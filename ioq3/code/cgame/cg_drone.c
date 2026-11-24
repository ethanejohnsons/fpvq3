/*
===========================================================================
fpvq3 - FPV quadcopter flight for ioquake3

This file is part of a Quake III Arena modification and is distributed
under the GNU General Public License version 2 or later, like the rest of
the ioquake3 game source it builds on.
===========================================================================
*/
//
// cg_drone.c -- FPV camera, OSD and drone rendering
//

#include "cg_local.h"

static droneParams_t	cg_droneServer;		// physics from serverinfo
static droneParams_t	cg_droneRates;		// rates from our dr_rates cvar
static droneParams_t	cg_droneCombined;
static float			cg_droneHitbox = 12.0f;	// server's dr_hitbox: where shots hit a drone
static int				cg_droneRatesModCount = -1;

static int				cg_droneArmTime;
static int				cg_droneLastFlags;
static int				cg_droneCamShowTime = -100000;	// when the camera angle last changed

/*
=================
CG_DroneCamUp_f / CG_DroneCamDown_f

Step the FPV camera uptilt by dr_camstep degrees, 0..80.
=================
*/
static void CG_DroneCamStep( float dir ) {
	float step = dr_camstep.value > 0 ? dr_camstep.value : 5.0f;
	float a = dr_camangle.value + dir * step;

	if ( a < 0 ) {
		a = 0;
	} else if ( a > 80 ) {
		a = 80;
	}
	trap_Cvar_Set( "dr_camangle", va( "%i", (int)( a + 0.5f ) ) );
	trap_Cvar_Update( &dr_camangle );
	cg_droneCamShowTime = cg.time;
}

void CG_DroneCamUp_f( void ) {
	CG_DroneCamStep( 1 );
}

void CG_DroneCamDown_f( void ) {
	CG_DroneCamStep( -1 );
}

/*
=================
CG_DroneRegisterMedia
=================
*/
void CG_DroneRegisterMedia( void ) {
	cgs.media.droneSolidShader = trap_R_RegisterShader( "fpvq3/drone_solid" );
	cgs.media.dronePropShader = trap_R_RegisterShader( "fpvq3/drone_prop" );
}

/*
=================
CG_DroneParseServerinfo

Read the server's dr_* physics values. Missing keys keep defaults so a
plain server still predicts sensibly.
=================
*/
static void CG_DroneInfoFloat( const char *info, const char *key, float *out ) {
	const char *v = Info_ValueForKey( info, key );

	if ( v[0] ) {
		*out = atof( v );
	}
}

void CG_DroneParseServerinfo( const char *info ) {
	const char *v;

	BG_DroneDefaults( &cg_droneServer );
	CG_DroneInfoFloat( info, "dr_scale", &cg_droneServer.scale );
	CG_DroneInfoFloat( info, "dr_gravity", &cg_droneServer.gravity );
	CG_DroneInfoFloat( info, "dr_twr", &cg_droneServer.twr );
	CG_DroneInfoFloat( info, "dr_dragside", &cg_droneServer.dragSide );
	CG_DroneInfoFloat( info, "dr_dragtop", &cg_droneServer.dragTop );
	CG_DroneInfoFloat( info, "dr_propspeed", &cg_droneServer.propSpeed );
	CG_DroneInfoFloat( info, "dr_motortau", &cg_droneServer.motorTau );
	CG_DroneInfoFloat( info, "dr_ratetau", &cg_droneServer.rateTau );
	CG_DroneInfoFloat( info, "dr_idle", &cg_droneServer.idle );
	CG_DroneInfoFloat( info, "dr_crash", &cg_droneServer.crashSpeed );
	CG_DroneInfoFloat( info, "dr_bounce", &cg_droneServer.bounce );
	CG_DroneInfoFloat( info, "dr_friction", &cg_droneServer.friction );
	v = Info_ValueForKey( info, "dr_weapons" );
	if ( v[0] ) {
		cg_droneServer.weapons = atoi( v );
	}
	cg_droneHitbox = 12.0f;
	CG_DroneInfoFloat( info, "dr_hitbox", &cg_droneHitbox );
	BG_DroneSanitize( &cg_droneServer );
}

/*
=================
CG_DroneParams

Combined parameters for prediction; must match G_DroneParams on the server.
=================
*/
const droneParams_t *CG_DroneParams( void ) {
	int i;

	if ( dr_rates.modificationCount != cg_droneRatesModCount ) {
		cg_droneRatesModCount = dr_rates.modificationCount;
		BG_DroneDefaults( &cg_droneRates );
		BG_DroneParseRates( dr_rates.string, &cg_droneRates );
	}

	cg_droneCombined = cg_droneServer;
	for ( i = 0; i < 3; i++ ) {
		cg_droneCombined.rcRate[i] = cg_droneRates.rcRate[i];
		cg_droneCombined.superRate[i] = cg_droneRates.superRate[i];
		cg_droneCombined.expo[i] = cg_droneRates.expo[i];
	}
	// same parsing as the server applies to our userinfo
	BG_DroneParsePilot( dr_mode.string, dr_camangle.string, &cg_droneCombined );
	return &cg_droneCombined;
}

/*
=================
CG_DroneDebug

dr_debug 1 prints the snapshot and predicted state twice a second.
A non-finite view is always reported once.
=================
*/
static qboolean CG_DroneVecOK( const vec3_t v ) {
	return v[0] == v[0] && v[1] == v[1] && v[2] == v[2]
		&& fabs( v[0] ) < 1e6f && fabs( v[1] ) < 1e6f && fabs( v[2] ) < 1e6f;
}

void CG_DroneDebug( void ) {
	static int nextPrint, badReported;
	const playerState_t *sp = &cg.snap->ps, *pp = &cg.predictedPlayerState;
	qboolean bad;

	bad = !CG_DroneVecOK( cg.refdef.vieworg ) || !CG_DroneVecOK( cg.refdef.viewaxis[0] )
		|| !CG_DroneVecOK( cg.refdef.viewaxis[2] ) || !CG_DroneVecOK( pp->viewangles );
	if ( bad && !badReported ) {
		badReported = 1;
		CG_Printf( "^3fpvq3: bad view state, reporting below\n" );
	}
	if ( !( dr_debug.integer || ( bad && badReported == 1 ) ) || cg.time < nextPrint ) {
		return;
	}
	if ( bad ) {
		badReported = 2;
	}
	nextPrint = cg.time + 500;
	CG_Printf( "dr t%i snap%i ct%i/%i org(%.1f %.1f %.1f) ang(%.1f %.1f %.1f) vel(%.1f %.1f %.1f) fl%i\n",
		cg.time, cg.snap->serverTime, sp->commandTime, pp->commandTime,
		pp->origin[0], pp->origin[1], pp->origin[2],
		pp->viewangles[0], pp->viewangles[1], pp->viewangles[2],
		pp->velocity[0], pp->velocity[1], pp->velocity[2], pp->stats[STAT_DRONE_FLAGS] );
	CG_Printf( "   snaporg(%.1f %.1f %.1f) snapang(%.1f %.1f %.1f) err(%.1f %.1f %.1f) vieworg(%.1f %.1f %.1f) rdflags%i hyper%i\n",
		sp->origin[0], sp->origin[1], sp->origin[2],
		sp->viewangles[0], sp->viewangles[1], sp->viewangles[2],
		cg.predictedError[0], cg.predictedError[1], cg.predictedError[2],
		cg.refdef.vieworg[0], cg.refdef.vieworg[1], cg.refdef.vieworg[2], cg.refdef.rdflags, cg.hyperspace );
}

/*
=================
CG_IsDroneView
=================
*/
qboolean CG_IsDroneView( const playerState_t *ps ) {
	if ( ps->pm_type == PM_DRONE ) {
		return qtrue;
	}
	return ps->pm_type == PM_DEAD && ( ps->generic1 & DRONE_ENT_FLAG );
}

/*
=================
CG_DroneCamAngle

Uptilt of the FPV camera. Our own comes from dr_camangle; for a pilot we
are following, it is the angle between their frame and their aim, which
in acro and level mode is their camera.
=================
*/
static float CG_DroneCamAngle( const playerState_t *ps, vec3_t body[3] ) {
	vec3_t	aim, fwd;

	if ( !( ps->pm_flags & PMF_FOLLOW ) ) {
		return dr_camangle.value;
	}
	BG_DroneAimAngles( ps, aim );
	AngleVectors( aim, fwd, NULL, NULL );
	return RAD2DEG( atan2( DotProduct( fwd, body[2] ), DotProduct( fwd, body[0] ) ) );
}

/*
=================
CG_DroneView

Set up the camera. Acro and level: the FPV camera, at a fixed uptilt to
the frame, with full roll. Easy: the view angles, like a normal player,
since they are what the pilot aims with.
cg.refdef.vieworg already holds the (error-decayed) origin.
=================
*/
void CG_DroneView( const playerState_t *ps ) {
	vec3_t	body[3], cam[3], aim;
	float	tilt, c, s;
	int		i;
	qboolean easy;

	AnglesToAxis( ps->viewangles, body );
	easy = ps->pm_type == PM_DRONE && BG_DroneMode( ps ) == DRONE_MODE_EASY;

	if ( easy ) {
		BG_DroneAimAngles( ps, aim );
		AnglesToAxis( aim, cam );
	} else {
		tilt = DEG2RAD( CG_DroneCamAngle( ps, body ) );
		c = cos( tilt );
		s = sin( tilt );
		for ( i = 0; i < 3; i++ ) {
			cam[0][i] = body[0][i] * c + body[2][i] * s;
			cam[1][i] = body[1][i];
			cam[2][i] = body[2][i] * c - body[0][i] * s;
		}
	}

	if ( cg.renderingThirdPerson ) {
		// simple chase camera behind the drone's heading (easy: behind the aim)
		vec3_t	flat, want, dir, angles;
		vec3_t	mins = { -4, -4, -4 }, maxs = { 4, 4, 4 };
		trace_t	tr;
		vec3_t	target;

		VectorCopy( cg.refdef.vieworg, target );
		if ( easy ) {
			VectorSet( flat, cam[0][0], cam[0][1], 0 );
		} else {
			VectorSet( flat, body[0][0], body[0][1], 0 );
		}
		if ( VectorNormalize( flat ) < 0.2f ) {
			VectorSet( flat, body[2][0], body[2][1], 0 );
			VectorNormalize( flat );
		}
		VectorMA( target, -cg_thirdPersonRange.value * 0.5f, flat, want );
		want[2] += 16;
		CG_Trace( &tr, target, mins, maxs, want, ps->clientNum, MASK_SOLID );
		VectorCopy( tr.endpos, cg.refdef.vieworg );

		VectorSubtract( target, cg.refdef.vieworg, dir );
		vectoangles( dir, angles );
		AnglesToAxis( angles, cg.refdef.viewaxis );
		VectorCopy( angles, cg.refdefViewAngles );
		return;
	}

	// camera sits at the nose
	VectorMA( cg.refdef.vieworg, 1.5f, body[0], cg.refdef.vieworg );
	AxisCopy( cam, cg.refdef.viewaxis );
	if ( easy ) {
		VectorCopy( aim, cg.refdefViewAngles );
	} else {
		BG_DroneAxisToAngles( cam, cg.refdefViewAngles );
	}
}

/*
=================
CG_DroneMuzzle

Where a drone's shots start and which way they go, for weapon effects.
Returns qfalse if cent is not a drone.
=================
*/
qboolean CG_DroneMuzzle( centity_t *cent, vec3_t muzzle, vec3_t forward ) {
	vec3_t	aim;

	if ( !( cent->currentState.generic1 & DRONE_ENT_FLAG ) ) {
		return qfalse;
	}
	if ( cent->currentState.number == cg.predictedPlayerState.clientNum ) {
		BG_DroneAimAngles( &cg.predictedPlayerState, aim );
		VectorCopy( cg.predictedPlayerState.origin, muzzle );
	} else {
		VectorCopy( cent->currentState.angles2, aim );
		VectorCopy( cent->lerpOrigin, muzzle );
	}
	AngleVectors( aim, forward, NULL, NULL );
	VectorMA( muzzle, 14, forward, muzzle );	// as CalcMuzzlePoint on the server
	return qtrue;
}

/*
===============================================================================

DRONE MODEL

Built from polys at runtime so the mod needs no model assets.

===============================================================================
*/

static vec3_t	drone_lightDir = { 0.35f, 0.25f, 0.9f };

#define DRONE_GUN_SCALE		0.25f	// weapon models are sized for a player's hands
#define DRONE_MODEL_RADIUS	4.4f	// centre to prop tip, model units
#define DRONE_MAX_DRAWSCALE	8.0f

#define DRONE_TRAIL_STEP	20		// ms between trail puffs
#define DRONE_TRAIL_LIFE	900		// ms a puff lasts
#define DRONE_TRAIL_MINSPEED	64.0f	// units/s; slower drones leave no trail
#define DRONE_TRAIL_RADIUS	2.5f	// model units, so it scales with the drone
#define DRONE_TRAIL_FADEIN	150		// ms; keeps the trail's head off the drone itself

typedef struct {
	vec3_t	origin;
	vec3_t	axis[3];
	float	scale;		// model units -> world units
} droneFrame_t;

static void CG_DronePoint( const droneFrame_t *f, const vec3_t local, vec3_t out ) {
	VectorCopy( f->origin, out );
	VectorMA( out, local[0] * f->scale, f->axis[0], out );
	VectorMA( out, local[1] * f->scale, f->axis[1], out );
	VectorMA( out, local[2] * f->scale, f->axis[2], out );
}

/*
=================
CG_DroneDrawScale

A real 5" quad is a tiny target, smaller than the box shots hit (dr_hitbox).
Draw other pilots' drones at that box's size so what you aim at is what
you can hit; dr_dronescale overrides it. Our own drone, seen from the
chase camera, keeps its real size.
=================
*/
static float CG_DroneDrawScale( int clientNum ) {
	float s, hitbox;

	if ( clientNum == cg.snap->ps.clientNum ) {
		return 1.0f;
	}
	if ( dr_dronescale.value > 0 ) {
		s = dr_dronescale.value;
	} else if ( cg_droneServer.weapons ) {
		hitbox = cg_droneHitbox;	// as clamped in ClientThink_real
		if ( hitbox > 32.0f ) {
			hitbox = 32.0f;
		}
		s = hitbox / DRONE_MODEL_RADIUS;
	} else {
		s = 1.0f;
	}
	if ( s < 1.0f ) {
		s = 1.0f;
	} else if ( s > DRONE_MAX_DRAWSCALE ) {
		s = DRONE_MAX_DRAWSCALE;
	}
	return s;
}

/*
=================
CG_DroneTrail

A fading trail of puffs in the pilot's colour behind a moving drone, so
fast drones can be followed by eye.
=================
*/
static void CG_DroneTrail( centity_t *cent, const vec4_t color, float scale ) {
	vec3_t	pos;
	float	frac, speed;
	int		t, elapsed;

	elapsed = cg.time - cent->droneTrailTime;
	if ( !dr_trail.integer || cent->droneTrailTime == 0 || elapsed < 0 || elapsed > 250 ) {
		// first frame, or the drone was out of view or respawned: start afresh
		cent->droneTrailTime = cg.time;
		VectorCopy( cent->lerpOrigin, cent->droneTrailPos );
		return;
	}
	if ( elapsed == 0 ) {
		return;
	}

	speed = VectorLength( cent->currentState.pos.trDelta );
	if ( speed >= DRONE_TRAIL_MINSPEED ) {
		// one puff per step of game time, spread along this frame's movement
		for ( t = ( cent->droneTrailTime / DRONE_TRAIL_STEP + 1 ) * DRONE_TRAIL_STEP; t <= cg.time; t += DRONE_TRAIL_STEP ) {
			frac = (float)( t - cent->droneTrailTime ) / elapsed;
			VectorSubtract( cent->lerpOrigin, cent->droneTrailPos, pos );
			VectorMA( cent->droneTrailPos, frac, pos, pos );
			CG_SmokePuff( pos, vec3_origin, DRONE_TRAIL_RADIUS * scale,
				color[0], color[1], color[2], 0.6f,
				DRONE_TRAIL_LIFE, t, t + DRONE_TRAIL_FADEIN, LEF_PUFF_DONT_SCALE, cgs.media.smokePuffShader );
		}
	}
	cent->droneTrailTime = cg.time;
	VectorCopy( cent->lerpOrigin, cent->droneTrailPos );
}

static void CG_DroneQuad( qhandle_t shader, vec3_t p[4], const vec4_t color ) {
	polyVert_t	verts[4];
	vec3_t		e1, e2, n, l;
	float		shade;
	int			i;

	VectorSubtract( p[1], p[0], e1 );
	VectorSubtract( p[2], p[0], e2 );
	CrossProduct( e1, e2, n );
	VectorNormalize( n );
	VectorCopy( drone_lightDir, l );
	VectorNormalize( l );
	shade = 0.5f + 0.5f * fabs( DotProduct( n, l ) );

	for ( i = 0; i < 4; i++ ) {
		VectorCopy( p[i], verts[i].xyz );
		verts[i].st[0] = ( i == 1 || i == 2 ) ? 1 : 0;
		verts[i].st[1] = ( i >= 2 ) ? 1 : 0;
		verts[i].modulate[0] = (byte)( 255 * color[0] * shade );
		verts[i].modulate[1] = (byte)( 255 * color[1] * shade );
		verts[i].modulate[2] = (byte)( 255 * color[2] * shade );
		verts[i].modulate[3] = (byte)( 255 * color[3] );
	}
	trap_R_AddPolyToScene( shader, 4, verts );
}

// box in drone-local space: centre c, local axes u/v/w scaled to half extents
static void CG_DroneBox( const droneFrame_t *f, const vec3_t c, const vec3_t u, const vec3_t v,
						 const vec3_t w, const vec4_t color ) {
	static const int faces[6][4] = {
		{ 0, 1, 3, 2 }, { 4, 6, 7, 5 },		// -u, +u
		{ 0, 4, 5, 1 }, { 2, 3, 7, 6 },		// -v, +v
		{ 0, 2, 6, 4 }, { 1, 5, 7, 3 }		// -w, +w
	};
	vec3_t	corner[8], local, quad[4];
	int		i, j;

	for ( i = 0; i < 8; i++ ) {
		VectorCopy( c, local );
		VectorMA( local, ( i & 4 ) ? 1 : -1, u, local );
		VectorMA( local, ( i & 2 ) ? 1 : -1, v, local );
		VectorMA( local, ( i & 1 ) ? 1 : -1, w, local );
		CG_DronePoint( f, local, corner[i] );
	}
	for ( i = 0; i < 6; i++ ) {
		for ( j = 0; j < 4; j++ ) {
			VectorCopy( corner[faces[i][j]], quad[j] );
		}
		CG_DroneQuad( cgs.media.droneSolidShader, quad, color );
	}
}

static void CG_DroneAABox( const droneFrame_t *f, float cx, float cy, float cz,
						   float hx, float hy, float hz, const vec4_t color ) {
	vec3_t c, u, v, w;

	VectorSet( c, cx, cy, cz );
	VectorSet( u, hx, 0, 0 );
	VectorSet( v, 0, hy, 0 );
	VectorSet( w, 0, 0, hz );
	CG_DroneBox( f, c, u, v, w, color );
}

static void CG_DroneDisc( const droneFrame_t *f, const vec3_t centre, float radius, const vec4_t color ) {
	polyVert_t	verts[16];
	vec3_t		local;
	float		a;
	int			i;

	for ( i = 0; i < 16; i++ ) {
		a = i * ( 2.0f * M_PI / 16.0f );
		VectorSet( local, centre[0] + cos( a ) * radius, centre[1] + sin( a ) * radius, centre[2] );
		CG_DronePoint( f, local, verts[i].xyz );
		verts[i].st[0] = 0.5f + 0.5f * cos( a );
		verts[i].st[1] = 0.5f + 0.5f * sin( a );
		verts[i].modulate[0] = (byte)( 255 * color[0] );
		verts[i].modulate[1] = (byte)( 255 * color[1] );
		verts[i].modulate[2] = (byte)( 255 * color[2] );
		verts[i].modulate[3] = (byte)( 255 * color[3] );
	}
	trap_R_AddPolyToScene( cgs.media.dronePropShader, 16, verts );
}

/*
=================
CG_DroneEntity

Draw a player that is flying a drone.
=================
*/
void CG_DroneEntity( centity_t *cent ) {
	static const vec4_t frameColor = { 0.12f, 0.12f, 0.13f, 1.0f };
	static const vec4_t motorColor = { 0.35f, 0.35f, 0.38f, 1.0f };
	static const vec4_t camColor = { 1.0f, 0.45f, 0.1f, 1.0f };
	droneFrame_t	f;
	vec4_t			accent, propColor;
	clientInfo_t	*ci;
	vec3_t			c, u, v, w;
	float			motor, spin, a;
	int				i, clientNum;

	clientNum = cent->currentState.clientNum;
	if ( clientNum == cg.snap->ps.clientNum && !cg.renderingThirdPerson ) {
		return;		// we are looking out of its camera
	}

	VectorCopy( cent->lerpOrigin, f.origin );
	AnglesToAxis( cent->lerpAngles, f.axis );
	f.scale = CG_DroneDrawScale( clientNum );

	ci = &cgs.clientinfo[clientNum];
	VectorCopy( ci->color1, accent );
	accent[3] = 1.0f;
	if ( VectorLength( accent ) < 0.1f ) {
		VectorSet( accent, 0.2f, 0.8f, 1.0f );
	}

	if ( !( cent->currentState.eFlags & EF_DEAD ) ) {
		CG_DroneTrail( cent, accent, f.scale );
	} else {
		cent->droneTrailTime = 0;
	}

	motor = ( cent->currentState.generic1 & DRONE_ENT_MOTORMASK ) / (float)DRONE_ENT_MOTORMASK;

	// centre stack and top plate
	CG_DroneAABox( &f, 0, 0, 0, 1.6f, 0.9f, 0.25f, frameColor );
	CG_DroneAABox( &f, -0.1f, 0, 0.55f, 1.0f, 0.65f, 0.3f, accent );
	// FPV camera at the nose
	CG_DroneAABox( &f, 1.35f, 0, 0.45f, 0.25f, 0.35f, 0.3f, camColor );

	for ( i = 0; i < 4; i++ ) {
		float sx = ( i & 1 ) ? 1.0f : -1.0f;
		float sy = ( i & 2 ) ? 1.0f : -1.0f;

		// arm from centre to motor
		VectorSet( c, sx * 1.25f, sy * 1.25f, 0 );
		VectorSet( u, sx * 1.25f, sy * 1.25f, 0 );
		VectorSet( v, -sy * 0.25f, sx * 0.25f, 0 );
		VectorSet( w, 0, 0, 0.12f );
		CG_DroneBox( &f, c, u, v, w, frameColor );

		// motor bell
		CG_DroneAABox( &f, sx * 2.5f, sy * 2.5f, 0.3f, 0.35f, 0.35f, 0.3f, motorColor );

		// prop: translucent disc when spinning, two blades when stopped
		VectorSet( c, sx * 2.5f, sy * 2.5f, 0.68f );
		if ( motor > 0.01f ) {
			propColor[0] = propColor[1] = propColor[2] = 0.75f;
			propColor[3] = 0.15f + 0.25f * motor;
			CG_DroneDisc( &f, c, 1.9f, propColor );
		} else {
			a = DEG2RAD( 30.0f + i * 47.0f );
			spin = 1.8f;
			VectorSet( u, cos( a ) * spin, sin( a ) * spin, 0 );
			VectorSet( v, -sin( a ) * 0.2f, cos( a ) * 0.2f, 0 );
			VectorSet( w, 0, 0, 0.03f );
			CG_DroneBox( &f, c, u, v, w, accent );
		}
	}

	// the gun: a shrunken weapon model slung under the frame, pointing
	// where the shots go (CG_AddPlayerWeapon adds its flash and beam)
	if ( CG_DroneParams()->weapons && cent->currentState.weapon > WP_NONE
			&& !( cent->currentState.eFlags & EF_DEAD ) ) {
		refEntity_t	gun;
		vec3_t		aim;

		memset( &gun, 0, sizeof( gun ) );
		if ( clientNum == cg.predictedPlayerState.clientNum ) {
			BG_DroneAimAngles( &cg.predictedPlayerState, aim );
		} else {
			VectorCopy( cent->currentState.angles2, aim );
		}
		AnglesToAxis( aim, gun.axis );
		for ( i = 0; i < 3; i++ ) {
			VectorScale( gun.axis[i], DRONE_GUN_SCALE * f.scale, gun.axis[i] );
		}
		gun.nonNormalizedAxes = qtrue;
		VectorMA( f.origin, -1.4f * f.scale, f.axis[2], gun.origin );
		VectorCopy( gun.origin, gun.lightingOrigin );
		gun.renderfx = RF_LIGHTING_ORIGIN;
		CG_AddPlayerWeapon( &gun, NULL, cent, ci->team );
	}
}

/*
===============================================================================

OSD

===============================================================================
*/

static void CG_DroneStickBox( float x, float y, float size, float sx, float sy ) {
	static const vec4_t frame = { 1, 1, 1, 0.5f };
	static const vec4_t dot = { 1, 1, 1, 0.9f };
	float half = size * 0.5f;

	if ( sx > 1 ) sx = 1; else if ( sx < -1 ) sx = -1;
	if ( sy > 1 ) sy = 1; else if ( sy < -1 ) sy = -1;

	CG_DrawRect( x, y, size, size, 1, frame );
	CG_FillRect( x + half + sx * half - 2, y + half - sy * half - 2, 4, 4, dot );
}

static void CG_DroneCenterText( int y, const char *s, const vec4_t color ) {
	int w = CG_DrawStrlen( s ) * BIGCHAR_WIDTH;

	CG_DrawBigStringColor( 320 - w / 2, y, s, (float *)color );
}

/*
=================
CG_DroneDraw2D
=================
*/
void CG_DroneDraw2D( void ) {
	static const vec4_t white = { 1, 1, 1, 1 };
	static const vec4_t bad = { 1, 0.3f, 0.25f, 1 };
	static const char	*modeNames[] = { "ACRO", "LEVEL", "EASY" };
	playerState_t	*ps = &cg.predictedPlayerState;
	usercmd_t		cmd, prev;
	int				flags, current, secs, dtMs, mode;
	float			throttle, speed, yaw;
	const char		*s;
	qboolean		selecting;

	if ( ps->pm_type != PM_DRONE || !dr_osd.integer ) {
		return;
	}

	flags = ps->stats[STAT_DRONE_FLAGS];
	if ( ( flags & DRF_ARMED ) && !( cg_droneLastFlags & DRF_ARMED ) ) {
		cg_droneArmTime = cg.time;
	}
	cg_droneLastFlags = flags;

	current = trap_GetCurrentCmdNumber();
	trap_GetUserCmd( current, &cmd );
	throttle = BG_DroneThrottle( cmd.upmove );
	mode = BG_DroneMode( ps );

	// the weapon select bar (shown for 1.4 s after a change) sits where the
	// timer and stick overlay are
	selecting = CG_DroneParams()->weapons && cg.time - cg.weaponSelectTime < 1400;

	// status line
	if ( flags & DRF_CRASHED ) {
		CG_DroneCenterText( 120, "CRASHED - PRESS RESET", bad );
	} else if ( !( flags & DRF_LIVE ) ) {
		CG_DroneCenterText( 120, mode == DRONE_MODE_EASY ? "MOVE OR LOOK TO START" : "MOVE THE STICKS TO START", white );
	} else if ( cg.time - cg_droneArmTime < 1500 ) {
		CG_DroneCenterText( 120, "ARMED", white );
	}

	if ( cg.time - cg_droneCamShowTime < 1500 ) {
		CG_DroneCenterText( 150, va( "CAMERA %i DEG", dr_camangle.integer ), white );
	}

	// mode, throttle (easy: motor output, since the stick sets a climb rate), timer, speed
	CG_DrawSmallStringColor( 24, 382, modeNames[mode], (float *)white );
	if ( mode == DRONE_MODE_EASY ) {
		s = va( "PWR %3i%%", ps->stats[STAT_DRONE_MOTOR] * 100 / DRONE_MOTOR_SCALE );
	} else {
		s = va( "THR %3i%%", (int)( throttle * 100 + 0.5f ) );
	}
	CG_DrawBigStringColor( 24, 400, s, (float *)white );

	secs = ( flags & DRF_ARMED ) ? ( cg.time - cg_droneArmTime ) / 1000 : 0;
	s = va( "%02i:%02i", secs / 60, secs % 60 );
	if ( !selecting ) {
		CG_DrawBigStringColor( 320 - CG_DrawStrlen( s ) * BIGCHAR_WIDTH / 2, 400, s, (float *)white );
	}

	speed = VectorLength( ps->velocity ) / CG_DroneParams()->scale * 3.6f;
	s = va( "%3i KM/H", (int)( speed + 0.5f ) );
	CG_DrawBigStringColor( 548 - CG_DrawStrlen( s ) * BIGCHAR_WIDTH, 400, s, (float *)white );

	// stick overlay: left = yaw/throttle, right = roll/pitch (mode 2)
	if ( dr_sticks.integer && !selecting ) {
		yaw = 0;
		trap_GetUserCmd( current - 1, &prev );
		dtMs = cmd.serverTime - prev.serverTime;
		if ( dtMs > 0 ) {
			int delta = ( cmd.angles[YAW] - prev.angles[YAW] ) & 65535;
			if ( delta > 32767 ) {
				delta -= 65536;
			}
			yaw = -SHORT2ANGLE( delta ) / ( dtMs * 0.001f ) / DRONE_YAW_FULLSCALE;
		}
		CG_DroneStickBox( 268, 350, 44, yaw, throttle * 2 - 1 );
		CG_DroneStickBox( 328, 350, 44, cmd.rightmove / 127.0f, cmd.forwardmove / 127.0f );
	}
}
