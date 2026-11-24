/*
===========================================================================
fpvq3 - FPV quadcopter flight for ioquake3

This file is part of a Quake III Arena modification and is distributed
under the GNU General Public License version 2 or later, like the rest of
the ioquake3 game source it builds on.
===========================================================================
*/
//
// bg_drone.c -- quadcopter flight model shared by game and cgame
//
// The model is a rigid body with:
//   - Betaflight-style rates mapping stick deflection to target body rates
//   - a first-order rate controller (stands in for a tuned PID loop, airmode on)
//   - optional self-levelling (level mode) and velocity / altitude hold (easy
//     mode) on top of the rate controller, like a flight controller's angle mode
//   - spooled motors, thrust ~ rpm^2, thrust loss with axial inflow speed
//   - anisotropic quadratic drag in the body frame
//   - impulse-based collisions at the box corner, so hits make it tumble
//   - a settle torque so a disarmed drone comes to rest on a face
//
// Everything is integrated in fixed substeps of at most 2 ms so behaviour
// does not depend on client frame rate, and identically on server and client.

#include "../qcommon/q_shared.h"
#include "bg_public.h"

#define DRONE_MAX_SUBSTEP_MS	2

// body half-extents used for contact points (arms + props), units
#define DRONE_BODY_X			3.0f
#define DRONE_BODY_Y			3.0f
#define DRONE_BODY_Z			1.2f
#define DRONE_INERTIA			6.5f	// moment of inertia / mass, units^2

// level mode (and the attitude part of easy mode)
#define DRONE_LEVEL_ANGLE		55.0f	// tilt at full roll/pitch stick, deg
#define DRONE_LEVEL_GAIN		10.0f	// attitude error (rad) to body rate (rad/s)
#define DRONE_LEVEL_MAXRATE		10.0f	// cap on the self-levelling rate, rad/s

// easy mode
#define DRONE_EASY_SPEED		14.0f	// horizontal speed at full stick, m/s
#define DRONE_EASY_CLIMB		5.0f	// climb or sink rate at full throttle input, m/s
#define DRONE_EASY_TILT			35.0f	// most it tilts to reach the commanded speed, deg
#define DRONE_EASY_VTAU			0.5f	// horizontal speed response, s
#define DRONE_EASY_ZTAU			0.25f	// climb rate response, s
#define DRONE_EASY_YAWGAIN		12.0f	// heading error to yaw rate, 1/s
#define DRONE_EASY_MAXYAW		720.0f	// deg/s
#define DRONE_EASY_MAXPITCH		88.0f	// view pitch limit, deg (as for normal players)

// what one move feeds every substep
typedef struct {
	int			mode;			// DRONE_MODE_*
	float		throttle;		// 0..1 (acro, level)
	float		sticks[3];		// roll, pitch, yaw, -1..1
	float		climb;			// -1..1 (easy)
	float		viewYaw;		// deg (easy: heading to fly and point at)
	qboolean	inWater;
} droneInput_t;

/*
==================
BG_DroneDefaults

Values tuned for a 5" freestyle quad.
==================
*/
void BG_DroneDefaults( droneParams_t *p ) {
	int i;

	p->scale = 32.0f;
	p->gravity = 9.81f;
	p->twr = 8.0f;
	p->dragSide = 0.022f;
	p->dragTop = 0.05f;
	p->propSpeed = 70.0f;
	p->motorTau = 0.03f;
	p->rateTau = 0.02f;
	p->idle = 0.05f;
	p->crashSpeed = 0.0f;
	p->bounce = 0.3f;
	p->friction = 0.6f;
	p->weapons = 1;

	p->mode = DRONE_MODE_ACRO;
	p->camAngle = 30.0f;

	for ( i = 0; i < 3; i++ ) {
		p->rcRate[i] = 1.0f;
		p->superRate[i] = 0.7f;
		p->expo[i] = 0.0f;
	}
}

/*
==================
BG_DroneParseRates

"roll_rc roll_super roll_expo pitch_rc pitch_super pitch_expo yaw_rc yaw_super yaw_expo"
Missing or invalid values keep what is already in p.
==================
*/
void BG_DroneParseRates( const char *s, droneParams_t *p ) {
	char	*text = (char *)s;
	char	*tok;
	float	v;
	int		i;

	if ( !s ) {
		return;
	}

	for ( i = 0; i < 9; i++ ) {
		tok = COM_Parse( &text );
		if ( !tok[0] ) {
			return;
		}
		v = atof( tok );

		switch ( i % 3 ) {
		case 0:
			if ( v >= 0.01f && v <= 3.0f ) {
				p->rcRate[i / 3] = v;
			}
			break;
		case 1:
			if ( v >= 0.0f && v <= 0.99f ) {
				p->superRate[i / 3] = v;
			}
			break;
		default:
			if ( v >= 0.0f && v <= 1.0f ) {
				p->expo[i / 3] = v;
			}
			break;
		}
	}
}

/*
==================
BG_DroneParsePilot

dr_mode and dr_camangle from a client. Missing or invalid values keep what
is already in p.
==================
*/
void BG_DroneParsePilot( const char *mode, const char *camAngle, droneParams_t *p ) {
	int		m;
	float	a;

	if ( mode && mode[0] ) {
		m = atoi( mode );
		if ( m >= DRONE_MODE_ACRO && m <= DRONE_MODE_EASY ) {
			p->mode = m;
		}
	}
	if ( camAngle && camAngle[0] ) {
		a = atof( camAngle );
		if ( a >= 0.0f && a <= 80.0f ) {
			p->camAngle = a;
		}
	}
}

/*
==================
BG_DroneRate

Betaflight "classic" rates: stick -1..1 to degrees per second.
==================
*/
float BG_DroneRate( const droneParams_t *p, int axis, float stick ) {
	float	x, ax, expo, rc, rate, f;

	x = stick;
	if ( x > 1.0f ) {
		x = 1.0f;
	} else if ( x < -1.0f ) {
		x = -1.0f;
	}
	ax = fabs( x );

	expo = p->expo[axis];
	if ( expo > 0.0f ) {
		x = x * ax * ax * ax * expo + x * ( 1.0f - expo );
	}

	rc = p->rcRate[axis];
	if ( rc > 2.0f ) {
		rc += 14.54f * ( rc - 2.0f );
	}

	rate = 200.0f * rc * x;

	if ( p->superRate[axis] > 0.0f ) {
		f = 1.0f - ax * p->superRate[axis];
		if ( f < 0.01f ) {
			f = 0.01f;
		} else if ( f > 1.0f ) {
			f = 1.0f;
		}
		rate /= f;
	}

	if ( rate > 1998.0f ) {
		rate = 1998.0f;
	} else if ( rate < -1998.0f ) {
		rate = -1998.0f;
	}
	return rate;
}

/*
==================
BG_DroneThrottle

upmove -127..127 to throttle 0..1
==================
*/
float BG_DroneThrottle( int upmove ) {
	float t = ( upmove + 127 ) / 254.0f;

	if ( t < 0.0f ) {
		return 0.0f;
	}
	if ( t > 1.0f ) {
		return 1.0f;
	}
	return t;
}

/*
==================
BG_DroneAxisToAngles

Inverse of AnglesToAxis (axis[0] forward, axis[1] left, axis[2] up).
Stays well conditioned through vertical so flips do not jump.
==================
*/
void BG_DroneAxisToAngles( vec3_t axis[3], vec3_t angles ) {
	float	horiz;

	horiz = sqrt( axis[0][0] * axis[0][0] + axis[0][1] * axis[0][1] );
	angles[PITCH] = RAD2DEG( atan2( -axis[0][2], horiz ) );

	if ( horiz > 1e-4f ) {
		angles[YAW] = RAD2DEG( atan2( axis[0][1], axis[0][0] ) );
		angles[ROLL] = RAD2DEG( atan2( axis[1][2], axis[2][2] ) );
	} else {
		// looking straight up or down: fold all heading into yaw
		angles[ROLL] = 0.0f;
		angles[YAW] = RAD2DEG( atan2( -axis[1][0], axis[1][1] ) );
	}
}

/*
==================
Drone_Orthonormalize
==================
*/
static void Drone_Orthonormalize( vec3_t axis[3] ) {
	float d;

	VectorNormalize( axis[0] );
	d = DotProduct( axis[1], axis[0] );
	VectorMA( axis[1], -d, axis[0], axis[1] );
	VectorNormalize( axis[1] );
	CrossProduct( axis[0], axis[1], axis[2] );
}

/*
==================
Drone_Rotate

Rotate all three axis vectors around the world-space vector ww by |ww| * dt.
==================
*/
static void Drone_Rotate( vec3_t axis[3], const vec3_t ww, float dt ) {
	vec3_t	k, kxa, out;
	float	len, angle, c, s, kd;
	int		i;

	len = VectorLength( ww );
	angle = len * dt;
	if ( angle < 1e-7f ) {
		return;
	}
	VectorScale( ww, 1.0f / len, k );
	c = cos( angle );
	s = sin( angle );

	// Rodrigues
	for ( i = 0; i < 3; i++ ) {
		CrossProduct( k, axis[i], kxa );
		kd = DotProduct( k, axis[i] ) * ( 1.0f - c );
		out[0] = axis[i][0] * c + kxa[0] * s + k[0] * kd;
		out[1] = axis[i][1] * c + kxa[1] * s + k[1] * kd;
		out[2] = axis[i][2] * c + kxa[2] * s + k[2] * kd;
		VectorCopy( out, axis[i] );
	}
	Drone_Orthonormalize( axis );
}

static void Drone_BodyToWorld( vec3_t axis[3], const vec3_t b, vec3_t w ) {
	w[0] = b[0] * axis[0][0] + b[1] * axis[1][0] + b[2] * axis[2][0];
	w[1] = b[0] * axis[0][1] + b[1] * axis[1][1] + b[2] * axis[2][1];
	w[2] = b[0] * axis[0][2] + b[1] * axis[1][2] + b[2] * axis[2][2];
}

static void Drone_WorldToBody( vec3_t axis[3], const vec3_t w, vec3_t b ) {
	b[0] = DotProduct( w, axis[0] );
	b[1] = DotProduct( w, axis[1] );
	b[2] = DotProduct( w, axis[2] );
}

static void Drone_AddTouch( pmove_t *pmv, int entityNum ) {
	int i;

	if ( entityNum == ENTITYNUM_WORLD || pmv->numtouch >= MAXTOUCH ) {
		return;
	}
	for ( i = 0; i < pmv->numtouch; i++ ) {
		if ( pmv->touchents[i] == entityNum ) {
			return;
		}
	}
	pmv->touchents[pmv->numtouch++] = entityNum;
}

/*
==================
Drone_Impact

Resolve one contact against plane normal n with an impulse at the body
corner that is deepest along -n. w is body angular velocity (rad/s).
==================
*/
static void Drone_Impact( const droneParams_t *p, vec3_t axis[3], vec3_t w, vec3_t v,
						  const vec3_t n, int *flags ) {
	static const float half[3] = { DRONE_BODY_X, DRONE_BODY_Y, DRONE_BODY_Z };
	vec3_t	r, ww, vc, rn, vt, t, rt, J, rJ;
	float	vn, impact, e, kn, jn, vtl, kt, jt;
	int		i;

	// contact point relative to centre (world space)
	VectorClear( r );
	for ( i = 0; i < 3; i++ ) {
		VectorMA( r, DotProduct( axis[i], n ) > 0.0f ? -half[i] : half[i], axis[i], r );
	}

	Drone_BodyToWorld( axis, w, ww );
	CrossProduct( ww, r, vc );
	VectorAdd( vc, v, vc );

	vn = DotProduct( vc, n );

	// slow contact (resting, sliding): treat as a face contact through the
	// centre so it does not rock between corners; the settle torque in
	// Drone_Step lays it flat instead
	if ( vn < 0.0f && -vn < 0.5f * p->scale ) {
		VectorClear( r );
		VectorCopy( v, vc );
		vn = DotProduct( vc, n );
	}

	if ( vn < 0.0f ) {
		impact = -vn;

		if ( p->crashSpeed > 0.0f && ( *flags & DRF_ARMED ) && impact > p->crashSpeed * p->scale ) {
			*flags &= ~DRF_ARMED;
			*flags |= DRF_CRASHED;
		}

		// only bounce on real hits so resting contact stays calm
		e = impact > p->scale ? p->bounce : 0.0f;

		CrossProduct( r, n, rn );
		kn = 1.0f + DotProduct( rn, rn ) / DRONE_INERTIA;
		jn = ( 1.0f + e ) * impact / kn;
		VectorScale( n, jn, J );

		// Coulomb friction at the contact point
		VectorMA( vc, -vn, n, vt );
		vtl = VectorLength( vt );
		if ( vtl > 1e-3f ) {
			VectorScale( vt, 1.0f / vtl, t );
			CrossProduct( r, t, rt );
			kt = 1.0f + DotProduct( rt, rt ) / DRONE_INERTIA;
			jt = vtl / kt;
			if ( jt > p->friction * jn ) {
				jt = p->friction * jn;
			}
			VectorMA( J, -jt, t, J );
		}

		VectorAdd( v, J, v );
		CrossProduct( r, J, rJ );
		VectorMA( ww, 1.0f / DRONE_INERTIA, rJ, ww );
		Drone_WorldToBody( axis, ww, w );
	}

	// the centre can never move into the plane
	vn = DotProduct( v, n );
	if ( vn < 0.0f ) {
		VectorMA( v, -vn, n, v );
	}
}

/*
==================
Drone_Heading

Unit horizontal vector the drone points along. With the nose straight up
or down, the top of the frame gives the heading instead.
==================
*/
static void Drone_Heading( vec3_t axis[3], vec3_t out ) {
	VectorSet( out, axis[0][0], axis[0][1], 0.0f );
	if ( VectorNormalize( out ) > 0.2f ) {
		return;
	}
	// nose down: the top points ahead; nose up: it points back
	if ( axis[0][2] < 0.0f ) {
		VectorSet( out, axis[2][0], axis[2][1], 0.0f );
	} else {
		VectorSet( out, -axis[2][0], -axis[2][1], 0.0f );
	}
	if ( VectorNormalize( out ) < 1e-3f ) {
		VectorSet( out, 1.0f, 0.0f, 0.0f );
	}
}

/*
==================
Drone_LevelTarget

Target body rates (rad/s) for level and easy mode: turn the thrust axis
toward a wanted tilt, and yaw about the world vertical.
==================
*/
static void Drone_LevelTarget( const droneParams_t *p, vec3_t axis[3], const vec3_t velocity,
							   const droneInput_t *in, vec3_t target ) {
	static const vec3_t	worldUp = { 0.0f, 0.0f, 1.0f };
	vec3_t	heading, right, tilt, upWant, err, ww, vWant, vfwd, vright;
	float	t, s, c, angle, rate, yawRate, g, vlen, maxTilt;

	Drone_Heading( axis, heading );
	VectorSet( right, heading[1], -heading[0], 0.0f );

	if ( in->mode == DRONE_MODE_EASY ) {
		// tilt toward the acceleration that reaches the commanded velocity
		t = DEG2RAD( in->viewYaw );
		VectorSet( vfwd, cos( t ), sin( t ), 0.0f );
		VectorSet( vright, sin( t ), -cos( t ), 0.0f );
		VectorScale( vfwd, in->sticks[1], vWant );
		VectorMA( vWant, in->sticks[0], vright, vWant );
		vlen = VectorLength( vWant );
		if ( vlen > 1.0f ) {
			VectorScale( vWant, 1.0f / vlen, vWant );
		}
		VectorScale( vWant, DRONE_EASY_SPEED * p->scale, vWant );

		// plus what it takes to hold that speed against drag
		g = p->gravity * p->scale;
		vlen = VectorLength( vWant ) * p->dragSide / p->scale;
		tilt[0] = ( vWant[0] * vlen + ( vWant[0] - velocity[0] ) / DRONE_EASY_VTAU ) / g;
		tilt[1] = ( vWant[1] * vlen + ( vWant[1] - velocity[1] ) / DRONE_EASY_VTAU ) / g;
		tilt[2] = 0.0f;
		maxTilt = tan( DEG2RAD( DRONE_EASY_TILT ) );
		vlen = VectorLength( tilt );
		if ( vlen > maxTilt ) {
			VectorScale( tilt, maxTilt / vlen, tilt );
		}
	} else {
		// sticks are the tilt: + pitch noses down, + roll rolls right
		VectorScale( heading, tan( DEG2RAD( in->sticks[1] * DRONE_LEVEL_ANGLE ) ), tilt );
		VectorMA( tilt, tan( DEG2RAD( in->sticks[0] * DRONE_LEVEL_ANGLE ) ), right, tilt );
	}
	VectorAdd( tilt, worldUp, upWant );
	VectorNormalize( upWant );

	// rotation that takes the current thrust axis onto the wanted one
	CrossProduct( axis[2], upWant, err );
	s = VectorNormalize( err );
	c = DotProduct( axis[2], upWant );
	if ( s < 1e-5f ) {
		if ( c > 0.0f ) {
			VectorClear( err );
		} else {
			VectorCopy( axis[0], err );		// upside down: roll back over
		}
	}
	angle = atan2( s, c );
	rate = angle * DRONE_LEVEL_GAIN;
	if ( rate > DRONE_LEVEL_MAXRATE ) {
		rate = DRONE_LEVEL_MAXRATE;
	}
	VectorScale( err, rate, ww );

	// yaw about the world vertical; + yaw stick (or a view to the right) yaws right
	if ( in->mode == DRONE_MODE_EASY ) {
		yawRate = AngleNormalize180( in->viewYaw - RAD2DEG( atan2( heading[1], heading[0] ) ) )
			* DRONE_EASY_YAWGAIN;
		if ( yawRate > DRONE_EASY_MAXYAW ) {
			yawRate = DRONE_EASY_MAXYAW;
		} else if ( yawRate < -DRONE_EASY_MAXYAW ) {
			yawRate = -DRONE_EASY_MAXYAW;
		}
	} else {
		yawRate = -BG_DroneRate( p, 2, in->sticks[2] );
	}
	ww[2] += DEG2RAD( yawRate );

	Drone_WorldToBody( axis, ww, target );
}

/*
==================
Drone_EasyMotor

Easy mode altitude hold: the motor output that gives the commanded climb
rate, allowing for tilt and for the thrust lost to inflow.
==================
*/
static float Drone_EasyMotor( const droneParams_t *p, vec3_t axis[3], const vec3_t velocity,
							  float inflow, float climb, qboolean grounded ) {
	float	g, climbWant, accel, thrust, m;

	climbWant = climb * DRONE_EASY_CLIMB * p->scale;
	if ( grounded && climbWant <= 0.0f ) {
		return p->idle;		// sit on the floor instead of hovering against it
	}
	if ( axis[2][2] < 0.3f ) {
		return p->idle;		// far from level: let the attitude loop turn it over first
	}

	g = p->gravity * p->scale;
	accel = ( climbWant - velocity[2] ) / DRONE_EASY_ZTAU
		+ climbWant * fabs( climbWant ) * p->dragTop / p->scale;	// drag at that climb rate
	if ( accel < -g ) {
		accel = -g;
	} else if ( accel > 2.0f * g ) {
		accel = 2.0f * g;
	}
	thrust = ( g + accel ) / axis[2][2];

	if ( inflow < 0.2f ) {
		inflow = 0.2f;
	}
	m = sqrt( thrust / ( p->twr * g * inflow ) );
	if ( m < p->idle ) {
		m = p->idle;
	} else if ( m > 1.0f ) {
		m = 1.0f;
	}
	return m;
}

/*
==================
Drone_Step

One fixed substep. Returns qtrue if anything was touched.
==================
*/
static qboolean Drone_Step( pmove_t *pmv, const droneParams_t *p, vec3_t axis[3], vec3_t w,
							float *motor, const droneInput_t *in, int *flags, float dt ) {
	playerState_t	*ps = pmv->ps;
	vec3_t	target, ww, accel, end, probe, down;
	trace_t	tr;
	float	k, motorTarget, g, vAxial, inflow, thrust, speed, cs, ct, dragScale;
	float	timeLeft, best, d;
	qboolean armed, touched, grounded, onGround;
	int		i, bump, bestAxis;

	armed = ( *flags & DRF_ARMED ) != 0;
	touched = qfalse;
	g = p->gravity * p->scale;

	// sitting on the floor without enough thrust to lift: the frame rests on
	// the ground instead of the rate loop holding whatever tilt it landed at
	onGround = ( *flags & DRF_GROUNDED ) && p->twr * ( *motor ) * ( *motor ) < 1.0f;

	//
	// rotation
	//
	if ( armed && !onGround ) {
		if ( in->mode == DRONE_MODE_ACRO ) {
			// + roll stick rolls right (+ about forward), + pitch stick noses down
			// (+ about left), + yaw stick yaws right (- about up)
			target[0] = DEG2RAD( BG_DroneRate( p, 0, in->sticks[0] ) );
			target[1] = DEG2RAD( BG_DroneRate( p, 1, in->sticks[1] ) );
			target[2] = -DEG2RAD( BG_DroneRate( p, 2, in->sticks[2] ) );
		} else {
			Drone_LevelTarget( p, axis, ps->velocity, in, target );
		}
		k = dt / ( p->rateTau + dt );
		for ( i = 0; i < 3; i++ ) {
			w[i] += ( target[i] - w[i] ) * k;
		}
	} else {
		// unpowered or resting: a little aerodynamic damping
		VectorScale( w, 1.0f / ( 1.0f + 0.5f * dt ), w );
	}

	Drone_BodyToWorld( axis, w, ww );
	Drone_Rotate( axis, ww, dt );

	//
	// motors
	//
	vAxial = DotProduct( ps->velocity, axis[2] ) / p->scale;
	inflow = 1.0f - vAxial / ( p->propSpeed * ( *motor > 0.1f ? *motor : 0.1f ) );
	if ( inflow < 0.0f ) {
		inflow = 0.0f;
	} else if ( inflow > 1.2f ) {
		inflow = 1.2f;
	}

	if ( !armed ) {
		motorTarget = 0.0f;
	} else if ( in->mode == DRONE_MODE_EASY ) {
		motorTarget = Drone_EasyMotor( p, axis, ps->velocity, inflow, in->climb,
			( *flags & DRF_GROUNDED ) != 0 );
	} else {
		motorTarget = p->idle + ( 1.0f - p->idle ) * in->throttle;
	}
	*motor += ( motorTarget - *motor ) * dt / ( p->motorTau + dt );

	//
	// forces, in units/s^2
	//
	thrust = p->twr * g * ( *motor ) * ( *motor ) * inflow;

	dragScale = in->inWater ? 20.0f : 1.0f;
	if ( in->inWater ) {
		thrust *= 0.3f;
	}

	speed = VectorLength( ps->velocity );
	cs = dragScale * p->dragSide / p->scale * speed;
	ct = dragScale * p->dragTop / p->scale * speed;

	VectorScale( axis[2], thrust, accel );
	accel[2] -= g;
	VectorMA( accel, -cs * DotProduct( ps->velocity, axis[0] ), axis[0], accel );
	VectorMA( accel, -cs * DotProduct( ps->velocity, axis[1] ), axis[1], accel );
	VectorMA( accel, -ct * DotProduct( ps->velocity, axis[2] ), axis[2], accel );

	VectorMA( ps->velocity, dt, accel, ps->velocity );

	//
	// move and collide
	//
	timeLeft = dt;
	for ( bump = 0; bump < 4 && timeLeft > 0.0f; bump++ ) {
		VectorMA( ps->origin, timeLeft, ps->velocity, end );
		pmv->trace( &tr, ps->origin, pmv->mins, pmv->maxs, end, ps->clientNum, pmv->tracemask );

		if ( tr.allsolid ) {
			// embedded (teleport or spawn overlap): try to pop upward
			for ( i = 1; i <= 16; i++ ) {
				VectorCopy( ps->origin, probe );
				probe[2] += i;
				pmv->trace( &tr, probe, pmv->mins, pmv->maxs, probe, ps->clientNum, pmv->tracemask );
				if ( !tr.allsolid ) {
					VectorCopy( probe, ps->origin );
					break;
				}
			}
			VectorClear( ps->velocity );
			touched = qtrue;
			break;
		}

		if ( tr.fraction > 0.0f ) {
			VectorCopy( tr.endpos, ps->origin );
		}
		if ( tr.fraction == 1.0f ) {
			break;
		}

		touched = qtrue;
		Drone_AddTouch( pmv, tr.entityNum );
		timeLeft -= timeLeft * tr.fraction;
		Drone_Impact( p, axis, w, ps->velocity, tr.plane.normal, flags );
	}

	//
	// resting on the floor: settle onto whichever face is down
	//
	VectorCopy( ps->origin, down );
	down[2] -= 0.25f;
	pmv->trace( &tr, ps->origin, pmv->mins, pmv->maxs, down, ps->clientNum, pmv->tracemask );
	grounded = tr.fraction < 1.0f && tr.plane.normal[2] > 0.7f;

	if ( grounded ) {
		*flags |= DRF_GROUNDED;
	} else {
		*flags &= ~DRF_GROUNDED;
	}

	if ( grounded ) {
		touched = qtrue;
		if ( thrust < g ) {
			static const float bodyHalf[3] = { DRONE_BODY_X, DRONE_BODY_Y, DRONE_BODY_Z };
			vec3_t	face, into, torque;

			// the face it will tip onto: a box rests on face i while
			// tan(tilt) < other extent / own extent, which is the same as
			// picking the largest |cos(tilt)| / half-extent along that normal
			VectorNegate( tr.plane.normal, into );
			best = -2.0f;
			bestAxis = 0;
			for ( i = 0; i < 3; i++ ) {
				d = fabs( DotProduct( axis[i], into ) ) / bodyHalf[i];
				if ( d > best ) {
					best = d;
					bestAxis = i;
				}
			}
			VectorCopy( axis[bestAxis], face );
			if ( DotProduct( face, into ) < 0.0f ) {
				VectorNegate( face, face );
			}

			Drone_BodyToWorld( axis, w, ww );
			CrossProduct( face, into, torque );
			VectorMA( ww, 60.0f * dt, torque, ww );
			VectorScale( ww, 1.0f / ( 1.0f + 12.0f * dt ), ww );
			Drone_WorldToBody( axis, ww, w );
		}
	}

	return touched;
}

/*
==================
PM_DroneMove

Called from PmoveSingle for PM_DRONE. msec is the time covered by this call.
==================
*/
void PM_DroneMove( pmove_t *pmv, int msec ) {
	playerState_t	*ps = pmv->ps;
	droneParams_t	defaults;
	const droneParams_t	*p;
	droneInput_t	in;
	vec3_t	axis[3], w, cam, aim;
	float	motor, dt, tilt, c, s;
	int		flags, steps, i, buttons;
	short	pitch;
	qboolean touched;

	p = pmv->drone;
	if ( !p ) {
		BG_DroneDefaults( &defaults );
		p = &defaults;
	}

	buttons = pmv->cmd.buttons;
	flags = ps->stats[STAT_DRONE_FLAGS];

	memset( &in, 0, sizeof( in ) );
	in.mode = p->mode;
	in.throttle = BG_DroneThrottle( pmv->cmd.upmove );
	in.climb = pmv->cmd.upmove / 127.0f;
	in.sticks[0] = pmv->cmd.rightmove / 127.0f;
	in.sticks[1] = pmv->cmd.forwardmove / 127.0f;
	in.sticks[2] = pmv->droneYawStick;

	if ( in.mode == DRONE_MODE_EASY ) {
		// the view angles are the aim: clamp the pitch the way
		// PM_UpdateViewAngles does for a normal player
		pitch = pmv->cmd.angles[PITCH] + ps->delta_angles[PITCH];
		if ( pitch > ANGLE2SHORT( DRONE_EASY_MAXPITCH ) ) {
			ps->delta_angles[PITCH] = ANGLE2SHORT( DRONE_EASY_MAXPITCH ) - pmv->cmd.angles[PITCH];
		} else if ( pitch < -ANGLE2SHORT( DRONE_EASY_MAXPITCH ) ) {
			ps->delta_angles[PITCH] = -ANGLE2SHORT( DRONE_EASY_MAXPITCH ) - pmv->cmd.angles[PITCH];
		}
		in.viewYaw = SHORT2ANGLE( (short)( pmv->cmd.angles[YAW] + ps->delta_angles[YAW] ) );
	}

	if ( buttons & BUTTON_TALK ) {
		ps->eFlags |= EF_TALK;
	} else {
		ps->eFlags &= ~EF_TALK;
	}

	// The drone is armed whenever it is alive, except right after a spawn:
	// the motors stay off until some stick input arrives. A gamepad reports
	// nothing until a stick moves, and an all-zero command reads as half
	// throttle, which would launch the drone into the ceiling. A crash
	// (dr_crash) disarms it until the next respawn.
	if ( !( flags & DRF_LIVE ) && ( pmv->cmd.upmove || pmv->cmd.rightmove
			|| pmv->cmd.forwardmove || in.sticks[2] != 0.0f ) ) {
		flags |= DRF_LIVE;
	}
	if ( ( flags & DRF_LIVE ) && !( flags & DRF_CRASHED ) ) {
		flags |= DRF_ARMED;
	} else {
		flags &= ~DRF_ARMED;
	}

	// timers (teleport knockback etc.) still run down
	if ( ps->pm_time ) {
		if ( msec >= ps->pm_time ) {
			ps->pm_flags &= ~PMF_ALL_TIMES;
			ps->pm_time = 0;
		} else {
			ps->pm_time -= msec;
		}
	}
	ps->pm_flags &= ~( PMF_JUMP_HELD | PMF_DUCKED | PMF_BACKWARDS_JUMP | PMF_BACKWARDS_RUN | PMF_GRAPPLE_PULL );

	VectorSet( pmv->mins, -DRONE_HALF_SIZE, -DRONE_HALF_SIZE, -DRONE_HALF_SIZE );
	VectorSet( pmv->maxs, DRONE_HALF_SIZE, DRONE_HALF_SIZE, DRONE_HALF_SIZE );
	ps->viewheight = 0;
	ps->groundEntityNum = ENTITYNUM_NONE;
	ps->bobCycle = 0;

	AnglesToAxis( ps->viewangles, axis );
	w[0] = DEG2RAD( ps->grapplePoint[0] );
	w[1] = DEG2RAD( ps->grapplePoint[1] );
	w[2] = DEG2RAD( ps->grapplePoint[2] );
	motor = ps->stats[STAT_DRONE_MOTOR] / (float)DRONE_MOTOR_SCALE;

	in.inWater = ( pmv->pointcontents( ps->origin, ps->clientNum ) & MASK_WATER ) != 0;

	steps = ( msec + DRONE_MAX_SUBSTEP_MS - 1 ) / DRONE_MAX_SUBSTEP_MS;
	if ( steps < 1 ) {
		steps = 1;
	}
	dt = msec * 0.001f / steps;

	touched = qfalse;
	for ( i = 0; i < steps; i++ ) {
		if ( Drone_Step( pmv, p, axis, w, &motor, &in, &flags, dt ) ) {
			touched = qtrue;
		}
	}

	if ( touched ) {
		flags |= DRF_CONTACT;
	} else {
		flags &= ~DRF_CONTACT;
	}

	// write state back
	BG_DroneAxisToAngles( axis, ps->viewangles );
	ps->grapplePoint[0] = RAD2DEG( w[0] );
	ps->grapplePoint[1] = RAD2DEG( w[1] );
	ps->grapplePoint[2] = RAD2DEG( w[2] );

	if ( motor < 0.0f ) {
		motor = 0.0f;
	} else if ( motor > 1.0f ) {
		motor = 1.0f;
	}
	ps->stats[STAT_DRONE_MOTOR] = (int)( motor * DRONE_MOTOR_SCALE + 0.5f );
	flags = ( flags & ~DRF_MODE_MASK ) | ( in.mode << DRF_MODE_SHIFT );
	ps->stats[STAT_DRONE_FLAGS] = flags;

	// aim: the view in easy mode, otherwise the FPV camera (tilted up from the frame)
	if ( in.mode == DRONE_MODE_EASY ) {
		aim[PITCH] = SHORT2ANGLE( (short)( pmv->cmd.angles[PITCH] + ps->delta_angles[PITCH] ) );
		aim[YAW] = in.viewYaw;
	} else {
		tilt = DEG2RAD( p->camAngle );
		c = cos( tilt );
		s = sin( tilt );
		for ( i = 0; i < 3; i++ ) {
			cam[i] = axis[0][i] * c + axis[2][i] * s;
		}
		vectoangles( cam, aim );
	}
	ps->stats[STAT_DRONE_AIM_PITCH] = (short)ANGLE2SHORT( aim[PITCH] );
	ps->stats[STAT_DRONE_AIM_YAW] = (short)ANGLE2SHORT( aim[YAW] );
	ps->generic1 = DRONE_ENT_FLAG | ( (int)( motor * DRONE_ENT_MOTORMASK + 0.5f ) & DRONE_ENT_MOTORMASK );

	// water level for drowning and splashes
	if ( pmv->pointcontents( ps->origin, ps->clientNum ) & MASK_WATER ) {
		pmv->waterlevel = 3;
		pmv->watertype = pmv->pointcontents( ps->origin, ps->clientNum );
	} else {
		pmv->waterlevel = 0;
		pmv->watertype = 0;
	}
}

/*
==================
BG_DroneAimAngles

Direction a drone's shots go (pitch, yaw; roll 0), as left by its last move.
==================
*/
void BG_DroneAimAngles( const playerState_t *ps, vec3_t angles ) {
	angles[PITCH] = SHORT2ANGLE( (short)ps->stats[STAT_DRONE_AIM_PITCH] );
	angles[YAW] = SHORT2ANGLE( (short)ps->stats[STAT_DRONE_AIM_YAW] );
	angles[ROLL] = 0.0f;
}

/*
==================
BG_DroneMode

Flight mode the drone flew its last move in.
==================
*/
int BG_DroneMode( const playerState_t *ps ) {
	return ( ps->stats[STAT_DRONE_FLAGS] & DRF_MODE_MASK ) >> DRF_MODE_SHIFT;
}

/*
==================
BG_DroneSanitize

Clamp physics values to something that cannot blow up the integrator.
Game and cgame both run this so prediction sees identical numbers.
==================
*/
static float Drone_Clamp( float v, float lo, float hi, float fallback ) {
	if ( !( v == v ) ) {	// NaN
		return fallback;
	}
	if ( v < lo ) {
		return lo;
	}
	if ( v > hi ) {
		return hi;
	}
	return v;
}

void BG_DroneSanitize( droneParams_t *p ) {
	p->scale = Drone_Clamp( p->scale, 4.0f, 256.0f, 32.0f );
	p->gravity = Drone_Clamp( p->gravity, 0.0f, 50.0f, 9.81f );
	p->twr = Drone_Clamp( p->twr, 0.5f, 30.0f, 8.0f );
	p->dragSide = Drone_Clamp( p->dragSide, 0.0f, 1.0f, 0.022f );
	p->dragTop = Drone_Clamp( p->dragTop, 0.0f, 1.0f, 0.05f );
	p->propSpeed = Drone_Clamp( p->propSpeed, 5.0f, 300.0f, 70.0f );
	p->motorTau = Drone_Clamp( p->motorTau, 0.001f, 1.0f, 0.03f );
	p->rateTau = Drone_Clamp( p->rateTau, 0.001f, 1.0f, 0.02f );
	p->idle = Drone_Clamp( p->idle, 0.0f, 0.5f, 0.05f );
	p->crashSpeed = Drone_Clamp( p->crashSpeed, 0.0f, 200.0f, 0.0f );
	p->bounce = Drone_Clamp( p->bounce, 0.0f, 1.0f, 0.3f );
	p->friction = Drone_Clamp( p->friction, 0.0f, 3.0f, 0.6f );
	p->weapons = p->weapons ? 1 : 0;
	if ( p->mode < DRONE_MODE_ACRO || p->mode > DRONE_MODE_EASY ) {
		p->mode = DRONE_MODE_ACRO;
	}
	p->camAngle = Drone_Clamp( p->camAngle, 0.0f, 80.0f, 30.0f );
}
