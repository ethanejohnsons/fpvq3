/*
===========================================================================
fpvq3 - FPV quadcopter flight for ioquake3

This file is part of a Quake III Arena modification and is distributed
under the GNU General Public License version 2 or later, like the rest of
the ioquake3 game source it builds on.
===========================================================================
*/
//
// ai_drone.c -- autopilot that lets the stock bots fly drones
//
// Bots keep their normal brain: goal selection, routing over the AAS walk
// graph, aiming and weapon choice. Three hooks turn that into flight:
//
//   - BotDroneSetupMovement: botlib plans for a walking player, so it is
//     given a "ghost" standing on the floor under the drone
//   - BotMoveToGoalDrone: after botlib picks the route, look ahead along it
//     for a point the drone can fly to in a straight line
//   - BotDroneInput: replace the walker's usercmd with easy-mode flight
//     input: a horizontal velocity toward that point, a climb rate that keeps
//     it at standing height over the route (or level with its enemy), and
//     the bot's own view angles, which in easy mode are its aim
//
// Bots always fly easy mode (see ClientUserinfoChanged), whose velocity and
// altitude hold run inside the shared flight model, so the bot only has to
// say where it wants to go.

#include "g_local.h"
#include "../botlib/botlib.h"
#include "../botlib/be_aas.h"
#include "../botlib/be_ea.h"
#include "../botlib/be_ai_char.h"
#include "../botlib/be_ai_chat.h"
#include "../botlib/be_ai_gen.h"
#include "../botlib/be_ai_goal.h"
#include "../botlib/be_ai_move.h"
#include "../botlib/be_ai_weap.h"
//
#include "ai_main.h"
#include "ai_drone.h"

#define DRONE_BOT_STAND			24.0f	// a standing player's origin above the floor
#define DRONE_BOT_LOOKFAR		192.0f	// route lookahead that sets the height to fly at
#define DRONE_BOT_LOOKNEAR		64.0f	// shorter lookahead when the far point is out of sight
#define DRONE_BOT_ARRIVE		48.0f	// slow down inside this distance of the target
#define DRONE_BOT_CLIMBTIME		0.5f	// close a height error in about this long, s
#define DRONE_BOT_DODGE			40.0f	// extra height while the bot "jumps" in a fight
#define DRONE_BOT_HOP			48.0f	// how far up or down to look for a way past a wall
#define DRONE_BOT_FIGHTSPEED	0.7f	// fraction of full speed while it has an enemy
#define DRONE_BOT_TARGETLIFE	0.5f	// a route point older than this is stale, s
#define DRONE_BOT_WALKSPEED		320.0f	// the bot input speed that means "run"

static const vec3_t droneMins = { -DRONE_HALF_SIZE, -DRONE_HALF_SIZE, -DRONE_HALF_SIZE };
static const vec3_t droneMaxs = { DRONE_HALF_SIZE, DRONE_HALF_SIZE, DRONE_HALF_SIZE };

/*
==================
BotIsDrone
==================
*/
static qboolean BotIsDrone( bot_state_t *bs ) {
	return g_entities[bs->client].client->ps.pm_type == PM_DRONE;
}

/*
==================
BotDroneFloor

Height of the floor below a point, or qfalse if there is none within reach.
==================
*/
static qboolean BotDroneFloor( int client, const vec3_t origin, float *floorZ ) {
	trace_t	tr;
	vec3_t	end;

	VectorCopy( origin, end );
	end[2] -= 1024;
	trap_Trace( &tr, origin, NULL, NULL, end, client, MASK_PLAYERSOLID );
	if ( tr.startsolid || tr.fraction >= 1.0f ) {
		return qfalse;
	}
	*floorZ = tr.endpos[2];
	return qtrue;
}

/*
==================
BotDroneCanFly

A straight flight from the drone to a point is clear of the world.
==================
*/
static qboolean BotDroneCanFly( int client, const vec3_t from, const vec3_t to ) {
	trace_t	tr;

	trap_Trace( &tr, from, droneMins, droneMaxs, to, client, MASK_SOLID | CONTENTS_PLAYERCLIP );
	return !tr.startsolid && tr.fraction >= 1.0f;
}

/*
==================
BotDroneClearAt

Moving dz up (or down) from here, and then on to the point ahead at that
height, are both clear.
==================
*/
static qboolean BotDroneClearAt( int client, const vec3_t origin, const vec3_t ahead, float dz ) {
	vec3_t	from, to;

	VectorCopy( origin, from );
	from[2] += dz;
	VectorCopy( ahead, to );
	to[2] += dz;
	return BotDroneCanFly( client, origin, from ) && BotDroneCanFly( client, from, to );
}

/*
==================
BotDroneSetupMovement

Called from BotSetupForMovement. botlib only knows how to walk, so hand it
a player standing on the floor under the drone.
==================
*/
void BotDroneSetupMovement( bot_state_t *bs, bot_initmove_t *initmove ) {
	float floorZ;

	if ( !BotIsDrone( bs ) ) {
		return;
	}
	initmove->viewoffset[2] = DEFAULT_VIEWHEIGHT;
	initmove->presencetype = PRESENCE_NORMAL;
	initmove->velocity[2] = 0;
	if ( BotDroneFloor( bs->client, bs->cur_ps.origin, &floorZ ) ) {
		initmove->origin[2] = floorZ + DRONE_BOT_STAND;
		initmove->or_moveflags |= MFL_ONGROUND;
	}
}

/*
==================
BotMoveToGoalDrone

Stands in for trap_BotMoveToGoal in the AI nodes. For a drone, also pick
the route point to fly toward until the next think.
==================
*/
void BotMoveToGoalDrone( bot_state_t *bs, bot_moveresult_t *result, bot_goal_t *goal, int travelflags ) {
	vec3_t	far, near;
	qboolean haveFar;

	trap_BotMoveToGoal( result, bs->ms, goal, travelflags );

	if ( !BotIsDrone( bs ) ) {
		return;
	}

	haveFar = trap_BotMovementViewTarget( bs->ms, goal, travelflags, DRONE_BOT_LOOKFAR, far );
	if ( !haveFar ) {
		// already in the goal's area (no reachability left to follow)
		if ( goal->areanum != bs->areanum && Distance( bs->origin, goal->origin ) > DRONE_BOT_LOOKFAR ) {
			bs->drone_targettime = 0;
			return;
		}
		VectorCopy( goal->origin, far );
	}

	// the far point sets the height, so ledges and jump pad landings are
	// climbed toward before the drone gets there
	VectorCopy( far, bs->drone_climbto );
	bs->drone_targettime = FloatTime();

	if ( BotDroneCanFly( bs->client, bs->origin, far ) ) {
		VectorCopy( far, bs->drone_target );
		bs->drone_targetvisible = qtrue;
	} else if ( trap_BotMovementViewTarget( bs->ms, goal, travelflags, DRONE_BOT_LOOKNEAR, near )
			&& BotDroneCanFly( bs->client, bs->origin, near ) ) {
		VectorCopy( near, bs->drone_target );
		bs->drone_targetvisible = qtrue;
	} else {
		bs->drone_targetvisible = qfalse;	// steer by botlib's walking direction
	}
}

/*
==================
BotDroneInput

Called from BotUpdateInput after the bot input became a usercmd. Rewrite
it as easy-mode flight input.
==================
*/
void BotDroneInput( bot_state_t *bs, bot_input_t *bi, usercmd_t *ucmd ) {
	gclient_t	*client = g_entities[bs->client].client;
	playerState_t *ps = &client->ps;
	droneParams_t	params;
	vec3_t	vWant, viewFwd, viewRight, ahead, angles;
	float	scale, maxSpeed, speed, dist, floorZ, zWant, climb, fwd, side, len;
	qboolean jumping, haveTarget;
	trace_t	tr;

	if ( ps->pm_type != PM_DRONE ) {
		return;
	}
	G_DroneParams( client, &params );
	scale = params.scale;
	maxSpeed = 14.0f * scale;	// DRONE_EASY_SPEED, full stick

	jumping = ( bi->actionflags & ACTION_JUMP ) != 0;

	// the getflag gesture shares its button with the drone reset switch;
	// use it only to get out of a crash
	ucmd->buttons &= ~BUTTON_DRONE_RESET;
	if ( ( ps->stats[STAT_DRONE_FLAGS] & DRF_CRASHED ) && !( client->pers.droneOldButtons & BUTTON_DRONE_RESET ) ) {
		ucmd->buttons |= BUTTON_DRONE_RESET;
	}

	//
	// where to go: toward the route point if it is fresh, else the way
	// botlib would walk (and the way the fight code strafes)
	//
	speed = bi->speed * 400.0f / 127.0f / DRONE_BOT_WALKSPEED;	// BotInputToUserCommand scaled it
	if ( speed > 1.0f ) {
		speed = 1.0f;
	}
	speed *= maxSpeed;
	if ( bs->enemy >= 0 ) {
		speed *= DRONE_BOT_FIGHTSPEED;	// strafe about as fast as a running player
	}

	haveTarget = bs->drone_targettime > FloatTime() - DRONE_BOT_TARGETLIFE;
	VectorClear( vWant );
	if ( haveTarget && bs->drone_targetvisible ) {
		VectorSubtract( bs->drone_target, ps->origin, vWant );
		vWant[2] = 0;
		dist = VectorNormalize( vWant );
		if ( dist < DRONE_BOT_ARRIVE ) {
			speed *= dist / DRONE_BOT_ARRIVE;
		}
		VectorScale( vWant, speed, vWant );
	} else {
		VectorSet( vWant, bi->dir[0], bi->dir[1], 0 );
		if ( VectorNormalize( vWant ) > 0.1f ) {
			VectorScale( vWant, speed, vWant );
		} else {
			VectorClear( vWant );
		}
	}

	//
	// how high: standing height over the floor and the route ahead, or
	// level with the enemy, plus a hop when the fight code jumps
	//
	if ( BotDroneFloor( bs->client, ps->origin, &floorZ ) ) {
		zWant = floorZ + DRONE_BOT_STAND;
	} else {
		zWant = ps->origin[2];
	}
	if ( haveTarget && bs->drone_climbto[2] > zWant ) {
		zWant = bs->drone_climbto[2];
	}
	if ( bs->enemy >= 0 && bs->enemy < MAX_GENTITIES && g_entities[bs->enemy].inuse
			&& g_entities[bs->enemy].r.currentOrigin[2] > zWant ) {
		zWant = g_entities[bs->enemy].r.currentOrigin[2];
	}
	if ( jumping ) {
		zWant += DRONE_BOT_DODGE;
	}

	climb = ( zWant - ps->origin[2] ) / ( DRONE_BOT_CLIMBTIME * 5.0f * scale );	// DRONE_EASY_CLIMB

	// a wall in the way at this height: go over it if that way is open,
	// else under it
	len = VectorLength( vWant );
	if ( len > 1.0f ) {
		VectorMA( ps->origin, 0.3f, vWant, ahead );
		trap_Trace( &tr, ps->origin, droneMins, droneMaxs, ahead, bs->client, MASK_SOLID | CONTENTS_PLAYERCLIP );
		if ( !tr.startsolid && tr.fraction < 1.0f && tr.plane.normal[2] < 0.7f ) {
			if ( BotDroneClearAt( bs->client, ps->origin, ahead, DRONE_BOT_HOP ) ) {
				climb = 1.0f;
			} else if ( BotDroneClearAt( bs->client, ps->origin, ahead, -DRONE_BOT_HOP ) ) {
				climb = -1.0f;
			}
		}
	}
	if ( climb > 1.0f ) {
		climb = 1.0f;
	} else if ( climb < -1.0f ) {
		climb = -1.0f;
	}

	//
	// easy mode wants the velocity relative to the view heading
	//
	VectorSet( angles, 0, bi->viewangles[YAW], 0 );
	AngleVectors( angles, viewFwd, viewRight, NULL );
	fwd = DotProduct( vWant, viewFwd ) / maxSpeed;
	side = DotProduct( vWant, viewRight ) / maxSpeed;

	ucmd->forwardmove = ClampChar( (int)( fwd * 127 ) );
	ucmd->rightmove = ClampChar( (int)( side * 127 ) );
	ucmd->upmove = ClampChar( (int)( climb * 127 ) );

	if ( dr_botdebug.integer && ( level.framenum % 20 ) == bs->client % 20 ) {
		G_Printf( "drbot %d org(%.0f %.0f %.0f) want(%.0f %.0f) z%.0f tgt%s cmd(%d %d %d) hp%d en%d%s w%d\n",
			bs->client, ps->origin[0], ps->origin[1], ps->origin[2], vWant[0], vWant[1], zWant,
			haveTarget ? ( bs->drone_targetvisible ? "+" : "~" ) : "-",
			ucmd->forwardmove, ucmd->rightmove, ucmd->upmove, ps->stats[STAT_HEALTH],
			bs->enemy, ( ucmd->buttons & BUTTON_ATTACK ) ? " FIRE" : "", ps->weapon );
	}
}
