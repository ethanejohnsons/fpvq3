// Headless checks for the fpvq3 flight model (bg_drone.c) against a flat floor at z=0.
// Build: see tests/run.sh
#include "../ioq3/code/qcommon/q_shared.h"
#include "../ioq3/code/game/bg_public.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

void QDECL Com_Error( int level, const char *fmt, ... ) { va_list a; va_start(a, fmt); vprintf(fmt, a); va_end(a); exit(1); }
void QDECL Com_Printf( const char *fmt, ... ) { va_list a; va_start(a, fmt); vprintf(fmt, a); va_end(a); }

static float floorZ = 0;

static void FloorTrace( trace_t *tr, const vec3_t start, const vec3_t mins, const vec3_t maxs,
						const vec3_t end, int pass, int mask ) {
	float s = start[2] + mins[2] - floorZ, e = end[2] + mins[2] - floorZ;
	memset( tr, 0, sizeof( *tr ) );
	tr->entityNum = ENTITYNUM_NONE;
	if ( s < 0 ) { tr->allsolid = tr->startsolid = qtrue; VectorCopy( start, tr->endpos ); return; }
	if ( e >= 0.125f ) { tr->fraction = 1; VectorCopy( end, tr->endpos ); return; }
	// stop 1/8 unit above the plane like the real collision code
	tr->fraction = ( s - 0.125f ) / ( s - e );
	if ( tr->fraction < 0 ) tr->fraction = 0;
	tr->endpos[0] = start[0] + tr->fraction * ( end[0] - start[0] );
	tr->endpos[1] = start[1] + tr->fraction * ( end[1] - start[1] );
	tr->endpos[2] = start[2] + tr->fraction * ( end[2] - start[2] );
	VectorSet( tr->plane.normal, 0, 0, 1 );
	tr->entityNum = ENTITYNUM_WORLD;
}
static int NoContents( const vec3_t p, int pass ) { return 0; }

static playerState_t ps;
static pmove_t pm;
static droneParams_t params;
static float viewPitch, viewYaw;	// easy mode: the view angles sent in the usercmd

static void Reset( float z, float pitch, float roll ) {
	memset( &ps, 0, sizeof( ps ) );
	ps.pm_type = PM_DRONE;
	ps.origin[2] = z;
	ps.viewangles[PITCH] = pitch;
	ps.viewangles[ROLL] = roll;
	memset( &pm, 0, sizeof( pm ) );
	pm.ps = &ps; pm.trace = FloorTrace; pm.pointcontents = NoContents;
	pm.tracemask = MASK_PLAYERSOLID; pm.drone = &params;
}

// run for ms milliseconds at 4 ms frames
static void Run( int ms, float throttle, float roll, float pitch, float yaw, int buttons ) {
	int t;
	for ( t = 0; t < ms; t += 4 ) {
		pm.cmd.upmove = (signed char)( throttle * 254 - 127 );
		pm.cmd.rightmove = (signed char)( roll * 127 );
		pm.cmd.forwardmove = (signed char)( pitch * 127 );
		pm.cmd.buttons = buttons;
		pm.cmd.angles[PITCH] = ANGLE2SHORT( viewPitch );
		pm.cmd.angles[YAW] = ANGLE2SHORT( viewYaw );
		pm.droneYawStick = yaw;
		PM_DroneMove( &pm, 4 );
	}
}

static float HorizSpeed( void ) {
	return sqrt( ps.velocity[0] * ps.velocity[0] + ps.velocity[1] * ps.velocity[1] ) / params.scale;
}

static int failures;
static void Check( const char *what, int ok, const char *fmt, ... ) {
	va_list a; printf( "%s  %-44s ", ok ? "PASS" : "FAIL", what );
	va_start( a, fmt ); vprintf( fmt, a ); va_end( a ); printf( "\n" );
	if ( !ok ) failures++;
}

int main( void ) {
	float speed, z0, hover, lo, hi;
	int i;

	BG_DroneDefaults( &params );

	// Throttle 0.5 with centred sticks is an all-zero command, which keeps a
	// freshly spawned drone disarmed (spawn hold); the drop tests use that.

	// 1. disarmed drop (30 cm) settles flat and stops
	Reset( 13, 20, 35 );
	Run( 6000, 0.5f, 0, 0, 0, 0 );
	Check( "disarmed drop comes to rest", VectorLength( ps.velocity ) < 1 && VectorLength( ps.grapplePoint ) < 0.5f,
		"z=%.2f |v|=%.3f |w|=%.2f", ps.origin[2], VectorLength( ps.velocity ), VectorLength( ps.grapplePoint ) );
	Check( "lands on its belly", fabs( ps.viewangles[PITCH] ) < 3 && fabs( ps.viewangles[ROLL] ) < 3,
		"pitch=%.1f roll=%.1f", ps.viewangles[PITCH], ps.viewangles[ROLL] );

	// 2. upside-down drop (30 cm) stays upside down
	Reset( 13, 0, 160 );
	Run( 6000, 0.5f, 0, 0, 0, 0 );
	Check( "upside-down drop rests on its back", fabs( fabs( ps.viewangles[ROLL] ) - 180 ) < 5,
		"roll=%.1f", ps.viewangles[ROLL] );

	// 2b. a 3 m drop with a hard corner hit still comes to rest on some face
	Reset( 100, 20, 35 );
	Run( 8000, 0.5f, 0, 0, 0, 0 );
	{
		vec3_t ax[3]; float m = 0; int k;
		AnglesToAxis( ps.viewangles, ax );
		for ( k = 0; k < 3; k++ ) if ( fabs( ax[k][2] ) > m ) m = fabs( ax[k][2] );
		Check( "3 m tumble ends at rest on a face", m > 0.99f && VectorLength( ps.grapplePoint ) < 0.5f,
			"face=%.3f |w|=%.2f", m, VectorLength( ps.grapplePoint ) );
	}

	// 3. spawn hold: no stick input yet keeps the motors off, any input arms
	Reset( 40.0f, 0, 0 );
	Run( 1000, 0.5f, 0, 0, 0, 0 );
	Check( "spawn hold: no input keeps motors off", !( ps.stats[STAT_DRONE_FLAGS] & DRF_ARMED )
		&& ps.stats[STAT_DRONE_MOTOR] == 0 && ps.origin[2] < 40.0f,
		"flags=%d motor=%d z=%.1f", ps.stats[STAT_DRONE_FLAGS], ps.stats[STAT_DRONE_MOTOR], ps.origin[2] );
	Run( 20, 0.6f, 0, 0, 0, 0 );
	Check( "spawn hold releases on stick input", ( ps.stats[STAT_DRONE_FLAGS] & ( DRF_ARMED | DRF_LIVE ) ) == ( DRF_ARMED | DRF_LIVE ),
		"flags=%d", ps.stats[STAT_DRONE_FLAGS] );

	// 4. take off from the ground: climbs and stays level
	Reset( 3.2f, 0, 0 );
	Run( 1000, 0, 0, 0, 0, 0 );
	Run( 100, 0, 0, 0, 0, 0 );
	Run( 1000, 0.6f, 0, 0, 0, 0 );
	Check( "takeoff climbs and stays level", ps.origin[2] > 100 && fabs( ps.viewangles[ROLL] ) < 1 &&
		fabs( ps.viewangles[PITCH] ) < 1, "z=%.0f pitch=%.2f roll=%.2f", ps.origin[2], ps.viewangles[PITCH], ps.viewangles[ROLL] );

	// 5. find hover throttle by bisection on vertical speed after 2 s
	lo = 0; hi = 1;
	for ( i = 0; i < 20; i++ ) {
		hover = ( lo + hi ) / 2;
		Reset( 500, 0, 0 );
		Run( 50, 0, 0, 0, 0, 0 );
		Run( 2000, hover, 0, 0, 0, 0 );
		if ( ps.velocity[2] > 0 ) hi = hover; else lo = hover;
	}
	Check( "hover throttle in a 5\" quad range (25-45%)", hover > 0.25f && hover < 0.45f, "hover=%.1f%%", hover * 100 );

	// 6. full-stick roll rate (rc 1.0, super 0.7 -> 667 deg/s)
	Reset( 500, 0, 0 );
	Run( 50, 0, 0, 0, 0, 0 );
	Run( 200, hover, 1, 0, 0, 0 );
	Check( "full roll rate ~667 deg/s", fabs( ps.grapplePoint[0] - 666.7f ) < 15, "rate=%.1f", ps.grapplePoint[0] );

	// 7. vertical punch-out speed
	Reset( 500, 0, 0 );
	Run( 50, 0, 0, 0, 0, 0 );
	Run( 6000, 1, 0, 0, 0, 0 );
	speed = ps.velocity[2] / params.scale * 3.6f;
	Check( "full-throttle climb 80-200 km/h", speed > 80 && speed < 200, "%.0f km/h", speed );

	// 8. level-ish forward flight at full throttle, nose down ~70 deg
	Reset( 5000, 70, 0 );
	Run( 50, 0, 0, 0, 0, 0 );
	ps.viewangles[PITCH] = 70;
	z0 = ps.origin[2];
	Run( 8000, 1, 0, 0, 0, 0 );
	speed = sqrt( ps.velocity[0] * ps.velocity[0] + ps.velocity[1] * ps.velocity[1] ) / params.scale * 3.6f;
	Check( "forward top speed 90-200 km/h", speed > 90 && speed < 200, "%.0f km/h, climb %.0f u/s",
		speed, ps.velocity[2] );

	// 9. flip through vertical: 360 deg pitch keeps orientation consistent
	Reset( 3000, 0, 0 );
	Run( 50, 0, 0, 0, 0, 0 );
	{
		// integrate rate and compare against the commanded total
		float total = 0;
		for ( i = 0; i < 2000 && total < 360; i++ ) {
			Run( 4, hover, 0, 1, 0, 0 );
			total += ps.grapplePoint[1] * 0.004f;
		}
		Run( 400, hover, 0, 0, 0, 0 );
		Check( "360 flip returns level", fabs( ps.viewangles[ROLL] ) < 3 && fabs( ps.viewangles[PITCH] ) < 15,
			"pitch=%.1f roll=%.1f yaw=%.1f", ps.viewangles[PITCH], ps.viewangles[ROLL], ps.viewangles[YAW] );
	}

	// 10. crash detection when enabled
	params.crashSpeed = 10;
	Reset( 3000, 0, 0 );
	Run( 50, 0, 0, 0, 0, 0 );
	Run( 10000, 0.0f, 0, 0, 0, 0 );
	Check( "hard landing with dr_crash 10 disarms", ( ps.stats[STAT_DRONE_FLAGS] & DRF_CRASHED ) != 0,
		"flags=%d", ps.stats[STAT_DRONE_FLAGS] );
	params.crashSpeed = 0;

	// 11. axis/angle round trip near vertical
	{
		vec3_t ang = { 89.99f, 37, 12 }, ax[3], back, ax2[3];
		float err = 0; int k, j;
		AnglesToAxis( ang, ax );
		BG_DroneAxisToAngles( ax, back );
		AnglesToAxis( back, ax2 );
		for ( k = 0; k < 3; k++ ) for ( j = 0; j < 3; j++ ) err += fabs( ax[k][j] - ax2[k][j] );
		Check( "axis->angles round trip near vertical", err < 1e-3f, "err=%g", err );
	}

	// 12. aim in acro: the FPV camera, tilted up from the frame
	Reset( 500, 0, 0 );
	Run( 50, 0, 0, 0, 0, 0 );
	{
		vec3_t aim;
		BG_DroneAimAngles( &ps, aim );
		Check( "acro aims along the 30 deg camera", fabs( aim[PITCH] + 30 ) < 0.5f && BG_DroneMode( &ps ) == DRONE_MODE_ACRO,
			"aim pitch=%.1f", aim[PITCH] );
	}

	//
	// level mode
	//
	params.mode = DRONE_MODE_LEVEL;

	// 13. full roll stick holds the angle limit, release levels it
	Reset( 3000, 0, 0 );
	Run( 50, 0, 0, 0, 0, 0 );
	Run( 600, hover, 1, 0, 0, 0 );
	Check( "level: full roll stick tilts ~55 deg", fabs( ps.viewangles[ROLL] - 55 ) < 3,
		"roll=%.1f", ps.viewangles[ROLL] );
	Run( 600, hover, 0, 0, 0, 0 );
	Check( "level: released sticks return to level", fabs( ps.viewangles[ROLL] ) < 1 && fabs( ps.viewangles[PITCH] ) < 1,
		"pitch=%.2f roll=%.2f", ps.viewangles[PITCH], ps.viewangles[ROLL] );

	// 14. full pitch stick noses down to the limit and stays there
	Run( 600, hover, 0, 1, 0, 0 );
	Check( "level: full pitch stick noses down ~55 deg", fabs( ps.viewangles[PITCH] - 55 ) < 3,
		"pitch=%.1f", ps.viewangles[PITCH] );

	// 15. upside down in the air: rights itself
	Reset( 3000, 0, 180 );
	Run( 50, 0.6f, 0, 0, 0, 0 );
	Run( 1000, hover, 0, 0, 0, 0 );
	Check( "level: recovers from upside down", fabs( ps.viewangles[ROLL] ) < 2 && fabs( ps.viewangles[PITCH] ) < 2,
		"pitch=%.1f roll=%.1f", ps.viewangles[PITCH], ps.viewangles[ROLL] );

	// 16. yaw stick still turns at the acro yaw rate
	Reset( 3000, 0, 0 );
	Run( 50, 0.6f, 0, 0, 0, 0 );
	Run( 300, hover, 0, 0, 1, 0 );
	Check( "level: full yaw stick ~667 deg/s", fabs( -ps.grapplePoint[2] - 666.7f ) < 20,
		"yaw rate=%.1f", -ps.grapplePoint[2] );

	//
	// easy mode
	//
	params.mode = DRONE_MODE_EASY;
	viewPitch = viewYaw = 0;

	// 17. centred sticks hold altitude
	Reset( 3000, 0, 0 );
	Run( 50, 0.6f, 0, 0, 0, 0 );		// any input arms it
	z0 = ps.origin[2];
	Run( 3000, 0.5f, 0, 0, 0, 0 );		// all-zero command
	Check( "easy: centred sticks hold altitude", fabs( ps.origin[2] - z0 ) < 16 && fabs( ps.velocity[2] ) < 4,
		"dz=%.1f vz=%.1f", ps.origin[2] - z0, ps.velocity[2] );

	// 18. full climb input climbs at ~5 m/s, full down sinks at ~5 m/s
	Run( 2000, 1, 0, 0, 0, 0 );
	Check( "easy: full climb ~5 m/s", fabs( ps.velocity[2] / params.scale - 5 ) < 0.5f,
		"%.2f m/s", ps.velocity[2] / params.scale );
	Run( 2000, 0, 0, 0, 0, 0 );
	Check( "easy: full sink ~5 m/s", fabs( ps.velocity[2] / params.scale + 5 ) < 0.5f,
		"%.2f m/s", ps.velocity[2] / params.scale );

	// 19. full forward stick reaches the set speed along the view, release stops it
	viewYaw = 90;
	Reset( 3000, 0, 0 );
	Run( 50, 0.6f, 0, 0, 0, 0 );
	Run( 5000, 0.5f, 0, 1, 0, 0 );
	Check( "easy: full stick flies ~14 m/s along the view", fabs( HorizSpeed() - 14 ) < 1.5f
		&& ps.velocity[1] > 0.95f * HorizSpeed() * params.scale,
		"%.1f m/s (vx %.0f vy %.0f)", HorizSpeed(), ps.velocity[0], ps.velocity[1] );
	Check( "easy: the frame turns to the view heading", fabs( AngleNormalize180( ps.viewangles[YAW] - 90 ) ) < 2,
		"yaw=%.1f", ps.viewangles[YAW] );
	Run( 3000, 0.5f, 0, 0, 0, 0 );
	Check( "easy: released sticks stop it", HorizSpeed() < 0.5f && fabs( ps.viewangles[PITCH] ) < 2,
		"%.2f m/s pitch=%.1f", HorizSpeed(), ps.viewangles[PITCH] );

	// 20. aim follows the view, not the frame
	viewPitch = 20;
	Run( 100, 0.5f, 1, 0, 0, 0 );
	{
		vec3_t aim;
		BG_DroneAimAngles( &ps, aim );
		Check( "easy: aims along the view", fabs( aim[PITCH] - 20 ) < 0.1f && fabs( aim[YAW] - 90 ) < 0.1f,
			"aim pitch=%.1f yaw=%.1f", aim[PITCH], aim[YAW] );
	}
	viewPitch = 0;

	// 21. sitting on the floor with centred sticks stays put
	Reset( 3.2f, 0, 0 );
	Run( 50, 0.6f, 0, 0, 0, 0 );
	Run( 2000, 0.5f, 0, 0, 0, 0 );
	Check( "easy: stays on the floor until climb input", ps.origin[2] < 4 && ps.stats[STAT_DRONE_MOTOR] < 0.1f * DRONE_MOTOR_SCALE,
		"z=%.2f motor=%d", ps.origin[2], ps.stats[STAT_DRONE_MOTOR] );
	Run( 1000, 1, 0, 0, 0, 0 );
	Check( "easy: takes off with climb input", ps.origin[2] > 64, "z=%.0f", ps.origin[2] );

	params.mode = DRONE_MODE_ACRO;

	printf( "\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL PASSED", failures, failures == 1 ? "" : "s" );
	return failures != 0;
}
