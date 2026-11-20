#!/bin/sh
# Launch ioquake3 with fpvq3 on a map (default q3dm17).
ROOT="$(cd "$(dirname "$0")" && pwd)"
ENGINE="${IOQ3:-/opt/quake3/ioquake3}"
BASE="${Q3BASE:-$HOME/Games/quake3}"
MAP="${1:-q3dm17}"
shift 2>/dev/null

# SDL auto-maps the Jumper T-Pro as a gamepad with the throttle on a
# half-range trigger. Remap all four channels onto full-range sticks
# (mode 2); fpv.cfg binds those sticks. Override for other radios.
export SDL_GAMECONTROLLERCONFIG="${SDL_GAMECONTROLLERCONFIG:-0300d60609120000544f000011010000,OpenTX Jumper TPRO Joystick,leftx:a3,lefty:a2,rightx:a0,righty:a1,platform:Linux,}"

cd "$(dirname "$ENGINE")"
exec "$ENGINE" \
	+set fs_basepath "$BASE" \
	+set fs_homepath "$ROOT/run" \
	+set fs_game fpvq3 \
	+set sv_pure 0 \
	+set vm_game 0 +set vm_cgame 0 +set vm_ui 0 \
	+set r_maxpolys 4000 +set r_maxpolyverts 20000 \
	+set g_drone 1 \
	+exec fpv.cfg \
	"$@" \
	+devmap "$MAP"
