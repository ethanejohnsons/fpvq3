#!/bin/sh
# Launch ioquake3 with fpvq3.
#
#   ./fly.sh [map] [engine args...]          fly on your own (default map q3dm17)
#   ./fly.sh --host [map] [engine args...]   host a game that others can join
#   ./fly.sh --join <address> [args...]      join someone's game
#   ./fly.sh --server [map] [args...]        dedicated server, no window
#
# Environment:
#   CONTROLS=radio|gamepad|keyboard   control setup (default radio)
#   BOTS=<n>      fill the game with n bots (BOT_SKILL=1-5, default 3)
#   IOQ3=, IOQ3DED=, Q3BASE=          engine binaries and Quake III data folder
ROOT="$(cd "$(dirname "$0")" && pwd)"
ENGINE="${IOQ3:-/opt/quake3/ioquake3}"
DEDICATED="${IOQ3DED:-/opt/quake3/ioq3ded}"
BASE="${Q3BASE:-$HOME/Games/quake3}"
CONTROLS="${CONTROLS:-radio}"

MODE=solo
case "$1" in
	--host) MODE=host; shift ;;
	--join) MODE=join; shift ;;
	--server) MODE=server; shift ;;
	-h|--help) sed -n '2,13p' "$0"; exit 0 ;;
esac

if [ "$MODE" = join ]; then
	ADDRESS="$1"
	if [ -z "$ADDRESS" ]; then
		echo "usage: $0 --join <address>" >&2
		exit 1
	fi
	shift
else
	MAP="${1:-q3dm17}"
	shift 2>/dev/null
fi

if [ ! -f "$ROOT/fpvq3/controls/$CONTROLS.cfg" ]; then
	echo "unknown CONTROLS=$CONTROLS (radio, gamepad or keyboard)" >&2
	exit 1
fi
if [ ! -f "$ROOT/run/fpvq3/fpvq3.pk3" ]; then
	echo "run ./install.sh first" >&2
	exit 1
fi

set -- +set fs_basepath "$BASE" +set fs_homepath "$ROOT/run" +set fs_game fpvq3 "$@"

BOTARGS=""
if [ -n "$BOTS" ] && [ "$BOTS" -gt 0 ]; then
	# bot_minplayers counts the human players too
	BOTARGS="+set bot_minplayers $((BOTS + 1)) +set g_spSkill ${BOT_SKILL:-3}"
fi

if [ "$MODE" = server ]; then
	# QVMs from fpvq3.pk3 on a pure server, so players without the mod
	# download it when they join
	exec "$DEDICATED" "$@" \
		+set dedicated 1 +set sv_pure 1 +set sv_allowDownload 1 +set vm_game 2 \
		+set g_drone 1 $BOTARGS \
		+map "$MAP"
fi

# SDL auto-maps the Jumper T-Pro as a gamepad with the throttle on a
# half-range trigger. Remap all four channels onto full-range sticks
# (mode 2); controls/radio.cfg binds those sticks. Override for other radios.
export SDL_GAMECONTROLLERCONFIG="${SDL_GAMECONTROLLERCONFIG:-0300d60609120000544f000011010000,OpenTX Jumper TPRO Joystick,leftx:a3,lefty:a2,rightx:a0,righty:a1,platform:Linux,}"

set -- "$@" \
	+set r_maxpolys 4000 +set r_maxpolyverts 20000 \
	+exec fpv.cfg +exec "controls/$CONTROLS.cfg"

cd "$(dirname "$ENGINE")"
case "$MODE" in
	solo)
		# native modules, and devmap so fpv.cfg's r_znear (a cheat cvar) applies
		exec "$ENGINE" "$@" \
			+set sv_pure 0 +set vm_game 0 +set vm_cgame 0 +set vm_ui 0 \
			+set g_drone 1 $BOTARGS \
			+devmap "$MAP" ;;
	host)
		# a pure server running the QVMs, so players without the mod download
		# fpvq3.pk3 when they join. devmap keeps r_znear working for everyone,
		# which also lets every player use cheat commands.
		exec "$ENGINE" "$@" \
			+set sv_pure 1 +set sv_allowDownload 1 \
			+set vm_game 2 +set vm_cgame 2 +set vm_ui 2 \
			+set g_drone 1 $BOTARGS \
			+devmap "$MAP" ;;
	join)
		exec "$ENGINE" "$@" \
			+set cl_allowDownload 1 \
			+connect "$ADDRESS" ;;
esac
