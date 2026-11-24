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
#   CHANNELS=AETR  the radio's channel order (A roll, E pitch, T throttle, R yaw)
#   FIRE_CHANNEL=5 radio channel (5-8) whose switch fires; empty for none
#   BOTS=<n>      fill the game with n bots (BOT_SKILL=1-5, default 3)
#   IOQ3=, IOQ3DED=, Q3BASE=          engine binaries and Quake III data folder
ROOT="$(cd "$(dirname "$0")" && pwd)"
ENGINE="${IOQ3:-/opt/quake3/ioquake3}"
DEDICATED="${IOQ3DED:-/opt/quake3/ioq3ded}"
BASE="${Q3BASE:-$HOME/Games/quake3}"
CONTROLS="${CONTROLS:-radio}"
CHANNELS="${CHANNELS:-AETR}"
FIRE_CHANNEL="${FIRE_CHANNEL-5}"

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
# (mode 2); controls/radio.cfg binds those sticks. CHANNELS is the radio's
# channel order, one letter per channel from CH1: A(ileron) = roll,
# E(levator) = pitch, T(hrottle), R(udder) = yaw. Set SDL_GAMECONTROLLERCONFIG
# yourself for other radios (tools/calibrate.py).
if [ -z "$SDL_GAMECONTROLLERCONFIG" ]; then
	order=$(printf %s "$CHANNELS" | tr a-z A-Z)
	case "$order" in
		????) ;;
		*) echo "CHANNELS=$CHANNELS: give the four letters A E T R in channel order, e.g. AETR or TAER" >&2; exit 1 ;;
	esac
	map=""
	for letter in A E T R; do
		rest="${order%%$letter*}"
		if [ "$rest" = "$order" ] || printf %s "${order#*$letter}" | grep -q "$letter"; then
			echo "CHANNELS=$CHANNELS: each of A, E, T and R must appear once" >&2
			exit 1
		fi
		axis=${#rest}		# CH1 is axis 0
		case $letter in
			A) map="${map}rightx:a$axis," ;;
			E) map="${map}righty:a$axis," ;;
			T) map="${map}lefty:a$axis," ;;
			R) map="${map}leftx:a$axis," ;;
		esac
	done
	# A switch channel fires while it is in its upper half: SDL turns the
	# axis into button A, which controls/radio.cfg binds to +attack.
	case "$FIRE_CHANNEL" in
		"") ;;
		5|6|7|8) map="${map}a:a$((FIRE_CHANNEL - 1))," ;;
		*) echo "FIRE_CHANNEL=$FIRE_CHANNEL: give a switch channel, 5 to 8 (or empty for none)" >&2; exit 1 ;;
	esac
	# The radio's SDL GUID (USB 1209:4f54 v0111) with the name checksum
	# (bytes 3-4) zeroed: SDL then matches it whatever name the firmware
	# reports ("TPRO", "TPro", OpenTX or EdgeTX).
	export SDL_GAMECONTROLLERCONFIG="0300000009120000544f000011010000,Jumper T-Pro,${map}platform:Linux,"
fi

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
