#!/bin/sh
# Compile and run the headless flight-model checks.
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SRC="$ROOT/ioq3/code"
OUT="${TMPDIR:-/tmp}/fpvq3_drone_sim"
cc -O2 -std=gnu99 -w -o "$OUT" "$ROOT/tests/drone_sim.c" \
	"$SRC/game/bg_drone.c" "$SRC/qcommon/q_math.c" "$SRC/qcommon/q_shared.c" -lm
"$OUT"
