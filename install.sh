#!/bin/sh
# Build the fpvq3 game modules and install them into the dev home folder:
#   fpvq3.pk3        QVMs, configs and shaders; what a server hands to
#                    players who join without the mod (see fly.sh --host)
#   *x86_64.so       native modules, used for local play (fly.sh)
set -e
ROOT="$(cd "$(dirname "$0")" && pwd)"
BUILD="$ROOT/ioq3/build"
MOD="${MOD:-$ROOT/run/fpvq3}"

# a build directory configured in another location (project moved) can't be reused
if [ -f "$BUILD/CMakeCache.txt" ] && ! grep -q "^CMAKE_HOME_DIRECTORY:INTERNAL=$ROOT/ioq3\$" "$BUILD/CMakeCache.txt"; then
	rm -rf "$BUILD"
fi
if [ ! -f "$BUILD/CMakeCache.txt" ]; then
	cmake -S "$ROOT/ioq3" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release \
		-DBUILD_CLIENT=OFF -DBUILD_SERVER=OFF \
		-DBUILD_RENDERER_GL1=OFF -DBUILD_RENDERER_GL2=OFF
fi
cmake --build "$BUILD" -j"$(nproc)" --target qagame_baseq3 cgame_baseq3 ui_baseq3 \
	qagameqvm_baseq3 cgameqvm_baseq3 uiqvm_baseq3

mkdir -p "$MOD"
for m in qagame cgame ui; do
	# copy then rename, so a game that is still running keeps its old files
	cp "$BUILD/Release/baseq3/$m.so" "$MOD/.${m}x86_64.so.new"
	mv -f "$MOD/.${m}x86_64.so.new" "$MOD/${m}x86_64.so"
done

# everything else goes in the pk3
STAGE="$BUILD/pk3"
rm -rf "$STAGE"
mkdir -p "$STAGE/vm"
cp "$BUILD/Release/baseq3/vm/"*.qvm "$STAGE/vm/"
cp -r "$ROOT/fpvq3/scripts" "$ROOT/fpvq3/controls" "$ROOT/fpvq3/fpv.cfg" "$STAGE/"
rm -f "$STAGE.pk3"
(cd "$STAGE" && zip -qr -X "$STAGE.pk3" .)
cp "$STAGE.pk3" "$MOD/.fpvq3.pk3.new"
mv -f "$MOD/.fpvq3.pk3.new" "$MOD/fpvq3.pk3"
cp "$ROOT/fpvq3/description.txt" "$MOD/"

# loose copies from older installs would shadow the pk3
rm -rf "$MOD/vm" "$MOD/scripts" "$MOD/controls" "$MOD/fpv.cfg"

echo "installed to $MOD"
