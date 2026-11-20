#!/bin/sh
# Build the fpvq3 game modules and install them into the dev home folder.
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

mkdir -p "$MOD/vm"
for m in qagame cgame ui; do
	# copy then rename, so a game that is still running keeps its old files
	cp "$BUILD/Release/baseq3/$m.so" "$MOD/.${m}x86_64.so.new"
	mv -f "$MOD/.${m}x86_64.so.new" "$MOD/${m}x86_64.so"
	cp "$BUILD/Release/baseq3/vm/$m.qvm" "$MOD/vm/.$m.qvm.new"
	mv -f "$MOD/vm/.$m.qvm.new" "$MOD/vm/$m.qvm"
done
cp -r "$ROOT/fpvq3/." "$MOD/"
echo "installed to $MOD"
