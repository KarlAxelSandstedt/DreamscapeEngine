#!/bin/bash
# run.sh for the assembly test box (test/asm): the same build, then DreamscapeAsm instead of DreamscapeTest
# run from the repository root, wherever the script is called from
cd "$(dirname "$0")/../.." || exit 1

if !(command -v cmake > /dev/null 2>&1); then
	echo "Error: CMake is not installed."
fi

if !(command -v nasm > /dev/null 2>&1); then
	echo "Error: nasm is not installed."
	exit 1
fi

if (command -v ninja > /dev/null 2>&1); then
	CMAKE_GENERATOR="Ninja"
else
	CMAKE_GENERATOR="Unix Makefiles"
fi

BUILD_CONFIG="build/build_config.txt"
BUILD_ID="Run"

if [ ! -d build ]; then
    mkdir build
    touch "$BUILD_CONFIG"
fi

read -r BUILD_ID_CURRENT < "$BUILD_CONFIG"
if [ "$BUILD_ID" != "$BUILD_ID_CURRENT" ]; then
    rm -r build
    mkdir build
    touch "$BUILD_CONFIG"
    echo "$BUILD_ID" | tee "$BUILD_CONFIG"
fi

cmake -S . -B build -DDS_TEST_PHYSICS=ON -DDS_DEBUG=OFF -DDS_PROFILE=ON -DDS_OPTIMIZE=ON -DCMAKE_BUILD_TYPE=Release -G $CMAKE_GENERATOR
cd build
cmake --build . --parallel --target DreamscapeAsm || exit 1

TRACY_PROFILER="../lib/tracy/profiler/build/tracy-profiler"
if [ -z $(pgrep -f "tracy-profiler")]; then
	nohup "$TRACY_PROFILER" > /dev/null 2>&1 &
fi

./DreamscapeAsm
cd ..
