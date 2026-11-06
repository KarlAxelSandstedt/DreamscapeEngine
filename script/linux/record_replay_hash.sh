#!/bin/bash
# run from the repository root, wherever the script is called from
cd "$(dirname "$0")/../.." || exit 1

if !(command -v cmake > /dev/null 2>&1); then
	echo "Error: CMake is not installed."
fi

if (command -v ninja > /dev/null 2>&1); then
	CMAKE_GENERATOR="Ninja"
else
	CMAKE_GENERATOR="Unix Makefiles"
fi

BUILD_CONFIG="build/build_config.txt"
BUILD_ID="Release"

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


cmake -S . -B build -DDS_TEST_PHYSICS=ON -DDS_DEBUG=OFF -DDS_PROFILE=OFF -DDS_OPTIMIZE=ON -DCMAKE_BUILD_TYPE=Release -G $CMAKE_GENERATOR
cd build
cmake --build . --parallel

# an interactive run recorded to record_replay_hash.rec (its commands) and record_replay_hash.hash (one hash per system
# tick); when it is closed, the same run replayed from the record with every tick's hash compared (a difference
# is fatal, a whole match is logged as a success). The logs: record.txt, replay.txt.
./DreamscapeTest config=../config/default.cfg log=record.txt record=record_replay_hash.rec hash=record_replay_hash.hash
./DreamscapeTest config=../config/default.cfg log=replay.txt replay=record_replay_hash.rec hash=record_replay_hash.hash
cd ..
