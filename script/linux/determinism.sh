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
BUILD_ID="Determinism"

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


#cmake -S . -B build -DDS_TEST_DETERMINISM=ON -DDS_DEBUG=ON -DDS_PROFILE=ON -DDS_OPTIMIZE=ON -DCMAKE_BUILD_TYPE=Release -G $CMAKE_GENERATOR
cmake -S . -B build -DDS_TEST_DETERMINISM=ON -DDS_DEBUG=OFF -DDS_PROFILE=OFF -DDS_OPTIMIZE=ON -DCMAKE_BUILD_TYPE=Release -G $CMAKE_GENERATOR
cd build
cmake --build . --parallel

# reference: serial, system seed (config/determinism.cfg); writes determinism.bin and the config it ran
# with, determinism.bin.cfg. Each replay uses that config with another thread_count (0: default).
#gdb --args ./DreamscapeTest ../config/determinism.cfg --generate determinism.bin
./DreamscapeTest ../config/determinism.cfg --generate determinism.bin
for THREAD_COUNT in 0 1 2; do # 3 4 5 6 7 8
    sed "s/^thread_count:.*/thread_count: $THREAD_COUNT/" determinism.bin.cfg > "determinism_$THREAD_COUNT.cfg"
    ./DreamscapeTest "determinism_$THREAD_COUNT.cfg" --load determinism.bin
done

# far from the origin: the same seed, the scene built around (10 km, 10 km, 10 km) (f64 world positions);
# determinism_far.bin stores the origin, so the replays run there too.
./DreamscapeTest determinism.bin.cfg --generate determinism_far.bin --origin 10000 10000 10000
for THREAD_COUNT in 0 1 2; do # 3 4 5 6 7 8
    sed "s/^thread_count:.*/thread_count: $THREAD_COUNT/" determinism_far.bin.cfg > "determinism_far_$THREAD_COUNT.cfg"
    ./DreamscapeTest "determinism_far_$THREAD_COUNT.cfg" --load determinism_far.bin
done

cd ..
