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
BUILD_ID="WebDeterminism"

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

source "$EMSDK/emsdk_env.sh"
emcmake cmake -S . -B build \
     -DCMAKE_TOOLCHAIN_FILE=$EMSDK/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake \
    -DCMAKE_FIND_ROOT_PATH=$EMSDK/upstream/emscripten/cache/sysroot \
    -DDS_TEST_DETERMINISM=ON \
    -DDS_DEBUG=OFF \
    -DDS_PROFILE=OFF \
    -DDS_OPTIMIZE=ON \
    -DCMAKE_BUILD_TYPE=Release \
    -G $CMAKE_GENERATOR
cd build
cmake --build . --parallel

# headless browser with its own profile in build/; emrun writes the page's output files into build/
# (--dump-out-directory) and closes the browser when the page exits. A page that never exits (or never
# connects) fails after --timeout seconds instead of hanging the script.
BROWSER="${BROWSER:-/usr/bin/brave-browser}"
web_run() {
    emrun --timeout=900 --kill_exit --dump-out-directory=. --browser="$BROWSER" \
        --browser_args="--headless=new --enable-unsafe-swiftshader --user-data-dir=$PWD/browser_profile --no-first-run --no-default-browser-check" \
        DreamscapeTest.html -- "$@"
}

# reference: serial, system seed (config/determinism.cfg); writes determinism.bin and the config it ran
# with, determinism.bin.cfg. Each replay uses that config with another thread_count (0: default).
web_run ../config/determinism.cfg --generate determinism.bin
for THREAD_COUNT in 0 1 2; do # 3 4 5 6 7 8
    sed "s/^thread_count:.*/thread_count: $THREAD_COUNT/" determinism.bin.cfg > "determinism_$THREAD_COUNT.cfg"
    web_run "determinism_$THREAD_COUNT.cfg" --load determinism.bin
done

# far from the origin: the same seed, the scene built around (10 km, 10 km, 10 km) (f64 world positions);
# determinism_far.bin stores the origin, so the replays run there too.
web_run determinism.bin.cfg --generate determinism_far.bin --origin 10000 10000 10000
for THREAD_COUNT in 0 1 2; do # 3 4 5 6 7 8
    sed "s/^thread_count:.*/thread_count: $THREAD_COUNT/" determinism_far.bin.cfg > "determinism_far_$THREAD_COUNT.cfg"
    web_run "determinism_far_$THREAD_COUNT.cfg" --load determinism_far.bin
done

cd ..
