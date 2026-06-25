#/bin/bash
#emcmake cmake -DCMAKE_BUILD_TYPE=Debug -S . -B build -Dkas_debug=ON -G Ninja

if !(command -v cmake > /dev/null 2>&1); then
	echo "Error: CMake is not installed."
fi

if (command -v ninja > /dev/null 2>&1); then
	CMAKE_GENERATOR="Ninja"
else
	CMAKE_GENERATOR="Unix Makefiles"
fi

BUILD_CONFIG="build/build_config.txt"
BUILD_ID="WebDebug"

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


emcmake cmake -S . -B build -Dkas_debug=ON -DCMAKE_BUILD_TYPE=Debug -G $CMAKE_GENERATOR
cd build
cmake --build . --parallel

emrun --browser=/usr/bin/brave-browser engine_sandbox.html
#node engine_sandbox.js
cd ..
