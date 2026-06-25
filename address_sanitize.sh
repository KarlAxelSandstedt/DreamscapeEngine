#/bin/bash

if !(command -v cmake > /dev/null 2>&1); then
	echo "Error: CMake is not installed."
fi

if (command -v ninja > /dev/null 2>&1); then
	CMAKE_GENERATOR="Ninja"
else
	CMAKE_GENERATOR="Unix Makefiles"
fi

BUILD_CONFIG="build/build_config.txt"
BUILD_ID="ASAN"

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


cmake -S . -B build -Dkas_debug=ON -DDS_TEST_PHYSICS=ON -DDS_DEBUG=ON -DDS_ASAN=ON -DDS_OPTIMIZE=ON -DCMAKE_BUILD_TYPE=Debug -G $CMAKE_GENERATOR
cd build
cmake --build . --parallel
./DreamscapeTest
cd ..
