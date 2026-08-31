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


cmake -S . -B build -DDS_TEST_DETERMINISM=ON -DDS_DEBUG=ON -DDS_PROFILE=ON -DDS_OPTIMIZE=ON -DCMAKE_BUILD_TYPE=Release -G $CMAKE_GENERATOR
#cmake -S . -B build -DDS_TEST_DETERMINISM=ON -DDS_DEBUG=ON -DDS_PROFILE=ON -DDS_OPTIMIZE=ON -DCMAKE_BUILD_TYPE=Release -G $CMAKE_GENERATOR
cd build
cmake --build . --parallel

gdb --args ./DreamscapeTest --generate "determinism.bin"
#./DreamscapeTest --generate "determinism.bin"
#./DreamscapeTest --load "determinism.bin" 
#./DreamscapeTest --load "determinism.bin" 1
#./DreamscapeTest --load "determinism.bin" 2
#./DreamscapeTest --load "determinism.bin" 3
#./DreamscapeTest --load "determinism.bin" 4
#./DreamscapeTest --load "determinism.bin" 5
#./DreamscapeTest --load "determinism.bin" 6
#./DreamscapeTest --load "determinism.bin" 7
#./DreamscapeTest --load "determinism.bin" 8

cd ..
