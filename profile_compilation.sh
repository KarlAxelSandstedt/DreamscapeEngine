#!/bin/bash

export CC=clang
export CXX=clang++

if !(command -v cmake > /dev/null 2>&1); then
	echo "Error: CMake is not installed."
fi

if (command -v ninja > /dev/null 2>&1); then
	CMAKE_GENERATOR="Ninja"
else
	CMAKE_GENERATOR="Unix Makefiles"
fi

rm -rf "build/src"
rm "build/DreamscapeTest"
time cmake -S . -B build -DDS_TEST_PHYSICS=ON -DDS_PROFILE=ON -DCMAKE_BUILD_TYPE=Debug -G $CMAKE_GENERATOR \
    --profiling-format=google-trace \
    --profiling-output="build/configure.trace.json"

cd build
time ninja -d explain -d stats -j$(nproc)
ninjatracing .ninja_log > ninja_trace.json

#NOTE: use Brave + brave://tracing to view profile graphically. 
