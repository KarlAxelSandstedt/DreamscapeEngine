#/bin/bash

if !(command -v cmake > /dev/null 2>&1); then
	echo "Error: CMake is not installed."
fi

if (command -v ninja > /dev/null 2>&1); then
	CMAKE_GENERATOR="Ninja"
else
	CMAKE_GENERATOR="Unix Makefiles"
fi

cmake -S . -B build -Dkas_debug=ON -DDS_TEST_PHYSICS=ON -DDS_PROFILE=ON -DCMAKE_BUILD_TYPE=Debug -G $CMAKE_GENERATOR
cd build
cmake --build . --parallel

TRACY_PROFILER="../lib/tracy/profiler/build/tracy-profiler"
if [ -z $(pgrep -f "tracy-profiler")]; then
	nohup "$TRACY_PROFILER" > /dev/null 2>&1 &
fi

gdb ./DreamscapeTest
cd ..
