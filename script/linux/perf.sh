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

#cmake -S . -B build -Dapply_optimization_options=ON -G $CMAKE_GENERATOR
cmake -S . -B build -Dkas_debug=ON  -DCMAKE_BUILD_TYPE=Debug -G $CMAKE_GENERATOR
cd build
cmake --build . --parallel
./engine_sandbox &
sleep 1s
ProcID=$(pidof engine_sandbox)
perf record -p ${ProcID} --call-graph dwarf
perf report
cd ..
