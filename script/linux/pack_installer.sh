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

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -Dapply_optimization_options=ON -G $CMAKE_GENERATOR
#cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug -Dkas_debug=ON -G Ninja
cd build
cmake --build . --parallel
cpack --config CPackConfig.cmake
cd ..
#readelf -d build/_CPack_Packages/Linux/DEB/game_1-0.1.0-Linux/opt/game_1/bin/game_1
