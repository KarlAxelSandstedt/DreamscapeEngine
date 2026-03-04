#/bin/bash

if !(command -v cmake > /dev/null 2>&1); then
	echo "Error: CMake is not installed."
fi

if (command -v ninja > /dev/null 2>&1); then
	CMAKE_GENERATOR="Ninja"
else
	CMAKE_GENERATOR="Unix Makefiles"
fi

#cmake -S . -B build -DDS_TEST_PERFORMANCE=ON -DDS_OPTIMIZE=ON -DCMAKE_BUILD_TYPE=Debug -G $CMAKE_GENERATOR
#cd build
#cmake --build . --parallel
#./DreamscapeTest
#cd ..

cmake -S . -B build -DDS_TEST_PERFORMANCE=ON -DDS_TSAN=ON -DDS_OPTIMIZE=ON -DCMAKE_BUILD_TYPE=Debug -G $CMAKE_GENERATOR
cd build
cmake --build . --parallel
gdb ./DreamscapeTest
cd ..

#cmake -S . -B build -DDS_TEST_CORRECTNESS=ON -DDS_TSAN=ON -DDS_OPTIMIZE=ON -DCMAKE_BUILD_TYPE=Debug -G $CMAKE_GENERATOR
#cd build
#cmake --build . --parallel
#gdb ./DreamscapeTest
#cd ..
