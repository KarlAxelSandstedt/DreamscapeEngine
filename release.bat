cmake -S . -B build -DDS_TEST_PHYSICS=ON -DDS_PROFILE=OFF -DDS_OPTIMIZE=ON -DCMAKE_BUILD_TYPE=Release -G Ninja 
cd build
cmake --build . --parallel
cpack --config CPackConfig.cmake
::cpack --config CPackSourceConfig.cmake
