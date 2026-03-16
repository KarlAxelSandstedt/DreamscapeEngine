cmake -S . -B build -Dkas_debug=ON -DDS_TEST_PHYSICS=ON -DDS_DEBUG=ON -DDS_ASAN=ON -DDS_OPTIMIZE=ON -DCMAKE_BUILD_TYPE=Debug
cd build
cmake --build . --parallel
DreamscapeTest
cd ..
