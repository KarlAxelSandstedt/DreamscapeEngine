cmake -S . -B build -DDS_TEST_PHYSICS=ON -DDS_PROFILE=OFF -DDS_OPTIMIZE=ON -DDS_ASAN=ON -DCMAKE_BUILD_TYPE=Release -G Ninja 
cd build
cmake --build . --parallel
DreamscapeTest
cd ..
