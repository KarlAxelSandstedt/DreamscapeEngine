cmake -S . -B build -DDS_TEST_PHYSICS=ON -DDS_PROFILE=OFF -DCMAKE_BUILD_TYPE=Debug
cd build
cmake --build . --parallel
raddbg Debug\DreamscapeTest
cd ..

