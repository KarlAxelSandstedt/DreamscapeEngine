cmake -S . -B build -DDS_TEST_PHYSICS=ON -DDS_PROFILE=ON -DDS_OPTIMIZE=ON -DCMAKE_BUILD_TYPE=Release -G Ninja 
cd build
cmake --build . --parallel
DreamscapeTest.exe
cd ..
