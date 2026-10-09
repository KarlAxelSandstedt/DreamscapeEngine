REM run from the repository root, wherever the script is called from
cd /d "%~dp0..\.."
cmake -S . -B build -DDS_TEST_PHYSICS=ON -DDS_PROFILE=OFF -DDS_OPTIMIZE=OFF -DCMAKE_BUILD_TYPE=Debug -G Ninja 
cd build
cmake --build . --parallel
raddbg DreamscapeTest.exe ..\config\default.cfg
cd ..

