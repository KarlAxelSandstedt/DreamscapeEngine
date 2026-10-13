@echo off
REM run from the repository root, wherever the script is called from
setlocal
cd /d "%~dp0..\.." || exit /b 1

REM Emscripten environment (emcc, emcmake, emrun), unless already on PATH
where emcc >nul 2>&1 || if defined EMSDK call "%EMSDK%\emsdk_env.bat" >nul 2>&1
where emcc >nul 2>&1 || (echo Error: Emscripten is not installed, see EMSDK. & exit /b 1)

REM Visual Studio ships cmake and ninja; its developer environment puts them on PATH
where cmake >nul 2>&1 && where ninja >nul 2>&1 || call :vcvars
where cmake >nul 2>&1 || (echo Error: CMake is not installed. & exit /b 1)
where ninja >nul 2>&1 || (echo Error: Ninja is not installed. & exit /b 1)
set "CMAKE_GENERATOR=Ninja"

REM override with the BROWSER environment variable
if not defined BROWSER set "BROWSER=C:\Program Files\BraveSoftware\Brave-Browser\Application\brave.exe"

set "BUILD_CONFIG=build\build_config.txt"
set "BUILD_ID=WebDebug"

if not exist build (
    mkdir build
    type nul > "%BUILD_CONFIG%"
)

set "BUILD_ID_CURRENT="
set /p BUILD_ID_CURRENT=<"%BUILD_CONFIG%" 2>nul
if not "%BUILD_ID%"=="%BUILD_ID_CURRENT%" (
    rmdir /s /q build
    mkdir build
    > "%BUILD_CONFIG%" echo %BUILD_ID%
    echo %BUILD_ID%
)

call emcmake cmake -S . -B build ^
    -DCMAKE_TOOLCHAIN_FILE="%EMSDK%/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake" ^
    -DCMAKE_FIND_ROOT_PATH="%EMSDK%/upstream/emscripten/cache/sysroot" ^
    -DDS_TEST_PHYSICS=ON ^
    -DDS_DEBUG=ON ^
    -DDS_OPTIMIZE=OFF ^
    -DCMAKE_BUILD_TYPE=Debug ^
    -G "%CMAKE_GENERATOR%" || exit /b 1
cd build
cmake --build . --parallel || exit /b 1

call emrun --browser="%BROWSER%" DreamscapeTest.html ../config/default.cfg
REM node engine_sandbox.js
cd ..
exit /b

:vcvars
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" exit /b 1
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VS_PATH=%%i"
call "%VS_PATH%\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
exit /b
