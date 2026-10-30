@echo off
REM run from the repository root, wherever the script is called from
setlocal
cd /d "%~dp0..\.." || exit /b 1

REM MSVC environment (cl, link, Windows SDK), unless already in a developer prompt
where cl >nul 2>&1 || call :vcvars
where cl >nul 2>&1 || (echo Error: MSVC is not installed. & exit /b 1)

where cmake >nul 2>&1 || (echo Error: CMake is not installed. & exit /b 1)

where ninja >nul 2>&1 && (set "CMAKE_GENERATOR=Ninja") || (set "CMAKE_GENERATOR=NMake Makefiles")

REM clang-cl (Visual Studio component "C++ Clang tools for Windows") if installed, else MSVC
where clang-cl >nul 2>&1 && (set "CC=clang-cl" & set "CXX=clang-cl")

if exist build\src rmdir /s /q build\src
if exist build\DreamscapeTest.exe del build\DreamscapeTest.exe
echo configure start %TIME%
cmake -S . -B build -DDS_TEST_PHYSICS=ON -DDS_PROFILE=ON -DCMAKE_BUILD_TYPE=Debug -G "%CMAKE_GENERATOR%" ^
    --profiling-format=google-trace ^
    --profiling-output="build\configure.trace.json" || exit /b 1
echo configure end   %TIME%

cd build
echo build start %TIME%
ninja -d explain -d stats -j%NUMBER_OF_PROCESSORS%
echo build end   %TIME%
where ninjatracing >nul 2>&1 && (ninjatracing .ninja_log > ninja_trace.json) || echo ninjatracing is not installed; skipping ninja_trace.json

REM NOTE: use Brave + brave://tracing to view profile graphically.
cd ..
exit /b

:vcvars
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" exit /b 1
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VS_PATH=%%i"
call "%VS_PATH%\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
exit /b
