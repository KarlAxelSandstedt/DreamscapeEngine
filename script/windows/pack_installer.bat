@echo off
REM run from the repository root, wherever the script is called from
setlocal
cd /d "%~dp0..\.." || exit /b 1

REM MSVC environment (cl, link, Windows SDK), unless already in a developer prompt
where cl >nul 2>&1 || call :vcvars
where cl >nul 2>&1 || (echo Error: MSVC is not installed. & exit /b 1)

where cmake >nul 2>&1 || (echo Error: CMake is not installed. & exit /b 1)

where ninja >nul 2>&1 && (set "CMAKE_GENERATOR=Ninja") || (set "CMAKE_GENERATOR=NMake Makefiles")

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DDS_OPTIMIZE=ON -G "%CMAKE_GENERATOR%" || exit /b 1
cd build
cmake --build . --parallel || exit /b 1
cpack --config CPackConfig.cmake
cd ..
exit /b

:vcvars
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" exit /b 1
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VS_PATH=%%i"
call "%VS_PATH%\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
exit /b
