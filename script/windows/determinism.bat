@echo off
REM run from the repository root, wherever the script is called from
setlocal
cd /d "%~dp0..\.." || exit /b 1

REM MSVC environment (cl, link, Windows SDK), unless already in a developer prompt
where cl >nul 2>&1 || call :vcvars
where cl >nul 2>&1 || (echo Error: MSVC is not installed. & exit /b 1)

where cmake >nul 2>&1 || (echo Error: CMake is not installed. & exit /b 1)

where ninja >nul 2>&1 && (set "CMAKE_GENERATOR=Ninja") || (set "CMAKE_GENERATOR=NMake Makefiles")

set "BUILD_CONFIG=build\build_config.txt"
set "BUILD_ID=Determinism"

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

REM cmake -S . -B build -DDS_TEST_DETERMINISM=ON -DDS_DEBUG=ON -DDS_PROFILE=ON -DDS_OPTIMIZE=ON -DCMAKE_BUILD_TYPE=Release -G "%CMAKE_GENERATOR%"
cmake -S . -B build -DDS_TEST_DETERMINISM=ON -DDS_DEBUG=OFF -DDS_PROFILE=OFF -DDS_OPTIMIZE=ON -DCMAKE_BUILD_TYPE=Release -G "%CMAKE_GENERATOR%" || exit /b 1
cd build
cmake --build . --parallel || exit /b 1

REM reference: serial, system seed (config/determinism.cfg); writes determinism.bin and the config it ran
REM with, determinism.bin.cfg. Each replay uses that config with another thread_count (0: default).
REM raddbg .\DreamscapeTest.exe ..\config\determinism.cfg --generate determinism.bin
.\DreamscapeTest.exe ..\config\determinism.cfg --generate determinism.bin
REM 3 4 5 6 7 8
for %%T in (0 1 2) do (
    findstr /v /b /c:"thread_count:" determinism.bin.cfg > determinism_%%T.cfg
    >> determinism_%%T.cfg echo thread_count: %%T
    .\DreamscapeTest.exe determinism_%%T.cfg --load determinism.bin
)

cd ..
exit /b

:vcvars
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" exit /b 1
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VS_PATH=%%i"
call "%VS_PATH%\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
exit /b
