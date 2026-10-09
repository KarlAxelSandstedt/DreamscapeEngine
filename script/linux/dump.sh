#!/bin/bash
# run from the repository root, wherever the script is called from
cd "$(dirname "$0")/../.." || exit 1
#cmake -S . -B build
cd build
#cmake --build .
objdump --source --disassembler-options="intel" --disassemble=$1 engine_sandbox
cd ..
