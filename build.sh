#!/bin/bash
type=Debug
target=pink_engine
while getopts 'Rt:' flag; do
        case "${flag}" in
                R) type=Release ;;
                t) target=${OPTARG} ;;
        esac
done
cmake -S . -B build -G"$NINJA_HOME/ninja.exe" -DCMAKE_BUILD_TYPE=$type
cmake --build build --target $target
