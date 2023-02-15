@echo off
set TYPE=Debug
set TARGET=pink_engine
set RUN=""

CD build 2>NUL && CD .. || MD build

:GETOPTS
if /I "%1" == "-R" set TYPE=Release
if /I "%1" == "-t" set TARGET=%2 & shift
if /I "%1" == "-run" set RUN=1
shift
if not "%1" == "" goto GETOPTS

cmake -S . -B build ^
-DCMAKE_MAKE_PROGRAM:STRING="%NINJA_HOME%\ninja.exe" ^
-DCMAKE_BUILD_TYPE=%type% ^
-DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=TRUE ^
-G Ninja

cmake --build build --target %target%

if not %RUN% == "" (
        CD ".\out\%TYPE%"
        START pink_engine.exe
)
