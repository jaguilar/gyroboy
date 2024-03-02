#! /bin/bash

set -e

cmake \
    -DCMAKE_TOOLCHAIN_FILE=armel.cmake \
    -S"$(pwd)" -B"$(pwd)/build" . 
(cd build && cmake --build . --target pid --config MinSizeRel -j 6)

# scp all python files and build/lib/libpid.so to a new subdirectory on ev3dev.local
INSTALL_DIR=/home/robot/gyroboy
ssh -t robot@ev3dev.local "rm -rf $INSTALL_DIR && mkdir -p $INSTALL_DIR"
scp -r *.py build/lib/libpid.so "robot@ev3dev.local:$INSTALL_DIR"
ssh -t robot@ev3dev.local "cd $INSTALL_DIR && LD_LIBRARY_PATH=$INSTALL_DIR brickrun -r -- pybricks-micropython main.py" || /bin/true
scp robot@ev3dev.local:"$INSTALL_DIR/*.csv" . 2>/dev/null || /bin/true