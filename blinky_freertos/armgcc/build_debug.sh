#!/bin/sh
cmake -DCMAKE_TOOLCHAIN_FILE="../../../sdk/tools/cmake_toolchain_files/armgcc.cmake" -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=debug  .
make -j4
