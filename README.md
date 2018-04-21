# frdm-k64f-projects

![FRDM-K64F)(images/FRDM-K64F.jpg)

this repo contains my personal projects using FRDM-K64F board.

## common build instructions
All my projects are based on GNU GCC and command line interface. I just don't like working with IDEs.
Whether you like it or not, Kinetis SDK is using CMake based Makefile generation. Although I am not a huge fan of CMake, all the samples are based on CMake. I'm just too lazy to come out with my own Makefile. Maybe sometime later when I get enough time to work on it.

Anyway original CMakeLists.txt is modified so that all you gotta do to build is change the SdkRoot CMake variable in CMakeLists.txt and path in build_xxx.sh.

Usually you gotta follow this sequence
1. clone this repo
2. of course, set ARMGCC_DIR env variable properly as required by Kinetis SDK
3. modify SdkRoot in armgcc/CMakeLists.txt
4. modify armgcc.cmake pathin build_debug.sh (or build_release.sh)
5. happy making!
