# frdm-k64f-projects

this repo contains my personal projects using FRDM-K64F board.

## common build instructions
All my projects are based on GNU GCC and command line interface. I just don't like working with IDEs.
Whether you like it or not, Kinetis SDK is using CMake based Makefile generation. Although I am not a huge fan of CMake, all the samples are based on CMake. I'm just too lazy to come out with my own Makefile. Maybe sometime later when I get enough time to work on it.

Anyway CMakeLists.txt are modified so that all you gotta do to build is change the SdkRoot CMake variable in CMakeLists.txt and path in build_xxx.sh.
