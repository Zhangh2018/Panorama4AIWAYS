##############################################################################
#
#  CONTENT
#    Build instructions for Openvx library
#
##############################################################################

  1) Download openvx version 1.2
  ----------------------------------
    - download openvx source from: https://www.khronos.org/registry/OpenVX/sample/openvx_sample_1.2.tar.bz2
    - extract its contents to working directory (let's say $SDK_ROOT/../) we will have "openvx_sample" directory
    - put "evb.cross.cmake" and "ovx_build.py" into same folder with "openvx_sample"

  2) Modify some file to compatible with your system:
  ----------------------------------
    - Edit COMPILER_FOLDER_NAME in evb.cross.cmake up to your compiler folder
    - Edit file /openvx_sample/cmake_utils/CMake_linux_tools.cmake
		set this line to comment: set(ARCH_BIT "-m32 -march=core2" )
     
  3) Build Openvx libraries
  -------------------------
    - Run ovx_build.py
