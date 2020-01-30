# this one is important
SET(CMAKE_SYSTEM_NAME Linux)

SET(COMPILER_FOLDER_NAME "/opt/adas/gcc-6.3.1-linaro-Xarmv8-linux/i686-linux")
SET(COMPILER_SUFFIX "")

if(WIN32)
	SET(COMPILER_FOLDER_NAME "gcc-6.3.1-linaro-Xarmv8-linux")
	SET(COMPILER_SUFFIX ".exe")
endif()

SET(CMAKE_C_COMPILER  "${COMPILER_FOLDER_NAME}/bin/aarch64-linux-gnu-gcc${COMPILER_SUFFIX}")
SET(CMAKE_CXX_COMPILER "${COMPILER_FOLDER_NAME}/bin/aarch64-linux-gnu-g++${COMPILER_SUFFIX}")

# where is the target environment 
SET(CMAKE_FIND_ROOT_PATH  ${COMPILER_FOLDER_NAME})

# common compiler flags (TBD)
set (CMAKE_C_FLAGS "-mcpu=generic -march=armv8-a -DPNG_ARM_NEON_OPT=0" CACHE STRING "C Flags" FORCE )
set (CMAKE_CXX_FLAGS  " ${CMAKE_C_FLAGS}"  CACHE STRING "CXX Flags" FORCE)

# search for programs in the build host directories
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

