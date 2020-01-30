
cmake_minimum_required (VERSION 2.6)

set (LibName "common")
#get_filename_component(LibName ${CMAKE_CURRENT_LIST_DIR} NAME)

set(comm_KERNEL_SRC "${CMAKE_CURRENT_LIST_DIR}/src")
set(comm_KERNEL_HEADER "${CMAKE_CURRENT_LIST_DIR}/include")

#message ("common lib dir " ${comm_KERNEL_SRC})
#CMAKE_REQUIRED_DEFINITIONS = (-DMyProjectRoot=${VISION_SDK})

# Maps to Visual Studio solution file (LibName.sln)
# The solution will have all targets (exe, lib, dll) 
# as Visual Studio projects (.vcproj)
#project (${LibName})

#file( GLOB srcFilesCPP "${OWN_KERNEL_SRC}/*.cpp")
#file( GLOB srcFilesC "${OWN_KERNEL_SRC}/*.c")
#file( GLOB headerFilesH "${OWN_KERNEL_HEADER}/*.h")
#file( GLOB headerFilesHPP "${OWN_KERNEL_HEADER}/*.hpp")

set (srcFilesC "")
set (headerFilesHPP "")

list(APPEND comm_srcFilesCPP "${comm_KERNEL_SRC}/time_measure.cpp" "${comm_KERNEL_SRC}/base_process_init.cpp")
list(APPEND comm_headerFilesH "${VISION_SDK}/common_stringify_macros.h" "${VISION_SDK}/common_time_measure.h" "${VISION_SDK}/common_config_time_measure.h" "${VISION_SDK}/common_base_process_init.h")

set (comm_srcFiles "")
list(APPEND comm_srcFiles ${comm_srcFilesCPP} ${comm_srcFilesC})
set (comm_headerFiles "")
list(APPEND comm_headerFiles ${comm_headerFilesHPP} ${comm_headerFilesH})

#message("common src files: " ${comm_srcFiles})
#message("own kernel cat H files: " ${headerFiles})

LIST(APPEND GEN_DEPENDENCIES ${LibName})

# set(LibFound "")
# find_library(LibFound ${LibName} "${CMAKE_CURRENT_LIST_DIR}/${VS_PROJ_DIR}/Release"  )
# message ("Lib ${LibName} found " ${LibFound})

#add_library(${LibName} STATIC IMPORTED)
#target_sources(${LibName} INTERFACE ${srcFiles} ${headerFiles})
#set_property(TARGET ${LibName} PROPERTY IMPORTED_LOCATION_DEBUG "${CMAKE_CURRENT_LIST_DIR}/${VS_PROJ_DIR}/Debug")
#set_property(TARGET ${LibName} PROPERTY IMPORTED_LOCATION_RELEASE "${CMAKE_CURRENT_LIST_DIR}/${VS_PROJ_DIR}/Release")
#unset(LibFound)

add_library(${LibName} STATIC ${comm_srcFiles} ${comm_headerFiles} )
source_group("Source Files" FILES ${comm_srcFiles})
source_group("Header Files" FILES ${comm_headerFiles} )

# Creates a folder "c${LIB_SOURCE_DIR}/build-deskwin32/mvc" and adds target 
# project (<LibName>.vcproj) under it
#set_property(TARGET ${LibName} PROPERTY FOLDER "${LIB_SOURCE_DIR}/build-deskwin32/mvc")

# Adds logic to INSTALL.vcproj to copy ${LibName}.lib to destination directory
#install (TARGETS ${LibName} 
#		ARCHIVE DESTINATION ${LIB_SOURCE_DIR}/build-deskwin32/mvc)
