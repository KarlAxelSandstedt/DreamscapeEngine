cmake_minimum_required(VERSION 3.14.3)

if (DEFINED EMSCRIPTEN)
	# Needed due to header issues in freetype/buids/unix/ftsystem.c (Does not find open/read)
	set(HAVE_UNISTD_H 1 CACHE BOOL "Trick freetype to use unistd.h" FORCE)
	set(HAVE_FCNTL_H 1 CACHE BOOL "Trick freetype to use fcntl.h" FORCE)
	#set(CMAKE_FIND_ROOT_PATH "$$ {EMSCRIPTEN_SYSROOT}" " $${CMAKE_FIND_ROOT_PATH}")
	#include_directories("${EMSCRIPTEN_SYSROOT}/include")
	#add_definitions(-D_POSIX_C_SOURCE=200809L -D_XOPEN_SOURCE=700)
endif()

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/SDL")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/freetype")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/dtoa")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/opengl")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/tracy")

add_library(stb_image INTERFACE)
target_include_directories(stb_image INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/stb_image")
  
add_library(xxHash STATIC "${CMAKE_CURRENT_SOURCE_DIR}/xxHash/xxhash.c" "${CMAKE_CURRENT_SOURCE_DIR}/xxHash/xxhash.h")
target_include_directories(xxHash INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/xxHash")
