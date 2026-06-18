cmake_minimum_required(VERSION 3.14.3)

if (DEFINED EMSCRIPTEN)
	# Needed due to header issues in freetype/buids/unix/ftsystem.c (Does not find open/read)
	set(HAVE_UNISTD_H 1 CACHE BOOL "Trick freetype to use unistd.h" FORCE)
	set(HAVE_FCNTL_H 1 CACHE BOOL "Trick freetype to use fcntl.h" FORCE)
	#set(CMAKE_FIND_ROOT_PATH "$$ {EMSCRIPTEN_SYSROOT}" " $${CMAKE_FIND_ROOT_PATH}")
	#include_directories("${EMSCRIPTEN_SYSROOT}/include")
	#add_definitions(-D_POSIX_C_SOURCE=200809L -D_XOPEN_SOURCE=700)
endif()

set(DS_FORWARD_VARS
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DCMAKE_INSTALL_PREFIX=${DS_INSTALL_PATH}
    -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
    -DCMAKE_C_FLAGS=${CMAKE_C_FLAGS}
    -DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}
    -DCMAKE_EXE_LINKER_FLAGS=${CMAKE_EXE_LINKER_FLAGS}
    -DCMAKE_SHARED_LINKER_FLAGS=${CMAKE_SHARED_LINKER_FLAGS}
    -DCMAKE_STATIC_LINKER_FLAGS=${CMAKE_STATIC_LINKER_FLAGS})

ExternalProject_Add(SDL3
    SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/SDL"
    BINARY_DIR "${CMAKE_BINARY_DIR}/lib/SDL"
    CMAKE_ARGS 
        ${DS_FORWARD_VARS}
        -DSDL_STATIC=ON
    BUILD_ALWAYS OFF
    INSTALL_DIR "${DS_INSTALL_PATH}"
    BUILD_BYPRODUCTS "${DS_INSTALL_PATH}/lib/${LIB_PREFIX}SDL3${STATIC_EXT}"
)
ExternalProject_Get_Property(SDL3 BINARY_DIR)
add_library(SDL3::SDL3 STATIC IMPORTED GLOBAL)
add_dependencies(SDL3::SDL3 SDL3)
set_target_properties(SDL3::SDL3 PROPERTIES
    IMPORTED_LOCATION "${DS_INSTALL_PATH}/lib/${LIB_PREFIX}SDL3${STATIC_EXT}"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/SDL/include"
)

ExternalProject_Add(freetype
    SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/freetype"
    BINARY_DIR "${CMAKE_BINARY_DIR}/lib/freetype"
    CMAKE_ARGS ${DS_FORWARD_VARS}
    BUILD_ALWAYS OFF
    INSTALL_DIR "${DS_INSTALL_PATH}"
    BUILD_BYPRODUCTS "${DS_INSTALL_PATH}/lib/${LIB_PREFIX}freetyped${STATIC_EXT}"
)
ExternalProject_Get_Property(freetype BINARY_DIR)
add_library(Freetype::Freetype STATIC IMPORTED GLOBAL)
add_dependencies(Freetype::Freetype freetype)
set_target_properties(Freetype::Freetype PROPERTIES
    IMPORTED_LOCATION "${DS_INSTALL_PATH}/lib/${LIB_PREFIX}freetyped${STATIC_EXT}"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/freetype/include"
)

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/tracy")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/dtoa")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/opengl")

add_library(stb_image INTERFACE)
target_include_directories(stb_image INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/stb_image")
  
add_library(xxHash STATIC "${CMAKE_CURRENT_SOURCE_DIR}/xxHash/xxhash.c" "${CMAKE_CURRENT_SOURCE_DIR}/xxHash/xxhash.h")
target_include_directories(xxHash INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/xxHash")
