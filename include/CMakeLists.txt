cmake_minimum_required(VERSION 3.14.3)

add_library(DreamscapeDefine INTERFACE)
add_library(Dreamscape::Define ALIAS DreamscapeDefine)

target_link_libraries(DreamscapeDefine INTERFACE opengl_headers)
target_compile_definitions(DreamscapeDefine INTERFACE DS_VERSION_CSTR="${CPACK_PACKAGE_VERSION}")
target_include_directories(DreamscapeDefine INTERFACE ${DS_INCLUDE_PATH})

if (DS_DEBUG)
	target_compile_definitions(DreamscapeDefine INTERFACE DS_DEBUG DS_ASSERT)
endif()

# Warnings of the engine's targets (not lib/'s). Linux's set (CMakeLists.txt) is the reference; Windows follows it.
if (CMAKE_SYSTEM_NAME STREQUAL "Windows")
	if (CMAKE_C_COMPILER_ID STREQUAL "Clang")
		# clang-cl: Linux's set (its /W4 is clang's -Wall -Wextra)
		target_compile_options(DreamscapeDefine INTERFACE /W4
			/clang:-Wno-switch /clang:-Wno-unused-parameter /clang:-Wno-unused-variable
			/clang:-Wno-unused-but-set-variable /clang:-Wno-unused-function)
	else ()
		# MSVC: /W4 without what Linux's set does not report: unused parameters, locals and functions (its -Wno-
		# list: C4100 C4101 C4189 C4505), narrowing (C4244 C4267 C4305, -Wconversion), shadowing (C4456 C4459,
		# -Wshadow), constant conditions (C4127), signed/unsigned conversion (C4245, -Wsign-conversion) and
		# flexible array members (C4200)
		target_compile_options(DreamscapeDefine INTERFACE /W4
			/wd4100 /wd4101 /wd4189 /wd4505 /wd4244 /wd4267 /wd4305 /wd4456 /wd4459 /wd4127 /wd4245 /wd4200)
	endif ()
endif ()

if (DS_PROFILE)
	target_compile_definitions(DreamscapeDefine INTERFACE DS_PROFILE)
	target_link_libraries(DreamscapeDefine INTERFACE Tracy::TracyClient)
	target_include_directories(DreamscapeDefine INTERFACE tracy/public)
endif ()

if (DS_PERFORMANCE_MEASUREMENT_API)
	target_compile_definitions(DreamscapeDefine INTERFACE DS_PERFORMANCE_MEASUREMENT_API)
endif ()

if (DS_LOG)
	target_compile_definitions(DreamscapeDefine INTERFACE DS_LOG)
endif ()

if (DS_ASAN)
	target_compile_definitions(DreamscapeDefine INTERFACE DS_ASAN)
endif ()

if (DS_TSAN)
	target_compile_definitions(DreamscapeDefine INTERFACE DS_TSAN)
endif ()

if (DS_UBSAN)
	target_compile_definitions(DreamscapeDefine INTERFACE DS_UBSAN)
endif ()
