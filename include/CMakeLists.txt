cmake_minimum_required(VERSION 3.14.3)

add_library(DreamscapeDefine INTERFACE)
add_library(Dreamscape::Define ALIAS DreamscapeDefine)

target_link_libraries(DreamscapeDefine INTERFACE opengl_headers)
target_compile_definitions(DreamscapeDefine INTERFACE DS_VERSION_CSTR="${CPACK_PACKAGE_VERSION}")
target_include_directories(DreamscapeDefine INTERFACE ${DS_INCLUDE_PATH})

if (DS_DEBUG)
	target_compile_definitions(DreamscapeDefine INTERFACE DS_DEBUG DS_ASSERT)
endif()

if (DS_PROFILE)
	target_compile_definitions(DreamscapeDefine INTERFACE DS_PROFILE)
	target_link_libraries(DreamscapeDefine INTERFACE Tracy::TracyClient)
	target_include_directories(DreamscapeDefine INTERFACE tracy/public)
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
