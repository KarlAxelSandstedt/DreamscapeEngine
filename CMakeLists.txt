cmake_minimum_required(VERSION 3.14.3)

project(Dreamscape LANGUAGES C)
set(CPACK_PACKAGE_MAJOR "0")
set(CPACK_PACKAGE_MINOR "1")
set(CPACK_PACKAGE_PATCH "2")
set(CPACK_PACKAGE_VERSION "${CPACK_PACKAGE_MAJOR}.${CPACK_PACKAGE_MINOR}.${CPACK_PACKAGE_PATCH}")

option(DS_OPTIMIZE "DS_OPTIMIZE enables compiler optimization flags in the library. Defaults 
		    to OFF." 
		OFF)

option(DS_DEBUG "DS_DEBUG builds the library with debug information. Furthermore, it turns on
	         general debug macros such as ds_Assert, ds_Breakpoint(cond) by defining. 
		 Defaults to ON."
		ON)

option(DS_PROFILE "DS_PROFILE enables internal profiling and the profiling API wrapping the Tracy Profiler API." 
		OFF)

option(DS_LOG "DS_LOG enabled the logging API and internal logging points in the library." ON)

option(DS_ASAN "DS_ASAN builds the library with Asan address sanitizing. If the user wishes to
		use Asan, the user should compile his or her code with Asan, AND set DS_ASAN;
		this is important as it enabled custom address poisoning code for both public
		and internal APIs. Defaults to OFF. Since sanitizing is costly, it is
		recommended to also set DS_OPTIMIZE to ON."
		OFF)

option(DS_TSAN "DS_TSAN builds the library with thread sanitizing. If the user wishes to
		use Tsan, the user should compile his or her code with Tsan, AND set DS_TSAN.
		Defaults to OFF. Since sanitizing is costly, it is recommended to also set 
		DS_OPTIMIZE to ON."
		OFF)

option(DS_UBSAN "DS_UBSAN builds the library with undefined behaviour sanitizing. If the user
		 wishes to use Ubsan, the user should compile his or her code with Ubsan, AND 
		 set DS_UBSAN.  Defaults to OFF. Since sanitizing is costly, it is recommended
 		 to also set DS_OPTIMIZE to ON."
		 OFF)

set(DS_INCLUDE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/include")
set(DS_SRC_PATH "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(DS_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib")

find_package(OpenGL REQUIRED)
find_package(Threads REQUIRED)

if (DS_PROFILE)
	set(TRACY_ENABLE ON)
endif ()

if (DEFINED EMSCRIPTEN)

	if (DS_ASAN)
		add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
		add_link_options(-fsanitize=address)
	endif ()

	if (DS_UBSAN)
		add_compile_options(-fsanitize=undefined -fno-omit-frame-pointer)
		add_link_options(-fsanitize=undefined)
	endif ()

	if (DS_DEBUG)
		# Add debug information and C<->wasm mapping
		add_compile_options(-g)
		add_link_options(-g -gsource-map)

		# high overhead; check every memory reference?
		add_link_options(-sASSERTIONS=2)
		add_link_options(-sSTACK_OVERFLOW_CHECK=2)
		if (NOT DS_ASAN)
			add_link_options(-sSAFE_HEAP=1)
		endif ()
	else ()
		add_link_options(-sASSERTIONS=0)
		# low overhead; end-of-frame checking?
		add_link_options(-sSTACK_OVERFLOW_CHECK=1)
	endif()

	if (DS_OPTIMIZE)
		add_compile_options(-O3)
	else ()
		add_compile_options(-O0)
	endif ()	
	
	add_compile_options(-W -Wall -Wextra -Wno-switch -Wno-unused-parameter -Wno-unused-variable -Wno-unused-function)

elseif (CMAKE_SYSTEM_NAME STREQUAL "Linux")

	if (DS_ASAN)
		add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
		add_link_options(-fsanitize=address)
	endif ()

	if (DS_UBSAN)
		add_compile_options(-fsanitize=undefined -fno-omit-frame-pointer)
		add_link_options(-fsanitize=undefined)
	endif ()

	if (DS_TSAN)
		add_compile_options(-fsanitize=thread -fno-omit-frame-pointer)
		add_link_options(-fsanitize=thread)
	endif ()

	if (DS_DEBUG)
		add_compile_options(-gdwarf)
	endif()

	if (DS_OPTIMIZE)
		add_compile_options(-O3)
	else ()
		add_compile_options(-O0)
	endif ()	

	# avoids x87 fp-instructions entirely (extended precision bad for consistency)
	add_compile_options(-mfpmath=sse)

	add_compile_options(-W -Wall -Wextra -Wno-switch -Wno-unused-parameter -Wno-unused-variable -Wno-unused-function)

elseif (CMAKE_SYSTEM_NAME STREQUAL "Windows")

	if (DS_ASAN)
		add_compile_options(/fsanitize=address)
		add_link_options(/incremental:no)
	endif ()

	if (DS_DEBUG)
		add_compile_definitions(DS_DEBUG)
		add_compile_options(/Od /Zi)
	endif ()

	if (DS_OPTIMIZE)
		add_compile_options(/O2)
	endif ()	

	# avoids x87 fp-instructions entirely (extended precision bad for consistency)
	add_compile_options(/arch:SSE2)

	#parallel compilation
	add_compile_options(/MP)

endif ()

add_subdirectory("${DS_INCLUDE_PATH}")
add_subdirectory("${DS_SRC_PATH}/base")
add_subdirectory("${DS_SRC_PATH}/containers")
add_subdirectory("${DS_LIB_PATH}")

add_library(${PROJECT_NAME} INTERFACE)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME} 
	INTERFACE 
		"${PROJECT_NAME}::Base"
		"${PROJECT_NAME}::Containers"
)

#TODO remove, consider using DS_DEBUG instead, or if required, add DS_DEV
if (NOT CMAKE_BUILD_TYPE STREQUAL "Release")
	add_compile_definitions(KAS_DEV)
endif()

#TODO How do we expose library testing for the user, (Or developer)?
#     For both the developer and the user, we want to expose the interative
#     porting process: both the user and the developer is interested in what
#     progressively more coupled sub-system is not working as expected, and
#     this should be exposed in some form or another.
set(DS_TEST_PATH "${CMAKE_CURRENT_SOURCE_DIR}/test")
#if (kas_test_correctness)
#	add_compile_definitions(KAS_TEST_CORRECTNESS)
#	add_subdirectory("${TEST_PATH}")
#	target_link_libraries(${PROJECT_NAME} PUBLIC kas_test)
#elseif (kas_test_performance)
#	add_compile_definitions(KAS_TEST_PERFORMANCE)
#	add_subdirectory("${TEST_PATH}")
#	target_link_libraries(${PROJECT_NAME} PUBLIC kas_test)
#endif ()

#TODO: link to base, so it propagates WITHIN LIBRARY but nowhere else
#if (TRACY_ENABLE)
#	link_libraries(Tracy::TracyClient)
#	include_directories(tracy/public)
#	add_compile_definitions(DS_PROFILE)
#endif ()


#add_subdirectory("${DS_SRC_PATH}/misc")
#add_subdirectory("${DS_SRC_PATH}/misc/ui")
#add_subdirectory("${DS_SRC_PATH}/misc/debug")
#add_subdirectory("${DS_SRC_PATH}/common")
#add_subdirectory("${DS_SRC_PATH}/math")
#add_subdirectory("${DS_SRC_PATH}/containers")
#add_subdirectory("${DS_SRC_PATH}/memory")
#add_subdirectory("${DS_SRC_PATH}/renderer")
#add_subdirectory("${DS_SRC_PATH}/sys")
#add_subdirectory("${DS_SRC_PATH}/sys/sdl3_wrapper")
#add_subdirectory("${DS_SRC_PATH}/led")
#add_subdirectory("${DS_SRC_PATH}/asset_system")
#
#link_libraries(kas_common)
#link_libraries(log)

#target_link_libraries(${PROJECT_NAME}
#	INTERFACE
#		system_graphics
#		renderer
#		asset_system
#		led
#	)
#if (DEFINED EMSCRIPTEN)
#	add_subdirectory(${DS_SRC_PATH}/sys/wasm)
#	target_link_libraries(${PROJECT_NAME} PUBLIC wasm_interface)
#elseif (CMAKE_SYSTEM_NAME STREQUAL "Linux")
#	add_subdirectory(${DS_SRC_PATH}/sys/linux)
#	target_link_libraries(${PROJECT_NAME} INTERFACE linux_interface)
#elseif (CMAKE_SYSTEM_NAME STREQUAL "Windows")
#	add_subdirectory(${DS_SRC_PATH}/sys/windows)
#	target_link_libraries(${PROJECT_NAME} INTERFACE windows_interface bcrypt)
#
#	# DLLs must be in same folder as exe? Not as on linux where we have rpath and such...
#	add_custom_command(
#		TARGET	${PROJECT_NAME} POST_BUILD
#		COMMAND ${CMAKE_COMMAND} -E copy_if_different
#			$<TARGET_FILE:SDL3::SDL3-shared>
#			$<TARGET_FILE_DIR:${PROJECT_NAME}>
#		VERBATIM
#	)
#endif ()

#
# For CPack and installation process, good information can be found at:
# https://martin-fieber.de/blog/cmake-cpack-cross-platform-distributables/
#
# Contains definition for CMAKE_INSTALL_***DIR
include(GNUInstallDirs)
# get common install destinations for various platforms 
include(InstallRequiredSystemLibraries)
# enable CPack to escape values when writing its config files
set(CPACK_PACKAGE_NAME ${PROJECT_NAME})
set(CPACK_PACKAGE_INSTALL_DIRECTORY ${CPACK_PACKAGE_NAME})
set(CPACK_VERBATIM_VARIABLES YES)
set(CPACK_PACKAGE_VENDOR "PeonDynamics")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/COPYING.txt")

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
	set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
	set(CPACK_DEB_COMPONENT_INSTALL YES)
	set(CPACK_GENERATOR TGZ DEB)
	set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Apeman")
	set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS YES)
	set(CPACK_PACKAGING_INSTALL_PREFIX "/opt/${PROJECT_NAME}")

	# set RPATH for executable in install package 
	set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH $ORIGIN/../${CMAKE_INSTALL_BINDIR})
	set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH $ORIGIN/../${CMAKE_INSTALL_LIBDIR})
	install(TARGETS SDL3-shared DESTINATION ${CMAKE_INSTALL_LIBDIR})

	#install(FILES $<TARGET_FILE:SDL3::SDL3> DESTINATION ${CMAKE_INSTALL_LIBDIR} RENAME libSDL3.so.0)
elseif (CMAKE_SYSTEM_NAME STREQUAL "Windows")
	set(CPACK_GENERATOR NSIS)
	set(CPACK_NSIS_MODIFY_PATH ON)

	install(TARGETS SDL3-shared DESTINATION ${CMAKE_INSTALL_BINDIR})
endif ()

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}	# dlls and such on windows
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}	# (shared libraries, except dlls on windows)
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}	# static libraries
)

install(DIRECTORY asset_components/led_projects DESTINATION asset_components)
install(DIRECTORY asset_components/textures DESTINATION asset_components)
install(DIRECTORY asset_components/ttf DESTINATION asset_components)

install(DIRECTORY assets/shaders DESTINATION assets)
install(DIRECTORY assets/fonts DESTINATION assets)
install(DIRECTORY assets/sprites DESTINATION assets)

# Must always be after all CPACK_* variables have been defined 
include(CPack)
