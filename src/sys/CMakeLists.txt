cmake_minimum_required(VERSION 3.14.3)

add_subdirectory("sdl3_wrapper")

add_library(DreamscapePlatform
	STATIC
		"${DS_INCLUDE_PATH}/ds_platform.h"
		"linux/linux_filesystem.c"
		"linux/linux_random.c"
		"sys_local.h"
		"sys_filesystem.c"
		"sys_event.c"
		"sys_init.c"
		"sys_input.c"
		#"sys_graphics"
)
add_library(Dreamscape::Platform ALIAS DreamscapePlatform)
target_link_libraries(DreamscapePlatform 
	PUBLIC 
		Dreamscape::Base
	PRIVATE
		sdl3_wrapper
)

add_library(DreamscapeJob STATIC 
	"${DS_INCLUDE_PATH}/ds_job.h"
	ds_job.c
)
add_library(Dreamscape::Job ALIAS DreamscapeJob)
target_link_libraries(DreamscapeJob PUBLIC Dreamscape::Containers)


#add_library(system_graphics
#	STATIC
#		sys_local.h
#		sys_public.h
#	)
#
#target_include_directories(system_graphics INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
#target_link_libraries(system_graphics
#	PUBLIC
#		system
#	PRIVATE
#		renderer
#		sdl3_wrapper
#	)

#target_include_directories(system INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
#target_link_libraries(system
#	PUBLIC
#		ui
#		cmd
#		parallel_containers
#)
