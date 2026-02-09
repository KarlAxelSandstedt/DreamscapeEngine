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


add_library(DreamscapeGraphics
	STATIC
		"${DS_INCLUDE_PATH}/ds_graphics.h"
		ds_graphics.c
	)
add_library(Dreamscape::Graphics ALIAS DreamscapeGraphics)
target_link_libraries(DreamscapeGraphics
	PRIVATE
		Dreamscape::Renderer
		Dreamscape::UI
		sdl3_wrapper
	)
