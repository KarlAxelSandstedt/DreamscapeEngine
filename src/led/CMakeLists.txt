cmake_minimum_required(VERSION 3.14.3)

add_library(DreamscapeEditor STATIC 
	"${DS_INCLUDE_PATH}/led_public.h"
	"${DS_INCLUDE_PATH}/csg.h"
	led_local.h
	led_init.c
	led_main.c
	led_utility.c
	led_visual.c
	led_ui.c
	led_core.c
	csg.c
	)
add_library(Dreamscape::Editor ALIAS DreamscapeEditor)

target_link_libraries(DreamscapeEditor
	PUBLIC
		Dreamscape::Containers
		Dreamscape::Renderer
)

