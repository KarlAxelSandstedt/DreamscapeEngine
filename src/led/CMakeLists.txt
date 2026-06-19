cmake_minimum_required(VERSION 3.14.3)

add_library(DreamscapeEditor STATIC 
	"${DS_INCLUDE_PATH}/ds_led.h"
	"${DS_INCLUDE_PATH}/csg.h"
	led_local.h
    ds_led.c
	)
add_library(Dreamscape::Editor ALIAS DreamscapeEditor)

target_link_libraries(DreamscapeEditor
	PUBLIC
		Dreamscape::Containers
		Dreamscape::Renderer
)

