cmake_minimum_required(VERSION 3.14.3)

add_library(misc
	STATIC 
		"${DS_INCLUDE_PATH}/ds_ui.h"
		"ui/ui_local.h"
        "ds_misc.c"
)
add_library(Dreamscape::Misc ALIAS misc)
target_link_libraries(misc
	PUBLIC
		Dreamscape::Asset
        Dreamscape::Containers
		Dreamscape::Graphics
		Dreamscape::Math
)
