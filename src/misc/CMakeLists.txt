cmake_minimum_required(VERSION 3.14.3)

add_library(cmd 
	STATIC 
		"${DS_INCLUDE_PATH}/cmd.h"
		cmd.c
	)
add_library(Dreamscape::Command ALIAS cmd)
target_link_libraries(cmd PUBLIC Dreamscape::Containers)

#add_library(ui 
#	STATIC 
#		"${DS_INCLUDE_PATH}/ui_public.h"
#		"ui/ui_core.c"
#		"ui/ui_build.c" 
#		"ui/ui_local.h"
#)
#add_library(Dreamscape::UI ALIAS ui)
#target_link_libraries(ui
#	PUBLIC
#		Dreamscape::Asset
#		Dreamscape::Command
#)
