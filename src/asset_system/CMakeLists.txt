cmake_minimum_required(VERSION 3.14.3)

add_library(asset_system STATIC 
	"${DS_INCLUDE_PATH}/asset_public.h"
	asset_local.h
	asset_ssff.c
	asset_font.c
	asset_database.c 
	asset_init.c
	)
add_library(Dreamscape::Asset ALIAS asset_system)
target_include_directories(asset_system INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(asset_system 
	PRIVATE 
		stb_image
		freetype
	PUBLIC
		Dreamscape::Platform
	)
