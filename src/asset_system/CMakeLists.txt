cmake_minimum_required(VERSION 3.14.3)

add_library(asset_system STATIC 
	"${DS_INCLUDE_PATH}/ds_asset.h"
	asset_local.h
    ds_asset.c
	)
add_library(Dreamscape::Asset ALIAS asset_system)
target_include_directories(asset_system INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(asset_system 
	PRIVATE 
		stb_image
        Freetype::Freetype
	PUBLIC
		Dreamscape::Platform
	)
