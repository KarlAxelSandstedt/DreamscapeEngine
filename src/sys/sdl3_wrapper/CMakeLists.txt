cmake_minimum_required(VERSION 3.14.3)

add_library(sdl3_wrapper STATIC 
	"${DS_INCLUDE_PATH}/ds_gl.h"
    ds_sdl3.c
	sdl3_wrapper_public.h
	sdl3_wrapper_local.h
)
target_include_directories(sdl3_wrapper INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(sdl3_wrapper PUBLIC Dreamscape::Containers)

if (DEFINED EMSCRIPTEN)
	target_link_libraries(sdl3_wrapper 
		PRIVATE 
            #SDL3::SDL3-static
            #SDL3::Headers
            SDL3::SDL3
	)
else ()
    #target_link_libraries(sdl3_wrapper PRIVATE SDL3::SDL3-shared)	
    target_link_libraries(sdl3_wrapper PRIVATE SDL3::SDL3)	
endif ()
