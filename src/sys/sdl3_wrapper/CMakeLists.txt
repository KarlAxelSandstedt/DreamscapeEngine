cmake_minimum_required(VERSION 3.14.3)

add_library(sdl3_wrapper STATIC 
	sdl3_wrapper_public.h
	sdl3_wrapper_local.h
	sdl3_wrapper_window.c
	sdl3_wrapper_event.c
	sdl3_wrapper_input.c
	sdl3_wrapper_gl.c
)
target_include_directories(sdl3_wrapper INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(sdl3_wrapper PUBLIC Dreamscape::Containers)

if (DEFINED EMSCRIPTEN)
	target_link_libraries(sdl3_wrapper 
		PRIVATE 
			SDL3::SDL3-static
			SDL3::Headers
	)
else ()
	target_link_libraries(sdl3_wrapper PRIVATE SDL3::SDL3-shared)	
endif ()
