cmake_minimum_required(VERSION 3.14.3)

add_library(DreamscapeServer
	STATIC
		"${DS_INCLUDE_PATH}/ds_server.h"
		ds_server.c
	)
add_library(Dreamscape::Server ALIAS DreamscapeServer)
target_link_libraries(DreamscapeServer
	PUBLIC
		Dreamscape::Base
		Dreamscape::Containers
		Dreamscape::Platform
		Dreamscape::Simulation
	)
