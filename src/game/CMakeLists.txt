cmake_minimum_required(VERSION 3.14.3)

add_library(DreamscapeGame
	STATIC
		"${DS_INCLUDE_PATH}/ds_game.h"
		ds_game.c
	)
add_library(Dreamscape::Game ALIAS DreamscapeGame)
target_link_libraries(DreamscapeGame
	PUBLIC
		Dreamscape::Base
		Dreamscape::Containers
		Dreamscape::Dynamics
	PRIVATE
		xxHash
	)
