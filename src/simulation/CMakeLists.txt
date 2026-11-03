cmake_minimum_required(VERSION 3.14.3)

add_library(DreamscapeSimulation
	STATIC
		"${DS_INCLUDE_PATH}/ds_simulation.h"
		ds_simulation.c
	)
add_library(Dreamscape::Simulation ALIAS DreamscapeSimulation)
target_link_libraries(DreamscapeSimulation
	PUBLIC
		Dreamscape::Base
		Dreamscape::Containers
		Dreamscape::Dynamics
	)
