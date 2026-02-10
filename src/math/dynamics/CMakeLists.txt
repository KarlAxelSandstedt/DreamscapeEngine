cmake_minimum_required(VERSION 3.14.3)

add_library(dynamics STATIC
	"${DS_INCLUDE_PATH}/dynamics.h"
	physics_pipeline.c
	contact_database.c
	contact_solver.c
	island.c
	ds_shape.c
	ds_body.c
)
add_library(Dreamscape::Dynamics ALIAS dynamics)
target_link_libraries(dynamics PUBLIC Dreamscape::Collision Dreamscape::Job)
