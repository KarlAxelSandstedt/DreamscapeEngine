cmake_minimum_required(VERSION 3.14.3)

add_library(collision STATIC
	"${DS_INCLUDE_PATH}/collision.h"
	collision.c
	bvh.c
)
add_library(Dreamscape::Collision ALIAS collision)

target_link_libraries(collision PUBLIC
	Dreamscape::Math
	Dreamscape::Containers	
)
