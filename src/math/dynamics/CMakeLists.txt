cmake_minimum_required(VERSION 3.14.3)

add_library(dynamics STATIC
	"${DS_INCLUDE_PATH}/ds_dynamics.h"
    ds_dynamics.c
)
add_library(Dreamscape::Dynamics ALIAS dynamics)
target_link_libraries(dynamics 
    PUBLIC 
        Dreamscape::Collision 
        Dreamscape::Job
    PRIVATE
        xxHash
    )
