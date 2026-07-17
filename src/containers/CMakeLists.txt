cmake_minimum_required(VERSION 3.14.3)

add_library(DreamscapeContainers STATIC
    "${DS_INCLUDE_PATH}/ds_hash_map.h"
    "${DS_INCLUDE_PATH}/queue.h"
    "${DS_INCLUDE_PATH}/bit_vector.h"
    "${DS_INCLUDE_PATH}/list.h"
    "${DS_INCLUDE_PATH}/hierarchy_index.h"
    "${DS_INCLUDE_PATH}/string_database.h"
    "${DS_INCLUDE_PATH}/tree.h"
	"ds_containers.c"
)       
add_library(Dreamscape::Containers ALIAS DreamscapeContainers)
target_link_libraries(DreamscapeContainers PUBLIC Dreamscape::Base)
