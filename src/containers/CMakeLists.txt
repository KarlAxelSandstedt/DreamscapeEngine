cmake_minimum_required(VERSION 3.14.3)

# files to link to later
add_library(DreamscapeContainers STATIC
	"${DS_INCLUDE_PATH}/hash_map.h"
	"${DS_INCLUDE_PATH}/queue.h"
	"${DS_INCLUDE_PATH}/bit_vector.h"
	"${DS_INCLUDE_PATH}/ds_vector.h"
	"${DS_INCLUDE_PATH}/list.h"
	"${DS_INCLUDE_PATH}/hierarchy_index.h"
	"${DS_INCLUDE_PATH}/string_database.h"
	"${DS_INCLUDE_PATH}/tree.h"
	"${DS_INCLUDE_PATH}/fifo_spmc.h"
	"hash_map.c"
	"queue.c"
	"bit_vector.c"
	"ds_vector.c"
	"list.c"
	"hierarchy_index.c"
	"string_database.c"
	"tree.c"
	"parallel/fifo_spmc.c"
)       
add_library(Dreamscape::Containers ALIAS DreamscapeContainers)
target_link_libraries(DreamscapeContainers PUBLIC Dreamscape::Base)
