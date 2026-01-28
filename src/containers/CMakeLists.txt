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
	"${DS_SRC_PATH}/containers/hash_map.c"
	"${DS_SRC_PATH}/containers/queue.c"
	"${DS_SRC_PATH}/containers/bit_vector.c"
	"${DS_SRC_PATH}/containers/ds_vector.c"
	"${DS_SRC_PATH}/containers/list.c"
	"${DS_SRC_PATH}/containers/hierarchy_index.c"
	"${DS_SRC_PATH}/containers/string_database.c"
	"${DS_SRC_PATH}/containers/tree.c"
	"${DS_SRC_PATH}/containers/parallel/fifo_spmc.c"
)       
add_library(Dreamscape::Containers ALIAS DreamscapeContainers)
target_link_libraries(DreamscapeContainers PUBLIC Dreamscape::Base)
