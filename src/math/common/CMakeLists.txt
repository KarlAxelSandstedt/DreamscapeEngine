cmake_minimum_required(VERSION 3.14.3)

# files to link to later
add_library(ds_math STATIC
	"${DS_SRC_PATH}/math/common/matrix.c"
	"${DS_SRC_PATH}/math/common/vector.c"
	"${DS_SRC_PATH}/math/common/float32.c"
	"${DS_SRC_PATH}/math/common/geometry.c"
	"${DS_SRC_PATH}/math/common/quaternion.c"
	"${DS_SRC_PATH}/math/common/transform.c"
	"${DS_INCLUDE_PATH}/matrix.h"
	"${DS_INCLUDE_PATH}/vector.h"
	"${DS_INCLUDE_PATH}/ds_math.h"
	"${DS_INCLUDE_PATH}/float32.h"
	"${DS_INCLUDE_PATH}/geometry.h"
	"${DS_INCLUDE_PATH}/quaternion.h"
	"${DS_INCLUDE_PATH}/transform.h"
)
add_library(Dreamscape::Math ALIAS ds_math)

target_link_libraries(ds_math PUBLIC Dreamscape::Base)
if (UNIX) 
	target_link_libraries(ds_math PUBLIC m)
endif ()

target_include_directories(ds_math PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
