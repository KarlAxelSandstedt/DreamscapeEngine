cmake_minimum_required(VERSION 3.14.3)

add_library(DreamscapeBase 
	STATIC 
	"${DS_INCLUDE_PATH}/ds_types.h"
	"${DS_INCLUDE_PATH}/ds_atomic.h"
	"${DS_INCLUDE_PATH}/ds_allocator.h"
	"${DS_INCLUDE_PATH}/ds_serialize.h"
	"${DS_INCLUDE_PATH}/ds_string.h"
	"${DS_INCLUDE_PATH}/ds_error.h"
	"${DS_INCLUDE_PATH}/ds_semaphore.h"
	"${DS_INCLUDE_PATH}/ticket_factory.h"
	"${DS_INCLUDE_PATH}/ds_time.h"
	"${DS_INCLUDE_PATH}/ds_log.h"
	"${DS_INCLUDE_PATH}/ds_arch.h"
	"${DS_INCLUDE_PATH}/ds_string.h"
	"${DS_INCLUDE_PATH}/ds_thread.h"
	"${DS_INCLUDE_PATH}/ds_base.h"
	"${DS_SRC_PATH}/base/ds_allocator.c"
	"${DS_SRC_PATH}/base/ds_serialize.c"
	"${DS_SRC_PATH}/base/ds_string.c"
	"${DS_SRC_PATH}/base/ds_error.c"
	"${DS_SRC_PATH}/base/ds_semaphore.c"
	"${DS_SRC_PATH}/base/ds_arch.c"
	"${DS_SRC_PATH}/base/ds_string.c"
	"${DS_SRC_PATH}/base/ds_time.c"
	"${DS_SRC_PATH}/base/ticket_factory.c"
	"${DS_SRC_PATH}/base/ds_thread.c"
	"${DS_SRC_PATH}/base/ds_log.c"
)
add_library(Dreamscape::Base ALIAS DreamscapeBase)
target_link_libraries(DreamscapeBase 
	PUBLIC 
		DreamscapeDefine
	PRIVATE
		dtoa
		xxHash
)
