cmake_minimum_required(VERSION 3.14.3)

set(COMMON_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/common")
#set(COLLISION_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/collision")
#set(DYNAMICS_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/dynamics")

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/common")
#add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/collision")
#add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/dynamics")
