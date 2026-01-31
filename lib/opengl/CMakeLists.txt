cmake_minimum_required(VERSION 3.14.3)

add_library(opengl_headers INTERFACE)
target_include_directories(opengl_headers INTERFACE "${LIB_PATH}/opengl")

