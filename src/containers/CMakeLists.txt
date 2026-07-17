cmake_minimum_required(VERSION 3.14.3)

# files to link to later
add_library(DreamscapeContainers STATIC
	"ds_containers.c"
)       
add_library(Dreamscape::Containers ALIAS DreamscapeContainers)
target_link_libraries(DreamscapeContainers PUBLIC Dreamscape::Base)
