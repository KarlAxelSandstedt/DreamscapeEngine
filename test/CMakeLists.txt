cmake_minimum_required(VERSION 3.14.3)

add_library(DreamscapeTestAPI
	ds_test.h
	test_repetition.c
	test_main.c 
    #test_container.c
    #test_math.c
	test_string.c
	test_serialize.c
	test_allocator.c
	test_hash.c
	test_rng.c
)

add_library(Dreamscape::Test ALIAS DreamscapeTestAPI)
target_link_libraries(DreamscapeTestAPI PRIVATE Dreamscape::Dreamscape dtoa xxHash) 
target_include_directories(DreamscapeTestAPI INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
