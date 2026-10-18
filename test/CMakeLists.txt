cmake_minimum_required(VERSION 3.14.3)

add_library(DreamscapeTestAPI
	ds_test.h
	test_main.c 
	test_memory.c
	test_asset.c
    test_job.c
    #test_container.c
    #test_math.c
	test_string.c
	test_serialize.c
	test_allocator.c
	test_allocator_correctness.c
	test_container_correctness.c
	test_base_correctness.c
    test_hash_map.c
    test_hierarchy.c
	test_hash.c
	test_rng.c
	test_geometry.c
	test_gjk.c
	test_contact.c
	test_config.c
	test_dynamics.c
)

add_library(Dreamscape::Test ALIAS DreamscapeTestAPI)
target_link_libraries(DreamscapeTestAPI PRIVATE Dreamscape::Dreamscape dtoa xxHash) 
target_include_directories(DreamscapeTestAPI INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
