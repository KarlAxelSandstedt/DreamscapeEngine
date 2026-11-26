cmake_minimum_required(VERSION 3.14.3)

add_library(DreamscapeTestAPI
	ds_test.h
	test_main.c 
	test_memory.c
	test_asset.c
	test_shape.c
	test_raycast.c
	test_bvh.c
	test_renderer.c
	test_ui.c
	test_cmd.c
    test_job.c
	test_string.c
	test_serialize.c
	test_allocator.c
	test_allocator_correctness.c
	test_allocator_benchmark.c
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
	test_geometric_mover.c
	test_toi.c
	test_simulation.c
)

add_library(Dreamscape::Test ALIAS DreamscapeTestAPI)
target_link_libraries(DreamscapeTestAPI PRIVATE Dreamscape::Dreamscape dtoa xxHash) 
target_include_directories(DreamscapeTestAPI INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})


# Assembly test box (test/asm; script/linux/asm_run.sh, asm_debug.sh): hand-rolled NASM functions checked against
# the C ones. Linux only: the .asm files use the System V x86-64 calling convention and ELF objects. nasm is run
# by hand: as a CMake language it would be handed the C compile options. Not part of the default build, so an
# unfinished .asm file never breaks it.
if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
	find_program(DS_NASM nasm)
	if (DS_NASM)
		set(DS_ASM_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/asm/c_sphere_contact_handrolled.asm")
		set(DS_ASM_OBJECT "${CMAKE_BINARY_DIR}/c_sphere_contact_handrolled.o")
		set(DS_ASM_LISTING "${CMAKE_BINARY_DIR}/c_sphere_contact_handrolled.lst")
		add_custom_command(
			OUTPUT "${DS_ASM_OBJECT}"
			COMMAND "${DS_NASM}" -f elf64 -g -F dwarf -l "${DS_ASM_LISTING}" -o "${DS_ASM_OBJECT}" "${DS_ASM_SOURCE}"
			DEPENDS "${DS_ASM_SOURCE}"
			VERBATIM
		)
		add_executable(DreamscapeAsm EXCLUDE_FROM_ALL asm/asm_main.c "${DS_ASM_OBJECT}")
		set_target_properties(DreamscapeAsm PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
		target_link_libraries(DreamscapeAsm PRIVATE Dreamscape::Dreamscape)
		# asm_main.c never calls ds_Init, so nothing pulls g_config's object out of its library before the
		# libraries that read g_config are scanned
		target_link_options(DreamscapeAsm PRIVATE "LINKER:--undefined=g_config")
	else ()
		message(STATUS "nasm not found: the assembly test box (DreamscapeAsm) is not available")
	endif ()
endif ()
