cmake_minimum_required(VERSION 3.14.3)

add_library(DreamscapeRenderer STATIC
	"${DS_INCLUDE_PATH}/ds_renderer.h"
	r_local.h
	r_init.c
	r_main.c
	r_mesh.c
	r_camera.c
	r_proxy3d.c
	r_scene.c
	r_ui.c
	r_gl.c
)
add_library(Dreamscape::Renderer ALIAS DreamscapeRenderer)

target_link_libraries(DreamscapeRenderer
	PUBLIC
		Dreamscape::Base
		Dreamscape::Dynamics
		Dreamscape::Editor
		Dreamscape::Asset
	PRIVATE
		Dreamscape::UI
		xxHash
)
