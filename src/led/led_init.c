/*
==========================================================================
    Copyright (C) 2025 Axel Sandstedt 

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
==========================================================================
*/

#include <string.h>
#include "led_local.h"

struct led g_editor_storage = { 0 };
struct led *g_editor = &g_editor_storage;

struct led_project_menu	led_project_menu_alloc(void)
{
	struct led_project_menu menu =
	{
		.projects_folder_allocated = 0,
		.projects_folder_refresh = 0,
		.selected_path = Utf8Empty(),
		.dir_nav = directory_navigator_alloc(4096, 64, 64),
		.dir_list = ui_list_init(AXIS_2_Y, 200.0f, 24.0f, UI_SELECTION_UNIQUE),
		.window = HI_NULL_INDEX,
		.popup_new_project = ui_popup_null(),
		.utf8_new_project = Utf8Empty(),
		.input_line_new_project = ui_text_input_empty(),
	};

	return menu;	
}

void led_project_menu_dealloc(struct led_project_menu *menu)
{
	directory_navigator_dealloc(&menu->dir_nav);
}

struct led *led_alloc(void)
{
	led_core_init_commands();
	g_editor->mem_persistent = ArenaAlloc(16*1024*1024);

	g_editor->window = system_process_root_window_alloc("Level Editor", Vec2u32Inline(400,400), Vec2u32Inline(1280, 720));

	g_editor->frame = ArenaAlloc(16*1024*1024);
	g_editor->project_menu = led_project_menu_alloc();
	g_editor->running = 1;
	g_editor->ns = ds_TimeNs();
	g_editor->root_folder = file_null();

	//const vec3 position = {-40.0f, 3.0f, -30.0f};
	//const vec3 left = {0.0f, 0.0f, 1.0f};
	//const vec3 up = {0.0f, 1.0f, 0.0f};
	//const vec3 dir = {1.0f, 0.0f, 0.0f};
	
	const vec3 position = {10.0f, 1.0f, 5.0f};
	//const vec3 position = {3.0f, 1.0f, -3.0f};
	const vec3 left = {1.0f, 0.0f, 0.0f};
	const vec3 up = {0.0f, 1.0f, 0.0f};
	const vec3 dir = {0.0f, 0.0f, 1.0f};
	vec2 size = { 1280.0f, 720.0f };
	r_camera_construct(&g_editor->cam, 
			position, 
			left,
			up,
			dir,
			0.0f,
			0.0f,
			0.0250f,
			1024.0f,
			(f32) size[0] / size[1],
			2.0f * MM_PI_F / 3.0f );

	g_editor->cam_left_velocity = 0.0f;
	g_editor->cam_forward_velocity = 0.0f;
	g_editor->ns_delta = 0;
	g_editor->ns_delta_modifier = 1.0f;

	g_editor->project.initialized = 0;
	g_editor->project.folder = file_null();
	g_editor->project.file = file_null();

	struct system_window *sys_win = system_window_address(g_editor->window);
	enum fs_error err; 
	if ((err = directory_try_create_at_cwd(&sys_win->mem_persistent, &g_editor->root_folder, LED_ROOT_FOLDER_PATH)) != FS_SUCCESS)
	{
		if ((err = directory_try_open_at_cwd(&sys_win->mem_persistent, &g_editor->root_folder, LED_ROOT_FOLDER_PATH)) != FS_SUCCESS)
		{
			LogString(T_SYSTEM, S_FATAL, "Failed to open projects folder, exiting.");
			FatalCleanupAndExit(ds_ThreadSelfTid());
		}
	}
	
	g_editor->viewport_id = Utf8Format(&sys_win->mem_persistent, "viewport_%u", g_editor->window);
	g_editor->node_pool = GPoolAlloc(NULL, 4096, struct led_node, GROWABLE);
	g_editor->node_map = HashMapAlloc(NULL, 4096, 4096, GROWABLE);
	g_editor->node_marked_list = dll_Init(struct led_node);
	g_editor->node_non_marked_list = dll_Init(struct led_node);
	g_editor->node_selected_list = dll2_Init(struct led_node);
	g_editor->csg = csg_alloc();
	g_editor->render_mesh_db = strdb_Alloc(NULL, 32, 32, struct r_mesh, GROWABLE);
	g_editor->rb_prefab_db = strdb_Alloc(NULL, 32, 32, struct rigid_body_prefab, GROWABLE);
	g_editor->cs_db = strdb_Alloc(NULL, 32, 32, struct collision_shape, GROWABLE);
	g_editor->physics = physics_pipeline_alloc(NULL, 1024, NSEC_PER_SEC / (u64) 60, 1024*1024, &g_editor->cs_db, &g_editor->rb_prefab_db);

	g_editor->pending_engine_running = 0;
	g_editor->pending_engine_initalized = 0;
	g_editor->pending_engine_paused = 0;
	g_editor->engine_running = 0;
	g_editor->engine_initalized = 0;
	g_editor->engine_paused = 0;
	g_editor->ns_engine_running = 0;

	struct r_mesh *r_mesh_stub = strdb_Address(&g_editor->render_mesh_db, STRING_DATABASE_STUB_INDEX);
	r_mesh_set_stub_box(r_mesh_stub);

	struct collision_shape *shape_stub = strdb_Address(&g_editor->cs_db, STRING_DATABASE_STUB_INDEX);
	shape_stub->type = COLLISION_SHAPE_CONVEX_HULL;
	shape_stub->hull = dcel_box(&sys_win->mem_persistent, Vec3Inline(0.5f, 0.5f, 0.5f));

	struct rigid_body_prefab *prefab_stub = strdb_Address(&g_editor->rb_prefab_db, STRING_DATABASE_STUB_INDEX);
	prefab_stub->shape = strdb_Reference(&g_editor->cs_db, Utf8Inline("")).index;
	prefab_stub->density = 1.0f;
	prefab_stub->restitution = 0.0f;
	prefab_stub->friction = 0.0f;
	prefab_stub->dynamic = 1;
	prefab_statics_setup(prefab_stub, shape_stub, prefab_stub->density);

	return g_editor;
}

void led_dealloc(struct led *led)
{
	ArenaFree(&led->mem_persistent);
	led_project_menu_dealloc(&led->project_menu);
	csg_dealloc(&led->csg);
	GPoolDealloc(&led->node_pool);
	ArenaFree(&g_editor->frame);
}
