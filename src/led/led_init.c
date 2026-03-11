/*
==========================================================================
    Copyright (C) 2025, 2026 Axel Sandstedt 

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

struct led_ProjectMenu led_ProjectMenuAlloc(void)
{
	struct led_ProjectMenu menu =
	{
		.projects_folder_allocated = 0,
		.projects_folder_refresh = 0,
		.selected_path = Utf8Empty(),
		.dir_nav = DirectoryNavigatorAlloc(4096, 64, 64),
		.dir_list = ui_ListInit(AXIS_2_Y, 200.0f, 24.0f, UI_SELECTION_UNIQUE),
		.window = HI_NULL_INDEX,
		.popup_new_project = ui_PopupNull(),
		.utf8_new_project = Utf8Empty(),
		.input_line_new_project = ui_TextInputEmpty(),
	};

	return menu;	
}

void led_ProjectMenuDealloc(struct led_ProjectMenu *menu)
{
	DirectoryNavigatorDealloc(&menu->dir_nav);
}

const char *body_color_mode_str_buf[RB_COLOR_MODE_COUNT] = 
{
	"RB_COLOR_MODE_BODY",
	"RB_COLOR_MODE_COLLISION",
	"RB_COLOR_MODE_ISLAND",
	"RB_COLOR_MODE_SLEEP",
};
const char **body_color_mode_str = body_color_mode_str_buf;


struct led *led_Alloc(void)
{
	led_CoreInitCommands();
	g_editor->mem_persistent = ArenaAlloc(16*1024*1024);

	g_editor->window = ds_RootWindowAlloc("Level Editor", Vec2U32Inline(400,400), Vec2U32Inline(1280, 720));

	g_editor->frame = ArenaAlloc(16*1024*1024);
	g_editor->project_menu = led_ProjectMenuAlloc();
	g_editor->running = 1;
	g_editor->ns = ds_TimeNs();
	g_editor->root_folder = FileNull();

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
	r_CameraConstruct(&g_editor->cam, 
			position, 
			left,
			up,
			dir,
			0.0f,
			0.0f,
			0.0250f,
			1024.0f,
			(f32) size[0] / size[1],
			2.0f * F32_PI / 3.0f );

	g_editor->cam_left_velocity = 0.0f;
	g_editor->cam_forward_velocity = 0.0f;
	g_editor->ns_delta = 0;
	g_editor->ns_delta_modifier = 1.0f;

	g_editor->project.initialized = 0;
	g_editor->project.folder = FileNull();
	g_editor->project.file = FileNull();

	struct ds_Window *sys_win = ds_WindowAddress(g_editor->window);
	enum fsError err; 
	if ((err = DirectoryTryCreateAtCwd(&sys_win->mem_persistent, &g_editor->root_folder, LED_ROOT_FOLDER_PATH)) != FS_SUCCESS)
	{
		if ((err = DirectoryTryOpenAtCwd(&sys_win->mem_persistent, &g_editor->root_folder, LED_ROOT_FOLDER_PATH)) != FS_SUCCESS)
		{
			LogString(T_SYSTEM, S_FATAL, "Failed to open projects folder, exiting.");
			FatalCleanupAndExit();
		}
	}
	
	g_editor->viewport_id = Utf8Format(&sys_win->mem_persistent, "viewport_%u", g_editor->window);
	g_editor->node_hierarchy = hi_Alloc(NULL, 4096, struct led_Node, GROWABLE);
	g_editor->node_map = ds_HashMapAlloc(NULL, 4096, 4096, GROWABLE);
	//g_editor->node_selected_list = dll2_Init(struct led_Node);
	g_editor->csg = csg_Alloc();
	g_editor->render_mesh_db = strdb_Alloc(NULL, 32, 32, struct r_Mesh, GROWABLE);
	g_editor->shape_prefab_db = strdb_Alloc(NULL, 32, 32, struct ds_ShapePrefab, GROWABLE);
    g_editor->shape_prefab_instance_pool = ds_PoolAlloc(NULL, 4096, struct ds_ShapePrefabInstance, GROWABLE);
	g_editor->body_prefab_db = strdb_Alloc(NULL, 32, 32, struct ds_RigidBodyPrefab, GROWABLE);
	g_editor->cs_db = strdb_Alloc(NULL, 32, 32, struct c_Shape, GROWABLE);
	g_editor->physics = PhysicsPipelineAlloc(&g_editor->mem_persistent, 1024, NSEC_PER_SEC / (u64) 60, 16*1024*1024, &g_editor->cs_db, &g_editor->body_prefab_db);

	g_editor->pending_engine_running = 0;
	g_editor->pending_engine_initalized = 0;
	g_editor->pending_engine_paused = 0;
	g_editor->engine_running = 0;
	g_editor->engine_initalized = 0;
	g_editor->engine_paused = 0;
	g_editor->ns_engine_running = 0;

	struct r_Mesh *r_mesh_stub = strdb_Address(&g_editor->render_mesh_db, STRING_DATABASE_STUB_INDEX);
	r_MeshStubBox(r_mesh_stub);

	struct c_Shape *cshape_stub = strdb_Address(&g_editor->cs_db, STRING_DATABASE_STUB_INDEX);
	cshape_stub->type = C_SHAPE_CONVEX_HULL;
	cshape_stub->hull = DcelBox(&sys_win->mem_persistent, Vec3Inline(0.5f, 0.5f, 0.5f));
	c_ShapeUpdateMassProperties(cshape_stub);

    struct ds_ShapePrefab *shape_stub = strdb_Address(&g_editor->shape_prefab_db, STRING_DATABASE_STUB_INDEX);
    shape_stub->cshape = strdb_Reference(&g_editor->cs_db, Utf8Inline("")).index;
	shape_stub->density = 1.0f;
	shape_stub->restitution = 0.0f;
	shape_stub->friction = 0.0f;
    shape_stub->render_mesh = strdb_Reference(&g_editor->render_mesh_db, Utf8Inline("")).index;

    struct slot slot = ds_PoolAdd(&g_editor->shape_prefab_instance_pool);
    const u32 instance_index = slot.index;
    struct ds_ShapePrefabInstance *instance = slot.address;
    instance->id = Utf8CstrBuffered(instance->id_buf, PREFAB_BUFSIZE, "Stub");
	instance->shape_prefab = strdb_Reference(&g_editor->shape_prefab_db, Utf8Inline("")).index;
    instance->t_local = ds_TransformIdentity();

	struct ds_RigidBodyPrefab *prefab_stub = strdb_Address(&g_editor->body_prefab_db, STRING_DATABASE_STUB_INDEX);
	prefab_stub->dynamic = 1;
    prefab_stub->shape_list = dll_Init(struct ds_ShapePrefabInstance);
    dll_Append(&prefab_stub->shape_list, g_editor->shape_prefab_instance_pool.buf, instance_index);

    slot = hi_Add(&g_editor->node_hierarchy, HI_ROOT_STUB_INDEX);
    ds_Assert(slot.index == LED_NODE_ROOT);

	g_editor->body_color_mode = RB_COLOR_MODE_BODY;
	g_editor->pending_body_color_mode = RB_COLOR_MODE_COLLISION;
	Vec4Set(g_editor->collision_color, 1.0f, 0.1f, 0.1f, 0.5f);
	Vec4Set(g_editor->static_color, 0.6f, 0.6f, 0.6f, 0.5f);
	Vec4Set(g_editor->sleep_color, 113.0f/256.0f, 241.0f/256.0f, 157.0f/256.0f, 0.7f);
	Vec4Set(g_editor->awake_color, 255.0f/256.0f, 36.0f/256.0f, 48.0f/256.0f, 0.7f);
	Vec4Set(g_editor->manifold_color, 0.6f, 0.6f, 0.9f, 1.0f);
	Vec4Set(g_editor->dbvh_color, 0.8f, 0.1f, 0.0f, 0.6f);
	Vec4Set(g_editor->sbvh_color, 0.0f, 0.8f, 0.1f, 0.6f);
	Vec4Set(g_editor->bounding_box_color, 0.8f, 0.1f, 0.6f, 1.0f);

	g_editor->draw_bounding_box = 0;
	g_editor->draw_dbvh = 0;
	g_editor->draw_sbvh = 1;
	g_editor->draw_manifold = 0;
	g_editor->draw_lines = 0;

	return g_editor;
}

void led_Dealloc(struct led *led)
{
	ArenaFree(&led->mem_persistent);
	led_ProjectMenuDealloc(&led->project_menu);
	csg_Dealloc(&led->csg);
	ds_HashMapDealloc(&led->node_map);
	hi_Dealloc(&led->node_hierarchy);
	ArenaFree(&g_editor->frame);
}
