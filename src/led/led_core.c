/*
==========================================================================
    Copyright (C) 2025,2026 Axel Sandstedt 

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

#include "led_local.h"
#include "ds_random.h"

void cmd_led_compile(void);
void cmd_led_run(void);
void cmd_led_pause(void);
void cmd_led_stop(void);

void cmd_led_node_add(void);
void cmd_led_node_remove(void);
void cmd_led_node_set_position(void);
void cmd_led_node_set_rb_prefab(void);
void cmd_led_node_set_csg_brush(void);
void cmd_led_node_set_proxy3d(void);

void cmd_collision_shape_add(void);
void cmd_collision_shape_remove(void);
void cmd_collision_tri_mesh_bvh_add(void);
void cmd_collision_dcel_add(void);
void cmd_collision_box_add(void);
void cmd_collision_sphere_add(void);
void cmd_collision_capsule_add(void);

void cmd_rb_prefab_add(void);
void cmd_rb_prefab_remove(void);

void cmd_render_mesh_add(void);
void cmd_render_mesh_remove(void);

u32 cmd_led_node_add_id;
u32 cmd_led_node_remove_id;
u32 cmd_led_node_set_position_id;
u32 cmd_led_node_set_rb_prefab_id;
u32 cmd_led_node_set_csg_brush_id;
u32 cmd_led_node_set_proxy3d_id;

u32	cmd_led_compile_id;
u32	cmd_led_run_id;
u32	cmd_led_pause_id;
u32	cmd_led_stop_id;

u32 cmd_rb_prefab_add_id;
u32 cmd_rb_prefab_remove_id;

u32 cmd_render_mesh_add_id;
u32 cmd_render_mesh_remove_id;

u32 cmd_collision_shape_add_id;
u32 cmd_collision_shape_remove_id;
u32 cmd_collision_box_add_id;
u32 cmd_collision_dcel_add_id;
u32 cmd_collision_sphere_add_id;
u32 cmd_collision_capsule_add_id;
u32 cmd_collision_tri_mesh_bvh_add_id;

void led_core_init_commands(void)
{
	cmd_led_node_add_id = cmd_function_register(Utf8Inline("led_node_add"), 1, &cmd_led_node_add).index;
	cmd_led_node_remove_id = cmd_function_register(Utf8Inline("led_node_remove"), 1, &cmd_led_node_remove).index;
	cmd_led_node_set_position_id = cmd_function_register(Utf8Inline("led_node_set_position"), 4, &cmd_led_node_set_position).index;
	cmd_led_node_set_rb_prefab_id = cmd_function_register(Utf8Inline("led_node_set_rb_prefab"), 2, &cmd_led_node_set_rb_prefab).index;
	cmd_led_node_set_csg_brush_id = cmd_function_register(Utf8Inline("led_node_set_csg_brush"), 2, &cmd_led_node_set_csg_brush).index;
	cmd_led_node_set_proxy3d_id = cmd_function_register(Utf8Inline("led_node_set_proxy3d"), 7, &cmd_led_node_set_proxy3d).index;

	cmd_led_compile_id = cmd_function_register(Utf8Inline("led_compile"), 0, &cmd_led_compile).index;
	cmd_led_run_id = cmd_function_register(Utf8Inline("led_run"), 0, &cmd_led_run).index;
	cmd_led_pause_id = cmd_function_register(Utf8Inline("led_pause"), 0, &cmd_led_pause).index;
	cmd_led_stop_id = cmd_function_register(Utf8Inline("led_stop"), 0, &cmd_led_stop).index;

	cmd_rb_prefab_add_id = cmd_function_register(Utf8Inline("rb_prefab_add"), 6, &cmd_rb_prefab_add).index;
	cmd_rb_prefab_remove_id = cmd_function_register(Utf8Inline("rb_prefab_remove"), 1, &cmd_rb_prefab_remove).index;

	cmd_render_mesh_add_id = cmd_function_register(Utf8Inline("render_mesh_add"), 2, &cmd_render_mesh_add).index;
	cmd_render_mesh_remove_id = cmd_function_register(Utf8Inline("render_mesh_remove"), 1, &cmd_render_mesh_remove).index;

	cmd_collision_shape_add_id = cmd_function_register(Utf8Inline("collision_shape_add"), 1, &cmd_collision_shape_add).index;
	cmd_collision_box_add_id = cmd_function_register(Utf8Inline("collision_box_add"), 4, &cmd_collision_box_add).index;
	cmd_collision_dcel_add_id = cmd_function_register(Utf8Inline("collision_dcel_add"), 2, &cmd_collision_dcel_add).index;
	cmd_collision_sphere_add_id = cmd_function_register(Utf8Inline("collision_sphere_add"), 2, &cmd_collision_sphere_add).index;
	cmd_collision_capsule_add_id = cmd_function_register(Utf8Inline("collision_capsule_add"), 3, &cmd_collision_capsule_add).index;
	cmd_collision_tri_mesh_bvh_add_id = cmd_function_register(Utf8Inline("collision_tri_mesh_bvh_add"), 2, &cmd_collision_tri_mesh_bvh_add).index;
	cmd_collision_shape_remove_id = cmd_function_register(Utf8Inline("collision_shape_remove"), 1, &cmd_collision_shape_remove).index;
}

void cmd_led_node_add(void)
{
	led_node_add(g_editor, g_queue->cmd_exec->arg[0].utf8);
}

void cmd_led_node_remove(void)
{
	led_node_remove(g_editor, g_queue->cmd_exec->arg[0].utf8);
}

void cmd_led_node_set_position(void)
{
	const vec3 position =
	{
		g_queue->cmd_exec->arg[1].f32,
		g_queue->cmd_exec->arg[2].f32,
		g_queue->cmd_exec->arg[3].f32,
	};
	led_node_set_position(g_editor, g_queue->cmd_exec->arg[0].utf8, position);
}

void cmd_led_node_set_rb_prefab(void)
{
	led_node_set_rb_prefab(g_editor, g_queue->cmd_exec->arg[0].utf8, g_queue->cmd_exec->arg[1].utf8);
}

void cmd_led_node_set_csg_brush(void)
{
	led_node_set_csg_brush(g_editor, g_queue->cmd_exec->arg[0].utf8, g_queue->cmd_exec->arg[1].utf8);
}

void cmd_led_node_set_proxy3d(void)
{
	const vec4 color = 
	{
		g_queue->cmd_exec->arg[2].f32,
		g_queue->cmd_exec->arg[3].f32,
		g_queue->cmd_exec->arg[4].f32,
		g_queue->cmd_exec->arg[5].f32,
	};
	const f32 blend = g_queue->cmd_exec->arg[6].f32;
	led_node_set_proxy3d(g_editor, g_queue->cmd_exec->arg[0].utf8, g_queue->cmd_exec->arg[1].utf8, color, blend);
}

void cmd_collision_shape_add(void)
{
	g_queue->cmd_exec->arg[1].f32 = 0.5f;
	g_queue->cmd_exec->arg[2].f32 = 0.5f;
	g_queue->cmd_exec->arg[3].f32 = 0.5f;
	cmd_collision_box_add();	
}

void cmd_collision_box_add(void)
{
	//TODO Need to come up with a memory allocation strategy for dcels
	struct system_window *sys_win = system_window_address(g_editor->window);
	const vec3 hw =
	{
		g_queue->cmd_exec->arg[1].f32,
		g_queue->cmd_exec->arg[2].f32,
		g_queue->cmd_exec->arg[3].f32,
	};

	if (hw[0] > 0.0f && hw[1] > 0.0f && hw[2] > 0.0f)
	{
		struct collision_shape shape =
		{
			.id = g_queue->cmd_exec->arg[0].utf8,
			.type = COLLISION_SHAPE_CONVEX_HULL,
			.hull = dcel_box(&sys_win->mem_persistent, hw), 
			.center_of_mass_localized = 0,
		};

		led_collision_shape_add(g_editor, &shape);
	}
	else
	{
		LogString(T_LED, S_WARNING, "Failed to allocate collision box: bad parameters");
	}
}

void cmd_collision_dcel_add(void)
{
	struct dcel *dcel = g_queue->cmd_exec->arg[1].ptr;
	if (dcel->v_count)
	{
		struct collision_shape shape =
		{
			.id = g_queue->cmd_exec->arg[0].utf8,
			.type = COLLISION_SHAPE_CONVEX_HULL,
			.hull = *dcel, 
			.center_of_mass_localized = 0,
		};

		led_collision_shape_add(g_editor, &shape);
	}
	else
	{
		LogString(T_LED, S_WARNING, "Failed to allocate collision dcel: bad parameters");
	}
}

void cmd_collision_tri_mesh_bvh_add(void)
{
	struct tri_mesh_bvh *mesh_bvh = g_queue->cmd_exec->arg[1].ptr;
	if (mesh_bvh->mesh->v_count && bt_NodeCount(&mesh_bvh->bvh.tree))
	{
		struct collision_shape shape =
		{
			.id = g_queue->cmd_exec->arg[0].utf8,
			.type = COLLISION_SHAPE_TRI_MESH,
			.mesh_bvh = *mesh_bvh, 
			.center_of_mass_localized = 1,
		};

		led_collision_shape_add(g_editor, &shape);
	}
	else
	{
		LogString(T_LED, S_WARNING, "Failed to allocate collision tri_mesh: bad parameters");
	}
}

void cmd_collision_sphere_add(void)
{
	struct collision_shape shape =
	{
		.id = g_queue->cmd_exec->arg[0].utf8,
		.type = COLLISION_SHAPE_SPHERE,
		.sphere = { .radius = g_queue->cmd_exec->arg[1].f32 },
		.center_of_mass_localized = 1,
	};

	if (shape.sphere.radius > 0.0f)
	{
		led_collision_shape_add(g_editor, &shape);
	}
	else
	{
		LogString(T_LED, S_WARNING, "Failed to allocate collision sphere: bad parameters");
	}
}

void cmd_collision_capsule_add(void)
{
	struct collision_shape shape =
	{
		.id = g_queue->cmd_exec->arg[0].utf8,
		.type = COLLISION_SHAPE_CAPSULE,
		.center_of_mass_localized = 1,
		.capsule = 
		{ 
			.radius = g_queue->cmd_exec->arg[1].f32,
			.half_height = g_queue->cmd_exec->arg[2].f32,
		},
	};

	if (shape.capsule.radius > 0.0f && shape.capsule.half_height > 0.0f)
	{
		led_collision_shape_add(g_editor, &shape);
	}
	else
	{
		LogString(T_LED, S_WARNING, "Failed to allocate collision capsule: bad parameters");
	}
}

void cmd_collision_shape_remove(void)
{
	led_collision_shape_remove(g_editor, g_queue->cmd_exec->arg[0].utf8);
}

struct slot led_collision_shape_add(struct led *led, const struct collision_shape *shape)
{
	struct slot slot = empty_slot;
	if (!shape->id.len)
	{
		LogString(T_LED, S_WARNING, "Failed to allocate collision shape: shape->id must not be empty");
	} 
	else if (strdb_Lookup(&led->cs_db, shape->id).index != STRING_DATABASE_STUB_INDEX) 
	{
		LogString(T_LED, S_WARNING, "Failed to allocate collision shape: shape with given id already exist");
	}
	else
	{ 
		u8 *buf = ThreadAlloc256B();
		const utf8 copy = Utf8CopyBuffered(buf, 256, shape->id);	
		if (!copy.len)
		{
			LogString(T_LED, S_WARNING, "Failed to allocate collision shape: shape->id size must be <= 256B");
			ThreadFree256B(buf);
		}
		else
		{
			slot = strdb_AddAndAlias(&led->cs_db, copy);
			struct collision_shape *new_shape = slot.address;
			new_shape->type = shape->type;
			new_shape->center_of_mass_localized = shape->center_of_mass_localized;
			switch (shape->type)
			{
				case COLLISION_SHAPE_SPHERE: 
				{ 
					new_shape->sphere = shape->sphere; 
				} break;

				case COLLISION_SHAPE_CAPSULE: 
				{ 
					new_shape->capsule = shape->capsule; 
				} break;

				case COLLISION_SHAPE_CONVEX_HULL: 
				{ 
					new_shape->hull = shape->hull; 
				} break;

				case COLLISION_SHAPE_TRI_MESH: 
				{ 
					new_shape->mesh_bvh = shape->mesh_bvh; 
				} break;
			};
		}
	}

	return slot;
}

void led_collision_shape_remove(struct led *led, const utf8 id)
{
	struct slot slot = led_collision_shape_lookup(led, id);
	struct collision_shape *shape = slot.address;
	if (slot.index != STRING_DATABASE_STUB_INDEX && shape->reference_count == 0)
	{
		void *buf = shape->id.buf;
		strdb_Remove(&led->cs_db, id);
		ThreadFree256B(buf);
	}
}

struct slot led_collision_shape_lookup(struct led *led, const utf8 id)
{
	return strdb_Lookup(&led->cs_db, id);
}

void cmd_render_mesh_add(void)
{
	led_render_mesh_add(g_editor, g_queue->cmd_exec->arg[0].utf8, g_queue->cmd_exec->arg[1].utf8);
}

void cmd_render_mesh_remove(void)
{
	led_render_mesh_remove(g_editor, g_queue->cmd_exec->arg[0].utf8);
}

struct slot led_render_mesh_add(struct led *led, const utf8 id, const utf8 shape)
{
	struct slot slot = empty_slot;
	if (!id.len)
	{
		LogString(T_LED, S_WARNING, "Failed to allocate render mesh: id must not be empty");
	} 
	else if (strdb_Lookup(&led->render_mesh_db, id).index != STRING_DATABASE_STUB_INDEX) 
	{
		LogString(T_LED, S_WARNING, "Failed to allocate render mesh: mesh with given id already exist");
	}
	else
	{ 
		u8 *buf = ThreadAlloc256B();
		const utf8 copy = Utf8CopyBuffered(buf, 256, id);	
		if (!copy.len)
		{
			LogString(T_LED, S_WARNING, "Failed to allocate render mesh: id size must be <= 256B");
			ThreadFree256B(buf);
		}
		else
		{
			slot = strdb_AddAndAlias(&led->render_mesh_db, copy);
			struct r_mesh *mesh = slot.address;

			struct slot ref = strdb_Lookup(&led->cs_db, shape);
			if (ref.index == STRING_DATABASE_STUB_INDEX)
			{
				LogString(T_LED, S_WARNING, "In render_mesh_add: shape not found, stub_shape chosen");
			}

			struct system_window *sys_win = system_window_address(g_editor->window);
			const struct collision_shape *s = ref.address;
			switch (s->type)
			{
				case COLLISION_SHAPE_SPHERE: { r_mesh_set_sphere(&sys_win->mem_persistent, mesh, s->sphere.radius, 12); } break;
				case COLLISION_SHAPE_CAPSULE: { r_mesh_set_capsule(&sys_win->mem_persistent, mesh, s->capsule.half_height, s->capsule.radius, 16); } break;
				case COLLISION_SHAPE_CONVEX_HULL: { r_mesh_set_hull(&sys_win->mem_persistent, mesh, &s->hull); } break;
				case COLLISION_SHAPE_TRI_MESH: { r_mesh_set_tri_mesh(&sys_win->mem_persistent, mesh, s->mesh_bvh.mesh); } break;
			}
		}
	}

	return slot;
}

void led_render_mesh_remove(struct led *led, const utf8 id)
{
	struct slot slot = strdb_Lookup(&led->render_mesh_db, id);
	struct r_mesh *mesh = slot.address;
	if (slot.index != STRING_DATABASE_STUB_INDEX && mesh->reference_count == 0)
	{
		void *buf = mesh->id.buf;
		strdb_Remove(&led->render_mesh_db, id);
		ThreadFree256B(buf);
	}
}

struct slot led_render_mesh_lookup(struct led *led, const utf8 id)
{
	return strdb_Lookup(&led->render_mesh_db, id);
}

void cmd_rb_prefab_add(void)
{
	const utf8 id = g_queue->cmd_exec->arg[0].utf8;
	const utf8 shape = g_queue->cmd_exec->arg[1].utf8;
	const f32 density = g_queue->cmd_exec->arg[2].f32;
	const f32 restitution = g_queue->cmd_exec->arg[3].f32;
	const f32 friction = g_queue->cmd_exec->arg[4].f32;
	const u32 dynamic = (u32) g_queue->cmd_exec->arg[5].u64;

	led_rigid_body_prefab_add(g_editor, id, shape, density, restitution, friction, dynamic);
}

void cmd_rb_prefab_remove(void)
{
	led_rigid_body_prefab_remove(g_editor, g_queue->cmd_exec->arg[0].utf8);
}

struct slot led_rigid_body_prefab_add(struct led *led, const utf8 id, const utf8 shape, const f32 density, const f32 restitution, const f32 friction, const u32 dynamic)
{
	struct slot slot = empty_slot;	
	if (!id.len)
	{
		LogString(T_LED, S_WARNING, "Failed to allocate rb_prefab: prefab->id must not be empty");
	} 
	else if (strdb_Lookup(&led->rb_prefab_db, id).index != STRING_DATABASE_STUB_INDEX) 
	{
		LogString(T_LED, S_WARNING, "Failed to allocate rb_prefab: prefab with given id already exist");
	}
	else
	{ 
		u8 *buf = ThreadAlloc256B();
		const utf8 copy = Utf8CopyBuffered(buf, 256, id);	
		if (!copy.len)
		{
			LogString(T_LED, S_WARNING, "Failed to allocate rb_prefab: prefab->id size must be <= 256B");
			ThreadFree256B(buf);
		}
		else
		{
			struct slot ref = strdb_Reference(&led->cs_db, shape);
			if (ref.index == STRING_DATABASE_STUB_INDEX)
			{
				LogString(T_LED, S_WARNING, "In rb_prefab: shape not found, stub_shape chosen");
			}
			slot = strdb_AddAndAlias(&led->rb_prefab_db, copy);
			struct rigid_body_prefab *prefab = slot.address;

			prefab->shape = ref.index;
			prefab->restitution = restitution;
			prefab->friction = friction;
			prefab->dynamic = dynamic;
			prefab->density = density;
			prefab_statics_setup(prefab, ref.address, density);
		}
	}

	return slot;
}

void led_rigid_body_prefab_remove(struct led *led, const utf8 id)
{
	struct slot slot = led_rigid_body_prefab_lookup(led, id);
	struct rigid_body_prefab *prefab = slot.address;
	if (slot.index != STRING_DATABASE_STUB_INDEX && prefab->reference_count == 0)
	{
		void *buf = prefab->id.buf;
		strdb_Dereference(&led->cs_db, prefab->shape);
		strdb_Remove(&led->rb_prefab_db, id);
		ThreadFree256B(buf);
	}	
}

struct slot led_rigid_body_prefab_lookup(struct led *led, const utf8 id)
{
	return strdb_Lookup(&led->rb_prefab_db, id);
}

struct slot led_node_add(struct led *led, const utf8 id)
{
	struct slot slot = empty_slot;
	if (!id.len)
	{
		LogString(T_LED, S_WARNING, "Failed to allocate led_node: id must not be empty");
	} 
	else if (led_node_lookup(led, id).address != STRING_DATABASE_STUB_INDEX) 
	{
		LogString(T_LED, S_WARNING, "Failed to allocate led_node: node with given id already exist");
	}
	else
	{ 
		void *buf = ThreadAlloc256B();
		const utf8 copy = Utf8CopyBuffered(buf, 256, id);	
		const u32 key = Utf8Hash(id);
		if (!copy.len)
		{
			LogString(T_LED, S_WARNING, "Failed to allocate led_node: id size must be <= 256B");
			ThreadFree256B(buf);
		} 
		else
		{
			slot = GPoolAdd(&led->node_pool);
			HashMapAdd(led->node_map, key, slot.index);
			dll_Append(&led->node_non_marked_list, led->node_pool.buf, slot.index);
			dll_SlotSetNotInList(&led->node_selected_list, slot.address);

			struct led_node *node = slot.address;
			node->flags = LED_FLAG_NONE;
			node->id = copy;
			node->key = key;
			node->cache = ui_node_cache_null();

			const vec3 axis = { 0.0f, 1.0f, 0.0f };
			Vec3Set(node->position, 0.0f, 0.0f, 0.0f);
			QuatAxisAngle(node->rotation, axis, 0.0f);

			node->rb_prefab = STRING_DATABASE_STUB_INDEX;
			node->proxy = HI_NULL_INDEX;
			node->csg_brush = STRING_DATABASE_STUB_INDEX;
		}
	}

	return slot;
}

struct slot led_node_lookup(struct led *led, const utf8 id)
{
	const u32 key = Utf8Hash(id);
	struct slot slot = empty_slot;
	for (u32 i = HashMapFirst(led->node_map, key); i != HASH_NULL; i = HashMapNext(led->node_map, i))
	{
		struct led_node *node = GPoolAddress(&led->node_pool, i);
		if (Utf8Equivalence(id, node->id))
		{
			slot.index = i;
			slot.address = node;
			break;
		}
	}

	return slot;
}

static void led_remove_marked_structs(struct led *led)
{
	struct led_node *node = NULL;
	for (u32 i = led->node_marked_list.first; i != DLL_NULL; i = dll_Next(node))
	{
		node = GPoolAddress(&led->node_pool, i);
		if (node->flags & LED_CONSTANT)
		{
			node->flags &= ~LED_MARKED_FOR_REMOVAL;
			dll_Remove(&led->node_marked_list, led->node_pool.buf, i);
			dll_Append(&led->node_non_marked_list, led->node_pool.buf, i);
			continue;
		}

		if (dll2_InList(node))
		{
			//fprintf(stderr, "Was selected, removing from list - ");
			Utf8DebugPrint(node->id);
			dll_Remove(&led->node_selected_list, led->node_pool.buf, i);
		}

		if (node->proxy != HI_NULL_INDEX)
		{
			r_proxy3d_dealloc(&led->frame, node->proxy);
		}

		strdb_Dereference(&led->rb_prefab_db, node->rb_prefab);
		strdb_Dereference(&led->csg.brush_db, node->csg_brush);

		node->rb_prefab = STRING_DATABASE_STUB_INDEX;
		node->csg_brush = STRING_DATABASE_STUB_INDEX;
		node->proxy = HI_NULL_INDEX;

		HashMapRemove(led->node_map, node->key, i);
		ThreadFree256B(node->id.buf);
		GPoolRemove(&led->node_pool, i);
	}

	dll_Flush(&led->node_marked_list);
}

void led_node_remove(struct led *led, const utf8 id)
{
	struct slot slot = led_node_lookup(led, id);
	struct led_node *node = slot.address;
	if (node)
	{
		node->flags |= LED_MARKED_FOR_REMOVAL;
		dll_Remove(&led->node_non_marked_list, led->node_pool.buf, slot.index);
		dll_Append(&led->node_marked_list, led->node_pool.buf, slot.index);
	}
}

void led_node_set_position(struct led *led, const utf8 id, const vec3 position)
{
	struct slot slot = led_node_lookup(led, id);
	struct led_node *node = slot.address;
	if (!node)
	{
		Log(T_LED, S_WARNING, "Failed to set position of led node %k, node not found.", &id);
	}
	else if (node->flags & LED_CONSTANT)
	{
		Log(T_LED, S_WARNING, "Failed to set position of led node %k, node is constant.", &id);
	}
	else
	{
		Vec3Copy(node->position, position);
	}
}

void led_node_set_rb_prefab(struct led *led, const utf8 id, const utf8 prefab)
{
	struct slot slot = led_node_lookup(led, id);
	struct led_node *node = slot.address;
	if (!node)
	{
		Log(T_LED, S_WARNING, "Failed to set of led node %k, node not found.", &id);
	}
	else if (node->flags & LED_CONSTANT)
	{
		Log(T_LED, S_WARNING, "Failed to set of led node %k, node is constant.", &id);
	}
	else
	{
		slot = strdb_Reference(&led->rb_prefab_db, prefab);
		if (slot.index == STRING_DATABASE_STUB_INDEX)
		{
			Log(T_LED, S_WARNING, "Failed to set of led node %k, prefab not found.", &id);
		}
		else
		{
			strdb_Dereference(&led->rb_prefab_db, node->rb_prefab);
			strdb_Dereference(&led->csg.brush_db, node->csg_brush);
			node->csg_brush = STRING_DATABASE_STUB_INDEX;

			node->rb_prefab = slot.index;
			node->flags &= ~(LED_CSG | LED_PHYSICS);
			node->flags |= LED_PHYSICS;
		}
	}
}

void led_node_set_csg_brush(struct led *led, const utf8 id, const utf8 brush)
{
	struct slot slot = led_node_lookup(led, id);
	struct led_node *node = slot.address;
	if (!node)
	{
		Log(T_LED, S_WARNING, "Failed to set of led node %k, node not found.", &id);
	}
	else if (node->flags & LED_CONSTANT)
	{
		Log(T_LED, S_WARNING, "Failed to set of led node %k, node is constant.", &id);
	}
	else
	{
		slot = strdb_Reference(&led->csg.brush_db, brush);
		if (slot.index == STRING_DATABASE_STUB_INDEX)
		{
			Log(T_LED, S_WARNING, "Failed to set of led node %k, brush not found.", &id);
		}
		else
		{
			strdb_Dereference(&led->rb_prefab_db, node->rb_prefab);
			strdb_Dereference(&led->csg.brush_db, node->csg_brush);
			node->rb_prefab = STRING_DATABASE_STUB_INDEX;

			node->csg_brush = slot.index;
			node->flags &= ~(LED_CSG | LED_PHYSICS);
			node->flags |= LED_CSG;
		}
	}
}

void led_node_set_proxy3d(struct led *led, const utf8 id, const utf8 mesh, const vec4 color, const f32 blend)
{
	struct slot slot = led_node_lookup(led, id);
	struct led_node *node = slot.address;
	if (!node)
	{
		Log(T_LED, S_WARNING, "Failed to set of led node %k, node not found.", &id);
	}
	else if (node->flags & LED_CONSTANT)
	{
		Log(T_LED, S_WARNING, "Failed to set of led node %k, node is constant.", &id);
	}
	else
	{
		if (node->proxy != HI_NULL_INDEX)
		{
			struct arena tmp = ArenaAlloc1MB();
			r_proxy3d_dealloc(&tmp, node->proxy);
			node->proxy = HI_NULL_INDEX;
			ArenaFree1MB(&tmp);
		}

		struct r_proxy3d_config config =
		{
			.parent = PROXY3D_ROOT,
			.linear_velocity = { 0.0f, 0.0f, 0.0f },
			.angular_velocity = { 0.0f, 0.0f, 0.0f },
			.mesh = mesh,
			.ns_time = led->ns,
		};
		Vec4Copy(config.color, color);
		config.blend = blend; 
		Vec3Copy(config.position, node->position);
		QuatCopy(config.rotation, node->rotation);
		node->proxy = r_proxy3d_alloc(&config);
		Vec4Copy(node->color, color);
	}
}

static struct tri_mesh tri_mesh_perlin_noise(struct arena *mem_persistent, const u32 n, const f32 width)
{
	ds_Assert(PowerOfTwoCheck(n) && n >= 32);

	struct arena tmp = ArenaAlloc1MB();

	struct tri_mesh mesh = 
	{
		.v_count = (n-1)*(n-1),
		.tri_count = 2*(n-2)*(n-2),
	};
	mesh.v = ArenaPush(mem_persistent, mesh.v_count*sizeof(vec3));
	mesh.tri = ArenaPush(mem_persistent, mesh.tri_count*sizeof(vec3u32));
	//TODO move out functino to tri_mesh_perlin_noise method
	//TODO return stub mesh
	ds_Assert(mesh.v && mesh.tri);

	const f32 unit = width / n;

#define OCTAVES	6
	vec2ptr grad[OCTAVES];
	for (u32 o = 0; o < OCTAVES; ++o)
	{
		const u32 on = n >> o;
		grad[o] = ArenaPush(&tmp, on*on*sizeof(vec2));
		for (u32 x = 0; x < on; ++x)
		{
			for (u32 z = 0; z < on; ++z)
			{
				const f32 angle = rng_f32_range(0.0f, MM_PI_2_F);
				grad[o][x*on + z][0] = f32_cos(angle);
				grad[o][x*on + z][1] = f32_sin(angle);
			}
		}
	}

	const vec3 offset =
	{
		-unit * (n >> 1),
		-20.0f,
		-unit * (n >> 1) + 30.0f,
	};

	for (u32 x = 0; x < n-1; ++x)
	{
		for (u32 z = 0; z < n-1; ++z)
		{
			mesh.v[x*(n-1) + z][0] = (0.5f + x) * unit;
			mesh.v[x*(n-1) + z][1] = 0.0f;
			mesh.v[x*(n-1) + z][2] = (0.5f + z) * unit;

			f32 amplitude = 1.0f / (1 << OCTAVES);
			for (u32 i = 0; i < OCTAVES; ++i)
			{
				const u32 on = n >> i;
				const u32 x_low = x - (x % (1 << i));
				const u32 z_low = z - (z % (1 << i));

				const u32 x_high = x_low + (1 << i);
				const u32 z_high = z_low + (1 << i);
	
				const vec2 bl_diff = 
				{
					mesh.v[x*(n-1) + z][0] - x_low * unit, 	
					mesh.v[x*(n-1) + z][2] - z_low * unit, 	
				};

				const vec2 tl_diff = 
				{
					mesh.v[x*(n-1) + z][0] - x_low * unit, 	
					mesh.v[x*(n-1) + z][2] - z_high * unit, 	
				};

				const vec2 br_diff = 
				{
					mesh.v[x*(n-1) + z][0] - x_high * unit, 	
					mesh.v[x*(n-1) + z][2] - z_low * unit, 	
				};

				const vec2 tr_diff = 
				{
					mesh.v[x*(n-1) + z][0] - x_high * unit, 	
					mesh.v[x*(n-1) + z][2] - z_high * unit, 	
				};

				//const f32 bl_dot = Vec2Dot(bl_diff, grad[i][(x_low >> i)*on + (z_low >> i)]);
				//const f32 br_dot = Vec2Dot(br_diff, grad[i][(x_high >> i)*on + (z_low >> i)]);
				//const f32 tl_dot = Vec2Dot(tl_diff, grad[i][(x_low >> i)*on + (z_high >> i)]);
				//const f32 tr_dot = Vec2Dot(tr_diff, grad[i][(x_high >> i)*on + (z_high >> i)]);
				
				const u32 xg_low = x_low >> i;
				const u32 xg_high = xg_low + 1;
				const u32 zg_low = z_low >> i;
				const u32 zg_high = zg_low + 1;
				const f32 bl_dot = Vec2Dot(bl_diff, grad[i][xg_low*on  + zg_low]);
				const f32 br_dot = Vec2Dot(br_diff, grad[i][xg_high*on + zg_low]);
				const f32 tl_dot = Vec2Dot(tl_diff, grad[i][xg_low*on  + zg_high]);
				const f32 tr_dot = Vec2Dot(tr_diff, grad[i][xg_high*on + zg_high]);

				//fprintf(stderr, "(%u, %u), (%u, %u)\n",
				//	(x_low >> i),
				//	(x_high >> i),
				//	(z_low >> i),
				//	(z_high >> i));

				const vec2 low = 
				{
					x_low * unit,
					z_low * unit,
				};

				const vec2 high = 
				{
					x_high * unit,
					z_high * unit,
				};

				const vec2 t = 
				{
					(mesh.v[x*(n-1) + z][0] - low[0]) / (high[0] - low[0]),
					(mesh.v[x*(n-1) + z][2] - low[1]) / (high[1] - low[1]),
				};

				const vec2 smoothstep =
				{
					6*t[0]*t[0]*t[0]*t[0]*t[0] - 15*t[0]*t[0]*t[0]*t[0] + 10*t[0]*t[0]*t[0],
					6*t[1]*t[1]*t[1]*t[1]*t[1] - 15*t[1]*t[1]*t[1]*t[1] + 10*t[1]*t[1]*t[1],
				};

				//if (i == OCTAVES-4)
				//{
				//fprintf(stderr, "(%u, %u) -> (%u, %u), (%u, %u)\n", x, z, x_low, z_low, x_high, z_high);
				////fprintf(stderr, "%f, %f\n", low[0], low[1]);
				//fprintf(stderr, "%f, %f\n", t[0], t[1]);
				////fprintf(stderr, "%f, %f\n", smoothstep[0], smoothstep[1]);
				//}

				const f32 vb = bl_dot*(1.0f - smoothstep[0]) + br_dot*smoothstep[0];
				const f32 vt = tl_dot*(1.0f - smoothstep[0]) + tr_dot*smoothstep[0];
				const f32 perlin = vb*(1.0f - smoothstep[1]) + vt*smoothstep[1];

				mesh.v[x*(n-1) + z][1] += perlin * amplitude;
				amplitude *= 2.0f;
			}

			mesh.v[x*(n-1) + z][0] += offset[0];
			mesh.v[x*(n-1) + z][1] += offset[1];
			mesh.v[x*(n-1) + z][2] += offset[2];
		}
	}

	for (u32 x = 0; x < n-2; ++x)
	{
		for (u32 z = 0; z < n-2; ++z)
		{
			mesh.tri[2*(x*(n-2) + z) + 0][0] = x*(n-1) + z; 
			mesh.tri[2*(x*(n-2) + z) + 0][1] = x*(n-1) + z+1;
			mesh.tri[2*(x*(n-2) + z) + 0][2] = (x+1)*(n-1) + z; 
			mesh.tri[2*(x*(n-2) + z) + 1][0] = (x+1)*(n-1) + z; 
			mesh.tri[2*(x*(n-2) + z) + 1][1] = x*(n-1) + z+1;
			mesh.tri[2*(x*(n-2) + z) + 1][2] = (x+1)*(n-1) + z+1;
		}
	}

	ArenaFree1MB(&tmp);

	struct AABB bbox = tri_mesh_bbox(&mesh);
	vec3 local_origin;
	Vec3Scale(local_origin, bbox.center, -1.0f);
	for (u32 i = 0; i < mesh.v_count; ++i)
	{
		Vec3Translate(mesh.v[i], local_origin);
	}

	return mesh;
}

void led_wall_smash_simulation_setup(struct led *led)
{
	struct system_window *sys_win = system_window_address(g_editor->window);

#define dsphere_v_count 	30
	const u32 dsphere_count = 40;
	const u32 capsule_count = 20;
	const u32 tower1_count = 2;
	const u32 tower2_count = 4;
	const u32 tower1_box_count = 40;
	const u32 tower2_box_count = 10;
	const u32 pyramid_layers = 15;
	const u32 pyramid_count = 3;
	const u32 bodies = tower1_box_count + tower2_box_count + 3 + pyramid_layers*(pyramid_layers+1) / 2;

	/* Setup rigid bodies */
	const f32 box_friction = 0.8f;
	const f32 ramp_friction = 0.1f;
	const f32 sphere_friction = 0.1f;
	const f32 floor_friction = 0.8f;
	const f32 capsule_friction = 0.8f;

	const f32 alpha1 = 0.7f;
	const f32 alpha2 = 0.5f;
	const vec4 dsphere_color = { 244.0f/256.0f, 0.1f, 0.4f, alpha2 };
	const vec4 tower1_color = { 154.0f/256.0f, 101.0f/256.0f, 182.0f/256.0f,alpha1 };
	const vec4 tower2_color = { 54.0f/256.0f, 183.0f/256.0f, 122.0f/256.0f, alpha2 };
	const vec4 pyramid_color = { 254.0f/256.0f, 181.0f/256.0f, 82.0f/256.0f,alpha2 };
	const vec4 floor_color = { 0.8f, 0.6f, 0.6f,                            alpha2 };
	const vec4 ramp_color = { 165.0f/256.0f, 242.0f/256.0f, 243.0f/256.0f,  alpha2 };
	const vec4 sphere_color = { 0.2f, 0.9f, 0.5f,                             alpha1 };
	const vec4 capsule_color = { 0.1f, 0.4f, 0.8f, 				alpha2 };
	const vec4 map_color = { 0.5f, 0.5f, 0.8f, 0.7f };

	const f32 box_side = 1.0f;
	struct AABB box_aabb = { .center = { 0.0f, 0.0f, 0.0f }, .hw = { box_side / 2.0f, box_side / 4.0f, box_side / 2.0f} };

	const f32 ramp_width = 10.0f;
	const f32 ramp_length = 60.0f;
	const f32 ramp_height = 34.0f;
	const u32 v_count = 6;
	vec3 ramp_vertices[6] = 
	{
		{0.0f, 		ramp_height,	-ramp_length},
		{ramp_width, 	ramp_height, 	-ramp_length},
		{0.0f, 		0.0f, 		-ramp_length},
		{ramp_width, 	0.0f, 		-ramp_length},
		{0.0f, 		0.0f, 		0.0f},
		{ramp_width, 	0.0f, 		0.0f},
	};

	vec3 dsphere_vertices[dsphere_v_count];
	const f32 phi = MM_PI_F * (3.0f - f32_sqrt(5.0f));
	for (u32 i = 0; i < dsphere_v_count; ++i)
	{
		const f32 y = 1.0 - i*2.0f/(dsphere_v_count-1);
		Vec3Set(dsphere_vertices[i]
				, f32_cos(i*phi)*f32_sqrt(1 - y*y)
				, y
				, f32_sin(i*phi)*f32_sqrt(1 - y*y));
	}

	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "c_floor");
	sys_win->cmd_queue->regs[1].f32 = 8.0f * ramp_width;
	sys_win->cmd_queue->regs[2].f32 = 0.5f;
	sys_win->cmd_queue->regs[3].f32 = ramp_length;
	cmd_queue_submit(sys_win->cmd_queue, cmd_collision_box_add_id);

	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "c_capsule");
	sys_win->cmd_queue->regs[1].f32 = 0.5f;
	sys_win->cmd_queue->regs[2].f32 = 1.0f;
	cmd_queue_submit(sys_win->cmd_queue, cmd_collision_capsule_add_id);

	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "c_box");
	sys_win->cmd_queue->regs[1].f32 = box_side / 2.0f;
	sys_win->cmd_queue->regs[2].f32 = box_side / 4.0f;
	sys_win->cmd_queue->regs[3].f32 = box_side / 2.0f;
	cmd_queue_submit(sys_win->cmd_queue, cmd_collision_box_add_id);

	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "c_sphere");
	sys_win->cmd_queue->regs[1].f32 = 2.0f;
	cmd_queue_submit(sys_win->cmd_queue, cmd_collision_sphere_add_id);

	struct dcel *c_ramp = ArenaPush(&sys_win->mem_persistent, sizeof(struct dcel));
	*c_ramp = dcel_convex_hull(&sys_win->mem_persistent, ramp_vertices, 6, F32_EPSILON * 100.0f);
	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "c_ramp");
	sys_win->cmd_queue->regs[1].ptr = c_ramp;
	cmd_queue_submit(sys_win->cmd_queue, cmd_collision_dcel_add_id);

	struct dcel *c_dsphere = ArenaPush(&sys_win->mem_persistent, sizeof(struct dcel));
	*c_dsphere = dcel_convex_hull(&sys_win->mem_persistent, dsphere_vertices, dsphere_v_count, F32_EPSILON * 100.0f);
	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "c_dsphere");
	sys_win->cmd_queue->regs[1].ptr = c_dsphere;
	cmd_queue_submit(sys_win->cmd_queue, cmd_collision_dcel_add_id);

	struct tri_mesh *map = ArenaPush(&led->mem_persistent, sizeof(struct tri_mesh));
	*map = tri_mesh_perlin_noise(&led->mem_persistent, 64, 100.0f);
	struct tri_mesh_bvh *mesh_bvh = ArenaPush(&led->mem_persistent, sizeof(struct tri_mesh_bvh));
	f32 best_cost = F32_INFINITY;
	u32 best_bin_count = 8;
	for (u32 i = 8; i < 16; ++i)
	{
		ArenaPushRecord(&led->mem_persistent);
		*mesh_bvh = tri_mesh_bvh_construct(&led->mem_persistent, map, i);
		const f32 cost = bvh_cost(&mesh_bvh->bvh);
		if (cost < best_cost)
		{
			best_cost = cost;
			best_bin_count = i;
		}
		ArenaPopRecord(&led->mem_persistent);
	}
	*mesh_bvh = tri_mesh_bvh_construct(&led->mem_persistent, map, best_bin_count);
	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "c_map");
	sys_win->cmd_queue->regs[1].ptr = mesh_bvh;
	cmd_queue_submit(sys_win->cmd_queue, cmd_collision_tri_mesh_bvh_add_id);

	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "rb_map");
	sys_win->cmd_queue->regs[1].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "c_map");
	sys_win->cmd_queue->regs[2].f32 = 1.0f;
	sys_win->cmd_queue->regs[3].f32 = 0.0f;
	sys_win->cmd_queue->regs[4].f32 = floor_friction;
	sys_win->cmd_queue->regs[5].u64 = 0;
	cmd_queue_submit(sys_win->cmd_queue, cmd_rb_prefab_add_id);

	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "rb_floor");
	sys_win->cmd_queue->regs[1].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "c_floor");
	sys_win->cmd_queue->regs[2].f32 = 1.0f;
	sys_win->cmd_queue->regs[3].f32 = 0.0f;
	sys_win->cmd_queue->regs[4].f32 = floor_friction;
	sys_win->cmd_queue->regs[5].u32 = 0;
	cmd_queue_submit(sys_win->cmd_queue, cmd_rb_prefab_add_id);

	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "rb_box");
	sys_win->cmd_queue->regs[1].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "c_box");
	sys_win->cmd_queue->regs[2].f32 = 1.0f;
	sys_win->cmd_queue->regs[3].f32 = 0.0f;
	sys_win->cmd_queue->regs[4].f32 = box_friction;
	sys_win->cmd_queue->regs[5].u32 = 1;
	cmd_queue_submit(sys_win->cmd_queue, cmd_rb_prefab_add_id);

	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "rb_capsule");
	sys_win->cmd_queue->regs[1].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "c_capsule");
	sys_win->cmd_queue->regs[2].f32 = 1.0f;
	sys_win->cmd_queue->regs[3].f32 = 0.0f;
	sys_win->cmd_queue->regs[4].f32 = capsule_friction;
	sys_win->cmd_queue->regs[5].u32 = 1;
	cmd_queue_submit(sys_win->cmd_queue, cmd_rb_prefab_add_id);

	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "rb_sphere");
	sys_win->cmd_queue->regs[1].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "c_sphere");
	sys_win->cmd_queue->regs[2].f32 = 100.0f;
	sys_win->cmd_queue->regs[3].f32 = 0.0f;
	sys_win->cmd_queue->regs[4].f32 = sphere_friction;
	sys_win->cmd_queue->regs[5].u32 = 1;
	cmd_queue_submit(sys_win->cmd_queue, cmd_rb_prefab_add_id);

	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "rb_dsphere");
	sys_win->cmd_queue->regs[1].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "c_dsphere");
	sys_win->cmd_queue->regs[2].f32 = 1.0f;
	sys_win->cmd_queue->regs[3].f32 = 0.0f;
	sys_win->cmd_queue->regs[4].f32 = sphere_friction;
	sys_win->cmd_queue->regs[5].u32 = 1;
	cmd_queue_submit(sys_win->cmd_queue, cmd_rb_prefab_add_id);

	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "rb_ramp");
	sys_win->cmd_queue->regs[1].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "c_ramp");
	sys_win->cmd_queue->regs[2].f32 = 1.0f;
	sys_win->cmd_queue->regs[3].f32 = 0.0f;
	sys_win->cmd_queue->regs[4].f32 = ramp_friction;
	sys_win->cmd_queue->regs[5].u32 = 0;
	cmd_queue_submit(sys_win->cmd_queue, cmd_rb_prefab_add_id);

	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "rm_map");
	sys_win->cmd_queue->regs[1].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "c_map");
	cmd_queue_submit(sys_win->cmd_queue, cmd_render_mesh_add_id);

	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "rm_floor");
	sys_win->cmd_queue->regs[1].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "c_floor");
	cmd_queue_submit(sys_win->cmd_queue, cmd_render_mesh_add_id);

	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "rm_ramp");
	sys_win->cmd_queue->regs[1].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "c_ramp");
	cmd_queue_submit(sys_win->cmd_queue, cmd_render_mesh_add_id);
	
	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "rm_capsule");
	sys_win->cmd_queue->regs[1].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "c_capsule");
	cmd_queue_submit(sys_win->cmd_queue, cmd_render_mesh_add_id);

	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "rm_tower1");
	sys_win->cmd_queue->regs[1].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "c_box");
	cmd_queue_submit(sys_win->cmd_queue, cmd_render_mesh_add_id);
	
	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "rm_tower2");
	sys_win->cmd_queue->regs[1].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "c_box");
	cmd_queue_submit(sys_win->cmd_queue, cmd_render_mesh_add_id);

	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "rm_box");
	sys_win->cmd_queue->regs[1].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "c_box");
	cmd_queue_submit(sys_win->cmd_queue, cmd_render_mesh_add_id);

	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "rm_sphere");
	sys_win->cmd_queue->regs[1].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "c_sphere");
	cmd_queue_submit(sys_win->cmd_queue, cmd_render_mesh_add_id);

	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "rm_dsphere");
	sys_win->cmd_queue->regs[1].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "c_dsphere");
	cmd_queue_submit(sys_win->cmd_queue, cmd_render_mesh_add_id);

	vec3 sphere_translation = { -0.5, 0.5f + ramp_height, -ramp_length };
	vec3 box_translation =  {-0.5f, 0.0f, -0.5f};
	vec3 ramp_translation = {0.0f , ramp_width, -ramp_length};
	vec3 floor_translation = { 0.0f, -ramp_width/2.0f - 1.0f, ramp_length / 2.0f -ramp_width/2.0f};
	vec3 box_base_translation = { 0.0f, floor_translation[1] + 1.0f, floor_translation[2] / 2.0f};
	vec3 dsphere_base_translation = { -15.0f, floor_translation[1] + 1.0f, floor_translation[2] / 2.0f + 20.0f};

	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "led_floor");
	cmd_queue_submit(sys_win->cmd_queue, cmd_led_node_add_id);
	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "led_floor");
	sys_win->cmd_queue->regs[1].f32 = floor_translation[0];
	sys_win->cmd_queue->regs[2].f32 = floor_translation[1];
	sys_win->cmd_queue->regs[3].f32 = floor_translation[2];
	cmd_queue_submit(sys_win->cmd_queue, cmd_led_node_set_position_id);
	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "led_floor");
	sys_win->cmd_queue->regs[1].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "rb_floor");
	cmd_queue_submit(sys_win->cmd_queue, cmd_led_node_set_rb_prefab_id);
	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "led_floor");
	sys_win->cmd_queue->regs[1].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "rm_floor");
	sys_win->cmd_queue->regs[2].f32 = floor_color[0];
	sys_win->cmd_queue->regs[3].f32 = floor_color[1];
	sys_win->cmd_queue->regs[4].f32 = floor_color[2];
	sys_win->cmd_queue->regs[5].f32 = floor_color[3];
	sys_win->cmd_queue->regs[6].f32 = 1.0f;
	cmd_queue_submit(sys_win->cmd_queue, cmd_led_node_set_proxy3d_id);
	
	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "led_map");
	cmd_queue_submit(sys_win->cmd_queue, cmd_led_node_add_id);
	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "led_map");
	sys_win->cmd_queue->regs[1].f32 = 0.0f;
	sys_win->cmd_queue->regs[2].f32 = -20.0f;
	sys_win->cmd_queue->regs[3].f32 = 0.0f;
	cmd_queue_submit(sys_win->cmd_queue, cmd_led_node_set_position_id);
	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "led_map");
	sys_win->cmd_queue->regs[1].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "rb_map");
	cmd_queue_submit(sys_win->cmd_queue, cmd_led_node_set_rb_prefab_id);
	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "led_map");
	sys_win->cmd_queue->regs[1].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "rm_map");
	sys_win->cmd_queue->regs[2].f32 = map_color[0];
	sys_win->cmd_queue->regs[3].f32 = map_color[1];
	sys_win->cmd_queue->regs[4].f32 = map_color[2];
	sys_win->cmd_queue->regs[5].f32 = map_color[3];
	sys_win->cmd_queue->regs[6].f32 = 1.0f;
	cmd_queue_submit(sys_win->cmd_queue, cmd_led_node_set_proxy3d_id);

	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "led_ramp");
	cmd_queue_submit(sys_win->cmd_queue, cmd_led_node_add_id);
	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "led_ramp");
	sys_win->cmd_queue->regs[1].f32 = ramp_translation[0];
	sys_win->cmd_queue->regs[2].f32 = ramp_translation[1];
	sys_win->cmd_queue->regs[3].f32 = ramp_translation[2];
	cmd_queue_submit(sys_win->cmd_queue, cmd_led_node_set_position_id);
	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "led_ramp");
	sys_win->cmd_queue->regs[1].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "rb_ramp");
	cmd_queue_submit(sys_win->cmd_queue, cmd_led_node_set_rb_prefab_id);
	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "led_ramp");
	sys_win->cmd_queue->regs[1].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "rm_ramp");
	sys_win->cmd_queue->regs[2].f32 = ramp_color[0];
	sys_win->cmd_queue->regs[3].f32 = ramp_color[1];
	sys_win->cmd_queue->regs[4].f32 = ramp_color[2];
	sys_win->cmd_queue->regs[5].f32 = ramp_color[3];
	sys_win->cmd_queue->regs[6].f32 = 1.0f;
	cmd_queue_submit(sys_win->cmd_queue, cmd_led_node_set_proxy3d_id);

	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "led_sphere");
	cmd_queue_submit(sys_win->cmd_queue, cmd_led_node_add_id);
	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "led_sphere");
	sys_win->cmd_queue->regs[1].f32 = sphere_translation[0];
	sys_win->cmd_queue->regs[2].f32 = sphere_translation[1];
	sys_win->cmd_queue->regs[3].f32 = sphere_translation[2];
	cmd_queue_submit(sys_win->cmd_queue, cmd_led_node_set_position_id);
	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "led_sphere");
	sys_win->cmd_queue->regs[1].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "rb_sphere");
	cmd_queue_submit(sys_win->cmd_queue, cmd_led_node_set_rb_prefab_id);
	sys_win->cmd_queue->regs[0].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "led_sphere");
	sys_win->cmd_queue->regs[1].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "rm_sphere");
	sys_win->cmd_queue->regs[2].f32 = sphere_color[0];
	sys_win->cmd_queue->regs[3].f32 = sphere_color[1];
	sys_win->cmd_queue->regs[4].f32 = sphere_color[2];
	sys_win->cmd_queue->regs[5].f32 = sphere_color[3];
	sys_win->cmd_queue->regs[6].f32 = 1.0f;
	cmd_queue_submit(sys_win->cmd_queue, cmd_led_node_set_proxy3d_id);
	
	for (u32 i = 0; i < capsule_count; ++i)
	{	
		vec3 translation;
		Vec3Copy(translation, dsphere_base_translation);
		translation[0] += (10.0f - 38.0f * (f32) i / dsphere_count) * f32_cos(i * MM_PI_F*37.0f/197.0f);
		translation[1] += 25.0f + (f32) i / 2.0f;
		translation[2] += (10.0f - 38.0f * (f32) i / dsphere_count) * f32_sin(i * MM_PI_F*37.0f/197.0f);

		utf8 id = Utf8Format(sys_win->ui->mem_frame, "capsule_%u", i);
		sys_win->cmd_queue->regs[0].utf8 = id;
		cmd_queue_submit(sys_win->cmd_queue, cmd_led_node_add_id);
		sys_win->cmd_queue->regs[0].utf8 = id;
		sys_win->cmd_queue->regs[1].f32 = translation[0];
		sys_win->cmd_queue->regs[2].f32 = translation[1];
		sys_win->cmd_queue->regs[3].f32 = translation[2];
		cmd_queue_submit(sys_win->cmd_queue, cmd_led_node_set_position_id);
		sys_win->cmd_queue->regs[0].utf8 = id;
		sys_win->cmd_queue->regs[1].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "rb_capsule");
		cmd_queue_submit(sys_win->cmd_queue, cmd_led_node_set_rb_prefab_id);
		sys_win->cmd_queue->regs[0].utf8 = id;
		sys_win->cmd_queue->regs[1].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "rm_capsule");
		sys_win->cmd_queue->regs[2].f32 = capsule_color[0];
		sys_win->cmd_queue->regs[3].f32 = capsule_color[1];
		sys_win->cmd_queue->regs[4].f32 = capsule_color[2];
		sys_win->cmd_queue->regs[5].f32 = capsule_color[3];
		sys_win->cmd_queue->regs[6].f32 = 1.0f;
		cmd_queue_submit(sys_win->cmd_queue, cmd_led_node_set_proxy3d_id);
	}

	for (u32 i = 0; i < dsphere_count; ++i)
	{	
		vec3 translation;
		Vec3Copy(translation, dsphere_base_translation);
		translation[0] += (10.0f - 38.0f * (f32) i / dsphere_count) * f32_cos(i * MM_PI_F*37.0f/197.0f);
		translation[1] += 5.0f + (f32) i / 2.0f;
		translation[2] += (10.0f - 38.0f * (f32) i / dsphere_count) * f32_sin(i * MM_PI_F*37.0f/197.0f);

		utf8 id = Utf8Format(sys_win->ui->mem_frame, "dsphere_%u", i);
		sys_win->cmd_queue->regs[0].utf8 = id;
		cmd_queue_submit(sys_win->cmd_queue, cmd_led_node_add_id);
		sys_win->cmd_queue->regs[0].utf8 = id;
		sys_win->cmd_queue->regs[1].f32 = translation[0];
		sys_win->cmd_queue->regs[2].f32 = translation[1];
		sys_win->cmd_queue->regs[3].f32 = translation[2];
		cmd_queue_submit(sys_win->cmd_queue, cmd_led_node_set_position_id);
		sys_win->cmd_queue->regs[0].utf8 = id;
		sys_win->cmd_queue->regs[1].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "rb_dsphere");
		cmd_queue_submit(sys_win->cmd_queue, cmd_led_node_set_rb_prefab_id);
		sys_win->cmd_queue->regs[0].utf8 = id;
		sys_win->cmd_queue->regs[1].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "rm_dsphere");
		sys_win->cmd_queue->regs[2].f32 = dsphere_color[0];
		sys_win->cmd_queue->regs[3].f32 = dsphere_color[1];
		sys_win->cmd_queue->regs[4].f32 = dsphere_color[2];
		sys_win->cmd_queue->regs[5].f32 = dsphere_color[3];
		sys_win->cmd_queue->regs[6].f32 = 1.0f;
		cmd_queue_submit(sys_win->cmd_queue, cmd_led_node_set_proxy3d_id);
	}

	for (u32 k = 0; k < pyramid_count; ++k)
	{
		for (u32 i = 0; i < pyramid_layers; ++i)
		{
			const f32 local_y = i * box_side;
			for (u32 j = 0; j < pyramid_layers-i; ++j)
			{
				const f32 local_x = j -(pyramid_layers-i-1) * box_side / 2.0f;
				vec3 translation;
				Vec3Copy(translation, box_base_translation);
				translation[0] += local_x;
				translation[1] += local_y;
				translation[2] += 10.0f * k;

				utf8 id = Utf8Format(sys_win->ui->mem_frame, "pyramid_%u_%u_%u", i, j, k);
				sys_win->cmd_queue->regs[0].utf8 = id;
				cmd_queue_submit(sys_win->cmd_queue, cmd_led_node_add_id);
				sys_win->cmd_queue->regs[0].utf8 = id;
				sys_win->cmd_queue->regs[1].f32 = translation[0];
				sys_win->cmd_queue->regs[2].f32 = translation[1];
				sys_win->cmd_queue->regs[3].f32 = translation[2];
				cmd_queue_submit(sys_win->cmd_queue, cmd_led_node_set_position_id);
				sys_win->cmd_queue->regs[0].utf8 = id;
				sys_win->cmd_queue->regs[1].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "rb_box");
				cmd_queue_submit(sys_win->cmd_queue, cmd_led_node_set_rb_prefab_id);
				sys_win->cmd_queue->regs[0].utf8 = id;
				sys_win->cmd_queue->regs[1].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "rm_box");
				sys_win->cmd_queue->regs[2].f32 = pyramid_color[0];
				sys_win->cmd_queue->regs[3].f32 = pyramid_color[1];
				sys_win->cmd_queue->regs[4].f32 = pyramid_color[2];
				sys_win->cmd_queue->regs[5].f32 = pyramid_color[3];
				sys_win->cmd_queue->regs[6].f32 = 1.0f;
				cmd_queue_submit(sys_win->cmd_queue, cmd_led_node_set_proxy3d_id);
			}
		}
	}

	for (u32 k = 0; k < tower1_count; ++k)
	{
		for (u32 j = 0; j < tower1_count; ++j)
		{
			for (u32 i = 0; i < tower1_box_count; ++i)
			{
				vec3 translation;
				Vec3Copy(translation, box_base_translation);
				translation[2] += 15.0f + 2.0f*k;
				translation[1] += (f32) i * box_aabb.hw[1] * 2.10f;
				translation[0] += 15.0f + 2.0f*j;

				utf8 id = Utf8Format(sys_win->ui->mem_frame, "tower1_%u_%u_%u", i, j, k);
				sys_win->cmd_queue->regs[0].utf8 = id;
				cmd_queue_submit(sys_win->cmd_queue, cmd_led_node_add_id);
				sys_win->cmd_queue->regs[0].utf8 = id;
				sys_win->cmd_queue->regs[1].f32 = translation[0];
				sys_win->cmd_queue->regs[2].f32 = translation[1];
				sys_win->cmd_queue->regs[3].f32 = translation[2];
				cmd_queue_submit(sys_win->cmd_queue, cmd_led_node_set_position_id);
				sys_win->cmd_queue->regs[0].utf8 = id;
				sys_win->cmd_queue->regs[1].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "rb_box");
				cmd_queue_submit(sys_win->cmd_queue, cmd_led_node_set_rb_prefab_id);
				sys_win->cmd_queue->regs[0].utf8 = id;
				sys_win->cmd_queue->regs[1].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "rm_box");
				sys_win->cmd_queue->regs[2].f32 = tower1_color[0];
				sys_win->cmd_queue->regs[3].f32 = tower1_color[1];
				sys_win->cmd_queue->regs[4].f32 = tower1_color[2];
				sys_win->cmd_queue->regs[5].f32 = tower1_color[3];
				sys_win->cmd_queue->regs[6].f32 = 1.0f;
				cmd_queue_submit(sys_win->cmd_queue, cmd_led_node_set_proxy3d_id);
			}
		}
	}

	for (u32 k = 0; k < tower2_count; ++k)
	{
		for (u32 j = 0; j < tower2_count; ++j)
		{
			for (u32 i = 0; i < tower2_box_count; ++i)
			{
				vec3 translation;
				Vec3Copy(translation, box_base_translation);
				translation[2] += 15.0f + 2.0f*k;
				translation[1] += (f32) i * box_aabb.hw[1] * 2.10f;
				translation[0] -= 15.0f + 2.0f*j;
			
				utf8 id = Utf8Format(sys_win->ui->mem_frame, "tower2_%u_%u_%u", i, j, k);
				sys_win->cmd_queue->regs[0].utf8 = id;
				cmd_queue_submit(sys_win->cmd_queue, cmd_led_node_add_id);
				sys_win->cmd_queue->regs[0].utf8 = id;
				sys_win->cmd_queue->regs[1].f32 = translation[0];
				sys_win->cmd_queue->regs[2].f32 = translation[1];
				sys_win->cmd_queue->regs[3].f32 = translation[2];
				cmd_queue_submit(sys_win->cmd_queue, cmd_led_node_set_position_id);
				sys_win->cmd_queue->regs[0].utf8 = id;
				sys_win->cmd_queue->regs[1].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "rb_box");
				cmd_queue_submit(sys_win->cmd_queue, cmd_led_node_set_rb_prefab_id);
				sys_win->cmd_queue->regs[0].utf8 = id;
				sys_win->cmd_queue->regs[1].utf8 = Utf8Cstr(sys_win->ui->mem_frame, "rm_box");
				sys_win->cmd_queue->regs[2].f32 = tower2_color[0];
				sys_win->cmd_queue->regs[3].f32 = tower2_color[1];
				sys_win->cmd_queue->regs[4].f32 = tower2_color[2];
				sys_win->cmd_queue->regs[5].f32 = tower2_color[3];
				sys_win->cmd_queue->regs[6].f32 = 1.0f;
				cmd_queue_submit(sys_win->cmd_queue, cmd_led_node_set_proxy3d_id);
			}
		}
	}
}

void cmd_led_compile(void)
{
	return led_compile(g_editor);
}

void cmd_led_run(void)
{
	return led_run(g_editor);
}

void cmd_led_pause(void)
{
	return led_pause(g_editor);
}

void cmd_led_stop(void)
{
	return led_stop(g_editor);
}

void led_compile(struct led *led)
{
}

void led_run(struct led *led)
{
	led->pending_engine_initalized = 1;
	led->pending_engine_running = 1;
	led->pending_engine_paused = 0;
}

void led_pause(struct led *led)
{
	led->pending_engine_paused = 1;
	led->pending_engine_running = 0;
}

void led_stop(struct led *led)
{
	led->pending_engine_initalized = 0;
	led->pending_engine_running = 0;
	led->pending_engine_paused = 0;
}

static void led_engine_flush(struct led *led)
{
	physics_pipeline_flush(&led->physics);
	struct led_node *node = NULL;
	for (u32 i = led->node_non_marked_list.first; i != DLL_NULL; i = dll_Next(node))
	{
		node = PoolAddress(&led->node_pool, i);
		r_proxy3d_set_linear_speculation(node->position
					, node->rotation
					, (vec3) { 0 } 
					, (vec3) { 0 } 
					, led->ns
					, node->proxy);
		struct r_proxy3d *proxy = r_proxy3d_address(node->proxy);
		Vec4Copy(proxy->color, node->color);
	}
}

static void led_engine_init(struct led *led)
{
	//TODO move this into engine flush
	physics_pipeline_flush(&led->physics);		
	led->physics.ns_start = led->ns;
	led->physics.ns_elapsed = -led->ns_delta;
	led->ns_engine_paused = 0;

	struct led_node *node = NULL;
	for (u32 i = led->node_non_marked_list.first; i != DLL_NULL; i = dll_Next(node))
	{
		node = GPoolAddress(&led->node_pool, i);
		if (node->flags & LED_PHYSICS)
		{
			struct rigid_body_prefab *prefab = strdb_Address(&led->rb_prefab_db, node->rb_prefab);
			if (Utf8Equivalence(prefab->id, Utf8Inline("rb_map")))
			{
				vec3 axis = { 0.6f, 1.0f, 0.6f };
				Vec3ScaleSelf(axis, 1.0f / f32_sqrt(Vec3Length(axis)));
				const f32 angle = MM_PI_F / 16.0f;
				QuatAxisAngle(node->rotation, axis, angle);
				vec3 linear_velocity = { 0.0f, 0.0f, 0.0f};
				vec3 angular_velocity = { 0.0f, 0.0f, 0.0f};
				r_proxy3d_set_linear_speculation(node->position
						, node->rotation
						, linear_velocity
						, angular_velocity
						, led->ns 
						, node->proxy);
			}
			physics_pipeline_rigid_body_alloc(&led->physics, prefab, node->position, node->rotation, i);
		}
	}
}

static void led_engine_color_bodies(struct led *led, const u32 island, const vec4 color)
{
	struct island *is = array_list_address(led->physics.is_db.islands, island);
	struct is_index_entry *entry = NULL;
	u32 k = is->body_first;
	for (u32 i = 0; i < is->body_count; ++i)
	{
		entry = array_list_address(led->physics.is_db.island_body_lists, k);
		k = entry->next;

		const struct rigid_body *body = PoolAddress(&led->physics.body_pool, entry->index);
		const struct led_node *node = PoolAddress(&led->node_pool, body->entity);
		struct r_proxy3d *proxy = r_proxy3d_address(node->proxy);
		Vec4Copy(proxy->color, color);
	}
}

static void led_engine_run(struct led *led)
{
	led->physics.ns_elapsed += led->ns_delta;

	//const u64 game_frames_to_run = (game->ns_elapsed - (game->frames_completed * game->ns_tick)) / game->ns_tick;
	const u64 physics_frames_to_run = (led->physics.ns_elapsed - (led->physics.frames_completed * led->physics.ns_tick)) / led->physics.ns_tick;

	//u64 ns_next_game_frame = game->frames_completed * game->ns_tick;
	u64 ns_next_physics_frame = (led->physics.frames_completed+1) * led->physics.ns_tick;

	for (u64 i = 0; i < /* game_frames_to_run + */ physics_frames_to_run; ++i)
	{
		//if (ns_next_game_frame <= ns_next_physics_frame)
		//{
		//	ArenaFlush(&game->frame);
		//	game_tick(game);
		//	game->frames_completed += 1;
		//	ns_next_game_frame += game->ns_tick;
		//}
		//else
		//{
			physics_pipeline_tick(&led->physics);
			ns_next_physics_frame += led->physics.ns_tick;
		//}
	}

	if (led->physics.pending_body_color_mode != led->physics.body_color_mode)
	{
		switch (led->physics.pending_body_color_mode)
		{
			case RB_COLOR_MODE_BODY: 
			{ 
				const struct rigid_body *body = NULL;
				for (u32 i = led->physics.body_non_marked_list.first; i != DLL_NULL; i = dll_Next(body))
				{
					body = PoolAddress(&led->physics.body_pool, i);
					const struct led_node *node = PoolAddress(&led->node_pool, body->entity);
					struct r_proxy3d *proxy = r_proxy3d_address(node->proxy);
					Vec4Copy(proxy->color, node->color);
				}
			} break;

			case RB_COLOR_MODE_COLLISION: 
			{ 
				const struct rigid_body *body = NULL;
				for (u32 i = led->physics.body_non_marked_list.first; i != DLL_NULL; i = dll_Next(body))
				{
					body = PoolAddress(&led->physics.body_pool, i);
					const struct led_node *node = PoolAddress(&led->node_pool, body->entity);
					struct r_proxy3d *proxy = r_proxy3d_address(node->proxy);
					if (RB_IS_DYNAMIC(body))
					{
						(body->first_contact_index == NLL_NULL)
							? Vec4Copy(proxy->color, node->color)
							: Vec4Copy(proxy->color, led->physics.collision_color);
					}
					else
					{
						Vec4Copy(proxy->color, led->physics.static_color);
					}
				}
			} break;

			case RB_COLOR_MODE_SLEEP: 
			{ 
				const struct rigid_body *body = NULL;
				for (u32 i = led->physics.body_non_marked_list.first; i != DLL_NULL; i = dll_Next(body))
				{
					body = PoolAddress(&led->physics.body_pool, i);
					const struct led_node *node = PoolAddress(&led->node_pool, body->entity);
					struct r_proxy3d *proxy = r_proxy3d_address(node->proxy);

					if (!RB_IS_DYNAMIC(body))
					{						
						Vec4Copy(proxy->color, led->physics.static_color);
					}
					else
					{
						(RB_IS_AWAKE(body))
							? Vec4Copy(proxy->color, led->physics.awake_color)
							: Vec4Copy(proxy->color, led->physics.sleep_color);
					}
				}
			} break;

			case RB_COLOR_MODE_ISLAND: 
			{ 
				const struct rigid_body *body = NULL;
				for (u32 i = led->physics.body_non_marked_list.first; i != DLL_NULL; i = dll_Next(body))
				{
					body = PoolAddress(&led->physics.body_pool, i);
					const struct led_node *node = PoolAddress(&led->node_pool, body->entity);
					const struct island *is = array_list_address(led->physics.is_db.islands, body->island_index);
					struct r_proxy3d *proxy = r_proxy3d_address(node->proxy);
					(RB_IS_DYNAMIC(body))
						? Vec4Copy(proxy->color, is->color)
						: Vec4Copy(proxy->color, led->physics.static_color);
				}
			} break;
		}
	}
	led->physics.body_color_mode = led->physics.pending_body_color_mode;

	struct physics_event *event = NULL;
	for (u32 i = led->physics.event_list.first; i != DLL_NULL; )
	{
		event = PoolAddress(&led->physics.event_pool, i);
		const u32 next = dll_Next(event);
		switch (event->type)
		{
			case PHYSICS_EVENT_CONTACT_NEW:
			{
				if (led->physics.body_color_mode == RB_COLOR_MODE_COLLISION)
				{
					const struct rigid_body *body1 = PoolAddress(&led->physics.body_pool, event->contact_bodies.body1);
					const struct rigid_body *body2 = PoolAddress(&led->physics.body_pool, event->contact_bodies.body2);
					const struct led_node *node1 = PoolAddress(&led->node_pool, body1->entity);
					const struct led_node *node2 = PoolAddress(&led->node_pool, body2->entity);
					if (RB_IS_DYNAMIC(body1))
					{
						struct r_proxy3d *proxy = r_proxy3d_address(node1->proxy);
						Vec4Copy(proxy->color, led->physics.collision_color);
					}
					if (RB_IS_DYNAMIC(body2))
					{
						struct r_proxy3d *proxy = r_proxy3d_address(node2->proxy);
						Vec4Copy(proxy->color, led->physics.collision_color);
					}
				}
			} break;

			case PHYSICS_EVENT_CONTACT_REMOVED:
			{
				if (led->physics.body_color_mode == RB_COLOR_MODE_COLLISION)
				{
					const struct rigid_body *body1 = PoolAddress(&led->physics.body_pool, event->contact_bodies.body1);
					const struct rigid_body *body2 = PoolAddress(&led->physics.body_pool, event->contact_bodies.body2);
					const struct led_node *node1 = PoolAddress(&led->node_pool, body1->entity);
					const struct led_node *node2 = PoolAddress(&led->node_pool, body2->entity);

					struct r_proxy3d *proxy1 = r_proxy3d_address(node1->proxy);
					struct r_proxy3d *proxy2 = r_proxy3d_address(node2->proxy);
					if (RB_IS_DYNAMIC(body1))
					{
						if (body1->first_contact_index == NLL_NULL)
						{
							Vec4Copy(proxy1->color, node1->color);
						}
					}
					else
					{
						Vec4Copy(proxy1->color, led->physics.static_color);
					}
		
					if (RB_IS_DYNAMIC(body2))
					{
						if (body2->first_contact_index == NLL_NULL)
						{
							Vec4Copy(proxy2->color, node2->color);
						}
					}
					else
					{
						Vec4Copy(proxy2->color, led->physics.static_color);
					}
				}
			} break;

#ifdef DS_PHYSICS_DEBUG
			case PHYSICS_EVENT_ISLAND_NEW:
			{
				struct island *is = array_list_address(led->physics.is_db.islands, event->island);
				Vec4Set(is->color, 
						rng_f32_normalized(), 
						rng_f32_normalized(), 
						rng_f32_normalized(), 
						0.7f);
				if (led->physics.body_color_mode == RB_COLOR_MODE_ISLAND)
				{
					led_engine_color_bodies(led, event->island, is->color);
				}
				else if (led->physics.body_color_mode == RB_COLOR_MODE_SLEEP)
				{
					led_engine_color_bodies(led, event->island, led->physics.awake_color);
				}
			} break;

			case PHYSICS_EVENT_ISLAND_EXPANDED:
			{
				if (led->physics.body_color_mode == RB_COLOR_MODE_ISLAND)
				{
					if (BitVecGetBit(&led->physics.is_db.island_usage, event->island))
					{
						const struct island *is = array_list_address(led->physics.is_db.islands, event->island);
						led_engine_color_bodies(led, event->island, is->color);
					}
				}
			} break;
#endif

			case PHYSICS_EVENT_ISLAND_REMOVED:
			{
			} break;

			case PHYSICS_EVENT_ISLAND_AWAKE:
			{
				if (led->physics.body_color_mode == RB_COLOR_MODE_SLEEP)
				{
					led_engine_color_bodies(led, event->island, led->physics.awake_color);
				}
			} break;

			case PHYSICS_EVENT_ISLAND_ASLEEP:
			{
				if (led->physics.body_color_mode == RB_COLOR_MODE_SLEEP)
				{
					led_engine_color_bodies(led, event->island, led->physics.sleep_color);
				}
			} break;
			
			case PHYSICS_EVENT_BODY_NEW:
			{
			} break;

			case PHYSICS_EVENT_BODY_REMOVED:
			{
			} break;

			case PHYSICS_EVENT_BODY_ORIENTATION:
			{
				const struct rigid_body *body = PoolAddress(&led->physics.body_pool, event->body);
				struct led_node *node = PoolAddress(&led->node_pool, body->entity);

				vec3 linear_velocity;
				Vec3Scale(linear_velocity, body->linear_momentum, 1.0f / body->mass);
				r_proxy3d_set_linear_speculation(body->position
						, body->rotation
						, linear_velocity
						, body->angular_velocity
						, event->ns
						, node->proxy);
			} break;
		}
		PoolRemove(&led->physics.event_pool, i);
		i = next;
	}
	
	dll_Flush(&led->physics.event_list);
}

void led_core(struct led *led)
{
	static u32 once = 1;
	struct system_window *sys_win = system_window_address(g_editor->window);
	if (once && sys_win)
	{
		once = 0;
		led_wall_smash_simulation_setup(led);
	}
	led_remove_marked_structs(led);

	if (led->engine_initalized && !led->pending_engine_initalized)
	{
		led_engine_flush(led);
	}

	if (!led->engine_initalized && led->pending_engine_initalized)
	{
		led_engine_init(led);
	}
	led->engine_initalized = led->pending_engine_initalized;
	led->engine_running = led->pending_engine_running;
	led->engine_paused = led->pending_engine_paused;
	if (led->engine_running)
	{
		led->ns_engine_running += led->ns_delta;
		led_engine_run(led);
	}

	if (led->engine_paused)
	{
		led->ns_engine_paused += led->ns_delta;
	}
}
