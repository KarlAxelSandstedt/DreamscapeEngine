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

#ifndef __LED_LOCAL_H__
#define __LED_LOCAL_H__

#include "led_public.h"
#include "r_public.h"
#include "ui_public.h"

#define LED_ROOT_FOLDER_PATH	"../asset_components/led_projects"

/*******************************************/
/*                 led_visual.c            */
/*******************************************/

struct led_Visual
{
	//TODO seperate cameras for seperate modes 
	struct r_Camera	cam;	

	/* general visual aspects */

	vec4		unit_grid_color;
	f32		unit_grid_equidistance;
	u32		unit_grid_lines_per_axis;
	u32		unit_grid_draw;			/* Boolean */
	u32		unit_r_handle;

	u32		axes_draw;
	u32		axes_r_handle;

	vec4		border_color;
	vec4		background_color;
	vec4		background_highlight_color;
	vec4		background_invalid_color;
	vec4		text_color;
	i32		border_size;
	f32		edge_softness;
	f32		corner_radius;
};

extern struct led_Visual *	g_visual;

/* init led visual defaults */
void 	led_VisualInitDefaults(const u32 window);


/*******************************************/
/*                 led_init.c              */
/*******************************************/

/* Allocate initial project menu resources */
struct led_ProjectMenu	led_ProjectMenuAlloc(void);
/* release project menu resources */
void			led_ProjectMenuDealloc(struct led_ProjectMenu *menu);

/*******************************************/
/*                 led_Main.c              */
/*******************************************/

/* main entrypoint */
void			led_ProjectMenuMain(struct led *led);

/*******************************************/
/*                 led_ui.c                */
/*******************************************/

/*******************************************/
/*                 led_Core.c              */
/*******************************************/

/* TODO: tmp, until we implenent cvars or something */
extern struct led *g_editor;

/* initate global commands and identifers */
void 		led_CoreInitCommands(void);
/* run level editor systems */
void 		led_Core(struct led *led);

/* compile level editor map */
void		led_Compile(struct led *led);
/* run level editor map, and  */
void		led_Run(struct led *led);
/* pause running level editor map */
void		led_Pause(struct led *led);
/* stop running level editor map */
void		led_Stop(struct led *led);

/* Allocate node with the given id. Returns (NULL, U32_MAX) if id.size > 256B or id.len == 0 */
struct slot 	led_NodeAdd(struct led *led, const utf8 id);
/* Mark node for remval if it exist; otherwise no-op.  */
void 		led_NodeRemove(struct led *led, const utf8 id);
/* Return node with the given id if it exist; otherwise return (NULL, U32_MAX).  */
struct slot 	led_NodeLookup(struct led *led, const utf8 id);
/* Set node position if it exist. */
void		led_NodeSetPosition(struct led *led, const utf8 id, const vec3 position);
/* Set node to be a physics instance if the node and the prefab exist */
void		led_NodeSetRigidBodyPrefab(struct led *led, const utf8 id, const utf8 prefab);
/* Set node to be a csg instance if the node and the csg brush exist */
void		led_NodeSetCsgBrush(struct led *led, const utf8 id, const utf8 brush);
/* Set node's render mesh if the node and mesh exists */
void		led_NodeSetProxy3d(struct led *led, const utf8 id, const utf8 mesh, const vec4 color, const f32 transparency);

/* Allocate node with the given id. Returns (NULL, U32_MAX) if id.size > 256B or bad shape paramters */
struct slot	led_CollisionShapeAdd(struct led *led, const struct collisionShape *shape);
/* Remove node if it exists and is not being referenced; otherwise no-op.  */
void 		led_CollisionShapeRemove(struct led *led, const utf8 id);
/* Return node with the given id if it exist; otherwise return (STUB_ADDRESS, STUB_INDEX).  */
struct slot 	led_CollisionShapeLookup(struct led *led, const utf8 id);

/* Allocate prefab with the given id. Returns (NULL, U32_MAX) if id.size > 256B or bad shape identifier */
struct slot 	led_RigidBodyPrefabAdd(struct led *led, const utf8 id, const utf8 shape, const f32 density, const f32 restitution, const f32 friction, const u32 dynamic);
/* Remove prefab if it exists and is not being referenced; otherwise no-op.  */
void 		led_RigidBodyPrefabRemove(struct led *led, const utf8 id);
/* Return prefab with the given id if it exist; otherwise return (STUB_ADDRESS, STUB_INDEX).  */
struct slot 	led_RigidBodyPrefabLookup(struct led *led, const utf8 id);

/* Allocate node with the given id. Returns (NULL, U32_MAX) if id.size > 256B or bad shape paramters */
struct slot	led_RenderMeshAdd(struct led *led, const utf8 id, const utf8 shape);
/* Remove render mesh if it exists and is not being referenced; otherwise no-op.  */
void 		led_RenderMeshRemove(struct led *led, const utf8 id);
/* Return node with the given id if it exist; otherwise return (STUB_ADDRESS, STUB_INDEX).  */
struct slot 	led_RenderMeshLookup(struct led *led, const utf8 id);

/* command identifiers */
extern u32 	cmd_led_node_add_id;
extern u32 	cmd_led_node_remove_id;
extern u32 	cmd_led_node_set_position_id;
extern u32 	cmd_led_node_set_rb_prefab_id;
extern u32 	cmd_led_node_set_csgBRush_id;
extern u32 	cmd_led_node_set_r_unit_id;

extern u32	cmd_led_compile_id;
extern u32	cmd_led_run_id;
extern u32	cmd_led_pause_id;
extern u32	cmd_led_stop_id;

extern u32 	cmd_rb_prefab_add_id;
extern u32 	cmd_rb_prefab_remove_id;

extern u32 	cmd_render_mesh_add_id;
extern u32 	cmd_render_mesh_remove_id;

extern u32 	cmd_collision_shape_add_id;
extern u32 	cmd_collision_shape_remove_id;
extern u32 	cmd_collision_box_add_id;
extern u32 	cmd_collision_sphere_add_id;
extern u32 	cmd_collision_capsule_add_id;

#endif
