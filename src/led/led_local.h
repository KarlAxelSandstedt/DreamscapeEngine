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

#include "ds_led.h"
#include "ds_renderer.h"
#include "ds_ui.h"

#define LED_ROOT_FOLDER_PATH	"../asset_components/led_projects"

/*******************************************/
/*                 led_visual.c            */
/*******************************************/

struct led_Visual
{
	//TODO seperate cameras for seperate modes 
	struct r_Camera	cam;	

	/* general visual aspects */
	vec4    unit_grid_color;
	f32		unit_grid_equidistance;
	u32		unit_grid_lines_per_axis;
	u32		unit_grid_draw;			/* Boolean */
	u32		unit_r_handle;

	u32		axes_draw;
	u32		axes_r_handle;

	vec4	border_color;
	vec4	background_color;
	vec4	background_highlight_color;
	vec4	background_invalid_color;
	vec4	text_color;
	i32		border_size;
	f32		edge_softness;
	f32		corner_radius;
};

extern struct led_Visual *	g_visual;

/* init led visual defaults */
void 	                led_VisualInitDefaults(const u32 window);


/*******************************************/
/*                 led_init.c              */
/*******************************************/

/* Allocate initial project menu resources */
struct led_ProjectMenu	led_ProjectMenuAlloc(void);
/* release project menu resources */
void			        led_ProjectMenuDealloc(struct led_ProjectMenu *menu);

/*******************************************/
/*                 led_Main.c              */
/*******************************************/

/* main entrypoint */
void			        led_ProjectMenuMain(struct led *led);

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

/*
led_Node
========
level editor fat struct node that interfaces with all sub-systems.
*/

#define LED_FLAG_NONE		((u64) 0)
#define LED_PROXY3D         ((u64) 1 << 16)
#define LED_RIGID_BODY      ((u64) 1 << 32)


#define LED_NODE_ID_SIZE    128

struct led_Node
{
    HI_SLOT_STATE;
    ds_Id           tagged_id;      /* Generational identifier */
	u64			    flags;
    u8              id_buf[LED_NODE_ID_SIZE];
	utf8			id;             /* User-provided identifier */

    ds_Transform    transform;      /* Transform relative to parent (or world origin if no parent) */

	u32			    rb_prefab;      

    u32             proxy;
	vec4			color;
};

/* Add a new node on success. If the parent_id is not empty, the new node will be a child of the parent node. 
 * If the allocation failed, return DS_ID_NULL. */
ds_Id               led_NodeAdd(struct led *led, const utf8 id, const utf8 parent_id);
/* Remove the node and its subhierarchy, and release all of their resources, if the node exist. Otherwise no-op. */
void                led_NodeRemoveId(struct led *led, const utf8 id);
/* Remove the node and its subhierarchy, and release all of their resources, if the node exist. Otherwise no-op. */
void                led_NodeRemove(struct led *led, const ds_Id id);
/* Return node with the given id if it exist; otherwise return (NULL, U32_MAX).  */
struct slot         led_NodeLookupId(struct led *led, const utf8 id);
/* Return node with the given ds_Id if it exist; otherwise return NULL.  */
struct led_Node *   led_NodeLookup(struct led *led, const ds_Id id);
/* Set node position if it exist. */
void		        led_NodeSetPositionId(struct led *led, const utf8 id, const vec3 position);
/* Set node position if it exist. */
void		        led_NodeSetPosition(struct led *led, const ds_Id id, const vec3 position);

/* Set node to contain a rigid body if the node and the prefab exist */
void		        led_NodeAttachRigidBodyPrefabId(struct led *led, const utf8 id, const utf8 prefab);
/* Set node to contain a rigid body if the node and the prefab exist */
void		        led_NodeAttachtRigidBodyPrefab(struct led *led, const ds_Id id, const utf8 prefab);

void		        led_NodeDetachRigidBodyPrefabId(struct led *led, const utf8 id);
void		        led_NodeDetachRigidBodyPrefab(struct led *led, const ds_Id id);



/*
led c_Shape API
===============
*/
/* Add and return a default collision shape. On failure, return (NULL, U32_MAX) */
struct slot led_CollisionShapeDefaultAdd(struct led *led, const utf8 id);
/* Add and return a dcel collision box with sides 2*hw[i]. On failure, return (NULL, U32_MAX) */
struct slot led_CollisionBoxAdd(struct led *led, const utf8 id, const vec3 hw);
/* Add and return a collision sphere. On failure, return (NULL, U32_MAX) */
struct slot led_CollisionSphereAdd(struct led *led, const utf8 id, const f32 radius);
/* Add and return a collision capsule. On failure, return (NULL, U32_MAX) */
struct slot led_CollisionCapsuleAdd(struct led *led, const utf8 id, const f32 radius, const f32 half_height);
/* Add and return a collision dcel. On failure, return (NULL, U32_MAX) */
struct slot led_CollisionDcelAdd(struct led *led, const utf8 id, struct dcel *dcel);
/* Add and return a collision triMeshBvh. On failure, return (NULL, U32_MAX) */
struct slot led_CollisionTriMeshBvhAdd(struct led *led, const utf8 id, struct triMeshBvh *mesh_bvh);
/* Remove the collision shape if it exists and is not being referenced; otherwise no-op.  */
void 		led_CollisionShapeRemove(struct led *led, const utf8 id);
/* Return the collsiion shape with the given id if it exist; otherwise return valid slot (STUB_ADDRESS, STUB_INDEX).*/
struct slot led_CollisionShapeLookup(struct led *led, const utf8 id);

/*
led ds_ShapePrefab API
======================
*/
/* Allocate prefab with the given id. Returns (NULL, U32_MAX) on failure. */
struct slot	led_ShapePrefabAdd(struct led *led, const utf8 id, const utf8 c_shape, const f32 density, const f32 restitution, const f32 friction, const f32 margin);
/* Remove prefab if it exists and is not being referenced; otherwise no-op.  */
void 		led_ShapePrefabRemove(struct led *led, const utf8 id);
/* Return prefab with the given id if it exist; otherwise return (STUB_ADDRESS, STUB_INDEX).  */
struct slot led_ShapePrefabLookup(struct led *led, const utf8 id);

/*
led ds_RigidBodyPrefab API
==========================
*/
/* Allocate prefab with the given id. On failure, return (NULL, U32_MAX). */
struct slot led_RigidBodyPrefabAdd(struct led *led, const utf8 id, const u32 dynamic);
/* Remove prefab if it exists and is not being referenced; otherwise no-op.  */
void        led_RigidBodyPrefabRemove(struct led *led, const utf8 id);
/* Return prefab with the given id if it exist; otherwise return (STUB_ADDRESS, STUB_INDEX).  */
struct slot led_RigidBodyPrefabLookup(struct led *led, const utf8 id);

/* Create and attach a shape prefab instance to the given body prefab if the body and shape exist; 
 * otherwise no-op. The body's local instance of the shape can be identified by local_id.  */
void        led_RigidBodyPrefabAttachShape(struct led *led, const utf8 rb_id, const utf8 shape_id, const utf8 local_shape_id, const ds_Transform *t_local);
/* Detach any shape prefab instace of the given body with local identifier local_id. On failure 
 * to lookup body or the local instance, the call results in a no-op. */
void        led_RigidBodyPrefabDetachShape(struct led *led, const utf8 rb_id, const utf8 local_id);
/* Return shape instance with the given id local_shape_id if the body prefab rb_id exists
 * and contains a shape instance with the given local_shape_id. Otherwise return (NULL, U32_MAX).  */
struct slot led_RigidBodyPrefabLookupShape(struct led *led, const utf8 rb_id, const utf8 local_shape_id);

/*
led r_Mesh API
==============
*/
/* Allocate node with the given id. Returns (NULL, U32_MAX) on failure. */
struct slot	led_RenderMeshAdd(struct led *led, const utf8 id, const utf8 shape);
/* Remove render mesh if it exists and is not being referenced; otherwise no-op.  */
void 		led_RenderMeshRemove(struct led *led, const utf8 id);
/* Return node with the given id if it exist; otherwise return (STUB_ADDRESS, STUB_INDEX).  */
struct slot led_RenderMeshLookup(struct led *led, const utf8 id);




//TODO

/* Set node's shape colors  */
void		        led_NodeSetProxy3d(struct led *led, const utf8 id, const vec4 color, const f32 transparency);


#endif
