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

#ifndef __LED_PUBLIC_H__
#define __LED_PUBLIC_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_base.h"
#include "ds_dynamics.h"
#include "ds_hash_map.h"
#include "list.h"
#include "cmd.h"
#include "ds_renderer.h"
#include "ds_ui.h"
#include "hierarchy_index.h"

/*
led_Node
========
level editor fat struct node that interfaces with all sub-systems.
*/

#define LED_FLAG_NONE		((u64) 0)
#define LED_ANONYMOUS       ((u64) 1 << 0)  /* Non-identifiable led_Node; usually the case when node spawn nodes */
#define LED_PROXY3D         ((u64) 1 << 16)
#define LED_BODY_PREFAB     ((u64) 1 << 32)
#define LED_SHAPE_PREFAB    ((u64) 1 << 33)


#define LED_NODE_ID_SIZE    128
#define LED_NODE_ROOT       2

typedef struct led_Node
{
    HI_NODE;
    ds_Id           tagged_id;      /* Generational identifier */
	u64			    flags;
    u8              id_buf[LED_NODE_ID_SIZE];
	utf8			id;             /* User-provided identifier */

    ds_Transform    transform;      /* Transform relative to parent (or world origin if no parent) */

	u32			    body_prefab; 
    u32             shape_prefab;

    u32             proxy;
	vec4			color;
    f32             blend;

    //TODO only used by shit joint on init temporarily
    ds_RigidBodyId  body;
} led_Node;
HI_DECLARE(led_Node);



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
void		        led_NodeSetColor(struct led *led, const ds_Id id, const vec4 color, const f32 blend);
/* Set node color and blend factor if it exist. */
void		        led_NodeSetColorId(struct led *led, const utf8 id, const vec4 color, const f32 blend);
/* Set node color and blend factor if it exist. */
void		        led_NodeSetPosition(struct led *led, const ds_Id id, const vec3 position);
/* Set node to contain a rigid body if the node and the prefab exist */
void		        led_NodeAttachRigidBodyPrefabId(struct led *led, const utf8 id, const utf8 prefab);
/* Set node to contain a rigid body if the node and the prefab exist */
void		        led_NodeAttachRigidBodyPrefab(struct led *led, const ds_Id id, const utf8 prefab);
/* Detach any existing rigid body from the node. If the node does not exist, or have no body attached, no-op. */
void		        led_NodeDetachRigidBodyPrefabId(struct led *led, const utf8 id);
/* Detach any existing rigid body from the node. If the node does not exist, or have no body attached, no-op. */
void		        led_NodeDetachRigidBodyPrefab(struct led *led, const ds_Id id);

/*******************************************/
/*                 led_init.c              */
/*******************************************/

/* project navigation menu */
struct led_ProjectMenu
{
	u32		window;

	utf8	selected_path;		        /* selected path in menu, or empty string */

	u32		projects_folder_allocated; /* Boolean : Is directory contents allocated */
	u32		projects_folder_refresh;    /* Boolean : on main entry, refresh projects folder contents */

	struct directoryNavigator	dir_nav;
	struct ui_List			    dir_list;
	
	struct ui_Popup		        popup_new_project;
	struct ui_Popup		        popup_new_project_extra;
	utf8			            utf8_new_project;
	struct ui_TextInput 	    input_line_new_project;
};

struct led_project
{
	u32			    initialized;	/* is project setup/loaded and initialized? 	*/	
	struct file		folder;		    /* project folder 				*/
	struct file		file;		    /* project main file 				*/
};

struct led_Joint
{
    struct ds_DistanceJointPrefab   prefab;
    ds_Id                           id[2];
    ds_Transform                    local_frame[2];
};
DEFINE_CPOOL_STRUCT(led_Joint);

/*
led
===
level editor main structure
 */
struct led
{
	u32			            window;
	struct file		        root_folder;

	struct arena		    mem_persistent;

	struct led_project	    project;
	struct led_ProjectMenu  project_menu;

	struct r_Camera		    cam;
	f32			            cam_left_velocity;
	f32			            cam_forward_velocity;
		
	u64			            ns;		/* current time in ns */
	u64			            ns_delta;
	f32			            ns_delta_modifier;
	u32			            running;

	u64			            ns_engine_running;
	u64			            ns_engine_paused;

	u32			            pending_engine_paused;
	u32			            pending_engine_running;
	u32			            pending_engine_initalized;

	u32			            engine_paused;
	u32			            engine_running;		
	u32			            engine_initalized;

	utf8		           	viewport_id;
	vec2		           	viewport_position;
	vec2		           	viewport_size;

    //TODO tmp
    ds_CPool(led_Joint)     joint_pool;

	/* TODO move stuff into led project/led_Core or something */
	struct arena 		    frame;
	struct ui_List 		    brush_list;

	struct ds_RigidBodyPipeline physics;
    c_ShapeSDB 		        cs_db;	
	struct ui_List 		    cs_list;
	struct ui_DropdownMenu  cs_mesh_menu;
	struct ui_DropdownMenu  rb_color_mode_menu;

	ds_ShapePrefabSDB       shape_prefab_db;
    ds_ShapePrefabInstancePool shape_prefab_instance_pool;    

	ds_RigidBodyPrefabSDB   body_prefab_db;
	struct ui_List 		    body_prefab_list;
	struct ui_DropdownMenu  body_prefab_mesh_menu;

    r_MeshSDB			    render_mesh_db;

	struct ds_HashMap 		node_map;
	led_NodeHI		        node_hierarchy;

	struct dll		        node_selected_list;
	struct ui_List		    node_ui_list;
	struct ui_List		    node_selected_ui_list;

	/* debug */
	enum rigidBodyColorMode	pending_body_color_mode;
	enum rigidBodyColorMode	body_color_mode;
	vec4			        collision_color;
	vec4			        static_color;
	vec4			        sleep_color;
	vec4			        awake_color;

	vec4			        bounding_box_color;
	vec4			        dbvh_color;
	vec4			        sbvh_color;
	vec4			        manifold_color;
    
    u32			            draw_bounding_box;
    u32			            draw_dbvh;
    u32			            draw_sbvh;
    u32			            draw_manifold;
    u32			            draw_lines;
};

extern const char **body_color_mode_str;

extern u32	cmd_led_compile;
extern u32	cmd_led_run;
extern u32	cmd_led_pause;
extern u32	cmd_led_stop;

/* Allocate initial led resources */
struct led *	led_Alloc(void);
/* deallocate led resources */
void		led_Dealloc(struct led *led);

/*******************************************/
/*                 led_utility.c           */
/*******************************************/

u32		led_FilenameValid(const utf8 filename);

/*******************************************/
/*                 led_Main.c              */
/*******************************************/

/* level editor entrypoint; handle ui interactions and update led state */
void		led_Main(struct led *led, const u64 ns_delta);

/*******************************************/
/*                 led_ui.c                */
/*******************************************/


/* level editor ui entrypoint */
void 		led_UiMain(struct led *led);

#ifdef __cplusplus
} 
#endif

#endif
