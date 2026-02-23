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
#include "csg.h"
#include "dynamics.h"
#include "hash_map.h"
#include "list.h"
#include "cmd.h"
#include "ds_renderer.h"

/*******************************************/
/*                 led_init.c              */
/*******************************************/

/* project navigation menu */
struct led_ProjectMenu
{
	u32		window;

	utf8		selected_path;		/* selected path in menu, or empty string */

	u32		projects_folder_allocated; /* Boolean : Is directory contents allocated */
	u32		projects_folder_refresh; /* Boolean : on main entry, refresh projects folder contents */

	struct directoryNavigator	dir_nav;
	struct ui_List			dir_list;
	
	struct ui_Popup		popup_new_project;
	struct ui_Popup		popup_new_project_extra;
	utf8			utf8_new_project;
	struct ui_TextInput 	input_line_new_project;
};

struct led_project
{
	u32			initialized;	/* is project setup/loaded and initialized? 	*/	
	struct file		folder;		/* project folder 				*/
	struct file		file;		/* project main file 				*/
};

/*
led_node
========
General level editor node; TODO
*/

#define LED_FLAG_NONE			((u64) 0)
#define LED_CONSTANT			((u64) 1 << 0)	/* node state is constant 			   	*/
#define LED_MARKED_FOR_REMOVAL		((u64) 1 << 1)	/* node is marked for removal; Whenever possible, 
							   remove the node. 					*/
#define LED_PHYSICS			((u64) 1 << 2)	/* node contains a physics constructor handle		*/
#define LED_CSG				((u64) 1 << 3)	/* node contains a csg constructor handle		*/

struct led_node
{
	GENERATIONAL_POOL_SLOT_STATE;
	DLL_SLOT_STATE;		/* marked/non_marked state */
	DLL2_SLOT_STATE;	/* selected list state */

	u64			        flags;
	utf8			    id;
	struct ui_NodeCache	cache;
	u32			        key;

	vec3			    position;
	quat			    rotation;
	vec4			    color;

	u32			        rb_prefab; 
	u32 			    proxy;	
	u32			        csgBRush;	
};

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

	/* TODO move stuff into led project/led_Core or something */
	struct arena 		    frame;
	struct csg 		        csg;
	struct ui_List 		    brush_list;

	struct ds_RigidBodyPipeline physics;
	struct strdb 		    cs_db;	
	struct ui_List 		    cs_list;
	struct ui_DropdownMenu  cs_mesh_menu;
	struct ui_DropdownMenu  rb_color_mode_menu;

	struct strdb		    shape_prefab_db;
    struct pool             shape_prefab_instance_pool;    

	struct strdb		    rb_prefab_db;
	struct ui_List 		    rb_prefab_list;
	struct ui_DropdownMenu  rb_prefab_mesh_menu;

	struct strdb		    render_mesh_db;

	struct hashMap 		    node_map;
	struct pool		        node_pool;
	struct dll		        node_marked_list;
	struct dll		        node_non_marked_list;
	struct dll		        node_selected_list;
	struct ui_List		    node_ui_list;
	struct ui_List		    node_selected_ui_list;
};

/* Allocate initial led resources */
struct led *	led_Alloc(void);
/* deallocate led resources */
void		led_Dealloc(struct led *led);

/*******************************************/
/*                 led_utility.c              */
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
