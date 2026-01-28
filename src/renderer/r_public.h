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

#ifndef __R_PUBLIC_H__
#define __R_PUBLIC_H__

#include "asset_public.h"
#include "array_list.h"
#include "allocator.h"
#include "string_database.h"
#include "geometry.h"

/********************************************************
 *			r_init.c			*
 ********************************************************/

/* initiate render state, ns_tick is ns per draw frame, or, if 0, redraw on every r_main() entry,  should be a power of 2 */
void 	r_init(struct arena *mem_persistent, const u64 ns_tick, const u64 frame_size, const u64 core_unit_count, struct strdb *mesh_database);

/********************************************************
 *			r_main.c			*
 ********************************************************/

struct led;

/* rendering entrypoint (level editor) */
void 	r_led_main(const struct led *led);

/********************************************************
 *			r_camera.c			*
 ********************************************************/

struct r_camera 
{
	vec3 position;
	vec3 up;
	vec3 forward;
	vec3 left;
	f32 yaw;
	f32 pitch;
	f32 fz_near;
	f32 fz_far;
	f32 aspect_ratio;
	f32 fov_x;
};

/*
 * setups up camera for 3d rendering 
 *
 * position - position in world
 * left, up, forward - normalized vectors representing orthogonal axes of camera
 * fov_x - field of view in radians
 * fz_near - distance to nearest frustum plane (AFFECTS Depth calculations, see Z-fighting)
 * fz_far - distance to farther frustum plane (AFFECTS Depth calculations, see Z-fighting)
 * aspect_ratio - w/h
 */
void r_camera_construct(struct r_camera *cam,
		const vec3 position,
	       	const vec3 left,
	       	const vec3 up,
	       	const vec3 forward,
		const f32 yaw,
		const f32 pitch,
	       	const f32 fz_near,
	       	const f32 fz_far,
	       	const f32 aspect_ratio,
	       	const f32 fov_x);
/*
 * setups up camera for 3d rendering 
 *
 * position - position in world
 * direction - direction of camera lense in world 
 * fov_x - field of view in radians
 * fz_near - distance to nearest frustum plane (AFFECTS Depth calculations, see Z-fighting)
 * fz_far - distance to farther frustum plane (AFFECTS Depth calculations, see Z-fighting)
 * aspect_ratio - w/h
 */
struct r_camera	r_camera_init(const vec3 position, const vec3 direction, const f32 fz_near, const f32 fz_far, const f32 aspect_ratio, const f32 fov_x);
/* update camera axes from its rotation */
void 		r_camera_update_axes(struct r_camera *cam);
/* update camera angles */
void 		r_camera_update_angles(struct r_camera *cam, const f32 yaw_delta, const f32 pitch_delta);
/* camera print state to stderr */
void 		r_camera_debug_print(const struct r_camera *cam);

/*
 * camera2d transform: transform world space coordinates to screen space
 *
 * W_to_AS: Column-Major matrix applying the transform [-w/2, -h/2] x [w/2, h/2] => [-1.0f, -1.0f] x [1.0f, 1.0f]
 * view_center: center of viewable zone 
 * view_size: size of viewable zone 
 */
void 		r_camera2d_transform(mat3 W_to_AS, const vec2 view_center, const f32 view_height, const f32 view_aspect_ratio);

/* Retrieve side lengths of frustum projection plane */
void 		frustum_projection_plane_sides(f32 *width, f32 *height, const f32 plane_distance, const f32 fov_x, const f32 aspect_ratio);
/* Retreive camera frustum plane in world space coordinates */
void 		frustum_projection_plane_world_space(vec3 bottom_left, vec3 upper_right, const struct r_camera *cam);
/* Retreive camera frustum plane in camera space coordinates */
void 		frustum_projection_plane_camera_space(vec3 bottom_left, vec3 upper_right, const struct r_camera *cam);

/* maps window pixel to position in world */
void 		window_space_to_world_space(vec3 world_pixel, const vec2 pixel, const vec2 win_size, const struct r_camera *cam);

/************************************** Draw Command Key Layout and Macros ***************************************/

/* r_command : draw command for a r_unit. Sortable for draw ordering. 
 * Larger values indicates priority in drawing.  
 */

struct r_command
{
	u64	key;		/* render command key, see render command layout and discussion above 	*/
	u32	instance;	/* render instance index 						*/
	u32	allocated;	/* boolean : is the command allocated 					*/
};

u64 	r_command_key(const u64 screen, const u64 depth, const u64 transparency, const u64 material, const u64 primitive, const u64 instanced, const u64 elements);
void 	r_command_key_print(const u64 key);
u64 	r_material_construct(const u64 program, const u64 mesh, const u64 texture);

#define	R_CMD_SCREEN_LAYER_BITS		1
#define	R_CMD_DEPTH_BITS		23
#define	R_CMD_TRANSPARENCY_BITS		2
#define	R_CMD_MATERIAL_BITS		30
#define R_CMD_ELEMENTS_BITS		1
#define	R_CMD_INSTANCED_BITS		1
#define	R_CMD_PRIMITIVE_BITS		1
#define R_CMD_UNUSED_BITS		(64 - R_CMD_SCREEN_LAYER_BITS - R_CMD_DEPTH_BITS - R_CMD_TRANSPARENCY_BITS - R_CMD_MATERIAL_BITS - R_CMD_PRIMITIVE_BITS - R_CMD_INSTANCED_BITS - R_CMD_ELEMENTS_BITS)

#define R_CMD_SCREEN_LAYER_LOW_BIT	(R_CMD_DEPTH_BITS + R_CMD_TRANSPARENCY_BITS + R_CMD_MATERIAL_BITS + R_CMD_PRIMITIVE_BITS + R_CMD_INSTANCED_BITS + R_CMD_ELEMENTS_BITS)
#define R_CMD_TRANSPARENCY_LOW_BIT	(R_CMD_DEPTH_BITS + R_CMD_MATERIAL_BITS + R_CMD_PRIMITIVE_BITS + R_CMD_INSTANCED_BITS + R_CMD_ELEMENTS_BITS)
#define R_CMD_DEPTH_LOW_BIT		(R_CMD_MATERIAL_BITS + R_CMD_PRIMITIVE_BITS + R_CMD_INSTANCED_BITS + R_CMD_ELEMENTS_BITS)
#define R_CMD_MATERIAL_LOW_BIT		(R_CMD_PRIMITIVE_BITS + R_CMD_INSTANCED_BITS + R_CMD_ELEMENTS_BITS)
#define R_CMD_PRIMITIVE_LOW_BIT		(R_CMD_INSTANCED_BITS + R_CMD_ELEMENTS_BITS)
#define R_CMD_INSTANCED_LOW_BIT		(R_CMD_ELEMENTS_BITS)
#define R_CMD_ELEMENTS_LOW_BIT		(0)
#define R_CMD_UNUSED_LOW_BIT		(R_CMD_SCREEN_LAYER_BITS + R_CMD_DEPTH_BITS + R_CMD_TRANSPARENCY_BITS + R_CMD_MATERIAL_BITS + R_CMD_PRIMITIVE_BITS + R_CMD_INSTANCED_BITS + R_CMD_ELEMENTS_BITS)

#define R_CMD_SCREEN_LAYER_MASK		((((u64) 1 << R_CMD_SCREEN_LAYER_BITS) - (u64) 1) << R_CMD_SCREEN_LAYER_LOW_BIT)
#define R_CMD_DEPTH_MASK		((((u64) 1 << R_CMD_DEPTH_BITS) - (u64) 1) << R_CMD_DEPTH_LOW_BIT)
#define R_CMD_TRANSPARENCY_MASK		((((u64) 1 << R_CMD_TRANSPARENCY_BITS) - (u64) 1) << R_CMD_TRANSPARENCY_LOW_BIT)
#define R_CMD_MATERIAL_MASK		((((u64) 1 << R_CMD_MATERIAL_BITS) - (u64) 1) << R_CMD_MATERIAL_LOW_BIT)
#define R_CMD_PRIMITIVE_MASK		((((u64) 1 << R_CMD_PRIMITIVE_BITS) - (u64) 1) << R_CMD_PRIMITIVE_LOW_BIT)
#define R_CMD_INSTANCED_MASK		((((u64) 1 << R_CMD_INSTANCED_BITS) - (u64) 1) << R_CMD_INSTANCED_LOW_BIT)
#define R_CMD_ELEMENTS_MASK		((((u64) 1 << R_CMD_ELEMENTS_BITS) - (u64) 1) << R_CMD_ELEMENTS_LOW_BIT)
#define R_CMD_UNUSED_MASK		((((u64) 1 << R_CMD_UNUSED_BITS) - (u64) 1) << R_CMD_UNUSED_LOW_BIT)

#define R_CMD_SCREEN_LAYER_GET(val64)	((val64 & R_CMD_SCREEN_LAYER_MASK) >> R_CMD_SCREEN_LAYER_LOW_BIT)
#define R_CMD_DEPTH_GET(val64)		((val64 & R_CMD_DEPTH_MASK) >> R_CMD_DEPTH_LOW_BIT)
#define R_CMD_TRANSPARENCY_GET(val64)	((val64 & R_CMD_TRANSPARENCY_MASK) >> R_CMD_TRANSPARENCY_LOW_BIT)
#define R_CMD_MATERIAL_GET(val64)	((val64 & R_CMD_MATERIAL_MASK) >> R_CMD_MATERIAL_LOW_BIT)
#define R_CMD_PRIMITIVE_GET(val64)	((val64 & R_CMD_PRIMITIVE_MASK) >> R_CMD_PRIMITIVE_LOW_BIT)
#define R_CMD_INSTANCED_GET(val64)	((val64 & R_CMD_INSTANCED_MASK) >> R_CMD_INSTANCED_LOW_BIT)
#define R_CMD_ELEMENTS_GET(val64)	((val64 & R_CMD_ELEMENTS_MASK) >> R_CMD_ELEMENTS_LOW_BIT)
#define R_CMD_UNUSED_GET(val64)		((val64 & R_CMD_UNUSED_MASK) >> R_CMD_UNUSED_LOW_BIT)

#define R_CMD_SCREEN_LAYER_GAME		((u64) 1)	/* The game itself */
#define R_CMD_SCREEN_LAYER_HUD		((u64) 0)	/* Game menus, mouse pointer, ... */ 

/* We draw transparent objects after opaque objects on the same layer */
#define R_CMD_TRANSPARENCY_OPAQUE 	((u64) 3)
#define R_CMD_TRANSPARENCY_ADDITIVE	((u64) 2)
#define R_CMD_TRANSPARENCY_SUBTRACTIVE	((u64) 1)
#define R_CMD_TRANSPARENCY_NORMAL	((u64) 0)

#define R_CMD_INSTANCED			((u64) 1)
#define R_CMD_NON_INSTANCED		((u64) 0)

#define R_CMD_ELEMENTS			((u64) 1)
#define R_CMD_ARRAYS			((u64) 0)

#define R_CMD_PRIMITIVE_LINE		((u64) 1)
#define R_CMD_PRIMITIVE_TRIANGLE	((u64) 0)

#define MATERIAL_PROGRAM_BITS		2
#define MATERIAL_MESH_BITS		10
#define MATERIAL_TEXTURE_BITS		3
#define MATERIAL_UNUSED_BITS 		(R_CMD_MATERIAL_BITS - MATERIAL_PROGRAM_BITS - MATERIAL_TEXTURE_BITS - MATERIAL_MESH_BITS)
#define MESH_NONE			0
#define MESH_STUB			0

#define MATERIAL_TEXTURE_LOW_BIT	0
#define MATERIAL_MESH_LOW_BIT		(MATERIAL_TEXTURE_BITS)
#define MATERIAL_PROGRAM_LOW_BIT	(MATERIAL_TEXTURE_BITS + MATERIAL_MESH_BITS)
#define MATERIAL_UNUSED_LOW_BIT		(MATERIAL_TEXTURE_BITS + MATERIAL_PROGRAM_BITS + MATERIAL_MESH_BITS)

#define MATERIAL_PROGRAM_MASK		((((u64) 1 << MATERIAL_PROGRAM_BITS) - (u64) 1) << MATERIAL_PROGRAM_LOW_BIT)
#define MATERIAL_MESH_MASK		((((u64) 1 << MATERIAL_MESH_BITS) - (u64) 1) << MATERIAL_MESH_LOW_BIT)
#define MATERIAL_TEXTURE_MASK		((((u64) 1 << MATERIAL_TEXTURE_BITS) - (u64) 1) << MATERIAL_TEXTURE_LOW_BIT)
#define MATERIAL_UNUSED_MASK		((((u64) 1 << MATERIAL_UNUSED_BITS) - (u64) 1) << MATERIAL_UNUSED_LOW_BIT)

#define MATERIAL_PROGRAM_GET(material)	((material & MATERIAL_PROGRAM_MASK) >> MATERIAL_PROGRAM_LOW_BIT)
#define MATERIAL_MESH_GET(material)	((material & MATERIAL_MESH_MASK) >> MATERIAL_MESH_LOW_BIT)
#define MATERIAL_TEXTURE_GET(material)	((material & MATERIAL_TEXTURE_MASK) >> MATERIAL_TEXTURE_LOW_BIT)

/********************************************************
 *			r_core.c			*
 ********************************************************/

/*
r_proxy3d
=========
Contains data for speculative movement; Since the physics engine runs at a fixed resolution, go get smooth movements
we must speculate on future positions. 
*/

#define PROXY3D_ROOT			2

#define PROXY3D_FLAG_NONE		((u32) 0)
#define PROXY3D_MOVING			((u32) 1 << 0)	/* Set if any velocity != 0 			    */
#define PROXY3D_SPECULATE_NONE		((u32) 1 << 1)	/* Set if transform should be non-speculative  	    */
#define PROXY3D_SPECULATE_LINEAR	((u32) 1 << 2)	/* Set if linear speculation			    */
#define PROXY3D_RELATIVE		((u32) 1 << 3)  /* Set if proxy is relative (has a non-root parent) */

#define PROXY3D_SPECULATE_FLAGS		(   PROXY3D_SPECULATE_NONE	\
					  | PROXY3D_SPECULATE_LINEAR	\
					)

struct r_proxy3d_config
{
	u64	ns_time;
	u32	parent;

	vec3	position;
	quat	rotation;
	vec3	linear_velocity;
	vec3	angular_velocity;

	vec4 	color;
	f32	blend;		/* percentage of color vs. texture */
	utf8	mesh;
};

/*
 * r_proxy3d - proxy structure containing information for speculatively drawing. 
 */
struct r_proxy3d
{
	struct hiNode	header; /* DO NOT MOVE! */

	u32	flags;
	vec3	spec_position;
	vec4	spec_rotation;

	u64	ns_at_update;	/* ns elapsed at the time of last update to position and rotation */
	vec3	position;	/* position of unit; interpreted according to its pos_type.  */
	quat	rotation;

	u32	mesh;
	vec4	color;		
	f32	blend;	

	union
	{
		struct 
		{
			vec3	linear_velocity;
			vec3	angular_velocity;
		} linear;
	};
};

/* return the handle of a newly allocated proxy3d. */
u32 			r_proxy3d_alloc(const struct r_proxy3d_config *config);
/* dealloc the given proxy3d unit. */
void			r_proxy3d_dealloc(struct arena *tmp, const u32 proxy);
/* return the proxy3d of the unit given that the unit exist and has a proxy3d; otherwise return NULL. */
struct r_proxy3d *	r_proxy3d_address(const u32 proxy);
/* set the proxy */
void 			r_proxy3d_set_linear_speculation(const vec3 position, const quat rotation, const vec3 linear_velocity, const vec3 angular_velocity, const u64 ns_time, const u32 proxy);

/********************************************************
 *			r_scene.c			*
 ********************************************************/

enum r_instance_type
{
	R_INSTANCE_PROXY3D,	/* instance of a proxy3d	*/
	R_INSTANCE_UI, 		/* instance of a ui bucket	*/
	R_INSTANCE_MESH,	/* instance of a mesh		*/
	R_INSTANCE_COUNT
};

struct r_instance
{
	struct array_list_intrusive_node header;

	u64			frame_last_touched;	/* last scene frame it was touched; if not touched during frame, then prune */
	struct r_command *	cmd;			/* points into arena, so safe to dereference */

	enum r_instance_type	type;			/* draw type of unit 				*/
	union
	{
		u32		       unit;	
		struct ui_draw_bucket *ui_bucket;
		struct r_mesh	      *mesh;
	};
};

struct r_instance *	r_instance_add(const u32 unit, const u64 cmd);
struct r_instance *	r_instance_add_non_cached(const u64 cmd); /* add non cached instance with no unit. This
								     gives us an immediate mode option that can
								     simplify what we want. */

/********************** Render Buffer  *************************/

struct r_buffer
{
	struct r_buffer *	next;
	u32 			shared_vbo;
	u32 			local_vbo;
	u32 			ebo;

	u64			shared_size;	/* total size of shared data in bucket (instanced) */
	u64			local_size;	/* total size of all vertices in bucket (vertex)   */
	u32			index_count;	/* number of indices in bucket   		   */
	u32			instance_count;	/* instance count          			   */

	u8 *			shared_data;	/* buf[shared_size] 	*/
	u8 *			local_data;	/* buf[local_size] 	*/
	u32 *			index_data;	/* u32[index_count]	*/

	/* draw command range [c_l, c_h]  related to buffer */
	u32			c_l;			
	u32			c_h;
};

/********************** Render Buffer Constructor  *************************/
 
/* r_buffer_constructor - utility to construct r_buffer arrays for r_buckets.  */
struct r_buffer_constructor
{
	struct r_buffer *first;
	struct r_buffer *last;	

	u32 count;
};

/* reset a r_buffer array constructor */
void			r_buffer_constructor_reset(struct r_buffer_constructor *constructor);
/* Alloc the next r_buffer beginning at command c_l and finish (if exists) the current r_buffer */
void			r_buffer_constructor_buffer_alloc(struct r_buffer_constructor *constructor, const u32 c_new_l);
/* add size to the current buffer being constructed  */
void 			r_buffer_constructor_buffer_add_size(struct r_buffer_constructor *constructor, const u64 local_size, const u64 shared_size, const u32 instance_count, const u32 index_count);
/* finish constructing the current r_buffer array with its upper bound draw command */
struct r_buffer **	r_buffer_constructor_finish( struct r_buffer_constructor *constructor, const u32 c_h);

/********************** Render Bucket *************************
 * r_bucket - A render bucket is a set of render commands or draw commands that can be drawn in a single 
 * 	      draw call. A bucket contains zero or more draw call instructions that determines how to draw
 * 	      the vertex data inside the bucket.
 */
struct r_bucket
{
	struct r_bucket *	next;

	struct r_buffer **	buffer_array;
	u32			buffer_count;

	u32	c_l;			/* index of first r_command in bucket 	*/
	u32	c_h;			/* index of last r_command in bucket  	*/

	u32	elements;
	u32	instanced;
	u32	primitive;
	u32	transparency;
	u32	material;
	u32	screen_layer;
};

/********************** Render Scene  *************************/
/* r_scene : A set of instances to be drawn. The structure is partially immediate; Every frame the user specifies
 * a set of draw commands for some render units. Each frame caches its new r_instances, and prunes ant instaces 
 * not recreated during the frame. */

struct r_scene
{
	struct arena 		mem_frame_arr[2];
	struct arena *		mem_frame;
	u64 			frame;

	struct hashMap *	proxy3d_to_instance_map;	/* map[ generation(32) | index(32) ] -> instance */
	struct array_list_intrusive *instance_list;		/* instance storage 					   */

	u32 			instance_new_first;	/* non-cached instance [ instance_dll ]	*/

	struct r_command *	cmd_cache;		/* cached commands 		*/
	struct r_command *	cmd_frame;		/* current frame commands 	*/
	u32			cmd_cache_count;	
	u32			cmd_frame_count;
	u32			cmd_new_count;		/* new command count (includes updated cached cmd's) */

	struct r_bucket *	frame_bucket_list;
};

extern struct r_scene *g_scene;

/* alloc r_scene resources 				*/
struct r_scene *r_scene_alloc(void);
/* free r_scene resources 				*/
void		r_scene_free(struct r_scene *scene);
/* set scene to be the current global scene 		*/
void		r_scene_set(struct r_scene *scene);
/* begin new frame 					*/
void		r_scene_frame_begin(void);
/* set global scene to NULL and process draw commands  	*/
void		r_scene_frame_end(void);

/********************************************************
 *			r_mesh.c			*
 ********************************************************/

struct r_mesh
{
	STRING_DATABASE_SLOT_STATE;				/* internal header, MAY NOT BE MOVED */
	u32				index_count;		
	u32 *				index_data; 		/* index_data[index_count] */
	u32				index_max_used;		/* max used index */
	u32				vertex_count;   	
	void *				vertex_data;		/* vertex_data[vertex_count] */
	u64				local_stride;
};

/**************** TEMPORARY: quick and dirty mesh generation *****************/

/* setup mesh stub */
void 		r_mesh_set_stub_box(struct r_mesh *mesh_stub);
/* setup mesh from sphere parameters */
void 		r_mesh_set_sphere(struct arena *mem, struct r_mesh *mesh, const f32 radius, const u32 refinement);
/* setup mesh from capsule parameters */
void 		r_mesh_set_capsule(struct arena *mem, struct r_mesh *mesh, const f32 half_height, const f32 radius, const u32 refinement);
/* setup mesh from collison hull */
void 		r_mesh_set_hull(struct arena *mem, struct r_mesh *mesh, const struct dcel *hull);
/* setup mesh from tri mesh */
void 		r_mesh_set_tri_mesh(struct arena *mem, struct r_mesh *mesh, const struct tri_mesh *tri_mesh);

/********************************************************
 *			r_gl.c				*
 ********************************************************/

/* alloc global gl state list */
void 	gl_state_list_alloc(void);
/* free global gl state list */
void 	gl_state_list_free(void);
/* alloc a new gl_state  */
u32	gl_state_alloc(void);
/* free a gl_state  */
void 	gl_state_free(const u32 gl_state);
/* set gl state to current global */
void 	gl_state_set_current(const u32 gl_state);

/*
   Some notes in order of development; initial documentation is wrong and should instead be viewed as the thought
   process as the library was developed. CLEAN UP and write proper later when things are changing less.

============================= r_core Persistent Render Units =============================

 	(1) identification: Given a render unit handle, we should be able to uniquely identify a render unit. Since
 	we expect no objects to share a render unit, so we may use an index based identifier.  
 
 	struct r_unit[id]
 	{
 	}
 
  	(2) hierarchy-based unit allocation / deallocation. render units may depend on / be related to each other in 
  	position, and this can be expressed in a hierarchy based way. Consider the following case:
 
  	Case 1. We have a character with 3 purely asthetic spinning orbs surronding it. the orbs may stop being
  	rendered if either the character stops  being rendered, or the individual orb render units gets
  	deallocated. 
 
 		[i0]	[i1, i2, i3]
 		unit <- { unit, unit, unit }
 
 	It seems quite reasonable to support dealloc(root) which then deallocates the whole sub-hierarchy of root;
 	the three orbs may not even be represented as some sort of game entities, the only reference to them
 	are as render units. So the only way we would know to deallocated them is through the character game entity.
 	The point of this case is that one game entity may have several render units, and we must support deallocating
 	all at once.
 
  	Thus, we add to our render unit structure the following hierarchy related variables:
 
  		struct r_unit[id]
  		{
 			hierarchy_node node
  		}
 
  	where a hierarchy_index_node is defined as follows:
 
  		struct hierarchy_node
  		{
 			u32	parent;	// parent
 			u32	prev;	// previous sibling
 			u32	next;	// next sibling
 			u32	first;	// first child
 			u32	last;	// last child 
  		}
 
 	(3) render state: For simplicity (and for now), all render units contain the following render state
 
 		struct r_unit[id]
 		{
 			u32 		draw_flags;		// 0 or more flags set 
 			vec4		color;			// Used on DRAW_COLOR
 			f32		transparency;		// Used on DRAW_TRASPARENT
 			f32		color_blending;		// Used on DRAW_COLOR & DRAW_**MATERIAL**
 			u32		material;		// Used on DRAW_**MATERIAL**
 			hierarchy_node 	node;
  		}
 
 	
 	(4) draw commands: each render unit must be drawn in a specific order, back-to-front to get correct a
 	result; we therefore introduce a corresponding sortable render command to each render unit. The render
 	unit will keep an updatable render command index to its command in the sortable command array.
 
  		struct r_unit[id]
 		{
 			u32 		draw_flags;		// 0 or more flags set 
 			vec4		color;			// Used on DRAW_COLOR
 			f32		transparency;		// Used on DRAW_TRASPARENT
 			f32		color_blending;		// Used on DRAW_COLOR & DRAW_**MATERIAL**
 			u32		material;		// Used on DRAW_**MATERIAL**
 			hierarchy_node 	node;
 			u32		r_cmd_i;		// index of r_unit's render command 
  		}
 
 
  	The render command array is simply an array of equal size to the unit array, with struct r_cmd's;
 
  		struct r_cmd[i]
  		{
 			u64	cmd;
 			u32	unit;
  		}
 
  	After having sorted the render command array, we update the indices of all units.
 
  	(5) draw command layout
 
 	r_cmd: Render command, sortable. the command itself is both a partial instruction and a draw order;  Higher
 	       valued commands is (and must) be drawn before and lower valued one. It is a partial instruction
 	       in the sense that it determines some of the instructions we should execute in order to draw the
 	       corresponding object.
 
 	63									0
 	[ ... | SCREEN_LAYER(1) | TRASPARENTCY(2) | DEPTH(23) | MATERIAL_ID(30) ]
 
 	NOTE: If one wanted to optimize the number of draw calls, one could make a single draw call for all
 	r_cmd's with transparency set to 0 the depth of all objects are instead view as actual z-values 
 	(normalized to some exponent if
 	we are dealing with floats), and we let the gpu draw and work with its depth buffer for the
 	non-transparent objects. Any transparent objects must still be sorted and drawn in order, but
 	for the non-transparent ones we skip sorting and simply draw them all.
 
 	For another overview (and discussion in comments) see https://realtimecollisiondetection.net/blog/?p=86.
 
 	----------------- float to depth bits -----------------
 
  	Suppose we have a camera with a positiog and a maximum view distance fz_far, and a scene of various 
  	transparent objects whose positions we know. Now, we want to generate an ordering of all transparent 
  	objects within our view according to their distance from our eye, back-to-front, using our depth bits.  
 
  	A normalized float has the following setup:
 
 	float a = (-1)^(sign_bit) * 2^(exponent_bits-127) * 1.b_22b_21...b_0
 		= [ sign_bit(1) | exponent(8) | fraction(23) ]
 
 	Therefore, in order to retain as much precision as possible in our depth keys, we normalize all floats
 	to the exponent (1 + fz_far.exponent) we no implicit 1 before the fracion. As we don't care for floats
 	with exponents larger than or equal two (1 + fz_far.exponent) (since they are outside of our view
 	distance) the computation becomes as follows:
 
  	exponent 			| key
  	----------------------------------------------------------------------
 	> (1 + fz_far.exponent) 	| ANY (They will most likely not be viewable)
  	----------------------------------------------------------------------
 	  (1 + fz_far.exponent) 	| (0x00800000 OR fraction(23)) >> 1
  	----------------------------------------------------------------------
  	...				| ...
  	----------------------------------------------------------------------
 	  (1 + fz_far.exponent - n) 	| (0x00800000 OR fraction(23)) >> (n+1)
  	----------------------------------------------------------------------
 	<= (1 + fz_far.exponent - 23) 	| 0, (At this point, we can no longer distinguish between distances)
 	
 	----------------- TODO -----------------
 
 	Now, a draw command is not enough to to draw something; it contains instructions of how to draw an object, 
 	but not the object itself. Skipping buffers and objects for a second, we state what the actual objective
 	is. We have drawing instructions for every entity in the scene; 
 
 	(1) We sort draw commands to get a drawing order of the entities.
 
 	(2) We now need to determine the actual draw calls; this equates to bucketing up as many draw commands 
 	    in a single buffer as possible. 
 
 	(3) for each draw bucket, we need to make sure the corresponding draw data will be stored at the correct place.
 
 	Now, how do we actually do the above?  To reach a solution, we first decouple the idea of public render 
 	api buffers and actual opengl buffers; They should not be viewed as equals. Two examples of why we want
 	this are: 1. Several api buffers may (or should) actually be able to be drawn together for whatever reason,
 	so they could share the same opengl buffer. 2. A api buffer may be to large to use a single opengl buffer, 
 	so we may instead work with several opengl buffers at the same time when drawing a single api buffer;
 	so it seems reasonable to have the two ideas decoupled.
 
  	Now begin by deriving a simple high-level solution to (2):
 
  	(2.0) Solution: 
  	{
  		draw_bucket = stub_finished 
  		FOR (i = 0; i < in sorted_commands.len; ++i)
  			IF (draw_bucket == full || r_cmd should be in new bucket)
  				draw_bucket.last = i;
  				draw_bucket = new
  				draw_bucket.first = i;
  			END
  		END
  	}
 
  	This begins the more intricate and meaty question
 	
 	(2.1) Question: How do we know when we should begin a new bucket?
 	(2.1) Solution: TODO 
 
 	Now that the draw buckets have been generated, we must solve issue (3). A solution to (3) equates to 
 	determining an architecture for our rendering api storage, and how to identify any stored render 
 	information given its r_cmd. First of all, we can bundle each render command r_cmd with an identifier
 	(r_cmd, id). The identifier is used to lookup whatever information we need in order to generate the
 	vertex data for the render command.
 
 	(3.0) Solution 1: the user render api's should handle code in something like the following manner,
 		        extending our rendering 2d api:
  	{
 		r_proxy2d_alloc(draw_info, draw_flags):
 			const ID unique_id = r_core_proxy2d_alloc(render_state, draw_flags);	
 			return unique_id;
 
 		struct r_proxy2d *proxy = r_core_lookup(unique_id) 	// should be O(1)
 
 		// Set any specific proxy state here
 		PROXY2D_PARENT(unqiue_id)
 		{
 			...
 		}
 	}
 
  	Giving some thought into this approach, the strategy seem to clash with our immediate mode thinking.
  	Storing context specific identifiers, indices and so on for different types of buffers; where are
  	buffers stored? and so on... is way to complex at the moment. 
 
  	(3.0) Solution 2: We will get by just fine with only a few hard coded 2d api buffers for the first games.
  		Suppose the engine / game needs N api buffers. we bundle each render command with a 64 identifer
  		that is used as follows:
 
  		63			32 31				0
  		[	Buffer ID	  | 	    Buffer Index	]
 
  		The Buffer ID determines what render api buffer we need to search in to find command's corresponding
  		data. The buffer index is self-explanatory. 
 
  	(3.1M)	Generating the vertex data for our api buffers will now be super easy; Since our buffers are hard-coded
  		and contains data not dependent on data outside of the buffer, vertex data generation becomes a single
  		thread-safe task. In the draw loop, we would get something like:
 
  		TASK_STREAM
  		{
  			TASK_POST(buffer_1 vertex generation);
  			TASK_POST(buffer_2 vertex generation);
 					...
  			TASK_POST(buffer_N vertex generation);
  		}
 
  		sort_draw_command();
  		task_context_master_finish_jobs();
 		WAIT(TASK_STREAM);
 
  		TASK_STREAM
  		{
  			TASK_POST(draw_bin_1 setup vertex_array);
  			TASK_POST(draw_bin_2 setup vertex_array);
 				           ...
  			TASK_POST(draw_bin_N setup_vertex_array);
 
 
  		}
 		WHILE (jobs left)
 			WHILE (jobs finished)
 				finished_bucket.glBufferData
 			END
 			task_context_master_single_job()
  		END
 
 		WHILE (last jobs finishing)
 			finished_bucket.glBufferData
 		END
 
  	--------------------------- Optimization ---------------------------
 
  	//TODO The renderer should be re-architectured to cache objects instead of being fully immediate.
  	//TODO BSP trees for occlusion...
 
  	--------------------------- Multithreading ---------------------------
  	(1M) Solution: TODO Many options here for multithreaded sorting, but profile first to see if needed
 
  	The bucket making can be done parallel by simply extending (2) to 
 	(2M, 3M) Solution: 
 		TASK_STREAM_BEGIN()
  		draw_bucket = stub_finished 
  		FOR (i = 0; i < in sorted_commands.len; ++i)
  			IF (draw_bucket == full || r_cmd should be in new bucket)
  				draw_bucket.last = i;
  				TASK_STREAM_ADD(draw_bucket_generate_gl_data)
  				draw_bucket = new
  				draw_bucket.first = i;
  			END
  		END
 		TASK_STREAM_END()
 
  		// TODO Option(1) simple: 
  		task_context_master_finish_jobs()
 		FOR (bucket.len)
 			bucket.glBufferData
 		END
 
  		// TODO Option(2) possibly optimized (PROFILE!): 
  		Optimized for utilizing CPU-GPU bandwidth
  		WHILE (jobs left)
 			WHILE (jobs finished)
 				finished_bucket.glBufferData
 			END
 			task_context_master_single_job()
  		END
 
 		WHILE (last jobs that has finished)
 			finished_bucket.glBufferData
 		END
 
============================= Multi-Window Rendering =============================

Suppose that in our program, we wish to view a DCEL in several differant way at the same time.
The DCEL itself gets a render handle, and the rendering core allocated a new render unit of type
DCEL.
			+-----------------+
			| R_CORE	  |
	      		|                 |
	DCEL <------------> R_UNIT        |
			|	          |
			+-----------------+

Now, the render unit itself only contains shared data from which we can generate draw data from,
but not the draw instructions itself. Suppose we now want to draw this unit, we call, in different
places in our program:

	r_unit_draw(dcel.r_unit, draw_command)
	...
	r_unit_draw(dcel.r_unit, draw_command)
	...
	r_unit_draw(dcel.r_unit, draw_command)

Each draw call construct a r_instance within the bound scene win->r_scene. Here lies an important
design choice; should the api be immediate under the hood? 

	(1) Immediate Mode:
		1. r_unit_draw(dcel.r_unit, draw_command) => (r_frame, r_unit, command_key)
		2. command_key is added to r_frame.keys
		3. sort key and generate draw data
		4. draw
		5. clear frame data
	
		Pros: Very simple
		Cons: For large amounts of data, we do not make use of frame coherency; almost 
			every frame the keys will stay the same, so we are almost always fully sorted.


	(2) Retained Mode:
		1. r_unit_draw(dcel.r_unit, draw_command) => (r_frame, r_unit, command_key)
		2. If (r_frame, r_unit) does not exist
			establish (r_unit <- r_frame.r_instace)
			establish (r_frame.r_instance <-> cmd_key)
		3. else if (cmd_key != r_unit->r_frame.r_instace->cmd_key)
			remove old key
			add new key
		4. update and sort cmd keys AND update (r_instace <-> cmd_key)
		5. draw

The Immediate mode choice is not viable; we shall go with the Retained mode. We can, similar to our ui system,
set up a caching system for instances to make it slightly simpler and just as fast.
In this strategy  we reconstruct draw commands every frame, and let the r_frame do the retained handling.
If we do not want to draw an instance anymore, we simple choose not to call r_draw_unit for that instance. If the
instance hasn't been drawn by the end of the frame, we remove it before the sorting. At the end of the frame,
we cache all old instances.

Memory handling:

	(1) If r_unit is deallocated,
		(X) Do nothing, any r_instances related to the unit will be released as they wount be touched again

	(2) If r_scene is deallocated,
		() del r_scene memory

To simplify the rendering even more, certain things we want to draw we know is completely local to a specific scene,
ui for example. In those cases, setting up r_units for every ui batch is unecessary. Instead, we should just use the
r_instance API directly to add units. This essentially becomes a fully Immediate API; Every frame our old  direct 
r_instance calls have their data cleaned up.

struct r_core
{
	//shared data
}

struct r_scene
{
	struct array_list_intrusive *instances;
}

[array allocator: intrusive double linked list}
struct r_instance
{
	0 = NULL
	U32 = MAX 
	struct dll_header; { prev, next };

	u32	last_frame_touched;
	u32	unit;	// ONLY SAFE TO USE IF last_frame_touched == current_frame
	u32	cmd;
}

struct r_cmd
{
	u64	cmd;	
	u32 	instance;
	u32	allocated;
}
*/

#endif
