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

#ifndef __R_PUBLIC_H__
#define __R_PUBLIC_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_base.h"
#include "ds_string_database.h"
#include "ds_hierarchy.h"
#include "ds_geometry.h"
#include "ds_list.h"

/*
ds_renderer.h
=============
The renderer: meshes, the camera, proxies (3d objects drawn with motion speculated between physics ticks), a
scene of draw commands per window and the GL state. Implemented in src/renderer, one section per file below;
the window and GL context are ds_graphics.h's, the GL loader ds_gl.h's.

A frame (r_EditorMain, once per elapsed render tick): for every window, r_SceneFrameBegin, the window's UI
(r_UiDraw, r_local.h) and for the root window the editor's 3d scene, r_SceneFrameEnd and the GL draw. Everything
drawn is an r_Instance with a draw command key (r_CommandKey). The keys sort descending, a larger key is drawn
first: screen layer, then transparency, depth, material, primitive, instanced, elements. A run of sorted
commands equal in every key field but depth and elements is one bucket, drawn with one draw call per buffer.
Proxy instances are cached by unit across frames and pruned after a frame that did not add them; UI and
immediate mesh instances live one frame. 3d draw data is uploaded relative to g_scene->view_origin (the
camera's position: only offsets from the eye are f32).

Conventions: metres, radians, right-handed, +y up; the camera looks along +z with +x to its left at
yaw = pitch = 0 (ds_math.h). Mesh triangles are CCW about their outward normal.
*/

/********************************************************
 *			r_mesh.c			*
 ********************************************************/

#define R_MESH_BUFSIZE  32

/*
 * a mesh in the mesh database (ds_SDB(r_Mesh), named by id_buf): vertex_count vertices of local_stride bytes,
 * drawn through index_data when index_count > 0 (R_CMD_ELEMENTS) and as arrays otherwise
 */
struct r_Mesh
{
    u8      id_buf[R_MESH_BUFSIZE];
    SDB_NODE;

	u32		index_count;		
	u32 *	index_data; 		/* index_data[index_count] */
	u32		index_max_used;		/* max used index */
	u32		vertex_count;   	
	void *	vertex_data;		/* vertex_data[vertex_count] */
	u64		local_stride;
};
DEFINE_SDB(r_Mesh);
/********************************************************
 *			r_Init.c			*
 ********************************************************/

/*
 * initiate render state: the four shader programs (fatal when one fails to read, compile or link; needs the GL
 * context current), the renderer's frame arena of frame_size bytes on mem_persistent (fatal on failure), the
 * proxy hierarchy with core_unit_count slots (growable) and its root PROXY3D_ROOT, the mesh database with the
 * stub box as its stub mesh. ns_tick is ns per draw frame (> 0: r_EditorMain redraws once per elapsed tick).
 * Called by the program's main after ds_Init and the window creation (the GL context).
 */
void 	r_Init(struct arena *mem_persistent, const u64 ns_tick, const u64 frame_size, const u32 core_unit_count, ds_SDB(r_Mesh) *mesh_database);

/********************************************************
 *			r_main.c			*
 ********************************************************/

struct led;

/*
 * rendering entrypoint (level editor): sets the renderer's clock from led->ns and, when at least one render tick
 * has elapsed since the last draw, draws every window that is not tagged for destruction (its scene's frame,
 * its UI; the root window also the editor's 3d scene) and leaves the root window's GL context current.
 */
void 	r_EditorMain(const struct led *led);

/********************************************************
 *			r_camera.c			*
 ********************************************************/

/*
 * the camera: position in world, the axes r_CameraUpdateAxes derives from yaw (about +y, in (-pi, pi]) then
 * pitch (about the camera's left, in (-pi/2 + 0.5, pi/2 - 0.5)), and the frustum
 */
struct r_Camera 
{
	v3f64 position;     /* world; 3d draw data is uploaded relative to it (camera-relative rendering) */
	v3 up;
	v3 forward;
	v3 left;
	f32 yaw;
	f32 pitch;
	f32 fz_near;
	f32 fz_far;
	f32 aspect_ratio;
	f32 fov_x;
};

/*
 * setups up camera for 3d rendering from the given axes and angles, stored as they are (not checked against
 * each other, not normalized)
 *
 * position - position in world
 * left, up, forward - normalized vectors representing orthogonal axes of camera
 * yaw, pitch - the angles the axes correspond to
 * fov_x - field of view in radians, in (0, pi)
 * fz_near - distance to nearest frustum plane, > 0 (AFFECTS Depth calculations, see Z-fighting)
 * fz_far - distance to farther frustum plane, > fz_near (AFFECTS Depth calculations, see Z-fighting)
 * aspect_ratio - w/h, > 0
 */
void r_CameraConstruct(struct r_Camera *cam,
		const v3f64 position,
	       	const v3 left,
	       	const v3 up,
	       	const v3 forward,
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
 * direction - direction of camera lense in world (any length > 0); sets yaw and pitch so that r_CameraUpdateAxes
 *             gives this frame; pitch is clamped as in r_CameraUpdateAngles (straight up/down: yaw 0, pitch at the limit)
 * fov_x - field of view in radians, in (0, pi)
 * fz_near - distance to nearest frustum plane, > 0 (AFFECTS Depth calculations, see Z-fighting)
 * fz_far - distance to farther frustum plane, > fz_near (AFFECTS Depth calculations, see Z-fighting)
 * aspect_ratio - w/h, > 0
 */
struct r_Camera	r_CameraInit(const v3f64 position, const v3 direction, const f32 fz_near, const f32 fz_far, const f32 aspect_ratio, const f32 fov_x);
/* update camera axes from its rotation: left, up, forward from yaw and pitch */
void 		r_CameraUpdateAxes(struct r_Camera *cam);
/* update camera angles by the deltas: yaw wrapped into (-pi, pi], pitch clamped to (-pi/2 + 0.5, pi/2 - 0.5); the axes are not updated */
void 		r_CameraUpdateAngles(struct r_Camera *cam, const f32 yaw_delta, const f32 pitch_delta);
/* camera print state to stderr */
void 		r_CameraDebugPrint(const struct r_Camera *cam);

/*
 * camera2d transform: transform world space coordinates to screen space
 *
 * return W_to_AS: Column-Major matrix applying the transform [-w/2, -h/2] x [w/2, h/2] => [-1.0f, -1.0f] x [1.0f, 1.0f]
 * view_center: center of viewable zone 
 * view_height, view_aspect_ratio: size of viewable zone, h and w/h
 */
m3 		r_Camera2dTransform(const v2 view_center, const f32 view_height, const f32 view_aspect_ratio);

/* Retrieve side lengths of frustum projection plane: the frustum's cross-section at plane_distance, width 2 d tan(fov_x / 2), height width / aspect_ratio */
void 		FrustumProjectionPlaneSides(f32 *width, f32 *height, const f32 plane_distance, const f32 fov_x, const f32 aspect_ratio);
/* Retreive camera frustum plane in world space coordinates: the near plane's bottom-left and upper-right corners, seen from the camera */
void 		FrustumProjectionPlaneWorldSpace(v3f64 *bottom_left, v3f64 *upper_right, const struct r_Camera *cam);
/* Retreive camera frustum plane in camera space coordinates (+x left, +y up, +z forward): bottom_left is (w/2, -h/2, fz_near) */
void 		FrustumProjectionPlaneCameraSpace(v3 *bottom_left, v3 *upper_right, const struct r_Camera *cam);

/* maps window pixel to position in world (on the near plane): (0, 0) is the plane's bottom-left corner, win_size its upper-right */
v3f64 		WindowSpaceToWorldSpace(const v2 pixel, const v2 win_size, const struct r_Camera *cam);

/************************************** Draw Command Key Layout and Macros ***************************************/

/* r_command : draw command for a r_unit. Sortable for draw ordering. 
 * Larger values indicates priority in drawing: the fields below, screen layer the most significant.
 */

struct r_Command
{
	u64	key;		/* render command key, see the field macros below 				*/
	u32	instance;	/* render instance index 						*/
	u32	allocated;	/* boolean : is the command allocated 					*/
};

/*
 * the key of the fields; every field must fit its bits (asserted): the R_CMD_* values below, a material from
 * r_MaterialConstruct. The depth of a 3d command is its distance from the camera, d, in 23 bits, larger farther
 * (so the sort draws back to front): with e_far = 1 + exponent(fz_far), a d with exponent e in
 * (e_far - 23, e_far] has depth (0x800000 | mantissa(d)) >> (e_far - e + 1), and one farther or nearer than
 * that range has 0 (r_main.c). The UI's depth is its own, reversed (r_ui.c).
 */
u64 	r_CommandKey(const u64 screen, const u64 depth, const u64 transparency, const u64 material, const u64 primitive, const u64 instanced, const u64 elements);
/* print the key's fields to stderr */
void 	r_CommandKeyPrint(const u64 key);
/* the material field: a program (PROGRAM_*, r_local.h), a mesh database index (MESH_NONE when the instance brings its own) and a texture; program and texture must fit their bits (asserted) */
u64 	r_MaterialConstruct(const u64 program, const u64 mesh, const u64 texture);

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
 *			r_proxy3d.c			*
 ********************************************************/

/*
r_proxy3d
=========
Contains data for speculative movement; Since the physics engine runs at a fixed resolution, go get smooth movements
we must speculate on future positions. A proxy is a node of a hierarchy under PROXY3D_ROOT, drawn with a mesh of
the mesh database, a color and a blend; it is addressed by its index (addresses move when the hierarchy grows).
A proxy with a parent other than the root is relative: its position and rotation are in the parent's frame and
composed with the parent's speculated pose every frame.
*/

#define PROXY3D_NULL                HI_ROOT
#define PROXY3D_ROOT			    2

#define PROXY3D_FLAG_NONE		    ((u32) 0)
#define PROXY3D_DRAW                ((u32) 1 << 0)
#define PROXY3D_MOVING			    ((u32) 1 << 1)	/* Set if any velocity != 0 			    */
#define PROXY3D_SPECULATE_NONE		((u32) 1 << 2)	/* Set if transform should be non-speculative  	    */
#define PROXY3D_SPECULATE_LINEAR	((u32) 1 << 3)	/* Set if linear speculation			    */
#define PROXY3D_RELATIVE		    ((u32) 1 << 4)  /* Set if proxy is relative (has a non-root parent) */

#define PROXY3D_SPECULATE_FLAGS		(   PROXY3D_SPECULATE_NONE	    \
					                  | PROXY3D_SPECULATE_LINEAR	\
					                )

/* what r_Proxy3dAlloc takes: the parent (PROXY3D_ROOT for a top-level proxy), the pose at ns_time with the velocities, the look */
struct r_Proxy3d_config
{
	u64	ns_time;
	u32	parent;

	v3f64	position;
	q	rotation;
	v3	linear_velocity;
	v3	angular_velocity;

	v4 	color;
	f32	blend;		/* percentage of color vs. texture */
	utf8	mesh;		/* the mesh's name in the mesh database; unknown: the stub box */
};

/*
 * r_proxy3d - proxy structure containing information for speculatively drawing. 
 */
typedef struct r_Proxy3d
{
    HI_NODE;

	u32	flags;
	v3f64	spec_position;
	q	spec_rotation;

	u64	ns_at_update;	/* ns elapsed at the time of last update to position and rotation */
	v3f64	position;	/* position of unit; interpreted according to its pos_type.  */
	q	rotation;

	u32	mesh;
	v4	color;		
	f32	blend;	

	union
	{
		struct 
		{
			v3	linear_velocity;
			v3	angular_velocity;
		} linear;
	};
} r_Proxy3d;
DEFINE_HI(r_Proxy3d);

/* return the handle of a newly allocated proxy3d, under config->parent, drawn, referencing its mesh, with r_Proxy3dLinearSpeculationSet applied */
u32 			r_Proxy3dAlloc(const struct r_Proxy3d_config *config);
/* dealloc the given proxy3d unit and its whole subtree, dereferencing their meshes (tmp: scratch for the walk); PROXY3D_NULL does nothing */
void			r_Proxy3dDealloc(struct arena *tmp, const u32 proxy);
/* return the proxy3d's address (not checked; valid until the hierarchy grows) */
struct r_Proxy3d *	r_Proxy3dAddress(const u32 proxy);
/*
 * set the proxy's pose at ns_time and the velocities it is extrapolated with: position + v t and the rotation
 * integrated by w for the time since ns_time at every draw; MOVING when any velocity is non-zero. For a relative
 * proxy the pose is in the parent's frame.
 */
void 			r_Proxy3dLinearSpeculationSet(const v3f64 position, const q rotation, const v3 linear_velocity, const v3 angular_velocity, const u64 ns_time, const u32 proxy);

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

/* something to draw this frame: its command (on the scene's frame arena) and, by type, the proxy, UI bucket or mesh it draws */
struct r_Instance
{
    POOL_NODE;
    i32         next;

	u64			frame_last_touched;	/* last scene frame it was touched; if not touched during frame, then prune */
	struct r_Command *	cmd;			/* points into arena, so safe to dereference */

	enum r_instance_type	type;			/* draw type of unit 				*/
	union
	{
		u32		       unit;	
		struct ui_DrawBucket *ui_bucket;
		struct r_Mesh	      *mesh;
	};
};
DEFINE_POOL(r_Instance);

/*
 * Draw the proxy3d unit with cmd this frame. The instance is cached across frames by unit (a changed cmd
 * re-sorts it) and pruned at the end of a frame that did not draw it. At most once per unit and frame.
 */
struct r_Instance *	r_InstanceAdd(const u32 unit, const u64 cmd);
/* add non cached instance with no unit (immediate mode: it lives this frame only); the caller sets type (R_INSTANCE_UI or R_INSTANCE_MESH) and its data */
struct r_Instance *	r_InstanceAddNonCached(const u64 cmd);

/********************** Render Buffer  *************************/

/* the GL buffers of a range of a bucket's commands: the per-vertex (local), per-instance (shared) and index data uploaded for them */
struct r_Buffer
{
	struct r_Buffer *	next;
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
 
/* r_buffer_constructor - utility to construct r_buffer arrays for r_buckets (used by the bucket generation; the buffers go on the scene's frame arena). */
struct r_BufferConstructor
{
	struct r_Buffer *first;
	struct r_Buffer *last;	

	u32 count;
};

/* reset a r_buffer array constructor */
void			r_BufferConstructorReset(struct r_BufferConstructor *constructor);
/* Alloc the next r_buffer beginning at command c_new_l and finish (if exists) the current r_buffer at c_new_l - 1 */
void			r_BufferConstructorBufferAlloc(struct r_BufferConstructor *constructor, const u32 c_new_l);
/* add size to the current buffer being constructed (one must exist) */
void 			r_BufferConstructorBufferAddSize(struct r_BufferConstructor *constructor, const u64 local_size, const u64 shared_size, const u32 instance_count, const u32 index_count);
/* finish constructing the current r_buffer array with its upper bound draw command: the array of the buffers so far (frame arena; NULL when none), the last one ending at c_h */
struct r_Buffer **	r_BufferConstructorFinish( struct r_BufferConstructor *constructor, const u32 c_h);

/********************** Render Bucket *************************
 * r_bucket - A render bucket is a set of render commands or draw commands that can be drawn in a single 
 * 	      draw call. A bucket contains zero or more draw call instructions that determines how to draw
 * 	      the vertex data inside the bucket. The commands [c_l, c_h] of a bucket are consecutive in the
 * 	      sorted order and equal in screen layer, transparency, material, primitive and instanced.
 */
struct r_Bucket
{
	struct r_Bucket *	next;

	struct r_Buffer **	buffer_array;
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
 * not recreated during the frame. One per window; the frame's commands, buffers and buckets live on the frame
 * arena of the frame (double-buffered: the previous frame's sorted commands, cmd_cache, are still readable). */

struct r_Scene
{
	struct arena 		mem_frame_arr[2];
	struct arena *		mem_frame;
	u64 			frame;

	struct ds_HashMap 		proxy3d_to_instance_map;/* map[ generation(32) | index(32) ] -> instance */
	ds_Pool(r_Instance) 	instance_pool;		/* instance storage */
	struct LL            instance_new_list;	/* non-cached instance 	*/

	struct r_Command *	cmd_cache;		/* cached commands 		*/
	struct r_Command *	cmd_frame;		/* current frame commands 	*/
	u32			cmd_cache_count;	
	u32			cmd_frame_count;

	struct r_Bucket *	frame_bucket_list;

	v3f64			view_origin;		/* world position 3d draw data is uploaded relative to (the camera; zero unless set this frame) */
};

extern struct r_Scene *g_scene;

/* alloc r_scene resources on the heap (two 64 MB frame arenas; fatal on failure) */
struct r_Scene *r_SceneAlloc(void);
/* free r_scene resources 				*/
void		r_SceneDealloc(struct r_Scene *scene);
/* set scene to be the current global scene (g_scene, what the functions below and r_InstanceAdd act on) */
void		r_SceneSetGlobal(struct r_Scene *scene);
/* begin new frame: switch and flush the frame arena, last frame's commands become the cache, view_origin is zero until set */
void		r_SceneFrameBegin(void);
/*
 * process draw commands: sort this frame's commands (descending key, the cached ones merged), prune the
 * instances not added this frame, build the buckets and their draw data on the frame arena. The GL draw is
 * r_SceneRender (r_local.h). g_scene stays set.
 */
void		r_SceneFrameEnd(void);


/**************** TEMPORARY: quick and dirty mesh generation *****************/

/*
 * Every mesh built here has (position, normal) vertices, 6 f32 per vertex. The sphere and capsule are indexed
 * (their data pushed on mem), the hull and tri mesh are triangle lists without indices, one (position, normal)
 * per corner, pushed packed on mem from mem's current position: nothing else may push on mem during the call.
 */
/* setup mesh stub: a unit box from static data, no allocation */
void 		r_MeshStubBox(struct r_Mesh *mesh_stub);
/* setup mesh from sphere parameters: refinement >= 3 strips of 2 refinement points */
void 		r_MeshSphere(struct arena *mem, struct r_Mesh *mesh, const f32 radius, const u32 refinement);
/* setup mesh from capsule parameters: the axis is y, half_height and radius > 0, refinement >= 2 rings per hemisphere */
void 		r_MeshCapsule(struct arena *mem, struct r_Mesh *mesh, const f32 half_height, const f32 radius, const u32 refinement);
/* setup mesh from collison hull: each face fanned into triangles with the face's normal */
void 		r_MeshHull(struct arena *mem, struct r_Mesh *mesh, const struct dcel *hull);
/* setup mesh from tri mesh: one triangle per tri with its normal */
void 		r_MeshTriMesh(struct arena *mem, struct r_Mesh *mesh, const struct triMesh *tri_mesh);

/********************************************************
 *			r_gl.c				*
 ********************************************************/

/*
 * A gl_State is the renderer's record of one GL context's state (functions, blending, texture unit bindings);
 * the current one (gl_StateSetCurrent) is what the ds_gl* calls act on. Textures are shared by all states;
 * texture index 0 is the stub, what the *Texture calls return on failure.
 */
/* alloc global gl state list: the texture, state and binding pools (heap, growable); before any gl_StateAlloc */
void 	gl_StateMemAlloc(void);
/* free global gl state list */
void 	gl_StateMemDealloc(void);
/*
 * alloc a new gl_state for the current GL context and make it current: loads the context's GL functions (the
 * first call also reads the GL limits and logs the vendor and version) and sets blending off, GL_FUNC_ADD,
 * GL_SRC_ALPHA / GL_ONE_MINUS_SRC_ALPHA. Its texture units are pushed on mem, which must outlive it (gl_StateDealloc).
 */
u32	gl_StateAlloc(struct arena *mem);
/* free a gl_state, unbinding its texture units; no state is current afterwards if it was */
void 	gl_StateDealloc(const u32 gl_state);
/* set gl state to current global */
void 	gl_StateSetCurrent(const u32 gl_state);

#ifdef __cplusplus
} 
#endif

#endif
