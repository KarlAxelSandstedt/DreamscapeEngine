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

#ifndef __DS_CSG_H__
#define __DS_CSG_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_base.h"
#include "string_database.h"
#include "list.h"
#include "quaternion.h"
#include "geometry.h"
#include "ds_ui.h"

#define CSG_FLAG_NONE		((u64) 0)
#define CSG_CONSTANT		((u64) 1 << 0)	/* If set, the struct's state is to be viewed as constant   */
#define CSG_DIRTY		((u64) 1 << 1)	/* If set, the struct's state has been modified and a delta 
	           			   	   is available. */
#define CSG_MARKED_FOR_REMOVAL	((u64) 1 << 2)	/* If set, the struct should be removed as soon as possible */

enum csg_Primitive
{
	CSG_PRIMITIVE_BOX,	
	CSG_PRIMITIVE_CUSTOM,	/* custom primitive constructed using csg_op */
	CSG_PRIMITIVE_COUNT
};

enum csg_Op
{
	CSG_OP_NONE,		/* No op, csg_node is leaf 			*/
	CSG_OP_UNION,		/* node is union of left and right 		*/
	CSG_OP_INTERSECTION,	/* node is intersection of left and right 	*/
	CSG_OP_DIFFERENCE,	/* node is left without intersection with right	*/
	CSG_OP_COUNT
};

/*
csg_bursh
=========  
*/
struct csg_Brush
{
	u64			flags;
	struct csg_Brush *	delta;
	enum csg_Primitive	primitive;		/* primitive type 	*/
	struct dcel		dcel;

	DLL_SLOT_STATE;
	STRING_DATABASE_SLOT_STATE;

	/* ui caching neeeded for proper list interactions */
	struct ui_NodeCache	cache;
};

/*
csg_instance
============  
*/
struct csg_Instance
{
	u64			flags;
	struct csg_instace *	delta;

	u32			brush;			/* brush 			*/
	u32			node;			/* csg_node (leaf) index 	*/

	quat			rotation;		/* normalized quaternion 	*/
	vec3			position;

	DLL_SLOT_STATE;
	POOL_SLOT_STATE;
};

/*
csg_node
========  
*/
struct csg_Node
{
	//TODO place below into a binary_tree_macro
	u32 		parent;
	u32		left;
	u32		right;

	enum csg_Op	op;

	DLL_SLOT_STATE;
	POOL_SLOT_STATE;
};

/*
csg
===  
*/
struct csg
{
	struct arena 	frame;		/* frame lifetime */

	struct strdb	brush_db;
	struct pool	instance_pool;
	struct pool	node_pool;

	struct dll	brush_marked_list;

	struct dll	instance_non_marked_list;
	struct dll	instance_marked_list;
};

/* allocate a csg structure */
struct csg 	csg_Alloc(void);
/* deallocate a csg structure */
void		csg_Dealloc(struct csg *csg);
/* flush a csg structure's resources */
void		csg_Flush(struct csg *csg);
/* serialize a csg structure and its resources */
void		csg_Serialize(struct serialStream *ss, const struct csg *csg);
/* deserialize a csg stream and return the csg struct. If mem is not NULL, alloc fixed size csg on arena.  */
struct csg	csg_Deserialize(struct arena *mem, struct serialStream *ss, const u32 growable);		
/* csg main method; apply deltas and update csg internals */
void		csg_Main(struct csg *csg);


/* Add a new csgBRush and copy the id onto the heap on success. 
 *
 * Failure 1: id requires a buffer size > 256.
 * Failure 2: An existing brush with the same id already exist.
 */
struct slot 	csg_BrushAdd(struct csg *csg, const utf8 id);
/* Tag a brush for removal. If the reference count is not zero, this is a no op. */
void 		csg_BrushMarkForRemoval(struct csg *csg, const utf8 id);

/*
global command identifiers
==========================
*/

/*
1. Definitions
==============

csg_primitive
-------------
csg_primitives are predefined geometric primitives; When building geometry
the user begins working with csgBRushes defined by these primitives and
iteratively construct more complex shapes. 

csgBRush
---------
csgBRushes are explicit versions of csg_primitives, or the calculated result 
of a csg_node's tree. csg_nodes that are leaves have their geometries defined
by brushes.

csg_node
--------
A csg_node is a node in a binary tree; If it is a leaf, it is defined by
a csgBRush. Otherwise, its geometry is defined by (csg_op, child1, child2).
When iteratively building a world using csg, it seems reasonable to save
a csg_node as a csgBRush if the geometry is to be used multiple times.

csg_instance
------------
instance of a brush somewhere in the world; always leaf nodes.

2. CSG state change
===================

In the csg engine we want to modify the state in as few areas as possible.
Furthermore, we may very well want to modify different csg data structure
in similar ways; flag changes for both brushes and instances, id changes
or marking for removal.  

In the most simple case when we just store the modified state until the next
frame and then apply it; we may find ourselves in the following situation:

| FRAME n:		| FRAME n+1:				|
| 			|         				|
| state: state0    	| state0 = state1, state1 = state2	|
| mod: state1,state2	|         				|

We modify the state two times in successsion, but only the last state is
written, so any intermediate state change is lost. In order to solve this,
we must keep track of intermediate deltas. This will also hold true in the
scenario where we implement field deltas. Continuing with we simple case,
each csg struct that we can modify must hold a handle to its corresponding
delta struct. Examples of how to interaction with and construct deltas of
such struct are:

// If delta hasn't been allocated, allocate within csg frame memory:
if (brush->flags & (CSG_FLAG_CONSTANT | CSG_FLAG_DIRTY) == CSG_FLAG_NONE)
{
	if (we_want_to_modify_data)
	{
		// delta == brush expect delta->delta = NULL and flags CONSTANT | DIRTY isn't set.
		brush->delta = ArenaPush_and_memcpy(csg->frame, sizeof(struct brush), brush);

		//Apply changes... 
	}	
}

// If delta is already allocated 
if (brush->flags & CSG_FLAG_DIRTY)
{
	struct csg_Brush *brush_delta = brush_delta(csg, brush_handle);

	//Apply changes... 
}

This system should work fine for small structs like csgBRush and 
csg_instance and in scenarios where we won't be applying deltas to often;
ui code and such. We must make sure to not mix csg state change; partially
modifying structures themselves, partially creating deltas of them.  One
reasonable way of viewing state ownership is:

--------------------------------------------
\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
\ EXTERNAL OWNERSHIP OVER CSG_STATE_CHANGE \
\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
csg_update()               \\\\\\\\\\\\\\\\\
{                          \\\\\\\\\\\\\\\\\
        csg.apply_deltas() \\\\\\\\\\\\\\\\\
                           \\\\\\\\\\\\\\\\\
\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
--------------------------------------------

  INTERNAL OWNERSHIP OVER CSG_STATE_CHANGE  
  (We may modify structs directly)

	ArenaFlush(csg.frame);
	csg.update_system_1();
	....
	csg.update_system_n();

--------------------------------------------
\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
                           \\\\\\\\\\\\\\\\\
}                          \\\\\\\\\\\\\\\\\
\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
\ EXTERNAL OWNERSHIP OVER CSG_STATE_CHANGE \
\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
--------------------------------------------

3. API
======

4. Algorithm
============

1. find intersection brushes (DBVH and collision tests)

2. get intersection volume vertices:

	For v <- A.vertices
		if (v.inside(B))
			intersection.add(v)
		else if (v.onPlane(B))
			intersection.add(v)
			

	For e <- A.edges
		For p <- B.planes
			if (e.intersects(p))
				intersection.add(e.intersection(p)

3. get intersection volume polygons 
*/

#ifdef __cplusplus
} 
#endif

#endif
