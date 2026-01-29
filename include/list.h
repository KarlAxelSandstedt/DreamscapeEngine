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

#ifndef __DS_LIST_H__
#define __DS_LIST_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_allocator.h"

/*
ll
==== 
Intrusive linked list for indexed structures. To use a struct as a list node, put
LL_SLOT_STATE in the structure. It is meant to be used for arrays < U32_MAX 
indices, where all structs are allocated in the same array. 
 */

#define LL_NULL				U32_MAX
#define LL_SLOT_STATE			u32 ll_next
#define ll_Next(structure_addr)		((structure_addr)->ll_next)

struct ll
{
	u32 	count;
	u32 	first;
	u32 	last;
	u64 	slot_size;
	u64	slot_state_offset;
};

/* initalize linked list  */
struct ll		ll_InitInternal(const u64 slot_size, const u64 slot_state_offset);
#define ll_Init(STRUCT)	ll_InitInternal(sizeof(STRUCT), (u64) &((STRUCT *)0)->ll_next)
/* flush list */
void			ll_Flush(struct ll *ll);
/* append to list */
void			ll_Append(struct ll *ll, void *array, const u32 index);
/* prepend to list */
void			ll_Prepend(struct ll *ll, void *array, const u32 index);

/*
dll
==== 
Intrusive doubly linked list for indexed structures. To use a struct as a list node,
put DLL_SLOT_STATE in the structure. It is meant to be used for arrays < U32_MAX 
indices, where all structs are allocated in the same array. 
 */

#define DLL_NULL			U32_MAX
#define DLL_NOT_IN_LIST			U32_MAX-1	/* if next, prev == DLL_STUB, then node is not in list */

#define DLL_SLOT_STATE			u32 dll_prev;			\
                       			u32 dll_next			
#define dll_Prev(structure_addr)	((structure_addr)->dll_prev)
#define dll_Next(structure_addr)	((structure_addr)->dll_next)
#define dll_InList(structure_addr)	((structure_addr)->dll_next != DLL_NOT_IN_LIST)

#define DLL2_SLOT_STATE			u32 dll2_prev;			\
                       			u32 dll2_next			
#define dll2_Prev(structure_addr)	((structure_addr)->dll2_prev)
#define dll2_Next(structure_addr)	((structure_addr)->dll2_next)
#define dll2_InList(structure_addr)	((structure_addr)->dll2_next != DLL_NOT_IN_LIST)

#define DLL3_SLOT_STATE			u32 dll3_prev;			\
                       			u32 dll3_next			
#define dll3_Prev(structure_addr)	((structure_addr)->dll3_prev)
#define dll3_Next(structure_addr)	((structure_addr)->dll3_next)
#define dll3_InList(structure_addr)	((structure_addr)->dll3_next != DLL_NOT_IN_LIST)

struct dll
{
	u32 	count;
	u32 	first;
	u32 	last;
	u64 	slot_size;
	u64	prev_offset;
	u64	next_offset;
};

/* initalize linked list  */
struct dll		  dll_InitInternal(const u64 slot_size, const u64 prev_offset, const u64 next_offset);
#define dll_Init(STRUCT)  dll_InitInternal(sizeof(STRUCT), (u64) &((STRUCT *)0)->dll_prev, (u64) &((STRUCT *)0)->dll_next)
#define dll2_Init(STRUCT) dll_InitInternal(sizeof(STRUCT), (u64) &((STRUCT *)0)->dll2_prev, (u64) &((STRUCT *)0)->dll2_next)
#define dll3_Init(STRUCT) dll_InitInternal(sizeof(STRUCT), (u64) &((STRUCT *)0)->dll3_prev, (u64) &((STRUCT *)0)->dll3_next)
/* flush list */
void			dll_Flush(struct dll *dll);
/* append to list */
void			dll_Append(struct dll *dll, void *array, const u32 index);
/* prepend to list */
void			dll_Prepend(struct dll *dll, void *array, const u32 index);
/* remove from list */
void			dll_Remove(struct dll *dll, void *array, const u32 index);
/* set slot state to indicate it is not in some list; WARNING: must not be in list!  */
void			dll_SlotSetNotInList(struct dll *dll, void *slot);

/*
nll
=== 
net_list : A set of intertwined lists. Each node represents a node in two lists, and adding or removing a net_node
affects two lists simultaneously. It is up to the user of the data structure to construct identifying data in their
own structures to identify who/what owns what parts of the node, i.e. which list owns prev_0, next_0 and
which list owns prev_1, next_1.
*/

#define NLL_NULL			0

#define NLL_SLOT_STATE			u32	nll_next[2];		\
					u32	nll_prev[2];		\
					POOL_SLOT_STATE

//#define NLL_PREV(structure_addr)	((structure_addr)->dll_prev)
//#define NLL_NEXT(structure_addr)	((structure_addr)->dll_next)
//#define NLL_IN_LIST(structure_addr)	((structure_addr)->dll_next != DLL_NOT_IN_LIST)

struct nll
{
	struct pool	pool;

	/* user provided identifier methods: 
	 *
	 * 	cur_index: the index used for the owner of prev[cur_index], next[cur_index]
	 * 	cur_node: [cur_node, cur_index], the node in a specific list we are inspecting
	 * 	prev/next_node: the node in which we wish to identify index owned by the owner of [cur_node, cur_index]
	 * 	returns the index owned by [cur_node, cur_index] in the prev/next index
	 *
	 * When removing a node in the middle of two lists, we must know how to identify the indices of the two
	 * corresponding lists within the prev/next nodes
	 */
	u32	(*index_in_prev_node)(struct nll *net, void **prev_node, const void *cur_node, const u32 cur_index);
	u32	(*index_in_next_node)(struct nll *net, void **next_node, const void *cur_node, const u32 cur_index);

	u32	heap_allocated;
	u64	next_offset;
	u64	prev_offset;
};

/* allocate net_list memory. If mem != NULL, the list cannot be growable. If mem == NULL, heap allocation is made */
struct nll 	nll_AllocInternal(struct arena *mem, 
				const u32 initial_length, 
				const u64 data_size, 
				const u64 pool_slot_offset, 
				const u64 next_offset, 
				const u64 prev_offset, 
				u32 (*index_in_prev_node)(struct nll *, void **, const void *, const u32),
	       			u32 (*index_in_next_node)(struct nll *, void **, const void *, const u32),
				const u32 growable);
#define 	nll_Alloc(mem, initial_length, STRUCT, index_in_prev_node, index_in_next_node, growable)  nll_AllocInternal(mem, initial_length, sizeof(STRUCT), (u64) &((STRUCT *)0)->slot_allocation_state, (u64) &((STRUCT *)0)->nll_next, (u64) &((STRUCT *)0)->nll_prev, index_in_prev_node, index_in_next_node, growable)
/* free allocated resources */
void		nll_Dealloc(struct nll *net);
/* flush / reset net_list  */
void 		nll_Flush(struct nll *net);
/* reserve a memory node and return the memory index and set the node's links. next_0 and next_1 MUST always be 
 * the first nodes, or static node references, of the two corresponding lists owning the node */
struct slot	nll_Add(struct nll *net, void *data, const u32 next_0, const u32 next_1);
/* free a memory node, updating both lists it is a part of */
void 		nll_Remove(struct nll *net, const u32 index);
/* get the node address given its index */
void *		nll_Address(const struct nll *net, const u32 index);
/* get the node index given its address  */
u32		nll_Index(const struct nll *net, const void *address);

#ifdef __cplusplus
} 
#endif

#endif
