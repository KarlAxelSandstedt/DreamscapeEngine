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

#ifndef __HIERARCHY_INDEX_H__
#define __HIERARCHY_INDEX_H__

#include "ds_allocator.h"

#ifdef __cplusplus
extern "C" { 
#endif

/*
hierarchy index
===============
Intrusive hierarchy data structure for indexed structures supporting 31 bit indices. 
*/


/* root stub is an internal node of the hierarchy; using this we can simplify Logic and have a nice "NULL" index to use */
#define HI_ROOT_STUB_INDEX	0
#define	HI_NULL_INDEX		0	
#define HI_ORPHAN_STUB_INDEX	1

#define HI_SLOT_STATE	u32	hi_parent;	\
			u32	hi_next;	\
			u32	hi_prev;	\
			u32	hi_first;	\
			u32	hi_last;	\
			u32	hi_child_count;	\
			POOL_SLOT_STATE

struct hi
{
	struct pool	pool;
	u32		parent_offset;	
	u32		next_offset;	
	u32		prev_offset;	
	u32		first_offset;	
	u32		last_offset;	
	u32		child_count_offset;	
};

/* alloc and init hierarchy resources (on arena if non-NULL and non-growable), returns { 0 } on failure */
struct hi 	hi_AllocInternal(struct arena *mem
			, const u32 length
			, const u64 data_size
			, const u32 growable
			, const u32 slot_allocation_offset	
			, const u32 parent_offset	
			, const u32 next_offset	
			, const u32 prev_offset	
			, const u32 first_offset	
			, const u32 last_offset	
			, const u32 child_count_offset);
#define		hi_Alloc(mem, length, STRUCT, growable)			\
		hi_AllocInternal(mem,					\
			, length					\
			, sizeof(STRUCT)				\
			, &((STRUCT *) 0)->slot_allocation_state	\
			, &((STRUCT *) 0)->hi_parent			\
			, &((STRUCT *) 0)->hi_next			\
			, &((STRUCT *) 0)->hi_prev			\
			, &((STRUCT *) 0)->hi_first			\
			, &((STRUCT *) 0)->hi_last			\
			, &((STRUCT *) 0)->hi_child_count)
/* free hierarchy allocated on the heap */
void		hi_Dealloc(struct hi *hi);
/* flush or reset hierarchy */
void		hi_Flush(struct hi *hi);
/* Allocate a hierarchy node and return the allocation slot on success, RETURNS (0, NULL) on failure */
struct slot	hi_Add(struct hi *hi, const u32 parent_index);
/* Deallocate a hierarchy node and its whole sub-hierarchy */
void 		hi_Remove(struct arena *tmp, struct hi *hi, const u32 node_index);
/* node's children (and their subtrees) are adopted by node's parent, and node's new parent becomes parent_index */
void 		hi_AdoptNodeExclusive(struct hi *hi, const u32 node_index, const u32 new_parent_index);
/* node's subtree is removed from current parent and added to new parent*/
void 		hi_AdoptNode(struct hi *hi, const u32 node_index, const u32 new_parent_index);
/* apply a custom free to and deallocate a hierarchy node and its whole sub-hierarchy; the custom free takes in the index to remove */
void		hi_ApplyCustomFreeAndRemove(struct arena *tmp, struct hi *hi, const u32 node_index, void (*custom_free)(const struct hi *hi, const u32, void *data), void *data);
/* Return node address corresponding to index */
void *		hi_Address(const struct hi *hi, const u32 node_index);

/*
 * hierarchy_index_iterator: iterator for traversing a supplied node and it's entire sub-hierarchy in the given
 * hierarchy. SHOULD always be supplied an arena large enough to store the sub-hierarchy. To check if memory
 * ran out, and forced heap allocations took place, simply run the following check after you are done iterating.
 *
 * if (iterator.forced_malloc)
 * {
 *	LOG_MESSAGE()
 * }
 */
struct hiIterator
{
	struct hi *hi; 	/* hierarchy index */
	struct arena *mem;		/* iterator memory */
	u64 stack_len;  		/* max stack size  */
	u32 *stack;			/* index stack 	   */
	u64 count;			/* stack count 	   */
	//u32 next;			/* next index	   */
	u32 forced_malloc;		/* internal state to use heap allocation if arena runs out of memory */
};

/* Setup hierarchy iterator at the given node root */
struct hiIterator	hi_IteratorInit(struct arena *ar_alias, struct hi *hi, const u32 root);
/* release / pop any used memory by iterator */
void 			hi_IteratorRelease(struct hiIterator *it);
/* Given it->count > 0, return the next index in the iterator */
u32 			hi_IteratorPeek(struct hiIterator *it);
/* Given it->count > 0, return the next index in the iterator, and push any new links (depth-first) related to the index */
u32 			hi_IteratorNextDf(struct hiIterator *it);
/* Given it->count > 0, skip the whole subtree corresponding to the next index in the iterator, and push the subtree's next sibling, if it exists. */
void			hi_IteratorSkip(struct hiIterator *it);

#ifdef __cplusplus
} 
#endif

#endif
