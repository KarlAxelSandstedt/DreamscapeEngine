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

#ifndef __ALLOCATOR_DEBUG_H__
#define __ALLOCATOR_DEBUG_H__

#include "ds_common.h"

#if defined(DS_ASAN)

#include "sanitizer/asan_interface.h"
#include "bitVector.h"

/*
 * allocator_debug_index: poison/unpoison slots in index allocators.
 */
struct allocator_debug_index
{
	const u8 *	array;
	struct bitVec	poisoned;	
	u32		slot_count;
	u32		max_unpoisoned_count;
	u64		slot_size;
	u64		slot_header_size;
	u64		slot_header_offset;
};

struct allocator_debug_index allocator_debug_index_alloc(const u8 *array, const u32 slot_count, const u64 slot_size, const u64 slot_header_size, const u64 slot_header_offset);
void allocator_debug_index_free(struct allocator_debug_index *debug);
void allocator_debug_index_flush(struct allocator_debug_index *debug);
void allocator_debug_index_poison(struct allocator_debug_index *debug, const u32 index);
void allocator_debug_index_unpoison(struct allocator_debug_index *debug, const u32 index);
void allocator_debug_index_alias_and_repoison(struct allocator_debug_index *debug, const u8 *reallocated_array, const u32 new_slot_count);

#define ALLOCATOR_DEBUG_INDEX_STRUCT	struct allocator_debug_index __allocator_poisoner
#define ALLOCATOR_DEBUG_INDEX_ALLOC(structure_addr, array, slot_count, slot_size, slot_header_size, slot_header_offset)	(structure_addr)->__allocator_poisoner = allocator_debug_index_alloc(array, slot_count, slot_size, slot_header_size, slot_header_offset)
#define ALLOCATOR_DEBUG_INDEX_FREE(structure_addr)	allocator_debug_index_free(&(structure_addr)->__allocator_poisoner)
#define ALLOCATOR_DEBUG_INDEX_FLUSH(structure_addr)	allocator_debug_index_flush(&(structure_addr)->__allocator_poisoner)
#define ALLOCATOR_DEBUG_INDEX_POISON(structure_addr, index) allocator_debug_index_poison(&(structure_addr)->__allocator_poisoner, index)
#define ALLOCATOR_DEBUG_INDEX_UNPOISON(structure_addr, index) allocator_debug_index_unpoison(&(structure_addr)->__allocator_poisoner, index)
#define ALLOCATOR_DEBUG_INDEX_ALIAS_AND_REPOISON(structure_addr, reallocated_array, new_slot_count) allocator_debug_index_alias_and_repoison(&(structure_addr)->__allocator_poisoner, reallocated_array, new_slot_count)

#else

#define ALLOCATOR_DEBUG_INDEX_STRUCT
#define ALLOCATOR_DEBUG_INDEX_ALLOC(structure_addr, array, slot_count, slot_size, slot_header_size, slot_header_offset)	
#define ALLOCATOR_DEBUG_INDEX_FREE(structure_addr)	
#define ALLOCATOR_DEBUG_INDEX_FLUSH(structure_addr)	
#define ALLOCATOR_DEBUG_INDEX_POISON(structure_addr, index) 
#define ALLOCATOR_DEBUG_INDEX_UNPOISON(structure_addr, index) 
#define ALLOCATOR_DEBUG_INDEX_ALIAS_AND_REPOISON(structure_addr, array, new_slot_count)

#endif

#endif
