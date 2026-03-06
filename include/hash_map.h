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

#ifndef __DS_HASH_MAP_H__
#define __DS_HASH_MAP_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_allocator.h"
#include "ds_serialize.h"

#define HASH_NULL 	POOL_NULL	

/*
hashMap
=======
hash map storing mapping a key to a set of possible indices. User dereferences
indices to check for equality between identifiers 
*/

struct ds_HashMap
{
	u32 *		        hash;
	u32 *		        index;
	u32		            hash_len;
	u32		            index_len;
	u32		            hash_mask;
	u32		            growable;
	struct ds_MemSlot	mem_hash;
	struct ds_MemSlot	mem_index;
};

/* allocate hash map on heap if mem == NULL, otherwise push memory onto arena. On failure, returns NULL  */
struct ds_HashMap	ds_HashMapAlloc(struct arena *mem, const u32 hash_len, const u32 index_len, const u32 growable);
/* free hash map memory */
void		        ds_HashMapFree(struct ds_HashMap *map);
/* flush / reset the hash map, removing any allocations within it */
void		        ds_HashMapFlush(struct ds_HashMap *map);
/* serialize hash map into stream  */
void 		        ds_HashMapSerialize(struct ss *ss, const struct ds_HashMap *map);
/* deserialize and construct hash_map on arena if defined, otherwise alloc on heap. On failure, returns NULL  */
struct ds_HashMap	ds_HashMapDeserialize(struct arena *mem, struct ss *ss, const u32 growable);
/* add the hash(key)-index pair to the hash map. return 1 on success, 0 on out-of-memory. */
u32		            ds_HashMapAdd(struct ds_HashMap *map, const u32 hash, const u32 index);
/* remove  hash(key)-index pair to the hash map. If the pair is not found, do nothing. */
void		        ds_HashMapRemove(struct ds_HashMap *map, const u32 hash, const u32 index);
/* Get the first (hash,index) pair of the map. If HASH_NULL is returned, no more pairs exist */
u32		            ds_HashMapFirst(const struct ds_HashMap *map, const u32 hash);
/* Get the next (hash,index) pair of the map. If HASH_NULL is returnsd, no more pairs exist */
u32		            ds_HashMapNext(const struct ds_HashMap *map, const u32 index);

#ifdef __cplusplus
} 
#endif

#endif
