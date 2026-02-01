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
 * hash map storing mapping a key to a set of possible indices. User dereferences indices to check for equality between identifiers 
 */
struct hashMap
{
	u32 *		hash;
	u32 *		index;
	u32		hash_len;
	u32		index_len;
	u32		hash_mask;
	u32		growable;
	struct memSlot	mem_hash;
	struct memSlot	mem_index;
};

/* allocate hash map on heap if mem == NULL, otherwise push memory onto arena. On failure, returns NULL  */
struct hashMap	HashMapAlloc(struct arena *mem, const u32 hash_len, const u32 index_len, const u32 growable);
/* free hash map memory */
void		HashMapFree(struct hashMap *map);
/* flush / reset the hash map, removing any allocations within it */
void		HashMapFlush(struct hashMap *map);
/* serialize hash map into stream  */
void 		HashMapSerialize(struct serialStream *ss, const struct hashMap *map);
/* deserialize and construct hash_map on arena if defined, otherwise alloc on heap. On failure, returns NULL  */
struct hashMap	HashMapDeserialize(struct arena *mem, struct serialStream *ss, const u32 growable);
/* add the key-index pair to the hash map. return 1 on success, 0 on out-of-memory. */
u32		HashMapAdd(struct hashMap *map, const u32 key, const u32 index);
/* remove  key-index pair to the hash map. If the pair is not found, do nothing. */
void		HashMapRemove(struct hashMap *map, const u32 key, const u32 index);
/* Get the first (key,index) pair of the map. If HASH_NULL is returned, no more pairs exist */
u32		HashMapFirst(const struct hashMap *map, const u32 key);
/* Get the next (key,index) pair of the map. If HASH_NULL is returnsd, no more pairs exist */
u32		HashMapNext(const struct hashMap *map, const u32 index);

/* keygen methods */
u64		KeyGenU32U32(const u32 k1, const u32 k2);

#ifdef __cplusplus
} 
#endif

#endif
