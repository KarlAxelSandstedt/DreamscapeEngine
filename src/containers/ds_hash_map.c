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

#include "ds_base.h"
#include "ds_hash_map.h"


struct ds_HashMap ds_HashMapAllocEx(struct arena *mem, const u32 hash_len, const u32 index_len, const u32 growable)
{
	ds_Assert(PowerOfTwoCheck(hash_len) && hash_len <= index_len && (index_len >> 31) == 0);
	ds_Assert(!(mem && growable));

	struct ds_HashMap map = 
	{ 
		.hash_len = hash_len,
		.index_len = index_len,
		.hash_mask = hash_len - 1,
		.growable = growable,
	};

	const u64 mem_left = (mem) ? mem->mem_left : 0;
	if (mem)
	{
		map.hash = ArenaPush(mem, map.hash_len * sizeof(u32)); 
		map.index = ArenaPush(mem, map.index_len * sizeof(u32)); 
	}
	else
	{
		map.hash = ds_Alloc(&map.mem_hash, map.hash_len * sizeof(u32), HUGE_PAGES);
		map.index = ds_Alloc(&map.mem_index, map.index_len * sizeof(u32), HUGE_PAGES);
	}

	if (!map.hash || !map.index)
	{
		if (mem)
		{
			ArenaPopPacked(mem, mem_left - mem->mem_left);
		}
		ds_Free(&map.mem_hash);
		ds_Free(&map.mem_index);
		return (struct ds_HashMap) { .growable = growable };
	}

	return map;
}

struct ds_HashMap ds_HashMapAlloc(struct arena *mem, const u32 hash_len, const u32 index_len, const u32 growable)
{
	ds_Assert(hash_len && index_len && (hash_len >> 31) == 0);

	u32 hash_len_used;
	u32 index_len_used;
	if (mem)
	{
		hash_len_used = (u32) PowerOfTwoCeil(hash_len);
		index_len_used = index_len;
	}
	else
	{
		/* use the whole heap allocation */
		hash_len_used = (u32) PowerOfTwoCeil( ds_AllocSizeCeil(hash_len * sizeof(u32))  / sizeof(u32) );
		index_len_used = (u32) ds_AllocSizeCeil(index_len * sizeof(u32)) / sizeof(u32);
	}
	index_len_used = (hash_len_used <= index_len_used)
		? index_len_used
		: hash_len_used;

	struct ds_HashMap map = ds_HashMapAllocEx(mem, hash_len_used, index_len_used, growable);
	for (u32 i = 0; i < map.hash_len; ++i)
	{
		map.hash[i] = HASH_NULL;
	}

	return map;
}

void ds_HashMapDealloc(struct ds_HashMap *map)
{
	if (map->mem_hash.address)
	{
		ds_Free(&map->mem_hash);
		ds_Free(&map->mem_index);
	}	
}

void ds_HashMapFlush(struct ds_HashMap *map)
{
	for (u32 i = 0; i < map->hash_len; ++i)
	{
		map->hash[i] = HASH_NULL;
	}
}

u64 ds_HashMapSerializeSize(const struct ds_HashMap *map)
{
	return 2*sizeof(u32) + ((u64) map->hash_len + map->index_len) * sizeof(u32);
}

void ds_HashMapSerialize(struct ss *ss, const struct ds_HashMap *map)
{
	ds_Assert(ss->bit_index % 8 == 0);
	ds_Assert(ds_HashMapSerializeSize(map) <= ss_BytesLeft(ss));

	ss_WriteU32Le(ss, map->hash_len);
	ss_WriteU32Le(ss, map->index_len);
	ss_WriteU32LeN(ss, map->hash, map->hash_len);
	ss_WriteU32LeN(ss, map->index, map->index_len);
}

u32 ds_HashMapTryDeserialize(struct arena *mem, struct ss *ss, struct ds_HashMap *map, const u32 growable)
{
	ds_Assert(ss->bit_index % 8 == 0);

	*map = (struct ds_HashMap) { .growable = growable };
	if (ss_BytesLeft(ss) < 2*sizeof(u32))
	{
		return 0;
	}

	const u64 bit_index = ss->bit_index;
	const u32 hash_len = ss_ReadU32Le(ss);
	const u32 index_len = ss_ReadU32Le(ss);
	/* hash_len is a mask base, so it must be an exact power of two; u64 sum: corrupt lengths must not overflow */
	if (!hash_len || !PowerOfTwoCheck(hash_len) || hash_len > index_len || (index_len >> 31)
		|| (u64) hash_len + index_len > ss_BytesLeft(ss) / sizeof(u32))
	{
		ss->bit_index = bit_index;
		return 0;
	}

	*map = ds_HashMapAllocEx(mem, hash_len, index_len, growable);
	if (!map->hash)
	{
		ss->bit_index = bit_index;
		return 0;
	}

	ss_ReadU32LeN(map->hash, ss, hash_len);
	ss_ReadU32LeN(map->index, ss, index_len);
	return 1;
}

u32 ds_HashMapAdd(struct ds_HashMap *map, const u32 hash, const u32 index)
{
	ds_Assert(index >> 31 == 0);

	if (map->index_len <= index)
	{
		if (map->growable)
		{
			const u64 size = (index < 2*map->index_len)
				? 2*map->mem_index.size
				: (index+1) * sizeof(u32);
			map->index = ds_Realloc(&map->mem_index, size);
			map->index_len = (u32) size / sizeof(u32);
		}
		else
		{
			return 0;	
		}
	}

	const u32 h = hash & map->hash_mask;
	
	map->index[index] = map->hash[h];
	map->hash[h] = index;

	return 1;
}

void ds_HashMapRemove(struct ds_HashMap *map, const u32 hash, const u32 index)
{
	ds_Assert(index < map->index_len);

	const u32 h = hash & map->hash_mask;
	if (map->hash[h] == index)
	{
		map->hash[h] = map->index[index];
	}
	else
	{
		for (u32 i = map->hash[h]; i != HASH_NULL; i = ds_HashMapNext(map, i))
		{
			if (map->index[i] == index)
			{
				map->index[i] = map->index[index];	
				break;
			}
		}
	}

	/* Only for debug purposes  */
	map->index[index] = HASH_NULL;
}

u32 ds_HashMapFirst(const struct ds_HashMap *map, const u32 hash)
{
	return map->hash[hash & map->hash_mask];
}

u32 ds_HashMapNext(const struct ds_HashMap *map, const u32 index)
{
	ds_Assert(index < map->index_len);
	return map->index[index];
}
