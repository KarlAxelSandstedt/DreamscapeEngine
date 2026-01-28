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
#include "hash_map.h"

static const struct hashMap map_empty = { 0 };

struct hashMap HashMapAlloc(struct arena *mem, const u32 hash_len, const u32 index_len, const u32 growable)
{
	ds_Assert(hash_len && index_len && (hash_len >> 31) == 0);

	struct hashMap map = 
	{ 
		.hash = NULL,
		.index = NULL,
		.growable = growable,
	};

	if (mem)
	{
		map.hash_len = (u32) PowerOfTwoCeil(hash_len);
		map.index_len = index_len;
		map.hash = ArenaPush(mem, map.hash_len * sizeof(u32)); 
		map.index = ArenaPush(mem, map.index_len * sizeof(u32)); 
	}
	else
	{
		map.hash_len = (u32) PowerOfTwoCeil( ds_AllocSizeCeil(hash_len) );
		map.index_len = (u32) PowerOfTwoCeil( ds_AllocSizeCeil(index_len * sizeof(u32)) ) / sizeof(u32);
		map.hash = ds_Alloc(&map.mem_hash, map.hash_len * sizeof(u32), HUGE_PAGES);
		map.index = ds_Alloc(&map.mem_index, map.index_len * sizeof(u32), HUGE_PAGES);
	}

	if (!map.hash || !map.index)
	{
		if (map.hash)
		{
			ds_Free(&map.mem_hash);
		}
		return map_empty;
	}

	ds_Assert(PowerOfTwoCheck(map.hash_len));
	map.hash_mask = map.hash_len-1;

	for (u32 i = 0; i < map.hash_len; ++i)
	{
		map.hash[i] = HASH_NULL;
	}

	return map;
}

void HashMapFree(struct hashMap *map)
{
	if (map->mem_hash.address)
	{
		ds_Free(&map->mem_hash);
		ds_Free(&map->mem_index);
	}	
}

void HashMapFlush(struct hashMap *map)
{
	for (u32 i = 0; i < map->hash_len; ++i)
	{
		map->hash[i] = HASH_NULL;
	}
}

void HashMapSerialize(struct serialStream *ss, const struct hashMap *map)
{
	if ((2 + map->hash_len + map->index_len) * sizeof(u32) <= ss_BytesLeft(ss))
	{
		ss_WriteU32Be(ss, map->hash_len);
		ss_WriteU32Be(ss, map->index_len);
		ss_WriteU32BeN(ss, map->hash, map->hash_len);
		ss_WriteU32BeN(ss, map->index, map->index_len);
	}
}

struct hashMap HashMapDeserialize(struct arena *mem, struct serialStream *ss, const u32 growable)
{
	ds_Assert(!(mem && growable));
	if (2 * sizeof(u32) > ss_BytesLeft(ss))
	{
		Log(T_SYSTEM, S_ERROR, "Deserializing hash map past byte boundary: Trying to read 8B with %luB left.", ss_BytesLeft(ss));
		return map_empty;
	}


	u32 *hash;
	u32 *index;
	struct hashMap map;

	const u32 hash_len = ss_ReadU32Be(ss);
	const u32 index_len = ss_ReadU32Be(ss);

	if (mem)
	{
		map.hash_len = hash_len;
		map.index_len = index_len;
		ArenaPushRecord(mem);
		map.hash = ArenaPush(mem, map.hash_len * sizeof(u32));
		map.index = ArenaPush(mem, map.index_len * sizeof(u32));
		if (!map.index || !map.hash)
		{
			ArenaPopRecord(mem);
			return map_empty;
		}
	}
	else
	{
		map.hash_len = (u32) PowerOfTwoCeil( ss_ReadU32Be(ss) );
		map.index_len = (u32) PowerOfTwoCeil( ds_AllocSizeCeil( ss_ReadU32Be(ss) * sizeof(u32)) ) / sizeof(u32);
		map.hash = (map.hash_len * sizeof(u32) > 1024*1024) 
			? ds_Alloc(&map.mem_hash, hash_len * sizeof(u32), HUGE_PAGES)
			: ds_Alloc(&map.mem_hash, hash_len * sizeof(u32), NO_HUGE_PAGES);
		if (!map.hash)
		{
			return map_empty;
		}

		map.index = (map.index_len * sizeof(u32) > 512*1024)
			? ds_Alloc(&map.mem_index, index_len * sizeof(u32), HUGE_PAGES)
			: ds_Alloc(&map.mem_index, index_len * sizeof(u32), NO_HUGE_PAGES);
		if (!map.index)
		{
			ds_Free(&map.mem_hash);
			return map_empty;
		}
	}

	map.growable = growable;
	map.hash_mask = map.hash_len-1;

	if (hash_len + index_len * sizeof(u32) > ss_BytesLeft(ss))
	{
		if (map.mem_index.address)
		{
			ds_Free(&map.mem_hash);
			ds_Free(&map.mem_index);
		}
		Log(T_SYSTEM, S_ERROR, "Deserializing hash map past byte boundary: Trying to read %luB with %luB left.", hash_len + index_len * sizeof(u32), ss_BytesLeft(ss));
		return map_empty;
	}

		ss_ReadU32BeN(map.hash, ss, hash_len);
		ss_ReadU32BeN(map.index, ss, index_len);

	return map;
}

u32 HashMapAdd(struct hashMap *map, const u32 key, const u32 index)
{
	ds_Assert(index >> 31 == 0);

	if (map->index_len <= index)
	{
		if (map->growable)
		{
			map->index_len = (u32) PowerOfTwoCeil(index+1);
			map->index = ds_Realloc(&map->mem_index, map->index_len * sizeof(u32));
			ds_Assert(map->index_len * sizeof(u32) == map->mem_index.size);
		}
		else
		{
			return 0;	
		}
	}

	const u32 h = key & map->hash_mask;
	
	map->index[index] = map->hash[h];
	map->hash[h] = index;
	return 1;
}

void HashMapRemove(struct hashMap *map, const u32 key, const u32 index)
{
	ds_Assert(index < map->index_len);

	const u32 h = key & map->hash_mask;
	if (map->hash[h] == index)
	{
		map->hash[h] = map->index[index];
	}
	else
	{
		for (u32 i = map->hash[h]; i != HASH_NULL; i = HashMapNext(map, i))
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

u32 HashMapFirst(const struct hashMap *map, const u32 key)
{
	return map->hash[key & map->hash_mask];
}

u32 HashMapNext(const struct hashMap *map, const u32 index)
{
	return (index < map->index_len)
		? map->index[index]
		: HASH_NULL;
}

u64 KeyGenU32U32(const u32 k1, const u32 k2)
{
       return ((u64) k1 << 32) | (u64) k2;
}
