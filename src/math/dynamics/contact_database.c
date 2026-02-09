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

#include "dynamics.h"

static u32 cdb_IndexInPreviousConctactNode(struct nll *net, void **prev_node, const void *cur_node, const u32 cur_index)
{
	ds_Assert(cur_index <= 1);
	const struct contact *c = cur_node;
	const u32 body = (1-cur_index) * CONTACT_KEY_TO_BODY_0(c->key) + cur_index * CONTACT_KEY_TO_BODY_1(c->key);
	
	*prev_node = nll_Address(net, c->nll_prev[cur_index]);
	const u64 key = ((struct contact *) *prev_node)->key;
	ds_Assert(c->nll_prev[cur_index] == NLL_NULL || body == CONTACT_KEY_TO_BODY_0(key) || body == CONTACT_KEY_TO_BODY_1(key));
	return (body == CONTACT_KEY_TO_BODY_0(key))
		? 0
		: 1;
}

static u32 cdb_IndexInNextConctactNode(struct nll *net, void **next_node, const void *cur_node, const u32 cur_index)
{
	ds_Assert(cur_index <= 1);
	const struct contact *c = cur_node;
	const u32 body = (1-cur_index) * CONTACT_KEY_TO_BODY_0(c->key) + cur_index * CONTACT_KEY_TO_BODY_1(c->key);
	
	*next_node = nll_Address(net, c->nll_next[cur_index]);
	const u64 key = ((struct contact *) *next_node)->key;
	ds_Assert(c->nll_next[cur_index] == NLL_NULL || body == CONTACT_KEY_TO_BODY_0(key) || body == CONTACT_KEY_TO_BODY_1(key));
	return (body == CONTACT_KEY_TO_BODY_0(key))
		? 0
		: 1;
}

struct cdb cdb_Alloc(struct arena *mem_persistent, const u32 size)
{
	struct cdb c_db = { 0 };
	ds_Assert(PowerOfTwoCheck(size));

	c_db.sat_cache_list = dll_Init(struct satCache);
	c_db.sat_cache_map = HashMapAlloc(NULL, size, size, GROWABLE);
	c_db.sat_cache_pool = PoolAlloc(NULL, size, struct satCache, GROWABLE);
	c_db.contact_net = nll_Alloc(NULL, size, struct contact, cdb_IndexInPreviousConctactNode, cdb_IndexInNextConctactNode, GROWABLE);
	c_db.contact_map = HashMapAlloc(NULL, size, size, GROWABLE);
	c_db.contacts_persistent_usage = BitVecAlloc(NULL, size, 0, 1);

	return c_db;
}

void cdb_Free(struct cdb *c_db)
{
	PoolDealloc(&c_db->sat_cache_pool);
	HashMapFree(&c_db->sat_cache_map);
	nll_Dealloc(&c_db->contact_net);
	HashMapFree(&c_db->contact_map);
	BitVecFree(&c_db->contacts_persistent_usage);
}

void cdb_Flush(struct cdb *c_db)
{
	cdb_ClearFrame(c_db);
	dll_Flush(&c_db->sat_cache_list);
	PoolFlush(&c_db->sat_cache_pool);
	HashMapFlush(&c_db->sat_cache_map);
	nll_Flush(&c_db->contact_net);
	HashMapFlush(&c_db->contact_map);
	BitVecClear(&c_db->contacts_persistent_usage, 0);
}

void cdb_Validate(const struct physicsPipeline *pipeline)
{
	for (u64 i = 0; i < pipeline->c_db.contacts_persistent_usage.bit_count; ++i)
	{
		if (BitVecGetBit(&pipeline->c_db.contacts_persistent_usage, i))
		{
			const struct contact *c = nll_Address(&pipeline->c_db.contact_net, (u32) i);
			ds_Assert(PoolSlotAllocated(c));

			//fprintf(stderr, "contact[%lu] (next[0], next[1], prev[0], prev[1]) : (%u,%u,%u,%u)\n",
			//	       i,
			//	       c->nll_next[0],	
			//	       c->nll_next[1],	
			//	       c->nll_prev[0],	
			//	       c->nll_prev[1]);

			const struct rigidBody *b1 = PoolAddress(&pipeline->body_pool, c->cm.i1);
			const struct rigidBody *b2 = PoolAddress(&pipeline->body_pool, c->cm.i2);

			u32 prev, k, found; 
			prev = NLL_NULL;
			k = b1->contact_first;
			found = 0;
			while (k != NLL_NULL)
			{
				if (k == i)
				{
					found = 1;
					break;
				}

				const struct contact *tmp = nll_Address(&pipeline->c_db.contact_net, k);
				ds_Assert(PoolSlotAllocated(tmp));
				if (CONTACT_KEY_TO_BODY_0(tmp->key) == c->cm.i1)
				{
					ds_Assert(prev == tmp->nll_prev[0]);
					prev = k;
					k = tmp->nll_next[0];
				}
				else
				{
					ds_Assert(CONTACT_KEY_TO_BODY_1(tmp->key) == c->cm.i1);
					ds_Assert(prev == tmp->nll_prev[1]);
					prev = k;
					k = tmp->nll_next[1];
				}
			}
			ds_Assert(found);
 
			prev = NLL_NULL;
			k = b2->contact_first;
			found = 0;
			while (k != NLL_NULL)
			{
				if (k == i)
				{
					found = 1;
					break;
				}

				const struct contact *tmp = nll_Address(&pipeline->c_db.contact_net, k);
				ds_Assert(PoolSlotAllocated(tmp));
				if (CONTACT_KEY_TO_BODY_0(tmp->key) == c->cm.i2)
				{
					ds_Assert(prev == tmp->nll_prev[0]);
					prev = k;
					k = tmp->nll_next[0];
				}
				else
				{
					ds_Assert(prev == tmp->nll_prev[1]);
					ds_Assert(CONTACT_KEY_TO_BODY_1(tmp->key) == c->cm.i2);
					prev = k;
					k = tmp->nll_next[1];
				}
			}
			ds_Assert(found);
		}
	}
}

void cdb_UpdatePersistentContactsUsage(struct cdb *c_db)
{
	ds_Assert(c_db->contacts_persistent_usage.block_count == c_db->contacts_frame_usage.block_count);
	for (u64 i = 0; i < c_db->contacts_frame_usage.block_count; ++i)
	{
		c_db->contacts_persistent_usage.bits[i] = c_db->contacts_frame_usage.bits[i];	
	}

	if (c_db->contacts_persistent_usage.bit_count < c_db->contact_net.pool.count_max)
	{
		const u64 low_bit = c_db->contacts_persistent_usage.bit_count;
		const u64 high_bit = c_db->contact_net.pool.count_max;
		BitVecIncreaseSize(&c_db->contacts_persistent_usage, c_db->contact_net.pool.length, 0);
		/* any new contacts that is in the appended region must now be set */
		for (u64 bit = low_bit; bit < high_bit; ++bit)
		{
			BitVecSetBit(&c_db->contacts_persistent_usage, bit, 1);
		}
	}
}

void cdb_ClearFrame(struct cdb *c_db)
{
	c_db->contacts_frame_usage.bits = NULL;
	c_db->contacts_frame_usage.bit_count = 0;
	c_db->contacts_frame_usage.block_count = 0;

	//fprintf(stderr, "count: %u\n", c_db->sat_cache_pool.count);
	for (u32 i = c_db->sat_cache_list.first; i != DLL_NULL; )
	{
		struct satCache *cache = PoolAddress(&c_db->sat_cache_pool, i);
		const u32 next = dll_Next(cache);
		if (cache->touched)
		{
			cache->touched = 0;
		}
		else
		{
			dll_Remove(&c_db->sat_cache_list, c_db->sat_cache_pool.buf, i);
			HashMapRemove(&c_db->sat_cache_map, (u32) cache->key, i);
			PoolRemove(&c_db->sat_cache_pool, i);
		}
		i = next;
	}
}

struct contact *cdb_ContactAdd(struct physicsPipeline *pipeline, const struct contactManifold *cm, const u32 i1, const u32 i2)
{
	u32 b1, b2;
	if (i1 < i2)
	{
		b1 = i1;
		b2 = i2;
	}
	else
	{
		b1 = i2;
		b2 = i1;
	}

	struct rigidBody *body1 = PoolAddress(&pipeline->body_pool, b1);
	struct rigidBody *body2 = PoolAddress(&pipeline->body_pool, b2);

	const u64 key = KeyGenU32U32(b1, b2);
	ds_Assert(b1 == CONTACT_KEY_TO_BODY_0(key));
	ds_Assert(b2 == CONTACT_KEY_TO_BODY_1(key));
	const u32 index = cdb_ContactLookupIndex(&pipeline->c_db, b1, b2);

	if (index == NLL_NULL)
	{
		/* smaller valued body owns slot 0, larger valued body owns slot 1 in node header */
		ds_Assert(PoolSlotAllocated(body1));
		ds_Assert(PoolSlotAllocated(body2));
		struct contact cpy =
		{
			.cm = *cm,
			.key = key,
			.cached_count = 0,
		};
		struct slot slot = nll_Add(&pipeline->c_db.contact_net, &cpy, body1->contact_first, body2->contact_first);
		const u32 ci = slot.index; 
		struct contact *c = slot.address; 

		body1->contact_first = ci;
		body2->contact_first = ci;

		HashMapAdd(&pipeline->c_db.contact_map, (u32) key, ci);

		if (ci < pipeline->c_db.contacts_frame_usage.bit_count)
		{
			BitVecSetBit(&pipeline->c_db.contacts_frame_usage, ci, 1);
		}
		PhysicsEventContactNew(pipeline, b1, b2);

		return c;
	}
	else
	{
		struct contact *c = nll_Address(&pipeline->c_db.contact_net, index);
		BitVecSetBit(&pipeline->c_db.contacts_frame_usage, index, 1);
		c->cm = *cm;
		return c;
	}
}

void cdb_ContactRemove(struct physicsPipeline *pipeline, const u64 key, const u32 index)
{
	struct contact *c = nll_Address(&pipeline->c_db.contact_net, index);
	struct rigidBody *body0 = PoolAddress(&pipeline->body_pool, (u32) CONTACT_KEY_TO_BODY_0(c->key));
	struct rigidBody *body1 = PoolAddress(&pipeline->body_pool, (u32) CONTACT_KEY_TO_BODY_1(c->key));
	
	if (body0->contact_first == index)
	{
		body0->contact_first = c->nll_next[0];
	}

	if (body1->contact_first == index)
	{
		body1->contact_first = c->nll_next[1];
	}

	PhysicsEventContactRemoved(pipeline, (u32) CONTACT_KEY_TO_BODY_0(c->key), (u32) CONTACT_KEY_TO_BODY_1(c->key));
	HashMapRemove(&pipeline->c_db.contact_map, (u32) key, index);
	nll_Remove(&pipeline->c_db.contact_net, index);
}

void cdb_BodyRemoveContacts(struct physicsPipeline *pipeline, const u32 body_index)
{
	struct rigidBody *body = PoolAddress(&pipeline->body_pool, body_index);
	u32 ci = body->contact_first;
	body->contact_first = NLL_NULL;
	while (ci != NLL_NULL)
	{
		struct contact *c = nll_Address(&pipeline->c_db.contact_net, ci);
		struct satCache *sat = SatCacheLookup(&pipeline->c_db, CONTACT_KEY_TO_BODY_0(c->key), CONTACT_KEY_TO_BODY_1(c->key));
		if (sat)
		{
			const u32 sat_index = PoolIndex(&pipeline->c_db.sat_cache_pool, sat);
			dll_Remove(&pipeline->c_db.sat_cache_list, pipeline->c_db.sat_cache_pool.buf, sat_index);
			HashMapRemove(&pipeline->c_db.sat_cache_map, (u32) c->key, sat_index);
			PoolRemove(&pipeline->c_db.sat_cache_pool, sat_index);
		}

		u32 next_i;
		if (body_index == CONTACT_KEY_TO_BODY_0(c->key))
		{
			next_i = 0;
			body = PoolAddress(&pipeline->body_pool, CONTACT_KEY_TO_BODY_1(c->key));
		}
		else
		{
			next_i = 1;
			body = PoolAddress(&pipeline->body_pool, CONTACT_KEY_TO_BODY_0(c->key));
		}

		if (body->contact_first == ci)
		{
			body->contact_first = c->nll_next[1-next_i];
		}
		const u32 ci_next = c->nll_next[next_i];

		PhysicsEventContactRemoved(pipeline, (u32) CONTACT_KEY_TO_BODY_0(c->key), (u32) CONTACT_KEY_TO_BODY_1(c->key));
		BitVecSetBit(&pipeline->c_db.contacts_persistent_usage, ci, 0);
		HashMapRemove(&pipeline->c_db.contact_map, (u32) c->key, ci);
		nll_Remove(&pipeline->c_db.contact_net, ci);
		ci = ci_next;
	}
}

void cdb_StaticRemoveContactsAndUpdateIslands(struct physicsPipeline *pipeline, const u32 static_index)
{
	ArenaPushRecord(&pipeline->frame);

	struct memArray arr = ArenaPushAlignedAll(&pipeline->frame, sizeof(u32), sizeof(u32));
	u32 *island = arr.addr;
	u32 island_count = 0;

	struct rigidBody *body = PoolAddress(&pipeline->body_pool, static_index);
	ds_Assert(body->island_index == ISLAND_STATIC);
	u32 ci = body->contact_first;
	body->contact_first = NLL_NULL;
	while (ci != NLL_NULL)
	{
		struct contact *c = nll_Address(&pipeline->c_db.contact_net, ci);
		u32 next_i;
		if (static_index == CONTACT_KEY_TO_BODY_0(c->key))
		{
			next_i = 0;
			body = PoolAddress(&pipeline->body_pool, CONTACT_KEY_TO_BODY_1(c->key));
		}
		else
		{
			next_i = 1;
			body = PoolAddress(&pipeline->body_pool, CONTACT_KEY_TO_BODY_0(c->key));
		}

		if (body->contact_first == ci)
		{
			body->contact_first = c->nll_next[1-next_i];
		}
		const u32 ci_next = c->nll_next[next_i];
		struct island *is = PoolAddress(&pipeline->is_db.island_pool, body->island_index);

		if ((is->flags & ISLAND_SPLIT) == 0)
		{
			if (island_count == arr.len)
			{
				LogString(T_SYSTEM, S_FATAL, "Stack OOM in cdb_StaticRemoveContactsAndUpdateIslands");
				FatalCleanupAndExit();
			}
			island[island_count++] = body->island_index;
			
			is->flags |= ISLAND_SPLIT;
			dll_Remove(&is->contact_list, pipeline->c_db.contact_net.pool.buf, ci);
		}

		PhysicsEventContactRemoved(pipeline, (u32) CONTACT_KEY_TO_BODY_0(c->key), (u32) CONTACT_KEY_TO_BODY_1(c->key));
		BitVecSetBit(&pipeline->c_db.contacts_persistent_usage, ci, 0);
		HashMapRemove(&pipeline->c_db.contact_map, (u32) c->key, ci);
		nll_Remove(&pipeline->c_db.contact_net, ci);
		ci = ci_next;
	}

	for (u32 i = 0; i < island_count; ++i)
	{
		struct island *is = PoolAddress(&pipeline->is_db.island_pool, island[i]);
		if (is->contact_list.count > 0)
		{
			isdb_SplitIsland(&pipeline->frame, pipeline, island[i]);
		}
		else
		{
			is->flags &= ~(ISLAND_SPLIT);
			if (!(is->flags & ISLAND_AWAKE))
			{
				PhysicsEventIslandAwake(pipeline, island[i]);	
			}
			is->flags |= ISLAND_SLEEP_RESET | ISLAND_AWAKE;
		}
	}

	ArenaPopRecord(&pipeline->frame);
}

struct contact *cdb_ContactLookup(const struct cdb *c_db, const u32 i1, const u32 i2)
{
	u32 b1, b2;
	if (i1 < i2)
	{
		b1 = i1;
		b2 = i2;
	}
	else
	{
		b1 = i2;
		b2 = i1;
	}

	const u64 key = KeyGenU32U32(b1, b2);
	for (u32 i = HashMapFirst(&c_db->contact_map, (u32) key); i != HASH_NULL; i = HashMapNext(&c_db->contact_map, i))
	{
		struct contact *c = nll_Address(&c_db->contact_net, i);
		if (c->key == key)
		{
			return c;
		}
	}

	return NULL;
}

u32 cdb_ContactLookupIndex(const struct cdb *c_db, const u32 i1, const u32 i2)
{
	u32 b1, b2;
	if (i1 < i2)
	{
		b1 = i1;
		b2 = i2;
	}
	else
	{
		b1 = i2;
		b2 = i1;
	}

	const u64 key = KeyGenU32U32(b1, b2);
	u32 ret = NLL_NULL;
	for (u32 i = HashMapFirst(&c_db->contact_map, (u32) key); i != HASH_NULL; i = HashMapNext(&c_db->contact_map, i))
	{
		struct contact *c = nll_Address(&c_db->contact_net, i);
		if (c->key == key)
		{
			ret = (u32) i;
			break;
		}
	}

	return ret;
}

void SatCacheAdd(struct cdb *c_db, const struct satCache *sat_cache)
{
	const u32 b0 = CONTACT_KEY_TO_BODY_0(sat_cache->key);
	const u32 b1 = CONTACT_KEY_TO_BODY_1(sat_cache->key);
	ds_Assert(SatCacheLookup(c_db, b0, b1) == NULL);

	//Breakpoint(b0 == 62 && b1 == 66);
	struct slot slot = PoolAdd(&c_db->sat_cache_pool);
	struct satCache *sat = slot.address;
	const u32 slot_allocation_state = sat->slot_allocation_state;
	*sat = *sat_cache;
	sat->slot_allocation_state = slot_allocation_state;
	dll_Append(&c_db->sat_cache_list, c_db->sat_cache_pool.buf, slot.index);
	HashMapAdd(&c_db->sat_cache_map, (u32) sat_cache->key, slot.index);
	sat->touched = 1;
}

struct satCache *SatCacheLookup(const struct cdb *c_db, const u32 b1, const u32 b2)
{
	ds_Assert(b1 < b2);
	const u64 key = KeyGenU32U32(b1, b2);
	struct satCache *ret = NULL;
	for (u32 i = HashMapFirst(&c_db->sat_cache_map, (u32) key); i != HASH_NULL; i = HashMapNext(&c_db->sat_cache_map, i))
	{
		struct satCache *sat = PoolAddress(&c_db->sat_cache_pool, i);
		if (sat->key == key)
		{
			ret = sat;
			break;
		}
	}

	return ret;
}
