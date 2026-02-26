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

#define XXH_INLINE_ALL
#include "xxhash.h"

#include "dynamics.h"

struct ds_ContactKey ds_ContactKeyCanonical(const u32 bodyA, const u32 shapeA, const u32 bodyB, const u32 shapeB)
{
    return (bodyA < bodyB)
        ? (struct ds_ContactKey) { .body0 = bodyA, .shape0 = shapeA, .body1 = bodyB, .shape1 = shapeB }
        : (struct ds_ContactKey) { .body0 = bodyB, .shape0 = shapeB, .body1 = bodyA, .shape1 = shapeA };
}

static u32 cdb_IndexInPreviousConctactNode(struct nll *net, void **prev_node, const void *cur_node, const u32 cur_index)
{
	ds_Assert(cur_index <= 1);
	const struct ds_Contact *c = cur_node;
	const u32 shape = (1-cur_index)*c->key.shape0 + cur_index*c->key.shape1;
	
	*prev_node = nll_Address(net, c->nll_prev[cur_index]);
    const struct ds_Contact *prev = *prev_node;
	ds_Assert(c->nll_prev[cur_index] == NLL_NULL || shape == prev->key.shape0 || shape == prev->key.shape1);
	return (shape == prev->key.shape0)
		? 0
		: 1;
}

static u32 cdb_IndexInNextConctactNode(struct nll *net, void **next_node, const void *cur_node, const u32 cur_index)
{
	ds_Assert(cur_index <= 1);
	const struct ds_Contact *c = cur_node;
	const u32 shape = (1-cur_index)*c->key.shape0 + cur_index*c->key.shape1;
	
	*next_node = nll_Address(net, c->nll_next[cur_index]);
    const struct ds_Contact *next = *next_node;
	ds_Assert(c->nll_next[cur_index] == NLL_NULL || shape == next->key.shape0 || shape == next->key.shape1);
	return (shape == next->key.shape0)
		? 0
		: 1;
}

struct cdb cdb_Alloc(struct arena *mem_persistent, const u32 size)
{
	struct cdb cdb = { 0 };
	ds_Assert(PowerOfTwoCheck(size));

	cdb.sat_cache_map = HashMapAlloc(NULL, size, size, GROWABLE);
	cdb.sat_cache_pool = PoolAlloc(NULL, 20000, struct sat_Cache, GROWABLE);
	//cdb.sat_cache_pool = PoolAlloc(NULL, size, struct sat_Cache, GROWABLE);
	cdb.contact_net = nll_Alloc(NULL, size, struct ds_Contact, cdb_IndexInPreviousConctactNode, cdb_IndexInNextConctactNode, GROWABLE);
	//cdb.contact_map = HashMapAlloc(NULL, size, size, GROWABLE);
	cdb.contact_map = HashMapAlloc(NULL, size, 20000, GROWABLE);
	cdb.contacts_persistent_usage = BitVecAlloc(NULL, size, 0, GROWABLE);

	return cdb;
}

void cdb_Free(struct cdb *cdb)
{
	PoolDealloc(&cdb->sat_cache_pool);
	HashMapFree(&cdb->sat_cache_map);
	nll_Dealloc(&cdb->contact_net);
	HashMapFree(&cdb->contact_map);
	BitVecFree(&cdb->contacts_persistent_usage);
}

void cdb_Flush(struct cdb *cdb)
{
	cdb_ClearFrame(cdb);
	PoolFlush(&cdb->sat_cache_pool);
	HashMapFlush(&cdb->sat_cache_map);
	nll_Flush(&cdb->contact_net);
	HashMapFlush(&cdb->contact_map);
	BitVecClear(&cdb->contacts_persistent_usage, 0);
}

//void cdb_Validate(const struct ds_RigidBodyPipeline *pipeline)
//{
//	for (u64 i = 0; i < pipeline->cdb.contacts_persistent_usage.bit_count; ++i)
//	{
//		if (BitVecGetBit(&pipeline->cdb.contacts_persistent_usage, i))
//		{
//			const struct ds_Contact *c = nll_Address(&pipeline->cdb.contact_net, (u32) i);
//			ds_Assert(PoolSlotAllocated(c));
//
//			//fprintf(stderr, "contact[%lu] (next[0], next[1], prev[0], prev[1]) : (%u,%u,%u,%u)\n",
//			//	       i,
//			//	       c->nll_next[0],	
//			//	       c->nll_next[1],	
//			//	       c->nll_prev[0],	
//			//	       c->nll_prev[1]);
//
//			const struct ds_RigidBody *b1 = PoolAddress(&pipeline->body_pool, c->cm.i1);
//			const struct ds_RigidBody *b2 = PoolAddress(&pipeline->body_pool, c->cm.i2);
//
//			u32 prev, k, found; 
//			prev = NLL_NULL;
//			k = b1->contact_first;
//			found = 0;
//			while (k != NLL_NULL)
//			{
//				if (k == i)
//				{
//					found = 1;
//					break;
//				}
//
//				const struct ds_Contact *tmp = nll_Address(&pipeline->cdb.contact_net, k);
//				ds_Assert(PoolSlotAllocated(tmp));
//				if (CONTACT_KEY_TO_BODY_0(tmp->key) == c->cm.i1)
//				{
//					ds_Assert(prev == tmp->nll_prev[0]);
//					prev = k;
//					k = tmp->nll_next[0];
//				}
//				else
//				{
//					ds_Assert(CONTACT_KEY_TO_BODY_1(tmp->key) == c->cm.i1);
//					ds_Assert(prev == tmp->nll_prev[1]);
//					prev = k;
//					k = tmp->nll_next[1];
//				}
//			}
//			ds_Assert(found);
// 
//			prev = NLL_NULL;
//			k = b2->contact_first;
//			found = 0;
//			while (k != NLL_NULL)
//			{
//				if (k == i)
//				{
//					found = 1;
//					break;
//				}
//
//				const struct ds_Contact *tmp = nll_Address(&pipeline->cdb.contact_net, k);
//				ds_Assert(PoolSlotAllocated(tmp));
//				if (CONTACT_KEY_TO_BODY_0(tmp->key) == c->cm.i2)
//				{
//					ds_Assert(prev == tmp->nll_prev[0]);
//					prev = k;
//					k = tmp->nll_next[0];
//				}
//				else
//				{
//					ds_Assert(prev == tmp->nll_prev[1]);
//					ds_Assert(CONTACT_KEY_TO_BODY_1(tmp->key) == c->cm.i2);
//					prev = k;
//					k = tmp->nll_next[1];
//				}
//			}
//			ds_Assert(found);
//		}
//	}
//}
//
//void cdb_UpdatePersistentContactsUsage(struct cdb *cdb)
//{
//	ds_Assert(cdb->contacts_persistent_usage.block_count == cdb->contacts_frame_usage.block_count);
//	for (u64 i = 0; i < cdb->contacts_frame_usage.block_count; ++i)
//	{
//		cdb->contacts_persistent_usage.bits[i] = cdb->contacts_frame_usage.bits[i];	
//	}
//
//	if (cdb->contacts_persistent_usage.bit_count < cdb->contact_net.pool.count_max)
//	{
//		const u64 low_bit = cdb->contacts_persistent_usage.bit_count;
//		const u64 high_bit = cdb->contact_net.pool.count_max;
//		BitVecIncreaseSize(&cdb->contacts_persistent_usage, cdb->contact_net.pool.length, 0);
//		/* any new contacts that is in the appended region must now be set */
//		for (u64 bit = low_bit; bit < high_bit; ++bit)
//		{
//			BitVecSetBit(&cdb->contacts_persistent_usage, bit, 1);
//		}
//	}
//}

void cdb_ClearFrame(struct cdb *cdb)
{
	cdb->contacts_frame_usage.bits = NULL;
	cdb->contacts_frame_usage.bit_count = 0;
	cdb->contacts_frame_usage.block_count = 0;	
}
//
//struct ds_Contact *cdb_ContactAdd(struct ds_RigidBodyPipeline *pipeline, const struct c_Manifold *cm, const u32 i1, const u32 i2)
//{
//	u32 b1, b2;
//	if (i1 < i2)
//	{
//		b1 = i1;
//		b2 = i2;
//	}
//	else
//	{
//		b1 = i2;
//		b2 = i1;
//	}
//
//	struct ds_RigidBody *body1 = PoolAddress(&pipeline->body_pool, b1);
//	struct ds_RigidBody *body2 = PoolAddress(&pipeline->body_pool, b2);
//
//	const u64 key = KeyGenU32U32(b1, b2);
//	ds_Assert(b1 == CONTACT_KEY_TO_BODY_0(key));
//	ds_Assert(b2 == CONTACT_KEY_TO_BODY_1(key));
//	const u32 index = cdb_ContactLookupIndex(&pipeline->cdb, b1, b2);
//
//	if (index == NLL_NULL)
//	{
//		/* smaller valued body owns slot 0, larger valued body owns slot 1 in node header */
//		ds_Assert(PoolSlotAllocated(body1));
//		ds_Assert(PoolSlotAllocated(body2));
//		struct ds_Contact cpy =
//		{
//			.cm = *cm,
//			.key = key,
//			.cached_count = 0,
//		};
//		struct slot slot = nll_Add(&pipeline->cdb.contact_net, &cpy, body1->contact_first, body2->contact_first);
//		const u32 ci = slot.index; 
//		struct ds_Contact *c = slot.address; 
//
//		body1->contact_first = ci;
//		body2->contact_first = ci;
//
//		HashMapAdd(&pipeline->cdb.contact_map, (u32) key, ci);
//
//		if (ci < pipeline->cdb.contacts_frame_usage.bit_count)
//		{
//			BitVecSetBit(&pipeline->cdb.contacts_frame_usage, ci, 1);
//		}
//		PhysicsEventContactNew(pipeline, b1, b2);
//
//		return c;
//	}
//	else
//	{
//		struct ds_Contact *c = nll_Address(&pipeline->cdb.contact_net, index);
//		BitVecSetBit(&pipeline->cdb.contacts_frame_usage, index, 1);
//		c->cm = *cm;
//		return c;
//	}
//}
//
//void cdb_ContactRemove(struct ds_RigidBodyPipeline *pipeline, const u64 key, const u32 index)
//{
//	struct ds_Contact *c = nll_Address(&pipeline->cdb.contact_net, index);
//	struct ds_RigidBody *body0 = PoolAddress(&pipeline->body_pool, (u32) CONTACT_KEY_TO_BODY_0(c->key));
//	struct ds_RigidBody *body1 = PoolAddress(&pipeline->body_pool, (u32) CONTACT_KEY_TO_BODY_1(c->key));
//	
//	if (body0->contact_first == index)
//	{
//		body0->contact_first = c->nll_next[0];
//	}
//
//	if (body1->contact_first == index)
//	{
//		body1->contact_first = c->nll_next[1];
//	}
//
//  TODO: if contact HULL-HULL, remove sat_cache
//  for (u32 i = cdb->sat_cache_list.first; i != DLL_NULL; )
//	{
//		struct sat_Cache *cache = PoolAddress(&cdb->sat_cache_pool, i);
//		const u32 next = dll_Next(cache);
//		if (cache->touched)
//		{
//			cache->touched = 0;
//		}
//		else
//		{
//            const u32 hash = (u32) XXH3_64bits(&cache->key, sizeof(cache->key));
//			dll_Remove(&cdb->sat_cache_list, cdb->sat_cache_pool.buf, i);
//			HashMapRemove(&cdb->sat_cache_map, hash, i);
//			PoolRemove(&cdb->sat_cache_pool, i);
//		}
//		i = next;
//	}
//	PhysicsEventContactRemoved(pipeline, (u32) CONTACT_KEY_TO_BODY_0(c->key), (u32) CONTACT_KEY_TO_BODY_1(c->key));
//	HashMapRemove(&pipeline->cdb.contact_map, (u32) key, index);
//	nll_Remove(&pipeline->cdb.contact_net, index);
//}
//
//void cdb_BodyRemoveContacts(struct ds_RigidBodyPipeline *pipeline, const u32 body_index)
//{
//	struct ds_RigidBody *body = PoolAddress(&pipeline->body_pool, body_index);
//	u32 ci = body->contact_first;
//	body->contact_first = NLL_NULL;
//	while (ci != NLL_NULL)
//	{
//		struct ds_Contact *c = nll_Address(&pipeline->cdb.contact_net, ci);
//		struct sat_Cache *sat = sat_CacheLookup(&pipeline->cdb, CONTACT_KEY_TO_BODY_0(c->key), CONTACT_KEY_TO_BODY_1(c->key));
//		if (sat)
//		{
//			const u32 sat_index = PoolIndex(&pipeline->cdb.sat_cache_pool, sat);
//			dll_Remove(&pipeline->cdb.sat_cache_list, pipeline->cdb.sat_cache_pool.buf, sat_index);
//			HashMapRemove(&pipeline->cdb.sat_cache_map, (u32) c->key, sat_index);
//			PoolRemove(&pipeline->cdb.sat_cache_pool, sat_index);
//		}
//
//		u32 next_i;
//		if (body_index == CONTACT_KEY_TO_BODY_0(c->key))
//		{
//			next_i = 0;
//			body = PoolAddress(&pipeline->body_pool, CONTACT_KEY_TO_BODY_1(c->key));
//		}
//		else
//		{
//			next_i = 1;
//			body = PoolAddress(&pipeline->body_pool, CONTACT_KEY_TO_BODY_0(c->key));
//		}
//
//		if (body->contact_first == ci)
//		{
//			body->contact_first = c->nll_next[1-next_i];
//		}
//		const u32 ci_next = c->nll_next[next_i];
//
//		PhysicsEventContactRemoved(pipeline, (u32) CONTACT_KEY_TO_BODY_0(c->key), (u32) CONTACT_KEY_TO_BODY_1(c->key));
//		BitVecSetBit(&pipeline->cdb.contacts_persistent_usage, ci, 0);
//		HashMapRemove(&pipeline->cdb.contact_map, (u32) c->key, ci);
//		nll_Remove(&pipeline->cdb.contact_net, ci);
//		ci = ci_next;
//	}
//}
//
//void cdb_StaticRemoveContactsAndUpdateIslands(struct ds_RigidBodyPipeline *pipeline, const u32 static_index)
//{
//	ArenaPushRecord(&pipeline->frame);
//
//	struct memArray arr = ArenaPushAlignedAll(&pipeline->frame, sizeof(u32), sizeof(u32));
//	u32 *island = arr.addr;
//	u32 island_count = 0;
//
//	struct ds_RigidBody *body = PoolAddress(&pipeline->body_pool, static_index);
//	ds_Assert(body->island_index == ISLAND_STATIC);
//	u32 ci = body->contact_first;
//	body->contact_first = NLL_NULL;
//	while (ci != NLL_NULL)
//	{
//		struct ds_Contact *c = nll_Address(&pipeline->cdb.contact_net, ci);
//		u32 next_i;
//		if (static_index == CONTACT_KEY_TO_BODY_0(c->key))
//		{
//			next_i = 0;
//			body = PoolAddress(&pipeline->body_pool, CONTACT_KEY_TO_BODY_1(c->key));
//		}
//		else
//		{
//			next_i = 1;
//			body = PoolAddress(&pipeline->body_pool, CONTACT_KEY_TO_BODY_0(c->key));
//		}
//
//		if (body->contact_first == ci)
//		{
//			body->contact_first = c->nll_next[1-next_i];
//		}
//		const u32 ci_next = c->nll_next[next_i];
//		struct island *is = PoolAddress(&pipeline->is_db.island_pool, body->island_index);
//
//		if ((is->flags & ISLAND_SPLIT) == 0)
//		{
//			if (island_count == arr.len)
//			{
//				LogString(T_SYSTEM, S_FATAL, "Stack OOM in cdb_StaticRemoveContactsAndUpdateIslands");
//				FatalCleanupAndExit();
//			}
//			island[island_count++] = body->island_index;
//			
//			is->flags |= ISLAND_SPLIT;
//			dll_Remove(&is->contact_list, pipeline->cdb.contact_net.pool.buf, ci);
//		}
//
//		PhysicsEventContactRemoved(pipeline, (u32) CONTACT_KEY_TO_BODY_0(c->key), (u32) CONTACT_KEY_TO_BODY_1(c->key));
//		BitVecSetBit(&pipeline->cdb.contacts_persistent_usage, ci, 0);
//		HashMapRemove(&pipeline->cdb.contact_map, (u32) c->key, ci);
//		nll_Remove(&pipeline->cdb.contact_net, ci);
//		ci = ci_next;
//	}
//
//	for (u32 i = 0; i < island_count; ++i)
//	{
//		struct island *is = PoolAddress(&pipeline->is_db.island_pool, island[i]);
//		if (is->contact_list.count > 0)
//		{
//			isdb_SplitIsland(&pipeline->frame, pipeline, island[i]);
//		}
//		else
//		{
//			is->flags &= ~(ISLAND_SPLIT);
//			if (!(is->flags & ISLAND_AWAKE))
//			{
//				PhysicsEventIslandAwake(pipeline, island[i]);	
//			}
//			is->flags |= ISLAND_SLEEP_RESET | ISLAND_AWAKE;
//		}
//	}
//
//	ArenaPopRecord(&pipeline->frame);
//}
//
//struct ds_Contact *cdb_ContactLookup(const struct cdb *cdb, const u32 i1, const u32 i2)
//{
//	u32 b1, b2;
//	if (i1 < i2)
//	{
//		b1 = i1;
//		b2 = i2;
//	}
//	else
//	{
//		b1 = i2;
//		b2 = i1;
//	}
//
//	const u64 key = KeyGenU32U32(b1, b2);
//	for (u32 i = HashMapFirst(&cdb->contact_map, (u32) key); i != HASH_NULL; i = HashMapNext(&cdb->contact_map, i))
//	{
//		struct ds_Contact *c = nll_Address(&cdb->contact_net, i);
//		if (c->key == key)
//		{
//			return c;
//		}
//	}
//
//	return NULL;
//}
//
//u32 cdb_ContactLookupIndex(const struct cdb *cdb, const u32 i1, const u32 i2)
//{
//	u32 b1, b2;
//	if (i1 < i2)
//	{
//		b1 = i1;
//		b2 = i2;
//	}
//	else
//	{
//		b1 = i2;
//		b2 = i1;
//	}
//
//	const u64 key = KeyGenU32U32(b1, b2);
//	u32 ret = NLL_NULL;
//	for (u32 i = HashMapFirst(&cdb->contact_map, (u32) key); i != HASH_NULL; i = HashMapNext(&cdb->contact_map, i))
//	{
//		struct ds_Contact *c = nll_Address(&cdb->contact_net, i);
//		if (c->key == key)
//		{
//			ret = (u32) i;
//			break;
//		}
//	}
//
//	return ret;
//}

struct slot sat_CacheAdd(struct cdb *cdb, const struct ds_ContactKey *key)
{
	ds_Assert(sat_CacheLookup(cdb, key).address == NULL);

    ds_AssertString(cdb->sat_cache_pool.count < cdb->sat_cache_pool.length, "Temporary: Currently   \
            we add new sat caches as we continuously dispatch contact calculation jobs to worker    \
            threads. Thus, if we happen to realloc here, we invalidate our thread pointers (most    \
            likely). The solution is to either make a pool allocator viable for multihtreading, or  \
            allocating all caches beforehand.");

    const u32 hash = (u32) XXH3_64bits(key, sizeof(*key));
	struct slot slot = PoolAdd(&cdb->sat_cache_pool);
	HashMapAdd(&cdb->sat_cache_map, hash, slot.index);

	struct sat_Cache *sat = slot.address;
    sat->key = *key;
    sat->type = SAT_CACHE_NOT_SET;

    return slot;
}

struct slot sat_CacheLookup(const struct cdb *cdb, const struct ds_ContactKey *key)
{
	ds_Assert(key->body0 < key->body1);
    const u32 hash = (u32) XXH3_64bits(key, sizeof(*key));
    struct slot slot = { .index = POOL_NULL, .address = NULL };
	for (u32 i = HashMapFirst(&cdb->sat_cache_map, hash); i != HASH_NULL; i = HashMapNext(&cdb->sat_cache_map, i))
	{
		struct sat_Cache *sat = PoolAddress(&cdb->sat_cache_pool, i);
		if (memcmp(&sat->key, key, sizeof(*key)) == 0)
		{
			slot.address = sat;
            slot.index = i;
			break;
		}
	}

	return slot;
}
