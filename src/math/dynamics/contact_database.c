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

u32 ds_ContactKeyHash(const struct ds_ContactKey *key)
{
    return (u32) XXH3_64bits(key, sizeof(struct ds_ContactKey));
}

u32 ds_ContactKeyEquivalence(const struct ds_ContactKey *keyA, const struct ds_ContactKey *keyB)
{
    return memcmp(keyA, keyB, sizeof(struct ds_ContactKey)) == 0;
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

struct cdb *cdb_Alloc(struct arena *mem_persistent, const u32 size)
{
    /* Note: requires allocation in persistent memory; thread structures initalizes pointers to pool storage... */
	struct cdb *cdb = ArenaPush(mem_persistent, sizeof(struct cdb));
	ds_Assert(PowerOfTwoCheck(size));

	sat_CacheTPoolAlloc(&cdb->sat_cache_pool, g_arch_config->logical_core_count, size);
	cdb->sat_cache_map = sat_CacheTHashMapAlloc(mem_persistent, &cdb->sat_cache_pool, 4096);

	cdb->contact_net = nll_Alloc(NULL, size, struct ds_Contact, cdb_IndexInPreviousConctactNode, cdb_IndexInNextConctactNode, GROWABLE);
	cdb->contact_map = ds_HashMapAlloc(NULL, size, size, GROWABLE);
	cdb->contact_persistent_usage = BitVecAlloc(NULL, size, 0, GROWABLE);
	cdb->sat_cache_persistent_usage = BitVecAlloc(NULL, size, 0, GROWABLE);

	return cdb;
}

void cdb_Free(struct cdb *cdb)
{
	sat_CacheTPoolDealloc(&cdb->sat_cache_pool);
	sat_CacheTHashMapDealloc(&cdb->sat_cache_map);
	nll_Dealloc(&cdb->contact_net);
	ds_HashMapDealloc(&cdb->contact_map);
	BitVecFree(&cdb->contact_persistent_usage);
	BitVecFree(&cdb->sat_cache_persistent_usage);
}

void cdb_Flush(struct cdb *cdb)
{
	cdb_ClearFrame(cdb);
	sat_CacheTPoolFlush(&cdb->sat_cache_pool);
	sat_CacheTHashMapFlush(&cdb->sat_cache_map);
	nll_Flush(&cdb->contact_net);
	ds_HashMapFlush(&cdb->contact_map);
	BitVecClear(&cdb->contact_persistent_usage, 0);
	BitVecClear(&cdb->sat_cache_persistent_usage, 0);
}

void cdb_Validate(const struct ds_RigidBodyPipeline *pipeline)
{
	for (u64 i = 0; i < pipeline->cdb->contact_persistent_usage.bit_count; ++i)
	{
		if (BitVecGetBit(&pipeline->cdb->contact_persistent_usage, i))
		{
			const struct ds_Contact *c = nll_Address(&pipeline->cdb->contact_net, (u32) i);
			ds_Assert(PoolSlotAllocated(c));

			//fprintf(stderr, "contact[%lu] (next[0], next[1], prev[0], prev[1]) : (%u,%u,%u,%u)\n",
			//	       i,
			//	       c->nll_next[0],	
			//	       c->nll_next[1],	
			//	       c->nll_prev[0],	
			//	       c->nll_prev[1]);

			const struct ds_RigidBody *b0 = ds_PoolAddress(&pipeline->body_pool, c->key.body0);
			const struct ds_RigidBody *b1 = ds_PoolAddress(&pipeline->body_pool, c->key.body1);
			const struct ds_Shape *s0 = ds_PoolAddress(&pipeline->shape_pool, c->key.shape0);
			const struct ds_Shape *s1 = ds_PoolAddress(&pipeline->shape_pool, c->key.shape1);

			u32 prev, k, found; 
			prev = NLL_NULL;
			k = s0->contact_first;
			found = 0;
			while (k != NLL_NULL)
			{
				if (k == i)
				{
					found = 1;
					break;
				}

				const struct ds_Contact *tmp = nll_Address(&pipeline->cdb->contact_net, k);
				ds_Assert(PoolSlotAllocated(tmp));
				if (tmp->key.shape0 == c->key.shape0)
				{
					ds_Assert(prev == tmp->nll_prev[0]);
					prev = k;
					k = tmp->nll_next[0];
				}
				else
				{
					ds_Assert(tmp->key.shape1 == c->key.shape0);
					ds_Assert(prev == tmp->nll_prev[1]);
					prev = k;
					k = tmp->nll_next[1];
				}
			}
			ds_Assert(found);
 
			prev = NLL_NULL;
			k = s1->contact_first;
			found = 0;
			while (k != NLL_NULL)
			{
				if (k == i)
				{
					found = 1;
					break;
				}

				const struct ds_Contact *tmp = nll_Address(&pipeline->cdb->contact_net, k);
				ds_Assert(PoolSlotAllocated(tmp));
				if (tmp->key.shape0 == c->key.shape1)
				{
					ds_Assert(prev == tmp->nll_prev[0]);
					prev = k;
					k = tmp->nll_next[0];
				}
				else
				{
					ds_Assert(prev == tmp->nll_prev[1]);
					ds_Assert(tmp->key.shape1 == c->key.shape1);
					prev = k;
					k = tmp->nll_next[1];
				}
			}
			ds_Assert(found);
		}
	}
}

void cdb_ClearFrame(struct cdb *cdb)
{
	cdb->sat_cache_frame_usage.bits = NULL;
	cdb->sat_cache_frame_usage.bit_count = 0;
	cdb->sat_cache_frame_usage.block_count = 0;	
    cdb->sat_cache_count = 0;

	cdb->contact_frame_usage.bits = NULL;
	cdb->contact_frame_usage.bit_count = 0;
	cdb->contact_frame_usage.block_count = 0;	
    cdb->contact_count = 0;
    cdb->contact_new_count = 0;
}

//void cdb_BodyRemoveContacts(struct ds_RigidBodyPipeline *pipeline, const u32 body_index)
//{
//	struct ds_RigidBody *body = ds_PoolAddress(&pipeline->body_pool, body_index);
//	u32 ci = body->contact_first;
//	body->contact_first = NLL_NULL;
//	while (ci != NLL_NULL)
//	{
//		struct ds_Contact *c = nll_Address(&pipeline->cdb->contact_net, ci);
//		struct sat_Cache *sat = sat_CacheLookup(&pipeline->cdb, CONTACT_KEY_TO_BODY_0(c->key), CONTACT_KEY_TO_BODY_1(c->key));
//		if (sat)
//		{
//			const u32 sat_index = ds_PoolIndex(&pipeline->cdb->sat_cache_pool, sat);
//			dll_Remove(&pipeline->cdb->sat_cache_list, pipeline->cdb->sat_cache_pool.buf, sat_index);
//			ds_HashMapRemove(&pipeline->cdb->sat_cache_map, (u32) c->key, sat_index);
//			ds_PoolRemove(&pipeline->cdb->sat_cache_pool, sat_index);
//		}
//
//		u32 next_i;
//		if (body_index == CONTACT_KEY_TO_BODY_0(c->key))
//		{
//			next_i = 0;
//			body = ds_PoolAddress(&pipeline->body_pool, CONTACT_KEY_TO_BODY_1(c->key));
//		}
//		else
//		{
//			next_i = 1;
//			body = ds_PoolAddress(&pipeline->body_pool, CONTACT_KEY_TO_BODY_0(c->key));
//		}
//
//		if (body->contact_first == ci)
//		{
//			body->contact_first = c->nll_next[1-next_i];
//		}
//		const u32 ci_next = c->nll_next[next_i];
//
//		PhysicsEventContactRemoved(pipeline, (u32) CONTACT_KEY_TO_BODY_0(c->key), (u32) CONTACT_KEY_TO_BODY_1(c->key));
//		BitVecSetBit(&pipeline->cdb->contact_persistent_usage, ci, 0);
//		ds_HashMapRemove(&pipeline->cdb->contact_map, (u32) c->key, ci);
//		nll_Remove(&pipeline->cdb->contact_net, ci);
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
//	struct ds_RigidBody *body = ds_PoolAddress(&pipeline->body_pool, static_index);
//	ds_Assert(body->island_index == ISLAND_STATIC);
//	u32 ci = body->contact_first;
//	body->contact_first = NLL_NULL;
//	while (ci != NLL_NULL)
//	{
//		struct ds_Contact *c = nll_Address(&pipeline->cdb->contact_net, ci);
//		u32 next_i;
//		if (static_index == CONTACT_KEY_TO_BODY_0(c->key))
//		{
//			next_i = 0;
//			body = ds_PoolAddress(&pipeline->body_pool, CONTACT_KEY_TO_BODY_1(c->key));
//		}
//		else
//		{
//			next_i = 1;
//			body = ds_PoolAddress(&pipeline->body_pool, CONTACT_KEY_TO_BODY_0(c->key));
//		}
//
//		if (body->contact_first == ci)
//		{
//			body->contact_first = c->nll_next[1-next_i];
//		}
//		const u32 ci_next = c->nll_next[next_i];
//		struct island *is = ds_PoolAddress(&pipeline->is_db.island_pool, body->island_index);
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
//			dll_Remove(&is->contact_list, pipeline->cdb->contact_net.pool.buf, ci);
//		}
//
//		PhysicsEventContactRemoved(pipeline, (u32) CONTACT_KEY_TO_BODY_0(c->key), (u32) CONTACT_KEY_TO_BODY_1(c->key));
//		BitVecSetBit(&pipeline->cdb->contact_persistent_usage, ci, 0);
//		ds_HashMapRemove(&pipeline->cdb->contact_map, (u32) c->key, ci);
//		nll_Remove(&pipeline->cdb->contact_net, ci);
//		ci = ci_next;
//	}
//
//	for (u32 i = 0; i < island_count; ++i)
//	{
//		struct island *is = ds_PoolAddress(&pipeline->is_db.island_pool, island[i]);
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

struct slot ds_ContactAdd(struct ds_RigidBodyPipeline *pipeline, const struct c_Manifold *cm, const struct ds_ContactKey *key)
{
    ds_Assert(ds_ContactKeyLookup(pipeline, key).address == NULL);

	struct ds_Shape *shape0 = ds_PoolAddress(&pipeline->shape_pool, key->shape0);
	struct ds_Shape *shape1 = ds_PoolAddress(&pipeline->shape_pool, key->shape1);

	struct ds_Contact cpy =
	{
		.cm = *cm,
		.key = *key,
		.cached_count = 0,
	};
	struct slot slot = nll_Add(&pipeline->cdb->contact_net, &cpy, shape0->contact_first, shape1->contact_first);
	ds_HashMapAdd(&pipeline->cdb->contact_map, ds_ContactKeyHash(key), slot.index);
    struct ds_Contact *c = slot.address;
    c->generation += 1;
    const ds_ContactId id = ((u64) c->generation << 32) | slot.index;
	shape0->contact_first = slot.index;
	shape1->contact_first = slot.index;

	if (slot.index < pipeline->cdb->contact_frame_usage.bit_count)
	{
		BitVecSetBit(&pipeline->cdb->contact_frame_usage, slot.index, 1);
	}
	PhysicsEventContactNew(pipeline, id);

    return slot;
}

void ds_ContactUpdate(struct ds_RigidBodyPipeline *pipeline, const struct slot slot, const struct c_Manifold *cm)
{
	struct ds_Contact *c = slot.address;
	BitVecSetBit(&pipeline->cdb->contact_frame_usage, slot.index, 1);
	c->cm = *cm;
}

void ds_ContactRemove(struct ds_RigidBodyPipeline *pipeline, const u32 index)
{
	struct ds_Contact *c = nll_Address(&pipeline->cdb->contact_net, index);
	struct ds_Shape *shape0 = ds_PoolAddress(&pipeline->shape_pool, c->key.shape0);
	struct ds_Shape *shape1 = ds_PoolAddress(&pipeline->shape_pool, c->key.shape1);
	
	if (shape0->contact_first == index)
	{
		shape0->contact_first = c->nll_next[0];
	}

	if (shape1->contact_first == index)
	{
		shape1->contact_first = c->nll_next[1];
	}

    if (shape0->cshape_type == C_SHAPE_CONVEX_HULL && shape1->cshape_type == C_SHAPE_CONVEX_HULL)
    {
    //TODO: if contact HULL-HULL, remove sat_cache????
        
    }
    
    //TODO: This doesn't work for us anymore... 
    //for (u32 i = cdb->sat_cache_list.first; i != DLL_NULL; )
	//{
	//	struct sat_Cache *cache = ds_PoolAddress(&cdb->sat_cache_pool, i);
	//	const u32 next = dll_Next(cache);
	//	if (cache->touched)
	//	{
	//		cache->touched = 0;
	//	}
	//	else
	//	{
    //        const u32 hash = (u32) XXH3_64bits(&cache->key, sizeof(cache->key));
	//		dll_Remove(&cdb->sat_cache_list, cdb->sat_cache_pool.buf, i);
	//		ds_HashMapRemove(&cdb->sat_cache_map, hash, i);
	//		ds_PoolRemove(&cdb->sat_cache_pool, i);
	//	}
	//	i = next;
	//}

    const ds_ContactId id = ((u64) c->generation << 32) | index;
	PhysicsEventContactRemoved(pipeline, id);
	ds_HashMapRemove(&pipeline->cdb->contact_map, ds_ContactKeyHash(&c->key), index);
	nll_Remove(&pipeline->cdb->contact_net, index);
}

struct slot ds_ContactKeyLookup(const struct ds_RigidBodyPipeline *pipeline, const struct ds_ContactKey *key)
{
    struct slot slot = { .address = NULL, .index = NLL_NULL };
	const u32 hash = ds_ContactKeyHash(key);
	for (u32 i = ds_HashMapFirst(&pipeline->cdb->contact_map, hash); i != HASH_NULL; i = ds_HashMapNext(&pipeline->cdb->contact_map, i))
	{
		struct ds_Contact *c = nll_Address(&pipeline->cdb->contact_net, i);
		if (ds_ContactKeyEquivalence(&c->key, key))
		{
            slot.address = c;
            slot.index = i;
            break;
		}
	}

	return slot;
}

struct slot ds_ContactLookup(const struct ds_RigidBodyPipeline *pipeline, const ds_ContactId id)
{
    struct slot slot = { .address = NULL, .index = NLL_NULL };
    struct ds_Contact *c = nll_Address(&pipeline->cdb->contact_net, ds_IdFIndex(id));
    if (id != DS_IDF_NULL && PoolSlotAllocated(c) && c->generation == ds_IdFGeneration(id))
    {
        slot.address = c;
        slot.index = ds_IdFIndex(id);
    }

    return slot;
}

struct sat_CacheKey sat_CacheKeyCanonical(const ds_RigidBodyId bodyA, const ds_ShapeId shapeA, const ds_RigidBodyId bodyB, const ds_ShapeId shapeB)
{
    return (ds_IdIndex(bodyA) < ds_IdIndex(bodyB))
        ? (struct sat_CacheKey) { .body0 = bodyA, .shape0 = shapeA, .body1 = bodyB, .shape1 = shapeB }
        : (struct sat_CacheKey) { .body0 = bodyB, .shape0 = shapeB, .body1 = bodyA, .shape1 = shapeA };
}

u32 sat_CacheKeyHash(const struct sat_CacheKey *key)
{
    return (u32) XXH3_64bits(key, sizeof(struct sat_CacheKey));
}

u32 sat_CacheKeyEquivalence(const struct sat_CacheKey *keyA, const struct sat_CacheKey *keyB)
{
    return memcmp(keyA, keyB, sizeof(struct sat_CacheKey)) == 0;
}

struct slot sat_CacheAdd(struct cdb *cdb, const struct sat_CacheKey *key)
{
	ds_Assert(sat_CacheLookup(cdb, key).address == NULL);

	struct slot slot = sat_CacheTPoolAdd(&cdb->sat_cache_pool);
	struct sat_Cache *sat = slot.address;
    sat->key = *key;
    sat->type = SAT_CACHE_NOT_SET;
	sat_CacheTHashMapAdd(&cdb->sat_cache_map, sat, slot.index);

    return slot;
}

void sat_CacheRemove(struct cdb *cdb, const u32 index)
{
	struct sat_Cache *sat = sat_CacheTPoolAddress(&cdb->sat_cache_pool, index);
    sat_CacheTHashMapRemove(&cdb->sat_cache_map, &sat->key);
    sat_CacheTPoolRemove(&cdb->sat_cache_pool, index);
}

struct slot sat_CacheLookup(struct cdb *cdb, const struct sat_CacheKey *key)
{
	ds_Assert(ds_IdIndex(key->body0) < ds_IdIndex(key->body1));
    return sat_CacheTHashMapLookup(&cdb->sat_cache_map, key);
}

TPOOL_DEFINE(sat_Cache)
THASH_DEFINE(sat_Cache, key, struct sat_CacheKey, sat_CacheKeyHash, sat_CacheKeyEquivalence)
