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

POOL_DEFINE(ds_Contact);

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

void ds_ContactKeyAddress(struct ds_RigidBody **b0, struct ds_Shape **s0, struct ds_RigidBody **b1, struct ds_Shape **s1, const struct ds_RigidBodyPipeline *pipeline, const struct ds_ContactKey *key)
{
    *b0 = pipeline->body_pool.buf + key->body0;
    *b1 = pipeline->body_pool.buf + key->body1;
    
    const u32 si0 = INDIRECT_SHAPE_CHECK(key->shape0)
        ? (u32) (*b0)->shape_list.first
        : key->shape0;
    const u32 si1 = INDIRECT_SHAPE_CHECK(key->shape1)
        ? (u32) (*b1)->shape_list.first
        : key->shape1;
    
    *s0 = (struct ds_Shape *) pipeline->shape_pool.buf + si0;
    *s1 = (struct ds_Shape *) pipeline->shape_pool.buf + si1;
}

struct cdb *cdb_Alloc(struct arena *mem_persistent, const u32 size)
{
    /* Note: requires allocation in persistent memory; thread structures initalizes pointers to pool storage... */
	struct cdb *cdb = ArenaPush(mem_persistent, sizeof(struct cdb));
	ds_Assert(PowerOfTwoCheck(size));

	sat_CacheTPoolAlloc(&cdb->sat_cache_pool, g_arch_config->logical_core_count, size);
	cdb->sat_cache_map = sat_CacheTHashMapAlloc(mem_persistent, &cdb->sat_cache_pool, 4096);

	cdb->contact_pool = ds_ContactPoolAlloc(NULL, size, GROWABLE);
	cdb->contact_map = ds_HashMapAlloc(NULL, size, size, GROWABLE);
	cdb->contact_persistent_usage = ds_BitSetAlloc(NULL, size, 0, GROWABLE);
	cdb->sat_cache_persistent_usage = ds_BitSetAlloc(NULL, size, 0, GROWABLE);

	return cdb;
}

void cdb_Free(struct cdb *cdb)
{
	sat_CacheTPoolDealloc(&cdb->sat_cache_pool);
	sat_CacheTHashMapDealloc(&cdb->sat_cache_map);
	ds_ContactPoolDealloc(&cdb->contact_pool);
	ds_HashMapDealloc(&cdb->contact_map);
	ds_BitSetDealloc(&cdb->contact_persistent_usage);
	ds_BitSetDealloc(&cdb->sat_cache_persistent_usage);
}

void cdb_Flush(struct cdb *cdb)
{
	cdb_ClearFrame(cdb);
	sat_CacheTPoolFlush(&cdb->sat_cache_pool);
	sat_CacheTHashMapFlush(&cdb->sat_cache_map);
	ds_ContactPoolFlush(&cdb->contact_pool);
	ds_HashMapFlush(&cdb->contact_map);
	ds_BitSetClear(&cdb->contact_persistent_usage, 0);
	ds_BitSetClear(&cdb->sat_cache_persistent_usage, 0);
}

void cdb_Validate(const struct ds_RigidBodyPipeline *pipeline)
{
	for (u64 i = 0; i < pipeline->cdb->contact_persistent_usage.bit_count; ++i)
	{
		if (ds_BitSetGet(&pipeline->cdb->contact_persistent_usage, i))
		{
			const struct ds_Contact *c = pipeline->cdb->contact_pool.buf + (u32) i;
			ds_Assert(ds_PoolSlotAllocated(c));

			//fprintf(stderr, "contact[%lu] (next[0], next[1], prev[0], prev[1]) : (%u,%u,%u,%u)\n",
			//	       i,
			//	       c->shape_contact[0].next,	
			//	       c->shape_contact[1].next,	
			//	       c->shape_contact[0].prev,	
			//	       c->shape_contact[1].prev);


            struct ds_RigidBody *b0, *b1;
            struct ds_Shape *s0, *s1;
            ds_ContactKeyAddress(&b0, &s0, &b1, &s1, pipeline, &c->key);

			i32 prev, k, found; 
			prev = DLL_SENTINEL;
			k = s0->contact_list.first;
			found = 0;
			while (k != DLL_SENTINEL)
			{
				if (k == (i32) i)
				{
					found = 1;
					break;
				}

				const struct ds_Contact *tmp = pipeline->cdb->contact_pool.buf + k;
				ds_Assert(ds_PoolSlotAllocated(tmp));
				if ((INDIRECT_SHAPE_CHECK(c->key.shape0) && INDIRECT_SHAPE_CHECK(tmp->key.shape0)) || tmp->key.shape0 == c->key.shape0)
				{
					ds_Assert(prev == tmp->shape_contact[0].prev);
					prev = k;
					k = tmp->shape_contact[0].next;
				}
				else
				{
					ds_Assert((INDIRECT_SHAPE_CHECK(c->key.shape0) && INDIRECT_SHAPE_CHECK(tmp->key.shape1)) || tmp->key.shape1 == c->key.shape0);
					ds_Assert(prev == tmp->shape_contact[1].prev);
					prev = k;
					k = tmp->shape_contact[1].next;
				}
			}
			ds_Assert(found);
 
			prev = DLL_SENTINEL;
			k = s1->contact_list.first;
			found = 0;
			while (k != DLL_SENTINEL)
			{
				if (k == (i32) i)
				{
					found = 1;
					break;
				}

				const struct ds_Contact *tmp = pipeline->cdb->contact_pool.buf + k;
				ds_Assert(ds_PoolSlotAllocated(tmp));
				if (tmp->key.shape0 == c->key.shape1)
				{
					ds_Assert(prev == tmp->shape_contact[0].prev);
					prev = k;
					k = tmp->shape_contact[0].next;
				}
				else
				{
					ds_Assert(prev == tmp->shape_contact[1].prev);
					ds_Assert(tmp->key.shape1 == c->key.shape1 || INDIRECT_SHAPE_CHECK(c->key.shape1));
					prev = k;
					k = tmp->shape_contact[1].next;
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

struct slot ds_ContactAdd(struct ds_RigidBodyPipeline *pipeline, const struct c_Manifold *cm, const struct ds_ContactKey *key)
{
    ds_Assert(ds_ContactKeyLookup(pipeline, key).address == NULL);

    struct ds_RigidBody *body0, *body1;
    struct ds_Shape *shape0, *shape1;
    ds_ContactKeyAddress(&body0, &shape0, &body1, &shape1, pipeline, key);

    const u32 old_max = pipeline->cdb->contact_pool.count_max;
	struct slot slot = ds_ContactPoolAdd(&pipeline->cdb->contact_pool);
    struct ds_Contact *c = slot.address;
    if (old_max != pipeline->cdb->contact_pool.count_max)
    {
        c->id = ds_IdFConstruct(slot.index, 0);
    }
    c->cm = *cm;
    c->key = *key;
    c->cached_count = 0;
    c->id += DS_IDF_GENERATION_INCREMENT;

    struct ds_Contact *buf = pipeline->cdb->contact_pool.buf;
    struct ds_Contact *c0 = buf + shape0->contact_list.last; 
    struct ds_Contact *c1 = buf + shape1->contact_list.last; 
    const i32 prev0 = (c0->key.shape1 == c->key.shape0);
    const i32 prev1 = (c1->key.shape1 == c->key.shape1);
    ds_DLLAppendEx(shape0->contact_list, buf, slot.index, shape_contact[prev0], shape_contact[0]);
    ds_DLLAppendEx(shape1->contact_list, buf, slot.index, shape_contact[prev1], shape_contact[1]);

	ds_HashMapAdd(&pipeline->cdb->contact_map, ds_ContactKeyHash(key), slot.index);

    ds_CGraphContactAdd(pipeline, c);

	if (slot.index < pipeline->cdb->contact_frame_usage.bit_count)
	{
		ds_BitSetSet(&pipeline->cdb->contact_frame_usage, slot.index, 1);
	}
	PhysicsEventContactNew(pipeline, c->id);

    return slot;
}

void ds_ContactUpdate(struct ds_RigidBodyPipeline *pipeline, const struct slot slot, const struct c_Manifold *cm)
{
	struct ds_Contact *c = slot.address;
	ds_BitSetSet(&pipeline->cdb->contact_frame_usage, slot.index, 1);
	c->cm = *cm;
}

void ds_ContactRemove(struct ds_RigidBodyPipeline *pipeline, const u32 index)
{
    struct ds_Contact *buf = pipeline->cdb->contact_pool.buf;
	struct ds_Contact *c = buf + index;
    ds_Assert(ds_PoolSlotAllocated(c));

    if (c->set == SOLVER_SET_NULL)
    {
        ds_CGraphContactRemove(pipeline, c);
    }
    else
    {
        ds_Assert(c->color >= CG_INVALID_COLOR);
        struct ds_SolverSet *set = pipeline->solver_set_pool.buf + c->set;

        ds_CPoolRemoveAndSwap(set->contact_pool, c->set_contact_index);
        if (c->set_contact_index < set->contact_pool.count)
        {
            const u32 moved_index = set->contact_pool.buf[ c->set_contact_index ];
            struct ds_Contact *moved = buf + moved_index;
            ds_Assert(moved->set_contact_index == set->contact_pool.count);
            moved->set_contact_index = c->set_contact_index;
        }
    }

	struct ds_RigidBody *body0, *body1;
    struct ds_Shape *shape0, *shape1;
    ds_ContactKeyAddress(&body0, &shape0, &body1, &shape1, pipeline, &c->key);
	
	PhysicsEventContactRemoved(pipeline, body0->id, shape0->id, body1->id, shape1->id);
	ds_BitSetSet(&pipeline->cdb->contact_persistent_usage, index, 0);
	ds_HashMapRemove(&pipeline->cdb->contact_map, ds_ContactKeyHash(&c->key), index);

    const i32 prev0 = (c->key.shape0 == buf[ c->shape_contact[0].prev ].key.shape1);
    const i32 prev1 = (c->key.shape1 == buf[ c->shape_contact[1].prev ].key.shape1);
    const i32 next0 = (c->key.shape0 == buf[ c->shape_contact[0].next ].key.shape1);
    const i32 next1 = (c->key.shape1 == buf[ c->shape_contact[1].next ].key.shape1);
    ds_DLLRemoveEx(shape0->contact_list, buf, index, shape_contact[prev0], shape_contact[0], shape_contact[next0]);
    ds_DLLRemoveEx(shape1->contact_list, buf, index, shape_contact[prev1], shape_contact[1], shape_contact[next1]);

    struct ds_Island *island = pipeline->island_pool.buf + c->island;
    ds_DLLRemove(island->contact_list, pipeline->cdb->contact_pool.buf, index, island_contact);
    ds_ContactPoolRemove(&pipeline->cdb->contact_pool, index);
}

struct slot ds_ContactKeyLookup(const struct ds_RigidBodyPipeline *pipeline, const struct ds_ContactKey *key)
{
    struct slot slot = { .address = NULL, .index = U32_MAX };
	const u32 hash = ds_ContactKeyHash(key);
	for (u32 i = ds_HashMapFirst(&pipeline->cdb->contact_map, hash); i != HASH_NULL; i = ds_HashMapNext(&pipeline->cdb->contact_map, i))
	{
		struct ds_Contact *c = pipeline->cdb->contact_pool.buf + i;
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
    struct slot slot = { .address = NULL, .index = U32_MAX };
    struct ds_Contact *c = pipeline->cdb->contact_pool.buf + ds_IdFIndex(id);
    if (id != DS_IDF_NULL && ds_PoolSlotAllocated(c) && c->id == id)
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
    sat->tri_cache_count = 0;
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
