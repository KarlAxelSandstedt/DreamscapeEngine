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

struct ds_ContactKey ds_ContactKeyCanonical(const u32 shape0, const u32 shape1)
{
    return (shape0 < shape1)
        ? (struct ds_ContactKey) { .shape = { shape0, shape1 } } 
        : (struct ds_ContactKey) { .shape = { shape1, shape0 } };
}

u32 ds_ContactKeyHash(const struct ds_ContactKey key)
{
    return (u32) XXH3_64bits(&key, sizeof(struct ds_ContactKey));
}

u32 ds_ContactKeyEquivalence(const struct ds_ContactKey key0, const struct ds_ContactKey key1)
{
    return memcmp(&key0, &key1, sizeof(struct ds_ContactKey)) == 0;
}

void ds_ContactKeyAddress(struct ds_RigidBody **b0, struct ds_Shape **s0, struct ds_RigidBody **b1, struct ds_Shape **s1, const struct ds_RigidBodyPipeline *pipeline, const struct ds_ContactKey key)
{    
    *s0 = pipeline->shape_pool.buf + key.shape[0];
    *s1 = pipeline->shape_pool.buf + key.shape[1];

    *b0 = pipeline->body_pool.buf + (*s0)->body;
    *b1 = pipeline->body_pool.buf + (*s1)->body;
}

struct slot ds_ContactAdd(struct ds_RigidBodyPipeline *pipeline, const struct ds_ContactKey key)
{
    ds_Assert(ds_ContactKeyLookup(pipeline, key).address == NULL);

    struct ds_Shape *shape[2] = 
    {
        pipeline->shape_pool.buf + key.shape[0],
        pipeline->shape_pool.buf + key.shape[1],
    };

    const u32 old_max = pipeline->contact_pool.count_max;
	const struct slot contact_slot = ds_ContactPoolAdd(&pipeline->contact_pool);
    struct ds_Contact *c = contact_slot.address;
    if (old_max != pipeline->contact_pool.count_max)
    {
        c->id = ds_IdFConstruct(contact_slot.index, 0);
    }

    c->key = key;
    c->id += DS_IDF_GENERATION_INCREMENT;
    c->narrowphase = (struct c_ContactResult) { 0 };

    struct ds_SolverSet *active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;
    c->compute = ds_CPoolPush(active->contact_pool).index;
    c->set = SOLVER_SET_ACTIVE;
    c->color = CG_INVALID_COLOR;
    active->contact_pool.buf[ c->compute ] = contact_slot.index;
        
    struct ds_Contact *buf = pipeline->contact_pool.buf;
    struct ds_Contact *c0 = buf + shape[0]->contact_list.last; 
    struct ds_Contact *c1 = buf + shape[1]->contact_list.last; 
    const i32 prev0 = (c0->key.shape[1] == c->key.shape[0]);
    const i32 prev1 = (c1->key.shape[1] == c->key.shape[1]);
    ds_DLLAppendEx(shape[0]->contact_list, buf, contact_slot.index, shape_contact[prev0], shape_contact[0]);
    ds_DLLAppendEx(shape[1]->contact_list, buf, contact_slot.index, shape_contact[prev1], shape_contact[1]);

	ds_HashMapAdd(&pipeline->contact_map, ds_ContactKeyHash(key), contact_slot.index);

	if (contact_slot.index < pipeline->contact_frame_usage.bit_count)
	{
		ds_BitSetSet(&pipeline->contact_frame_usage, contact_slot.index, 1);
	}
	PhysicsEventContactNew(pipeline, c->id);

    return contact_slot;
}

void ds_ContactRemove(struct ds_RigidBodyPipeline *pipeline, const u32 index)
{
    struct ds_Contact *buf = pipeline->contact_pool.buf;
	struct ds_Contact *c = buf + index;
    ds_Assert(ds_PoolSlotAllocated(c));

    if (c->set == SOLVER_SET_NULL)
    {
        //TODO
        //ds_CGraphContactRemove(pipeline, c);
    }
    else
    {
        ds_Assert(c->color >= CG_INVALID_COLOR);
        struct ds_SolverSet *set = pipeline->solver_set_pool.buf + c->set;

        ds_CPoolRemoveAndSwap(set->contact_pool, c->compute);
        if (c->compute < set->contact_pool.count)
        {
            const u32 moved_index = set->contact_pool.buf[ c->compute ];
            struct ds_Contact *moved = buf + moved_index;
            ds_Assert(moved->compute == set->contact_pool.count);
            moved->compute = c->compute;
        }
    }

	struct ds_RigidBody *body0, *body1;
    struct ds_Shape *shape0, *shape1;
    ds_ContactKeyAddress(&body0, &shape0, &body1, &shape1, pipeline, c->key);
	
	PhysicsEventContactRemoved(pipeline, body0->id, shape0->id, body1->id, shape1->id);
	ds_BitSetSet(&pipeline->contact_persistent_usage, index, 0);
	ds_HashMapRemove(&pipeline->contact_map, ds_ContactKeyHash(c->key), index);

    const i32 prev0 = (c->key.shape[0] == buf[ c->shape_contact[0].prev ].key.shape[1]);
    const i32 prev1 = (c->key.shape[1] == buf[ c->shape_contact[1].prev ].key.shape[1]);
    const i32 next0 = (c->key.shape[0] == buf[ c->shape_contact[0].next ].key.shape[1]);
    const i32 next1 = (c->key.shape[1] == buf[ c->shape_contact[1].next ].key.shape[1]);
    ds_DLLRemoveEx(shape0->contact_list, buf, index, shape_contact[prev0], shape_contact[0], shape_contact[next0]);
    ds_DLLRemoveEx(shape1->contact_list, buf, index, shape_contact[prev1], shape_contact[1], shape_contact[next1]);

    struct ds_Island *island = pipeline->island_pool.buf + c->island;
    ds_DLLRemove(island->contact_list, pipeline->contact_pool.buf, index, island_contact);
    ds_ContactPoolRemove(&pipeline->contact_pool, index);
}

struct slot ds_ContactKeyLookup(const struct ds_RigidBodyPipeline *pipeline, const struct ds_ContactKey key)
{
    struct slot slot = { .address = NULL, .index = U32_MAX };
	const u32 hash = ds_ContactKeyHash(key);
	for (u32 i = ds_HashMapFirst(&pipeline->contact_map, hash); i != HASH_NULL; i = ds_HashMapNext(&pipeline->contact_map, i))
	{
		struct ds_Contact *c = (struct ds_Contact *) pipeline->contact_pool.buf + i;
		if (ds_ContactKeyEquivalence(c->key, key))
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
    struct ds_Contact *c = pipeline->contact_pool.buf + ds_IdFIndex(id);
    if (id != DS_IDF_NULL && ds_PoolSlotAllocated(c) && c->id == id)
    {
        slot.address = c;
        slot.index = ds_IdFIndex(id);
    }

    return slot;
}

void ds_ContactValidateAll(const struct ds_RigidBodyPipeline *pipeline)
{
//	for (u64 i = 0; i < pipeline->cdb->contact_persistent_usage.bit_count; ++i)
//	{
//		if (ds_BitSetGet(&pipeline->cdb->contact_persistent_usage, i))
//		{
//			const struct ds_Contact *c = pipeline->cdb->contact_pool.buf + (u32) i;
//			ds_Assert(ds_PoolSlotAllocated(c));
//
//			//fprintf(stderr, "contact[%lu] (next[0], next[1], prev[0], prev[1]) : (%u,%u,%u,%u)\n",
//			//	       i,
//			//	       c->shape_contact[0].next,	
//			//	       c->shape_contact[1].next,	
//			//	       c->shape_contact[0].prev,	
//			//	       c->shape_contact[1].prev);
//
//
//            struct ds_RigidBody *b0, *b1;
//            struct ds_Shape *s0, *s1;
//            ds_ContactKeyAddress(&b0, &s0, &b1, &s1, pipeline, &c->key);
//
//			i32 prev, k, found; 
//			prev = DLL_SENTINEL;
//			k = s0->contact_list.first;
//			found = 0;
//			while (k != DLL_SENTINEL)
//			{
//				if (k == (i32) i)
//				{
//					found = 1;
//					break;
//				}
//
//				const struct ds_Contact *tmp = pipeline->cdb->contact_pool.buf + k;
//				ds_Assert(ds_PoolSlotAllocated(tmp));
//				if ((INDIRECT_SHAPE_CHECK(c->key.shape0) && INDIRECT_SHAPE_CHECK(tmp->key.shape0)) || tmp->key.shape0 == c->key.shape0)
//				{
//					ds_Assert(prev == tmp->shape_contact[0].prev);
//					prev = k;
//					k = tmp->shape_contact[0].next;
//				}
//				else
//				{
//					ds_Assert((INDIRECT_SHAPE_CHECK(c->key.shape0) && INDIRECT_SHAPE_CHECK(tmp->key.shape1)) || tmp->key.shape1 == c->key.shape0);
//					ds_Assert(prev == tmp->shape_contact[1].prev);
//					prev = k;
//					k = tmp->shape_contact[1].next;
//				}
//			}
//			ds_Assert(found);
// 
//			prev = DLL_SENTINEL;
//			k = s1->contact_list.first;
//			found = 0;
//			while (k != DLL_SENTINEL)
//			{
//				if (k == (i32) i)
//				{
//					found = 1;
//					break;
//				}
//
//				const struct ds_Contact *tmp = pipeline->cdb->contact_pool.buf + k;
//				ds_Assert(ds_PoolSlotAllocated(tmp));
//				if (tmp->key.shape0 == c->key.shape1)
//				{
//					ds_Assert(prev == tmp->shape_contact[0].prev);
//					prev = k;
//					k = tmp->shape_contact[0].next;
//				}
//				else
//				{
//					ds_Assert(prev == tmp->shape_contact[1].prev);
//					ds_Assert(tmp->key.shape1 == c->key.shape1 || INDIRECT_SHAPE_CHECK(c->key.shape1));
//					prev = k;
//					k = tmp->shape_contact[1].next;
//				}
//			}
//			ds_Assert(found);
//		}
//	}
}
