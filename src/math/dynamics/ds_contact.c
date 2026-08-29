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
        ds_CGraphContactRemove(pipeline, c);

        struct ds_Island *island = pipeline->island_pool.buf + c->island;
        island->constraint_remove_count += 1;
        ds_DLLRemove(island->contact_list, pipeline->contact_pool.buf, index, island_contact);
    }
    else
    {
        ds_Assert(c->color == CG_INVALID_COLOR);
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
	ds_HashMapRemove(&pipeline->contact_map, ds_ContactKeyHash(c->key), index);

    const i32 prev0 = (c->key.shape[0] == buf[ c->shape_contact[0].prev ].key.shape[1]);
    const i32 prev1 = (c->key.shape[1] == buf[ c->shape_contact[1].prev ].key.shape[1]);
    const i32 next0 = (c->key.shape[0] == buf[ c->shape_contact[0].next ].key.shape[1]);
    const i32 next1 = (c->key.shape[1] == buf[ c->shape_contact[1].next ].key.shape[1]);
    ds_DLLRemoveEx(shape0->contact_list, buf, index, shape_contact[prev0], shape_contact[0], shape_contact[next0]);
    ds_DLLRemoveEx(shape1->contact_list, buf, index, shape_contact[prev1], shape_contact[1], shape_contact[next1]);
        
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

u32 ds_ContactCheckBvhOverlap(const struct ds_RigidBodyPipeline *pipeline, const u32 contact_index)
{
    const struct ds_Contact *contact = pipeline->contact_pool.buf + contact_index;

    const struct ds_Shape *shape[2] =
    {
        pipeline->shape_pool.buf + contact->key.shape[0],
        pipeline->shape_pool.buf + contact->key.shape[1],
    };
    
    const struct ds_RigidBody *body[2] =
    {
	    pipeline->body_pool.buf + shape[0]->body,
	    pipeline->body_pool.buf + shape[1]->body,
    };

    const struct bvhNode *s_node = (const struct bvhNode *) pipeline->static_bvh.tree.pool.buf;
    const struct bvhNode *d_node = (const struct bvhNode *) pipeline->dynamic_bvh.tree.pool.buf;

    const struct aabb *bbox[2];
    bbox[0] = RB_IS_DYNAMIC(body[0]) 
            ? &d_node[ shape[0]->proxy ].bbox 
            : &s_node[ shape[0]->proxy ].bbox;
    bbox[1] = RB_IS_DYNAMIC(body[1]) 
            ? &d_node[ shape[1]->proxy ].bbox 
            : &s_node[ shape[1]->proxy ].bbox;
    
    return AabbTest(bbox[0], bbox[1]);
}

void ds_ContactPromote(struct ds_RigidBodyPipeline *pipeline, const u32 contact)
{
    struct ds_Contact *c = pipeline->contact_pool.buf + contact;
    struct ds_SolverSet *active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;
    ds_Assert(c->set == SOLVER_SET_ACTIVE);

    const u32 compute = c->compute;
    ds_CGraphContactAdd(pipeline, c);

    ds_CPoolRemoveAndSwap(active->contact_pool, compute);
    if (compute < active->contact_pool.count)
    {
        const u32 moved_index = active->contact_pool.buf[ compute ];
        struct ds_Contact *moved = pipeline->contact_pool.buf + moved_index;
        ds_Assert(moved->set == SOLVER_SET_ACTIVE);
        ds_Assert(moved->compute == active->contact_pool.count);
        moved->compute = compute;
    }

    const struct ds_Shape *shape[2] =
    {
        pipeline->shape_pool.buf + c->key.shape[0],
        pipeline->shape_pool.buf + c->key.shape[1],
    };
    
    const struct ds_RigidBody *body[2] =
    {
	    pipeline->body_pool.buf + shape[0]->body,
	    pipeline->body_pool.buf + shape[1]->body,
    };

    const u32 dynamic[2] = { RB_DYNAMIC_BIT(body[0]), RB_DYNAMIC_BIT(body[1]) };
    const u32 expand_index = body[ dynamic[1] ]->island;
    const u32 merge_index = body[ 1-dynamic[1] ]->island;
    ds_Assert(dynamic[0] || dynamic[1]);

	struct ds_Island *expand = pipeline->island_pool.buf + expand_index;
    if (expand->set >= SOLVER_SET_SLEEPING_FIRST)
    {
        ds_SolverSetWakeUp(pipeline, expand->set);
	    PhysicsEventIslandAwake(pipeline, expand->id);	
    }

    if (dynamic[0] && dynamic[1])
    {
		ds_IslandMerge(pipeline, expand_index, merge_index);
    }

    c->island = expand_index;
	ds_DLLAppend(expand->contact_list, pipeline->contact_pool.buf, contact, island_contact);
	PhysicsEventIslandExpanded(pipeline, expand->id);	

}

void ds_ContactDemote(struct ds_RigidBodyPipeline *pipeline, const u32 contact)
{
    struct ds_Contact *c = pipeline->contact_pool.buf + contact;
    struct ds_Island *island = pipeline->island_pool.buf + c->island;
    struct ds_SolverSet *active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;

    ds_CGraphContactRemove(pipeline, c); 

    island->constraint_remove_count += 1;
    ds_DLLRemove(island->contact_list, pipeline->contact_pool.buf, contact, island_contact);

    c->island = U32_MAX;
    c->set = SOLVER_SET_ACTIVE;
    c->compute = ds_CPoolPush(active->contact_pool).index;
    active->contact_pool.buf[ c->compute ] = contact;
}

void ds_ContactWakeUp(struct arena *frame, struct ds_RigidBodyPipeline *pipeline, const u32 contact_index)
{
    struct ds_Contact *c = pipeline->contact_pool.buf + contact_index;
    ds_Assert(c->set >= SOLVER_SET_SLEEPING_FIRST && c->color == CG_INVALID_COLOR);

    if (c->narrowphase.manifold)
    {
        c->narrowphase.manifold = ArenaPushAlignedMemcpy(frame, c->narrowphase.manifold, c->narrowphase.manifold_count*sizeof(struct c_Manifold), 1);
    }

    if (c->narrowphase.cache)
    {
        c->narrowphase.cache = ArenaPushAlignedMemcpy(frame, c->narrowphase.cache, c->narrowphase.cache_count*sizeof(struct c_SatCache), 1);
    }

    if (c->narrowphase.tri)
    {
        c->narrowphase.tri = ArenaPushAlignedMemcpy(frame, c->narrowphase.tri, c->narrowphase.manifold_count*sizeof(u32), 1);
        c->narrowphase.tri_manifold = ArenaPushAlignedMemcpy(frame, c->narrowphase.tri_manifold, c->narrowphase.manifold_count*sizeof(u32), 1);
    }

    ds_Assert(c->narrowphase.manifold_count);
    ds_CGraphContactAdd(pipeline, c);
}

u64 ds_ContactMemoryRequirement(const struct ds_RigidBodyPipeline *pipeline, const u32 contact)
{
    const struct ds_Contact *c = pipeline->contact_pool.buf + contact;

    const u64 mem_req_manifold = c->narrowphase.manifold_count*sizeof(struct c_Manifold);
    const u64 mem_req_cache = c->narrowphase.cache_count*sizeof(struct c_SatCache);

    u64 mem_req_tri = 0;
    u64 mem_req_tri_manifold = 0;
    if (c->narrowphase.tri)
    {
        mem_req_tri = c->narrowphase.manifold_count*sizeof(u32);
        mem_req_tri_manifold = c->narrowphase.manifold_count*sizeof(u32);
    }

    return mem_req_manifold + mem_req_cache + mem_req_tri + mem_req_tri_manifold;
}

void ds_ContactSleep(struct arena *mem_sleep, struct ds_RigidBodyPipeline *pipeline, const u32 contact, const u32 set_index)
{
    ds_Assert(ds_ContactMemoryRequirement(pipeline, contact) <= mem_sleep->mem_left);
    struct ds_SolverSet *set = pipeline->solver_set_pool.buf + set_index;
    struct ds_Contact *c = pipeline->contact_pool.buf + contact;

    ds_Assert(c->color != CG_INVALID_COLOR)
    const u64 mem_req_manifold = c->narrowphase.manifold_count*sizeof(struct c_Manifold);
    c->narrowphase.manifold = ArenaPushAlignedMemcpy(mem_sleep, c->narrowphase.manifold, mem_req_manifold, 1);
    ds_CGraphContactRemove(pipeline, c); 

    if (c->narrowphase.cache)
    {
        const u64 mem_req_cache = c->narrowphase.cache_count*sizeof(struct c_SatCache);
        c->narrowphase.cache = ArenaPushAlignedMemcpy(mem_sleep, c->narrowphase.cache, mem_req_cache, 1);
    }

    if (c->narrowphase.tri)
    {
        const u64 mem_req_tri = c->narrowphase.manifold_count*sizeof(u32);
        const u64 mem_req_tri_manifold = c->narrowphase.manifold_count*sizeof(u32);
        c->narrowphase.tri = ArenaPushAlignedMemcpy(mem_sleep, c->narrowphase.tri, mem_req_tri, 1);
        c->narrowphase.tri_manifold = ArenaPushAlignedMemcpy(mem_sleep, c->narrowphase.tri_manifold, mem_req_tri_manifold, 1);
    }

    c->set = set_index;
    c->compute = ds_CPoolPush(set->contact_pool).index;
    set->contact_pool.buf[ c->compute ] = contact;
}

void ds_ContactValidateAll(const struct ds_RigidBodyPipeline *pipeline)
{
    const struct ds_SolverSet *active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;
    for (u32 i = 0; i < active->contact_pool.count; ++i)
    {
        const u32 ci = active->contact_pool.buf[i];
        const struct ds_Contact *c = pipeline->contact_pool.buf + ci;
		ds_Assert(ds_PoolSlotAllocated(c));
        ds_Assert(c->compute == i);
        ds_Assert(c->set == SOLVER_SET_ACTIVE);
        ds_Assert(c->color == CG_INVALID_COLOR);
        ds_Assert(c->narrowphase.manifold_count == 0);
    }

    for (u32 color_index = 0; color_index < CG_COLOR_COUNT; ++color_index)
    {
        const struct ds_CGraphColor *color = pipeline->cgraph.color + color_index;
        for (u32 i = 0; i < color->contact_pool.count; ++i)
        {
            const u32 ci = color->contact_pool.buf[i];
            const struct ds_Contact *c = pipeline->contact_pool.buf + ci;
			ds_Assert(ds_PoolSlotAllocated(c));
            ds_Assert(c->compute == i);
            ds_Assert(c->set == SOLVER_SET_NULL);
            ds_Assert(c->color == color_index);
            ds_Assert(c->narrowphase.manifold_count > 0);
        }
    }

    for (u32 set_index = SOLVER_SET_SLEEPING_FIRST; set_index < pipeline->solver_set_pool.count; ++set_index)
    {
        const struct ds_SolverSet *set = pipeline->solver_set_pool.buf + set_index;
        for (u32 i = 0; i < set->contact_pool.count; ++i)
        {
            const u32 ci = set->contact_pool.buf[i];
            const struct ds_Contact *c = pipeline->contact_pool.buf + ci;
		    ds_Assert(ds_PoolSlotAllocated(c));
            ds_Assert(c->compute == i);
            ds_Assert(c->set == set_index);
            ds_Assert(c->color == CG_INVALID_COLOR);
            ds_Assert(c->narrowphase.manifold_count > 0);
        }
    }
}
