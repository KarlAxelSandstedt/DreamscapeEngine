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

#include "quaternion.h"
#include "ds_job.h"

POOL_DEFINE(ds_Island);

/* Add new body to island */
static void ds_IslandAddBody(struct ds_RigidBodyPipeline *pipeline, const u32 island_index, const u32 body)
{
    struct ds_Island *is = pipeline->island_pool.buf + island_index;
	struct ds_RigidBody *b = pipeline->body_pool.buf + body;
	b->island = ds_IslandPoolIndex(&pipeline->island_pool, is);
    if (b->set != is->set)
    {
        ds_SolverSetMoveBody(pipeline, body, is->set);
    }
	ds_DLLAppend(is->body_list, pipeline->body_pool.buf, body, island_body);
}

static struct slot ds_IslandAlloc(struct ds_RigidBodyPipeline *pipeline, const u32 set)
{
    ds_Assert(set != SOLVER_SET_STATIC);

    const u32 old_max = pipeline->island_pool.count;
	struct slot slot = ds_IslandPoolAdd(&pipeline->island_pool);
    if (pipeline->island_high_energy_set.bit_count < pipeline->island_pool.count)
    {
        ds_BitSetIncreaseSize(&pipeline->island_high_energy_set, pipeline->island_pool.count, 0);
    }

	struct ds_Island *is = slot.address;
	ds_DLLFlush(&is->body_list);
	ds_DLLFlush(&is->contact_list);
    ds_DLLFlush(&is->joint_list);
	is->constraint_remove_count = 0;

    if (old_max != pipeline->island_pool.count)
    {
        is->id = ds_IdConstruct(slot.index, 0);
    }
            
    is->id += DS_ID_GENERATION_INCREMENT;

    struct ds_SolverSet *s = pipeline->solver_set_pool.buf + set;
    is->set = set;
    is->set_island_index = ds_CPoolPush(s->island_pool).index;
    s->island_pool.buf[ is->set_island_index ] = slot.index;
	PhysicsEventIslandNew(pipeline, is->id);

	return slot;
}

struct slot ds_IslandLookup(struct ds_RigidBodyPipeline *pipeline, const ds_IslandId id)
{
    struct slot slot = { .address = NULL, .index = U32_MAX };
    struct ds_Island *island = pipeline->island_pool.buf + ds_IdIndex(id);
    if (ds_PoolSlotAllocated(island) && island->id == id)
    {
        slot.address = island;
        slot.index = ds_IdIndex(id);
    }

    return slot;
}

void ds_IslandPrint(FILE *file, const struct ds_RigidBodyPipeline *pipeline, const u32 island, const char *desc)
{
	const struct ds_Island *is = pipeline->island_pool.buf + island;
	if (!is) { return; }

	const struct ds_Contact *c;
	const struct ds_RigidBody *b;

	fprintf(file, "Island %u %s:\n{\n", island, desc);

	fprintf(file, "\tbody_list.count: %u\n", is->body_list.count);
	fprintf(file, "\tcontact_list.count: %u\n", is->contact_list.count);
		
	fprintf(file, "\t(Body):                     { ");
	for (i32 i = is->body_list.first; i != DLL_SENTINEL; i = b->island_body.next)
	{
		fprintf(file, "(%u) ", i);
		b = pipeline->body_pool.buf + i;
	}
	fprintf(file, "}\n");

	fprintf(file, "\t(Contact):                  { ");
	for (i32 i = is->contact_list.first; i != DLL_SENTINEL; i = c->island_contact.next)
	{
		fprintf(file, "(%u) ", i);
		c = pipeline->contact_pool.buf + i;
	}
	fprintf(file, "}\n");

	fprintf(file, "\tContacts (Shape0, Shape1):     { ");
	for (i32 i = is->contact_list.first; i != DLL_SENTINEL; i = c->island_contact.next)
	{
		c = pipeline->contact_pool.buf + i;
		fprintf(file, "(%u,%u)) ", c->key.shape[0], c->key.shape[1]);
	}
	fprintf(file, "}\n");

	fprintf(file, "\tflags:\n\t{\n");
	fprintf(file, "\t\tawake: %u\n", is->set == SOLVER_SET_ACTIVE);
	fprintf(file, "\t\tconstraint remove count: %u\n", is->constraint_remove_count);
	fprintf(file, "\t}\n");

	fprintf(file, "}\n");
}

void ds_IslandValidateAll(const struct ds_RigidBodyPipeline *pipeline)
{
	const struct ds_Island *is = NULL;
	const struct ds_RigidBody *body = NULL;

    for (u32 index = 0; index < pipeline->island_pool.count_max; ++index)
    {
	    is = pipeline->island_pool.buf + index;
        if (!ds_PoolSlotAllocated(is))
        {
            continue;
        }

 	    /* 1. verify body-island map count == island.body_list.count */
	    u32 count = 0;
	    for (u32 j = 0; j < pipeline->body_pool.count_max; ++j)
	    {
	    	const struct ds_RigidBody *b = pipeline->body_pool.buf + j;
	    	if (ds_PoolSlotAllocated(b) && b->island == index)
	    	{
	    		count += 1;
	    	}	
	    }
	    
	    ds_Assert(count == is->body_list.count && "Body count of island should be equal to the number of bodies mapped to the island");
 
	    /* 2. verify body-island map  == island.bodies */
	    u32 list_length = 0; 
	    for (u32 bi = is->body_list.first; (i32) bi != DLL_SENTINEL; bi = body->island_body.next)
	    {
	    	list_length += 1;
	    	body = pipeline->body_pool.buf + bi;
	    	ds_Assert(ds_PoolSlotAllocated(body) && body->island == index);
	    }
	    ds_Assert(list_length == is->body_list.count);

	    /* 3. check island contacts, ds_Assert body.contacts == NULL */
	    {
	    	/* 
	    	 * 4. For each contact in island
	    	 * 	1. check contact exist
	    	 * 	2. check bodies in contact are mapped to island
	    	 */
            u32 touching_contacts = 0;
	    	struct ds_Contact *c = NULL;
	    	for (u32 ci = is->contact_list.first; (i32) ci != DLL_SENTINEL; ci = c->island_contact.next)
	    	{
	    		c = pipeline->contact_pool.buf + ci;
	    		ds_Assert(ds_PoolSlotAllocated(c));
                const struct ds_Shape *s0 = pipeline->shape_pool.buf + c->key.shape[0];
                const struct ds_Shape *s1 = pipeline->shape_pool.buf + c->key.shape[1];
	    		const struct ds_RigidBody *b0 = pipeline->body_pool.buf + s0->body;
	    		const struct ds_RigidBody *b1 = pipeline->body_pool.buf + s1->body;
	    		ds_Assert((b0->island == index) || RB_IS_STATIC(b0));
	    		ds_Assert((b1->island == index) || RB_IS_STATIC(b1));
                ds_Assert(c->island == index);
                if (c->color != CG_INVALID_COLOR)
                {
                    touching_contacts += 1;
                }
	    	}
	    	ds_Assert(touching_contacts == is->contact_list.count);
	    }
	}

	/* 5. verify no body points to invalid island */
	for (u32 i = 0; i < pipeline->body_pool.count_max; ++i)
	{
		struct ds_RigidBody *body = pipeline->body_pool.buf + i;
		if (ds_PoolSlotAllocated(body) && RB_IS_DYNAMIC(body))
		{
			struct ds_Island *is = pipeline->island_pool.buf + body->island;
			ds_Assert(ds_PoolSlotAllocated(is));
		}
	}
}

void ds_IslandMerge(struct ds_RigidBodyPipeline *pipeline, const u32 expand_index, const u32 merge_index)
{
    ProfZone;
    
	struct ds_Island *expand = pipeline->island_pool.buf + expand_index;
	struct ds_Island *merge = pipeline->island_pool.buf + merge_index;
    
    ds_Assert(expand->set == SOLVER_SET_ACTIVE)
    ds_Assert(expand_index != merge_index)

    ds_SolverSetWakeUp(pipeline, merge->set);

    {
        struct ds_Contact *expand_last = pipeline->contact_pool.buf + expand->contact_list.last;
        struct ds_Contact *merge_first = pipeline->contact_pool.buf + merge->contact_list.first;
        expand_last->island_contact.next = merge->contact_list.first;
        merge_first->island_contact.prev = expand->contact_list.last;

	    if (expand->contact_list.count == 0)
	    {
	    	expand->contact_list.first = merge->contact_list.first;
	    }

        if (merge->contact_list.count > 0)
        {
	        expand->contact_list.last = merge->contact_list.last;
        }
        
        struct ds_Contact *merge_contact;
        for (i32 i = merge->contact_list.first; i != DLL_SENTINEL; i = merge_contact->island_contact.next)
        {
            merge_contact = pipeline->contact_pool.buf + i;
            merge_contact->island = expand_index;
        }
    }

    {
	    struct ds_RigidBody *expand_body_last = pipeline->body_pool.buf + expand->body_list.last;
	    struct ds_RigidBody *merge_body_first = pipeline->body_pool.buf + merge->body_list.first;
	    ds_Assert(expand_body_last->island_body.next == DLL_SENTINEL);
	    ds_Assert(merge_body_first->island_body.prev == DLL_SENTINEL);

	    expand_body_last->island_body.next = merge->body_list.first;
	    merge_body_first->island_body.prev = expand->body_list.last;
        struct ds_RigidBody *body;
	    for (i32 i = merge->body_list.first; i != DLL_SENTINEL; i = body->island_body.next)
	    {
	    	body = pipeline->body_pool.buf + i;
	    	body->island = expand_index;
	    }
    }

    expand->constraint_remove_count += merge->constraint_remove_count;
	expand->contact_list.count += merge->contact_list.count;
	expand->body_list.count += merge->body_list.count;
	expand->body_list.last = merge->body_list.last;
    
	ds_IslandRemove(pipeline, merge_index);
	
    ProfZoneEnd;
}

void ds_IslandRemove(struct ds_RigidBodyPipeline *pipeline, const u32 island_index)
{
    const struct ds_Island *island = pipeline->island_pool.buf + island_index;
    if (island->set >= SOLVER_SET_SLEEPING_FIRST)
    {
        ds_SolverSetRemove(pipeline, island->set);
    }
    else
    {
        struct ds_SolverSet *set = pipeline->solver_set_pool.buf + island->set;
        ds_CPoolRemoveAndSwap(set->island_pool, island->set_island_index);
        if (island->set_island_index < set->island_pool.count)
        {
            const u32 update_index = set->island_pool.buf[ island->set_island_index ];
            struct ds_Island *island_to_update = pipeline->island_pool.buf + update_index;
            ds_Assert(island_to_update->set_island_index == set->island_pool.count);
            island_to_update->set_island_index = island->set_island_index; 
        }
    }
	ds_IslandPoolRemove(&pipeline->island_pool, island_index);
}

void ds_IslandSplit(struct ds_RigidBodyPipeline *pipeline, const u32 island_to_split)
{
    ProfZone;

	struct ds_Island *split = pipeline->island_pool.buf + island_to_split;
    struct arena *mem_tmp = ArenaPushScratch();
    ds_Assert(split->set == SOLVER_SET_ACTIVE);
    ds_Assert(split->constraint_remove_count > 0);

	u32 *body_stack = ArenaPush(mem_tmp, split->body_list.count*sizeof(u32));
	u32 sc;
    for (i32 bi = split->body_list.first; bi != DLL_SENTINEL; bi = split->body_list.first)
	{
	    struct ds_RigidBody *body_anchor = pipeline->body_pool.buf + bi;
		ds_Assert(body_anchor->island == island_to_split);

	    const struct slot slot = ds_IslandAlloc(pipeline, SOLVER_SET_ACTIVE);
        const u32 new_island_index = slot.index;
        struct ds_Island *new_island = slot.address;
		split = pipeline->island_pool.buf + island_to_split;

		ds_DLLRemove(split->body_list, pipeline->body_pool.buf, bi, island_body);
		ds_IslandAddBody(pipeline, new_island_index, bi);
        body_stack[0] = bi;
        sc = 1;

        while (sc--)
        {
            const u32 body_index = body_stack[sc];
			struct ds_RigidBody *body = pipeline->body_pool.buf + body_index;
            struct ds_Shape *shape;
            for (i32 si = body->shape_list.first; si != DLL_SENTINEL; si = shape->body_shape.next)
            {
                shape = pipeline->shape_pool.buf + si;
                i32 ci = shape->contact_list.first;
                while (ci != DLL_SENTINEL)
                {
		    	    struct ds_Contact *c = pipeline->contact_pool.buf + ci;
                    const u32 n = ((u32) si == c->key.shape[1]);
                    const i32 ci_next = c->shape_contact[n].next;
                    if (c->island == island_to_split)
                    {
                        const struct ds_Shape *neighbour_shape = pipeline->shape_pool.buf + c->key.shape[1-n]; 
                        const struct ds_RigidBody *neighbour_body = pipeline->body_pool.buf + neighbour_shape->body; 
                        if (neighbour_body->island == island_to_split)
                        {
		      		    	ds_DLLRemove(split->body_list, pipeline->body_pool.buf, neighbour_shape->body, island_body);
		      		    	ds_IslandAddBody(pipeline, new_island_index, neighbour_shape->body);
		      		    	body_stack[sc++] = neighbour_shape->body;
                        }
	                    ds_DLLAppend(new_island->contact_list, pipeline->contact_pool.buf, (u32) ci, island_contact);
                        c->island = new_island_index;
                    }
                    ci = ci_next;
                }
            }
        }
	}

    //TODO issue: joints/(bodies???) in old island, need to update 
    
	ds_IslandRemove(pipeline, island_to_split);
    ArenaPopScratch();
    
    ProfZoneEnd;
}
