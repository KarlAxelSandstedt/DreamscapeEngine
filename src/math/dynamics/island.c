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
	PhysicsEventIslandNew(pipeline, slot.index);
    if (pipeline->island_high_energy_set.bit_count < pipeline->island_pool.count)
    {
        ds_BitSetIncreaseSize(&pipeline->island_high_energy_set, pipeline->island_pool.count, 0);
    }

	struct ds_Island *is = slot.address;
	ds_DLLFlush(&is->body_list);
	ds_DLLFlush(&is->contact_list);
    ds_DLLFlush(&is->joint_list);
	is->constraint_remove_count = 0;

    if (old_max == pipeline->island_pool.count)
    {
        is->id = ds_IdConstruct(slot.index, 0);
    }
            
    is->id += DS_ID_GENERATION_INCREMENT;

    struct ds_SolverSet *s = pipeline->solver_set_pool.buf + set;
    is->set = set;
    is->set_island_index = ds_CPoolPush(s->island_pool).index;
    s->island_pool.buf[ is->set_island_index ] = slot.index;

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
		c = pipeline->cdb->contact_pool.buf + i;
	}
	fprintf(file, "}\n");

	fprintf(file, "\tContacts (Body0, Shape0, Body1, Shape1):     { ");
	for (i32 i = is->contact_list.first; i != DLL_SENTINEL; i = c->island_contact.next)
	{
		c = pipeline->cdb->contact_pool.buf + i;
		fprintf(file, "((%u,%u)(%u,%u)) ", c->key.body0, c->key.shape0, c->key.body1, c->key.shape1);
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
	const struct cdb *c_db = pipeline->cdb;

	const struct ds_Island *is = NULL;
	const struct ds_RigidBody *body = NULL;
	const struct ds_Contact *c = NULL;

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

	    /* 3. if island no contacts, ds_Assert body.contacts == NULL */
	    if (is->contact_list.count == 0)
	    {
	    	ds_Assert(is->body_list.count == 1 || is->constraint_remove_count);
	    	body = pipeline->body_pool.buf + is->body_list.first;
	    	ds_Assert(ds_PoolSlotAllocated(body));
            const struct ds_Shape *shape = NULL;
            for (u32 s = body->shape_list.first; (i32) s != DLL_SENTINEL; s = shape->body_shape.next)
            {
                shape = pipeline->shape_pool.buf + s;
                ds_Assert(ds_PoolSlotAllocated(shape) && shape->contact_list.first == DLL_SENTINEL);
            }
	    }
	    else
	    {
	    	/* 
	    	 * 4. For each contact in island
	    	 * 	1. check contact exist
	    	 * 	2. check bodies in contact are mapped to island
	    	 */
	    	list_length = 0;
	    	struct ds_Contact *c = NULL;
	    	for (u32 ci = is->contact_list.first; (i32) ci != DLL_SENTINEL; ci = c->island_contact.next)
	    	{
	    		list_length += 1;
	    		c = c_db->contact_pool.buf + ci;
	    		ds_Assert(ds_PoolSlotAllocated(c));
	    		const struct ds_RigidBody *b0 = pipeline->body_pool.buf + c->key.body0;
	    		const struct ds_RigidBody *b1 = pipeline->body_pool.buf + c->key.body1;
	    		ds_Assert((b0->island == index) || RB_IS_STATIC(b0));
	    		ds_Assert((b1->island == index) || RB_IS_STATIC(b1));
                ds_Assert(c->island == index);
	    	}
	    	ds_Assert(list_length == is->contact_list.count);
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

    /* 6. verify no contact points to invalid island */
	for (u32 i = 0; i < pipeline->cdb->contact_pool.count_max; ++i)
	{
		struct ds_Contact *c = pipeline->cdb->contact_pool.buf + i;
		if (ds_PoolSlotAllocated(c))
		{
			struct ds_Island *is = pipeline->island_pool.buf + c->island;
			ds_Assert(ds_PoolSlotAllocated(is));
		}
	}
}

void ds_IslandMerge(struct ds_RigidBodyPipeline *pipeline, const u32 expand, const u32 merge, const u32 ci)
{
    ProfZone;
    struct ds_Contact *c = pipeline->cdb->contact_pool.buf + ci;
    
	//ds_IslandPrint(stderr, pipeline, expand, "To Expand");
	//ds_IslandPrint(stderr, pipeline, merge, "To Merge");
    
	struct ds_Island *is_expand = pipeline->island_pool.buf + expand;
	struct ds_Island *is_merge = pipeline->island_pool.buf + merge;
    
    if (is_expand->set >= SOLVER_SET_SLEEPING_FIRST)
    {
        ds_SolverSetWakeUp(pipeline, is_expand->set);
	    PhysicsEventIslandAwake(pipeline, expand);	
    }

    c->island = expand;

	/* new local contact within island */
	if (expand == merge)
	{
		struct ds_Island *is = pipeline->island_pool.buf + expand;
		ds_Assert(is->contact_list.count != 0 || is->constraint_remove_count);
		ds_DLLAppend(is->contact_list, pipeline->cdb->contact_pool.buf, ci, island_contact);
	}
	/* new contact between distinct islands */
	else
	{
        is_expand->constraint_remove_count += is_merge->constraint_remove_count;
        if (is_merge->set >= SOLVER_SET_SLEEPING_FIRST)
        {
            ds_SolverSetMerge(pipeline, SOLVER_SET_ACTIVE, is_merge->set);
        }

		if (is_expand->contact_list.count == 0)
		{
			is_expand->contact_list.first = ci;
			c->island_contact.prev = DLL_SENTINEL;
		}
		else
		{
			struct ds_Contact *contact = pipeline->cdb->contact_pool.buf + is_expand->contact_list.last;
			ds_Assert(contact->island_contact.next == DLL_SENTINEL);
			contact->island_contact.next = ci;
			c->island_contact.prev = is_expand->contact_list.last;
		}

		if (is_merge->contact_list.count == 0)
		{
			is_expand->contact_list.last = ci;
			c->island_contact.next = DLL_SENTINEL;
		}
		else
		{
			is_expand->contact_list.last = is_merge->contact_list.last;
			struct ds_Contact *contact = pipeline->cdb->contact_pool.buf + is_merge->contact_list.first;
			ds_Assert(contact->island_contact.prev == DLL_SENTINEL);
			contact->island_contact.prev = ci;
			c->island_contact.next = is_merge->contact_list.first;
		}

		is_expand->body_list.count += is_merge->body_list.count;
		is_expand->contact_list.count += is_merge->contact_list.count + 1;

		struct ds_RigidBody *body = pipeline->body_pool.buf + is_expand->body_list.last;
		struct ds_RigidBody *body2 = pipeline->body_pool.buf + is_merge->body_list.first;
		ds_Assert(body->island_body.next == DLL_SENTINEL);
		ds_Assert(body2->island_body.prev == DLL_SENTINEL);
		body->island_body.next = is_merge->body_list.first;
		body2->island_body.prev = is_expand->body_list.last;
		is_expand->body_list.last = is_merge->body_list.last;

		for (u32 i = is_merge->body_list.first; (i32) i != DLL_SENTINEL; i = body->island_body.next)
		{
			body = pipeline->body_pool.buf + i;
			body->island = expand;
		}

        struct ds_Contact *merge_contact;
        for (u32 i = is_merge->contact_list.first; (i32) i != DLL_SENTINEL; i = merge_contact->island_contact.next)
        {
            merge_contact = pipeline->cdb->contact_pool.buf + i;
            merge_contact->island = expand;
        }

		PhysicsEventIslandExpanded(pipeline, expand);
		ds_IslandRemove(pipeline, merge);
	}

	//ds_IslandPrint(stderr, pipeline, expand, "Expanded");
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
	PhysicsEventIslandRemoved(pipeline, island_index);
}

void ds_IslandSplit(struct ds_RigidBodyPipeline *pipeline, const u32 island_to_split)
{
    ProfZone;
    struct arena *mem_tmp = ArenaPushScratch();

	struct ds_Island *split_island = pipeline->island_pool.buf + island_to_split;
	//isdb_PrintIsland(stderr, pipeline, island_to_split, "To Split");
	u32 *body_stack = ArenaPush(mem_tmp, split_island->body_list.count*sizeof(u32));
	u32 sc;
    const u32 new_set = (split_island->set == SOLVER_SET_DISABLED)
                      ? SOLVER_SET_DISABLED
                      : SOLVER_SET_ACTIVE;

	for (u32 bi = split_island->body_list.first; (i32) bi != DLL_SENTINEL; )
	{
	    struct ds_RigidBody *body_last = pipeline->body_pool.buf + bi;
		ds_Assert(body_last->island == island_to_split);
		const struct slot slot = ds_IslandAlloc(pipeline, new_set);
        const u32 new_island = slot.index;
		split_island = pipeline->island_pool.buf + island_to_split;
        /* Note: we set this manually here as to skip the check 
         * neighbour_island == island_to_split for the body */
		body_last->island = slot.index;

        body_stack[0] = bi;
        sc = 1;
        while (sc--)
        {
            const u32 bi_cur = body_stack[sc];
			struct ds_RigidBody *body = pipeline->body_pool.buf + bi_cur;
            struct ds_Shape *shape = NULL;
            for (u32 si = body->shape_list.first; (i32) si != DLL_SENTINEL; si = shape->body_shape.next)
            {
                shape = pipeline->shape_pool.buf + si;
            	u32 ci = shape->contact_list.first;
                for (u32 co = 0; co < shape->contact_list.count; ++co)
                {
		    		const struct ds_Contact *c = pipeline->cdb->contact_pool.buf + ci;
		    		ds_Assert(ci >= pipeline->cdb->contact_frame_usage.bit_count 
		    				|| ds_BitSetGet(&pipeline->cdb->contact_frame_usage, ci) == 1);

                    u32 neighbour_index;
                    if (bi_cur == c->key.body0)
                    {
                        neighbour_index = c->key.body1;
		    		    ci = c->shape_contact[0].next;
                    }
                    else
                    {
                        neighbour_index = c->key.body0;
		    		    ci = c->shape_contact[1].next;
                    }

		    		body = pipeline->body_pool.buf + neighbour_index;
		    		const u32 neighbour_island = body->island;
              		if (neighbour_island == island_to_split)
		      		{
		      			ds_DLLRemove(split_island->body_list, pipeline->body_pool.buf, neighbour_index, island_body);
		      			ds_IslandAddBody(pipeline, new_island, neighbour_index);
		      			body_stack[sc++] = neighbour_index;
		      		}
		    	}
            }
        }

		const u32 tmp = body_last->island_body.next;
		ds_DLLRemove(split_island->body_list, pipeline->body_pool.buf, bi, island_body);
		ds_IslandAddBody(pipeline, new_island, bi);
		bi = tmp;
		//isdb_PrintIsland(stderr, pipeline, slot.index, "New Island (without contacts)");
	}

    struct ds_SolverSet *set = pipeline->solver_set_pool.buf + split_island->set;
	/* create contact lists of new islands */
    if (split_island->set >= SOLVER_SET_SLEEPING_FIRST)
    {
        /* 
         * We only move contacts if split was sleeping since ACTIVE/DISABLED contacts 
         * stay in the same set as before 
         */
        for (u32 i = 0; i < set->contact_pool.count; ++i)
        {
            const u32 ci = set->contact_pool.buf[i];
	    	if (ci >= pipeline->cdb->contact_frame_usage.bit_count || ds_BitSetGet(&pipeline->cdb->contact_frame_usage, ci) == 1)
	    	{
                struct ds_Contact *c = pipeline->cdb->contact_pool.buf + ci;
                ds_CGraphContactAdd(pipeline, c);

                const struct ds_RigidBody *body0 = pipeline->body_pool.buf + c->key.body0;
	    	    const struct ds_RigidBody *body1 = pipeline->body_pool.buf + c->key.body1;
	    	    const u32 island0 = body0->island;
	    	    const u32 island1 = body1->island;
	    	    struct ds_Island *is = RB_IS_DYNAMIC(body0)
	    	    	? pipeline->island_pool.buf + island0
	    	    	: pipeline->island_pool.buf + island1;
	    	    ds_DLLAppend(is->contact_list, pipeline->cdb->contact_pool.buf, ci, island_contact);
                c->island = ds_IslandPoolIndex(&pipeline->island_pool, is);
            }
        }
    }
    else
    {
	    struct ds_Contact *c;
	    u32 next;
	    for (u32 i = split_island->contact_list.first; (i32) i != DLL_SENTINEL; i = next)
	    {
	        c = pipeline->cdb->contact_pool.buf + i;
	    	next = c->island_contact.next;
	    	if (i >= pipeline->cdb->contact_frame_usage.bit_count || ds_BitSetGet(&pipeline->cdb->contact_frame_usage, i) == 1)
	    	{

	    		const struct ds_RigidBody *body0 = pipeline->body_pool.buf + c->key.body0;
	    		const struct ds_RigidBody *body1 = pipeline->body_pool.buf + c->key.body1;
	    		const u32 island0 = body0->island;
	    		const u32 island1 = body1->island;
	    		struct ds_Island *is = RB_IS_DYNAMIC(body0)
	    			? pipeline->island_pool.buf + island0
	    			: pipeline->island_pool.buf + island1;
	    		ds_DLLAppend(is->contact_list, pipeline->cdb->contact_pool.buf, i, island_contact);
                c->island = ds_IslandPoolIndex(&pipeline->island_pool, is);
	    	}
	    }
    }
                
    //TODO issue: joints/(bodies???) in old island, need to update 
	ds_IslandRemove(pipeline, island_to_split);
    ArenaPopScratch();
    ProfZoneEnd;
}
