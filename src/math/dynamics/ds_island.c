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

#include "ds_job.h"

/* Add new body to island */
static void ds_IslandAddBody(struct ds_Dynamics *pipeline, const u32 island_index, const u32 body)
{
    struct ds_Island *is = pipeline->island_pool.buf + island_index;
	struct ds_Body *b = pipeline->body_pool.buf + body;
	b->island = ds_PoolIndex(pipeline->island_pool, is);
    if (b->set != is->set)
    {
        ds_SolverSetMoveBody(pipeline, body, is->set);
    }
	DLLAppend(is->body_list, pipeline->body_pool.buf, body, island_body);
}

static struct slot ds_IslandAlloc(struct ds_Dynamics *pipeline, const u32 set)
{
    ds_Assert(set != SOLVER_SET_STATIC);

    /* count_max, the high-water mark: a reused slot keeps its id and only advances the generation (as ds_BodyAdd) */
    const u32 old_max = pipeline->island_pool.count_max;
	struct slot slot = ds_PoolAdd(pipeline->island_pool);
    if (pipeline->island_high_energy_set.bit_count < pipeline->island_pool.count)
    {
        ds_BitSetIncreaseSize(&pipeline->island_high_energy_set, pipeline->island_pool.count, 0);
    }

	struct ds_Island *is = slot.address;
	DLLFlush(is->body_list);
	DLLFlush(is->contact_list);
    DLLFlush(is->joint_list);
	is->constraint_remove_count = 0;

    if (old_max != pipeline->island_pool.count_max)
    {
        is->id = ds_IslandIdInit(slot.index, 0);
    }
            
    is->id = ds_IslandIdNextGeneration(is->id);

    struct ds_SolverSet *s = pipeline->solver_set_pool.buf + set;
    is->set = set;
    is->set_island_index = ds_CPoolPush(s->island_pool).index;
    s->island_pool.buf[ is->set_island_index ] = slot.index;
	PhysicsEventIslandNew(pipeline, is->id);

	return slot;
}

struct slot ds_IslandLookup(struct ds_Dynamics *pipeline, const ds_IslandId id)
{
    struct slot slot = { .address = NULL, .index = U32_MAX };
    /* the index is bounded before the slot is read (as ds_BodyLookup) */
    const u32 index = ds_IslandIdIndex(id);
    if (index < pipeline->island_pool.count_max)
    {
        struct ds_Island *island = pipeline->island_pool.buf + index;
        if (ds_PoolSlotAllocated(island) && ds_IslandIdEqual(island->id, id))
        {
            slot.address = island;
            slot.index = index;
        }
    }

    return slot;
}

void ds_IslandPrint(FILE *file, const struct ds_Dynamics *pipeline, const u32 island, const char *desc)
{
	const struct ds_Island *is = pipeline->island_pool.buf + island;

	const struct ds_Contact *c;
	const struct ds_Body *b;

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

void ds_IslandValidateAll(const struct ds_Dynamics *pipeline)
{
	const struct ds_Island *is = NULL;
	const struct ds_Body *body = NULL;

    for (u32 index = 0; index < pipeline->island_pool.count_max; ++index)
    {
	    is = pipeline->island_pool.buf + index;
        if (!ds_PoolSlotAllocated(is))
        {
            continue;
        }

	    u32 list_length = 0; 
	    for (u32 bi = is->body_list.first; (i32) bi != DLL_SENTINEL; bi = body->island_body.next)
	    {
	    	list_length += 1;
	    	body = pipeline->body_pool.buf + bi;
	    	ds_Assert(ds_PoolSlotAllocated(body) && body->island == index);
	    }
	    ds_Assert(list_length == is->body_list.count);

        u32 touching_contacts = 0; 
	    u32 colored_contacts = 0;
	    struct ds_Contact *c = NULL;
	    for (u32 ci = is->contact_list.first; (i32) ci != DLL_SENTINEL; ci = c->island_contact.next)
	    {
	    	c = pipeline->contact_pool.buf + ci;
	    	ds_Assert(ds_PoolSlotAllocated(c));
	    	const struct ds_Body *b0 = pipeline->body_pool.buf + c->body[0];
	    	const struct ds_Body *b1 = pipeline->body_pool.buf + c->body[1];
	    	ds_Assert((b0->island == index) || ds_BodyStaticCheck(b0));
	    	ds_Assert((b1->island == index) || ds_BodyStaticCheck(b1));
            ds_Assert(c->island == index);
            if (c->color != CG_INVALID_COLOR)
            {
                colored_contacts += 1;
            }

            if (pipeline->contact_narrow_pool.buf[ci].count)
            {
                touching_contacts += 1;
            }
	    }

	    ds_Assert(touching_contacts == is->contact_list.count);
        if (is->set == SOLVER_SET_ACTIVE)
        {
	        ds_Assert(colored_contacts == is->contact_list.count);
        }

        u32 joint_list_length = 0;
        const struct ds_Joint *joint = NULL;
        for (u32 ji = is->joint_list.first; (i32) ji != DLL_SENTINEL; ji = joint->island_joint.next)
        {
            joint_list_length += 1;
            joint = pipeline->joint_pool.buf + ji;
            ds_Assert(ds_PoolSlotAllocated(joint));
            ds_Assert(joint->island == index);
            const struct ds_Body *b0 = pipeline->body_pool.buf + joint->body[0];
            const struct ds_Body *b1 = pipeline->body_pool.buf + joint->body[1];
            ds_Assert(b0->island == index);
            ds_Assert((b1->island == index) || ds_JointStaticCheck(joint));
            if (is->set == SOLVER_SET_ACTIVE)
            {
                ds_Assert(joint->set == SOLVER_SET_NULL && joint->color != CG_INVALID_COLOR);
            }
            else
            {
                ds_Assert(joint->set == is->set && joint->color == CG_INVALID_COLOR);
            }
        }
        ds_Assert(joint_list_length == is->joint_list.count);
	}

	/* 5. verify no body points to invalid island */
	for (u32 i = 0; i < pipeline->body_pool.count_max; ++i)
	{
		struct ds_Body *body = pipeline->body_pool.buf + i;
		if (ds_PoolSlotAllocated(body) && ds_BodyDynamicCheck(body))
		{
			struct ds_Island *is = pipeline->island_pool.buf + body->island;
			ds_Assert(ds_PoolSlotAllocated(is));
		}
	}
}

u32 ds_IslandMerge(struct ds_Dynamics *pipeline, const u32 island_index0, const u32 island_index1)
{
    ProfZone;
    ds_Assert(island_index0 != island_index1);

    /* the smaller island is relabelled */
    const struct ds_Island *island0 = pipeline->island_pool.buf + island_index0;
    const struct ds_Island *island1 = pipeline->island_pool.buf + island_index1;
    const u32 count0 = island0->body_list.count + island0->contact_list.count + island0->joint_list.count;
    const u32 count1 = island1->body_list.count + island1->contact_list.count + island1->joint_list.count;

    const u32 expand_index = (count1 > count0 || (count1 == count0 && island_index1 < island_index0)) ? island_index1 : island_index0;
    const u32 merge_index = (expand_index == island_index0) ? island_index1 : island_index0;

	struct ds_Island *expand = pipeline->island_pool.buf + expand_index;
	struct ds_Island *merge = pipeline->island_pool.buf + merge_index;
    ds_SolverSetWakeUp(pipeline, expand->set);
    ds_SolverSetWakeUp(pipeline, merge->set);

    {
        struct ds_Contact *expand_last = pipeline->contact_pool.buf + expand->contact_list.last;
        struct ds_Contact *merge_first = pipeline->contact_pool.buf + merge->contact_list.first;
	    ds_Assert(expand->contact_list.count == 0 || expand_last->island_contact.next == DLL_SENTINEL);
	    ds_Assert(merge->contact_list.count == 0 || merge_first->island_contact.prev == DLL_SENTINEL);

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

        expand->contact_list.count += merge->contact_list.count;
    }

    {
        struct ds_Joint *expand_last = pipeline->joint_pool.buf + expand->joint_list.last;
        struct ds_Joint *merge_first = pipeline->joint_pool.buf + merge->joint_list.first;
	    ds_Assert(expand->joint_list.count == 0 || expand_last->island_joint.next == DLL_SENTINEL);
	    ds_Assert(merge->joint_list.count == 0 || merge_first->island_joint.prev == DLL_SENTINEL);

        expand_last->island_joint.next = merge->joint_list.first;
        merge_first->island_joint.prev = expand->joint_list.last;

        if (expand->joint_list.count == 0)
        {
            expand->joint_list.first = merge->joint_list.first;
        }

        if (merge->joint_list.count > 0)
        {
            expand->joint_list.last = merge->joint_list.last;
        }

        struct ds_Joint *merge_joint;
        for (i32 i = merge->joint_list.first; i != DLL_SENTINEL; i = merge_joint->island_joint.next)
        {
            merge_joint = pipeline->joint_pool.buf + i;
            merge_joint->island = expand_index;
        }

        expand->joint_list.count += merge->joint_list.count;
    }

    {
	    struct ds_Body *expand_last = pipeline->body_pool.buf + expand->body_list.last;
	    struct ds_Body *merge_first = pipeline->body_pool.buf + merge->body_list.first;
	    ds_Assert(expand_last->island_body.next == DLL_SENTINEL);
	    ds_Assert(merge_first->island_body.prev == DLL_SENTINEL);

	    expand_last->island_body.next = merge->body_list.first;
	    merge_first->island_body.prev = expand->body_list.last;

        struct ds_Body *body;
	    for (i32 i = merge->body_list.first; i != DLL_SENTINEL; i = body->island_body.next)
	    {
	    	body = pipeline->body_pool.buf + i;
	    	body->island = expand_index;
	    }
	
        expand->body_list.count += merge->body_list.count;
	    expand->body_list.last = merge->body_list.last;
    }

    expand->constraint_remove_count += merge->constraint_remove_count;
	ds_IslandRemove(pipeline, merge_index);
	
    ProfZoneEnd;
    return expand_index;
}

void ds_IslandRemove(struct ds_Dynamics *pipeline, const u32 island_index)
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
	ds_PoolRemove(pipeline->island_pool, island_index);
}

struct ds_PartialIsland
{
    u32 component;
};

struct ds_PartialIslandMember
{
    u32 partial_island;
    u32 index;
};

struct ds_IslandSplitState
{
    struct ds_PartialIsland *       partial;
    struct ds_PartialIslandMember * body_to_partial;
    struct ds_PartialIslandMember * contact_to_partial;
    struct ds_PartialIslandMember * joint_to_partial;
    u32                             partial_count;
    u32                             fuse_count;
    u32                             body_to_partial_count;
    u32                             contact_to_partial_count;
    u32                             joint_to_partial_count;
    u32                             island;
};

#define DS_PARTIAL_ISLAND_VISITED_BIT   ((u32) 1 << 31)

/* Push the split's arrays for island onto mem and start with no partial island; fatal if mem is too small */
static struct ds_IslandSplitState ds_IslandSplitStateAlloc(struct arena *mem, const struct ds_Dynamics *pipeline, const u32 island)
{
    const struct ds_Island *split = pipeline->island_pool.buf + island;

    /* ZERO island is reserved for denoting no-island. island.component == 0 means it is
     * root component, and any islands with component != 0, is a sub-component of partial
     * island indexed by component.*/
    struct ds_IslandSplitState s =
    {
        .partial_count = 1,
        .fuse_count = 0,
        .body_to_partial_count = 0,
        .contact_to_partial_count = 0,
        .joint_to_partial_count = 0,
        .island = island,
    };
    s.partial = ArenaPush(mem, (split->body_list.count+1)*sizeof(struct ds_PartialIsland));
    s.body_to_partial = ArenaPushZero(mem, split->body_list.count*sizeof(struct ds_PartialIslandMember));
    s.contact_to_partial = ArenaPush(mem, split->contact_list.count*sizeof(struct ds_PartialIslandMember));
    s.joint_to_partial = ArenaPush(mem, split->joint_list.count*sizeof(struct ds_PartialIslandMember));

    if (!s.partial || !s.body_to_partial || (split->contact_list.count && !s.contact_to_partial) || (split->joint_list.count && !s.joint_to_partial))
    {
        LogString(T_PHYSICS, S_FATAL, "out-of-memory in island split, increase dynamics_worker_framesize!");
        FatalCleanupAndExit();
    }

    return s;
}

/* Return the component partial island p belongs to; shortens the fused chain it walks */
static u32 ds_PartialIslandComponent(struct ds_PartialIsland *partial, u32 p)
{
    while (partial[p].component)
    {
        const u32 fused = partial[p].component;
        if (!partial[fused].component)
        {
            return fused;
        }
        partial[p].component = partial[fused].component;
        p = partial[fused].component;
    }
    return p;
}

/* Return the body's index in body_to_partial; maps it on its first visit */
static u32 ds_IslandSplitBody(struct ds_IslandSplitState *s, struct ds_Dynamics *pipeline, const u32 body)
{
    struct ds_Body *b = pipeline->body_pool.buf + body;
    if (b->island == s->island)
    {
        b->island = DS_PARTIAL_ISLAND_VISITED_BIT | s->body_to_partial_count;
        s->body_to_partial[ s->body_to_partial_count ].index = body;
        s->body_to_partial_count += 1;
    }
    return b->island & ~DS_PARTIAL_ISLAND_VISITED_BIT;
}

/*
 * Return the partial island of a contact or joint between body[0] and body[1] (static_index: the static body's slot,
 * U32_MAX if both are dynamic); maps unvisited dynamic bodies and fuses their components (counted in fuse_count)
 */
static u32 ds_IslandSplitConstraint(struct ds_IslandSplitState *s, struct ds_Dynamics *pipeline, const u32 body[2], const u32 static_index)
{
    if (static_index != U32_MAX)
    {
        const u32 m = ds_IslandSplitBody(s, pipeline, body[1 - static_index]);

        u32 partial_island = s->body_to_partial[m].partial_island;
        if (!partial_island)
        {
            /* Begin a new component and island */
            partial_island = s->partial_count;
            s->partial_count += 1;
            s->partial[ partial_island ].component = 0;
            s->body_to_partial[m].partial_island = partial_island;
        }
        return partial_island;
    }

    u32 m[2];
    m[0] = ds_IslandSplitBody(s, pipeline, body[0]);
    m[1] = ds_IslandSplitBody(s, pipeline, body[1]);

    const u32 pi[2] =
    {
        s->body_to_partial[ m[0] ].partial_island,
        s->body_to_partial[ m[1] ].partial_island,
    };
    
    u32 partial_island = (pi[0]) ? pi[0] : pi[1];
    if (pi[0] && pi[1])
    {
        /* Begin a two components and islands exist, one partial island
         * becomes sub-component and points to other component */
        const u32 component0 = ds_PartialIslandComponent(s->partial, pi[0]);
        const u32 component1 = ds_PartialIslandComponent(s->partial, pi[1]);

        /* fused entries point to lower indices */
        partial_island = U32Min(component0, component1);
        if (component0 != component1)
        {
            s->partial[ U32Max(component0, component1) ].component = partial_island;
            s->fuse_count += 1;
        }
    }
    else if (!partial_island)
    {
        /* Begin a new component and island */
        partial_island = s->partial_count;
        s->partial_count += 1;
        s->partial[ partial_island ].component = 0;
    }

    s->body_to_partial[ m[0] ].partial_island = partial_island;
    s->body_to_partial[ m[1] ].partial_island = partial_island;
    return partial_island;
}

void ds_IslandSplit(struct ds_Dynamics *pipeline, const u32 island_to_split)
{
    ProfZone;

    struct arena *mem = pipeline->runtime.worker[ds_ThreadSelfIndex()].frame;
    ArenaPushRecord(mem);

    struct ds_Island *split = pipeline->island_pool.buf + island_to_split;
    ds_Assert(split->set == SOLVER_SET_ACTIVE);
    ds_Assert(split->constraint_remove_count > 0);

    struct ds_IslandSplitState s = ds_IslandSplitStateAlloc(mem, pipeline, island_to_split);

    {
        ProfZoneNamed("Contact Walk");
        const struct ds_Contact *c = NULL;
        for (i32 ci = split->contact_list.first; ci != DLL_SENTINEL; ci = c->island_contact.next)
        {
            c = pipeline->contact_pool.buf + ci;

            const u32 static_index = (ds_ContactStaticCheck(c)) ? ds_ContactStaticIndex(c) : U32_MAX;
            const u32 partial_island = ds_IslandSplitConstraint(&s, pipeline, c->body, static_index);
            s.contact_to_partial[ s.contact_to_partial_count++ ] = (struct ds_PartialIslandMember) { .index = (u32) ci, .partial_island = partial_island };
        }
        ProfZoneEnd;
    }

    {
        ProfZoneNamed("Joint Walk");
        const struct ds_Joint *j = NULL;
        for (i32 ji = split->joint_list.first; ji != DLL_SENTINEL; ji = j->island_joint.next)
        {
            j = pipeline->joint_pool.buf + ji;

            const u32 static_index = (ds_JointStaticCheck(j)) ? 1 : U32_MAX;
            const u32 partial_island = ds_IslandSplitConstraint(&s, pipeline, j->body, static_index);
            s.joint_to_partial[ s.joint_to_partial_count++ ] = (struct ds_PartialIslandMember) { .index = (u32) ji, .partial_island = partial_island };
        }
        ProfZoneEnd;
    }

    const u32 component_count = (s.partial_count - 1) - s.fuse_count + (split->body_list.count - s.body_to_partial_count);
    if (component_count <= 1)
    {
        for (u32 i = 0; i < s.body_to_partial_count; ++i)
        {
            pipeline->body_pool.buf[ s.body_to_partial[i].index ].island = island_to_split;
        }
        split->constraint_remove_count = 0;
        goto end;
    }

    struct DLL body_list = split->body_list;
    ds_IslandRemove(pipeline, island_to_split);

    {
        ProfZoneNamed("Component Array Walk");
        /* a component gets a new island; a fused partial island takes the island of the lower index it points to */
        for (u32 p = 1; p < s.partial_count; ++p)
        {
            if (s.partial[p].component == 0)
            {
                s.partial[p].component = ds_IslandAlloc(pipeline, SOLVER_SET_ACTIVE).index;
            }
            else
            {
                s.partial[p].component = s.partial[ s.partial[p].component ].component;
            }
        }
        ProfZoneEnd;
    }

    {
        ProfZoneNamed("Body Array Walk");
        for (u32 i = 0; i < s.body_to_partial_count; ++i)
        {
            const u32 bi = s.body_to_partial[i].index;
            DLLRemove(body_list, pipeline->body_pool.buf, bi, island_body);
            ds_IslandAddBody(pipeline, s.partial[ s.body_to_partial[i].partial_island ].component, bi);
        }
        ProfZoneEnd;
    }

    {
        ProfZoneNamed("Contact Array Walk");
        for (u32 i = 0; i < s.contact_to_partial_count; ++i)
        {
            const u32 ci = s.contact_to_partial[i].index;
            const u32 island_index = s.partial[ s.contact_to_partial[i].partial_island ].component;
            struct ds_Island *island = pipeline->island_pool.buf + island_index;
            pipeline->contact_pool.buf[ci].island = island_index;
            DLLAppend(island->contact_list, pipeline->contact_pool.buf, ci, island_contact);
        }
        ProfZoneEnd;
    }

    {
        ProfZoneNamed("Joint Array Walk");
        for (u32 i = 0; i < s.joint_to_partial_count; ++i)
        {
            const u32 ji = s.joint_to_partial[i].index;
            const u32 island_index = s.partial[ s.joint_to_partial[i].partial_island ].component;
            struct ds_Island *island = pipeline->island_pool.buf + island_index;
            pipeline->joint_pool.buf[ji].island = island_index;
            DLLAppend(island->joint_list, pipeline->joint_pool.buf, ji, island_joint);
        }
        ProfZoneEnd;
    }

    /* the bodies left in the old list have no contact or joint: an island each */
    while (body_list.first != DLL_SENTINEL)
    {
        const i32 bi = body_list.first;
        DLLRemove(body_list, pipeline->body_pool.buf, bi, island_body);
        const u32 island_index = ds_IslandAlloc(pipeline, SOLVER_SET_ACTIVE).index;
        ds_IslandAddBody(pipeline, island_index, (u32) bi);
    }

end:
    ArenaPopRecord(mem);

    ProfZoneEnd;
}
