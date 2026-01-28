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

#include "sys_common.h"
#include "dynamics.h"
#include "quaternion.h"

/* Add new body to island */
static void is_db_internal_add_body_to_island(struct physics_pipeline *pipeline, struct island *is, const u32 body)
{
	ds_Assert(is->body_first != ISLAND_NULL && is->body_last != ISLAND_NULL);

	const u32 is_index = (u32) array_list_index(pipeline->is_db.islands, is);

	struct rigid_body *b = PoolAddress(&pipeline->body_pool, body);
	b->island_index = is_index;

	const u32 b_index = (u32) array_list_reserve_index(pipeline->is_db.island_body_lists);
	struct is_index_entry *entry = array_list_address(pipeline->is_db.island_body_lists, b_index);

	entry->next = is->body_first;
	entry->index = body;
	is->body_first = b_index;
	is->body_count += 1;
}

/* Add new contact to island */
static void is_db_internal_add_contact_to_island(struct island_database *is_db, struct island *is, const u32 contact)
{
	const u32 c_index = (u32) array_list_reserve_index(is_db->island_contact_lists);
	struct is_index_entry *entry = array_list_address(is_db->island_contact_lists, c_index);

	entry->next = is->contact_first;
	entry->index = contact;
	is->contact_first = c_index;
	is->contact_count += 1;

	if (is->contact_last == ISLAND_NULL)
	{
		is->contact_last = c_index;
	}
}

void is_db_add_contact_to_island(struct island_database *is_db, const u32 island, const u32 contact)
{
	struct island *is = array_list_address(is_db->islands, island);
	is_db_internal_add_contact_to_island(is_db, is, contact);
}

struct island *is_db_init_island_from_body(struct physics_pipeline *pipeline, const u32 body)
{
	struct rigid_body *b = PoolAddress(&pipeline->body_pool, body);
	b->island_index = (u32) array_list_reserve_index(pipeline->is_db.islands);
	PHYSICS_EVENT_ISLAND_NEW(pipeline, b->island_index);
	if (pipeline->is_db.island_usage.bit_count <= b->island_index)
	{
		BitVecIncreaseSize(&pipeline->is_db.island_usage, PowerOfTwoCeil(b->island_index+1), 0);
	}
	BitVecSetBit(&pipeline->is_db.island_usage, b->island_index, 1);

	struct island *is = array_list_address(pipeline->is_db.islands, b->island_index);
	is->contact_first = ISLAND_NULL;
	is->contact_last = ISLAND_NULL;
	is->contact_count = 0;
	is->body_first = (u32) array_list_reserve_index(pipeline->is_db.island_body_lists);
	is->body_last = is->body_first;
	is->body_count = 1;
	is->flags = g_solver_config->sleep_enabled * (ISLAND_AWAKE | ISLAND_SLEEP_RESET);

	struct is_index_entry *entry = array_list_address(pipeline->is_db.island_body_lists, is->body_first);
	entry->next = ISLAND_NULL;
	entry->index = body;

	return is;
}

void is_db_print_island(FILE *file, const struct island_database *is_db, const struct contact_database *c_db, const u32 island, const char *desc)
{
	const struct island *is = array_list_address(is_db->islands, island);
	if (!is) { return; }

	const struct is_index_entry *entry;

	fprintf(file, "Island %u %s:\n{\n", island, desc);

	fprintf(file, "\tbody_count: %u\n", is->body_count);
	fprintf(file, "\tcontact_count: %u\n", is->contact_count);
		
	fprintf(file, "\t(ListIndex, Body):                     { ");
	for (u32 i = is->body_first; i != ISLAND_NULL; i = entry->next)
	{
		entry = array_list_address(is_db->island_body_lists, i);
		fprintf(file, "(%u,%u) ", i, entry->index);
	}
	fprintf(file, "}\n");

	fprintf(file, "\tContact Pointers (ListIndex, Contact): { ");
	for (u32 i = is->contact_first; i != ISLAND_NULL; i = entry->next)
	{
		entry = array_list_address(is_db->island_contact_lists, i);
		fprintf(file, "(%u,%u) ", i, entry->index);
	}
	fprintf(file, "}\n");

	fprintf(file, "\tContacts (Body1, Body2):               { ");
	for (u32 i = is->contact_first; i != ISLAND_NULL; i = entry->next)
	{
		entry = array_list_address(is_db->island_contact_lists, i);
		const struct contact *c = nll_Address(&c_db->contact_net, entry->index);
		fprintf(file, "(%u,%u) ", c->cm.i1, c->cm.i2);
	}
	fprintf(file, "}\n");

	fprintf(file, "\tflags:\n\t{\n");
	fprintf(file, "\t\tawake: %u\n", ISLAND_AWAKE_BIT(is));
	fprintf(file, "\t\tsleep_reset: %u\n", ISLAND_SLEEP_RESET_BIT(is));
	fprintf(file, "\t\tsplit: %u\n", ISLAND_SPLIT_BIT(is));
	fprintf(file, "\t}\n");

	fprintf(file, "}\n");
}

struct island_database is_db_alloc(struct arena *mem_persistent, const u32 initial_size)
{
	struct island_database is_db = { 0 };

	is_db.island_usage = BitVecAlloc(NULL, initial_size, 0, 1);
	is_db.islands = array_list_alloc(NULL, initial_size, sizeof(struct island), ARRAY_LIST_GROWABLE);
	is_db.island_contact_lists = array_list_alloc(NULL, initial_size, sizeof(struct is_index_entry), ARRAY_LIST_GROWABLE);
	is_db.island_body_lists = array_list_alloc(NULL, initial_size, sizeof(struct is_index_entry), ARRAY_LIST_GROWABLE);

	return is_db;
}

void is_db_free(struct island_database *is_db)
{
	array_list_free(is_db->island_contact_lists);
	array_list_free(is_db->island_body_lists);
	array_list_free(is_db->islands);
	BitVecFree(&is_db->island_usage);
}

void is_db_flush(struct island_database *is_db)
{
	is_db_clear_frame(is_db);
	BitVecClear(&is_db->island_usage, 0);	
	array_list_flush(is_db->islands);
	array_list_flush(is_db->island_contact_lists);
	array_list_flush(is_db->island_body_lists);
}

void is_db_clear_frame(struct island_database *is_db)
{
	is_db->possible_splits = NULL;
	is_db->possible_splits_count = 0;
}

void is_db_validate(const struct physics_pipeline *pipeline)
{
	const struct island_database *is_db = &pipeline->is_db;
	const struct contact_database *c_db = &pipeline->c_db;
	u32 base = 0;
	for (u64 block = 0; block < is_db->island_usage.block_count; ++block)
	{
		u64 island_block = is_db->island_usage.bits[block];
		u32 offset = 0;
		while (island_block)
		{
			const u32 tzc = Ctz64(island_block);
			offset += tzc;
			const u32 is_index = base + offset;
			offset += 1;
			island_block = ((tzc + 1) < 64) 
				? island_block >> (tzc + 1)
				: 0;
			const struct island *is = array_list_address(is_db->islands, is_index);

	 		/* 1. verify body-island map count == island.body_count */
			u32 count = 0;
			for (u32 j = 0; j < pipeline->body_pool.count_max; ++j)
			{
	
				const struct rigid_body *b = PoolAddress(&pipeline->body_pool, j);
				if (PoolSlotAllocated(b) && b->island_index == is_index)
				{
					count += 1;
				}	
			}
			
			ds_Assert(count == is->body_count && "Body count of island should be equal to the number of bodies mapped to the island");
	 
			/* 2. verify body-island map  == island.bodies */
			u32 list_length = 0; 
			const struct is_index_entry *entry;
			for (u32 index = is->body_first; index != ISLAND_NULL; index = entry->next)
			{
				list_length += 1;
				entry = array_list_address(is_db->island_body_lists, index);
				const struct rigid_body *b = PoolAddress(&pipeline->body_pool, entry->index);
				ds_Assert(b->island_index == is_index && PoolSlotAllocated(b));
			}
			ds_Assert(list_length == is->body_count);

			/* 3. if island no contacts, assert body.contacts == NULL */
			if (is->contact_count == 0)
			{
				ds_Assert(is->body_count == 1);
				entry = array_list_address(is_db->island_body_lists, is->body_first);
				struct rigid_body *body = PoolAddress(&pipeline->body_pool, entry->index);
				ds_Assert(body && body->first_contact_index == NLL_NULL);
			}
			else
			{
				/* 
				 * 4. For each contact in island
				 * 	1. check contact exist
				 * 	2. check bodies in contact are mapped to island
				 */
				list_length = 0;
				for (u32 index = is->contact_first; index != ISLAND_NULL; index = entry->next)
				{
					list_length += 1;
					entry = array_list_address(is_db->island_contact_lists, index);
					struct contact *c = nll_Address(&c_db->contact_net, entry->index);
					const struct rigid_body *b1 = PoolAddress(&pipeline->body_pool, c->cm.i1);
					const struct rigid_body *b2 = PoolAddress(&pipeline->body_pool, c->cm.i2);
					ds_Assert(PoolSlotAllocated(c));
					ds_Assert(c != NULL);
					ds_Assert((b1->island_index == is_index) || (b1->island_index == ISLAND_STATIC));
					ds_Assert((b2->island_index == is_index) || (b2->island_index == ISLAND_STATIC));
				}
				ds_Assert(list_length == is->contact_count);
			}
		}
		base += 64;
	}

	/* 5. verify no body points to invalid island */
	for (u32 i = 0; i < pipeline->body_pool.count_max; ++i)
	{
		struct rigid_body *body = PoolAddress(&pipeline->body_pool, i);
		if (PoolSlotAllocated(body) && body->island_index != ISLAND_NULL && body->island_index != ISLAND_STATIC)
		{
			const u32 island_valid = BitVecGetBit(&is_db->island_usage, body->island_index);
			ds_Assert(island_valid == 1);
		}
	}
}

struct island *is_db_body_to_island(struct physics_pipeline *pipeline, const u32 body)
{
	const u32 is_index = ((struct rigid_body *) PoolAddress(&pipeline->body_pool, body))->island_index;
	return (is_index != ISLAND_NULL && is_index != ISLAND_STATIC)
		? array_list_address(pipeline->is_db.islands, is_index)
		: NULL;
}

void is_db_reserve_splits_memory(struct arena *mem_frame, struct island_database *is_db)
{
	is_db->possible_splits = ArenaPush(mem_frame, is_db->islands->length * sizeof(u32));
}

void is_db_release_unused_splits_memory(struct arena *mem_frame, struct island_database *is_db)
{
	ArenaPopPacked(mem_frame, (is_db->islands->length - is_db->possible_splits_count) * sizeof(u32));
}

void is_db_tag_for_splitting(struct physics_pipeline *pipeline, const u32 body)
{
	const struct rigid_body *b = PoolAddress(&pipeline->body_pool, body);
	ds_Assert(b->island_index != U32_MAX);

	const u32 is_index = b->island_index;
	struct island *is = array_list_address(pipeline->is_db.islands, is_index);
	if (!(is->flags & ISLAND_SPLIT))
	{
		ds_Assert(pipeline->is_db.possible_splits_count < pipeline->is_db.islands->length);
		is->flags |= ISLAND_SPLIT;
		pipeline->is_db.possible_splits[pipeline->is_db.possible_splits_count++] = is_index;
	}
}

void is_db_merge_islands(struct physics_pipeline *pipeline, const u32 ci, const u32 b1, const u32 b2)
{
	const struct rigid_body *body1 = PoolAddress(&pipeline->body_pool, b1);
	const struct rigid_body *body2 = PoolAddress(&pipeline->body_pool, b2);

	const u32 expand = body1->island_index;
	const u32 merge = body2->island_index;

	//fprintf(stderr, "merging islands (%u, %u) -> (%u)\n", expand, merge, merge);

	const u32 new_index = (u32) array_list_reserve_index(pipeline->is_db.island_contact_lists);
	struct is_index_entry *new_contact = array_list_address(pipeline->is_db.island_contact_lists, new_index);
	new_contact->index = ci;

	/* new local contact within island */
	if (expand == merge)
	{
		struct island *is = array_list_address(pipeline->is_db.islands, expand);
		ds_Assert(is->contact_count != 0);
		ds_Assert(is->contact_last != ISLAND_NULL);

		new_contact->next = is->contact_first;
		is->contact_first = new_index;
		is->contact_count += 1;	
	}
	/* new contact between distinct islands */
	else
	{
		struct island *is_expand = array_list_address(pipeline->is_db.islands, expand);
		struct island *is_merge = array_list_address(pipeline->is_db.islands, merge);

		if (g_solver_config->sleep_enabled)
		{
			const u32 island_sleep_interrupted = 1 - ISLAND_AWAKE_BIT(is_merge)*ISLAND_AWAKE_BIT(is_expand)
						+ ISLAND_TRY_SLEEP_BIT(is_merge) + ISLAND_TRY_SLEEP_BIT(is_expand);
			ds_Assert(!(ISLAND_AWAKE_BIT(is_merge) == 0 && ISLAND_AWAKE_BIT(is_expand) == 0));
			if (island_sleep_interrupted)
			{
				if (!ISLAND_AWAKE_BIT(is_expand))
				{
					PHYSICS_EVENT_ISLAND_AWAKE(pipeline, expand);	
				}
				is_expand->flags = ISLAND_AWAKE | ISLAND_SLEEP_RESET;
			}
		}

		struct is_index_entry *link = NULL;

		is_expand->body_count += is_merge->body_count;
		is_expand->contact_count += is_merge->contact_count + 1;

		new_contact->next = is_expand->contact_first;
		is_expand->contact_first = new_index;
		is_expand->contact_last = (is_expand->contact_last == ISLAND_NULL)
					? new_index
					: is_expand->contact_last;

		if (is_merge->contact_count > 0)
		{
			link = array_list_address(pipeline->is_db.island_contact_lists, is_expand->contact_last);
			ds_Assert(link->next == ISLAND_NULL);
			link->next = is_merge->contact_first;
			is_expand->contact_last = is_merge->contact_last;
		}

		link = array_list_address(pipeline->is_db.island_body_lists, is_expand->body_last);
		ds_Assert(link->next == ISLAND_NULL);
		link->next = is_merge->body_first;
		is_expand->body_last = is_merge->body_last;

		for (u32 i = is_merge->body_first; i != ISLAND_NULL; i = link->next)
		{
			link = array_list_address(pipeline->is_db.island_body_lists, i);
			struct rigid_body *b = PoolAddress(&pipeline->body_pool, link->index);
			b->island_index = expand;
		}

		is_merge->contact_first = ISLAND_NULL;
		is_merge->body_first = ISLAND_NULL;
		is_merge->contact_last = ISLAND_NULL;
		is_merge->body_last = ISLAND_NULL;
		is_merge->contact_count = 0;
		is_merge->body_count = 0;
		array_list_remove_index(pipeline->is_db.islands, merge);
		BitVecSetBit(&pipeline->is_db.island_usage, merge, 0);
		PHYSICS_EVENT_ISLAND_EXPANDED(pipeline, expand);
		PHYSICS_EVENT_ISLAND_REMOVED(pipeline, merge);
	}
}

void is_db_island_remove(struct physics_pipeline *pipeline, struct island *island)
{
	struct is_index_entry *entry = NULL;
	u32 i = island->contact_first; 
	while (i != ISLAND_NULL)
	{
		entry = array_list_address(pipeline->is_db.island_contact_lists, i);
		const u32 tmp = entry->next;
		entry->next = ISLAND_NULL;
		entry->index = ISLAND_NULL;
		array_list_remove_index(pipeline->is_db.island_contact_lists, i);
		i = tmp;
	}

	i = island->body_first; 
	while (i != ISLAND_NULL)
	{
		entry = array_list_address(pipeline->is_db.island_body_lists, i);
		const u32 tmp = entry->next;
		entry->next = ISLAND_NULL;
		entry->index = ISLAND_NULL;
		array_list_remove_index(pipeline->is_db.island_body_lists, i);
		i = tmp;
	}

	island->contact_first = ISLAND_NULL;
	island->body_first = ISLAND_NULL;
	island->contact_last = ISLAND_NULL;
	island->body_last = ISLAND_NULL;
	island->contact_count = 0;
	island->body_count = 0;
	array_list_remove(pipeline->is_db.islands, island);
	const u32 island_index = array_list_index(pipeline->is_db.islands, island);
	BitVecSetBit(&pipeline->is_db.island_usage, island_index, 0);
	PHYSICS_EVENT_ISLAND_REMOVED(pipeline, island_index);
}

void is_db_island_remove_body_resources(struct physics_pipeline *pipeline, const u32 island_index, const u32 body)
{
	ds_Assert(BitVecGetBit(&pipeline->is_db.island_usage, island_index));

	struct island *island = array_list_address(pipeline->is_db.islands, island_index);
	struct is_index_entry *entry = NULL;
	struct is_index_entry *prev = NULL;
	u32 i = island->contact_first; 
	u32 prev_i = ISLAND_NULL;
	while (i != ISLAND_NULL)
	{
		entry = array_list_address(pipeline->is_db.island_contact_lists, i);
		const u32 next = entry->next;
		const struct contact *c = nll_Address(&pipeline->c_db.contact_net, entry->index);
		if (body == CONTACT_KEY_TO_BODY_0(c->key) || body == CONTACT_KEY_TO_BODY_1(c->key))
		{
			if (prev)
			{
				prev->next = entry->next;
			}
			else
			{
				island->contact_first = entry->next;
			}
			const u32 next = entry->next;
			entry->next = ISLAND_NULL;
			entry->index = ISLAND_NULL;
			array_list_remove_index(pipeline->is_db.island_contact_lists, i);
			island->contact_count -= 1;
		}
		else
		{
			prev_i = i;
			prev = entry;
		}
		i = next;
	}
	island->contact_last = prev_i;

	i = island->body_first; 
	prev_i = ISLAND_NULL;
	prev = NULL;
	while (i != ISLAND_NULL)
	{
		entry = array_list_address(pipeline->is_db.island_body_lists, i);
		if (entry->index == body)
		{
			if (prev)
			{
				prev->next = entry->next;
			}
			else
			{
				island->body_first = entry->next;
			}
			entry->next = ISLAND_NULL;
			entry->index = ISLAND_NULL;
			array_list_remove_index(pipeline->is_db.island_body_lists, i);
			island->body_count -= 1;
			if (i == island->body_last)
			{
				island->body_last = prev_i;
			}
			break;
		}	
		prev_i = i;
		prev = entry;
		i = entry->next;
	}
	ds_Assert(i != ISLAND_NULL);

	if (island->body_count == 0)
	{
		ds_Assert(island->contact_first == ISLAND_NULL);
		ds_Assert(island->body_first == ISLAND_NULL);
		ds_Assert(island->contact_last == ISLAND_NULL);
		ds_Assert(island->body_last == ISLAND_NULL);
		ds_Assert(island->contact_count == 0);
		ds_Assert(island->body_count == 0);
		array_list_remove_index(pipeline->is_db.islands, island_index);
		BitVecSetBit(&pipeline->is_db.island_usage, island_index, 0);
		PHYSICS_EVENT_ISLAND_REMOVED(pipeline, island_index);
	}
}

void is_db_split_island(struct arena *mem_tmp, struct physics_pipeline *pipeline, const u32 island_to_split)
{
	ArenaPushRecord(mem_tmp);

	struct island *split = array_list_address(pipeline->is_db.islands, island_to_split);
	//is_db_print_island(stderr, &pipeline->is_db, &pipeline->c_db, island_to_split, "SPLIT");
	ds_Assert(split->contact_count > 0);
	u32 sc = 0;
	u32 *body_stack = ArenaPush(mem_tmp, split->body_count * sizeof(u32));

	struct is_index_entry *entry = NULL;
	for (u32 i = split->body_first; i != ISLAND_NULL;)
	{
		entry = array_list_address(pipeline->is_db.island_body_lists, i);
		u32 body = entry->index;
		i = entry->next;
		struct rigid_body *b = PoolAddress(&pipeline->body_pool, body);
		/* 
		 * Build new island from the connected component of the body under consideration, 
		 * if we havn't already. We must add the contacts later, as to not add the same
		 * contact twice.
		 */
		if (b->island_index == island_to_split)
		{
			/* TODO: Make thread-safe */
			struct island *new_island = is_db_init_island_from_body(pipeline, body);

			/* Body-Contact breadth-first search */
			while (1)
			{
				b = PoolAddress(&pipeline->body_pool, body);
				u32 ci = b->first_contact_index;
				ds_Assert(ci == NLL_NULL ||
					((body == CONTACT_KEY_TO_BODY_0(((struct contact *) nll_Address(&pipeline->c_db.contact_net, ci))->key)) 
					 && ((struct contact *) nll_Address(&pipeline->c_db.contact_net, ci))->nll_prev[0] == NLL_NULL) ||
					((body == CONTACT_KEY_TO_BODY_1(((struct contact *) nll_Address(&pipeline->c_db.contact_net, ci))->key)) 
					 && ((struct contact *) nll_Address(&pipeline->c_db.contact_net, ci))->nll_prev[1] == NLL_NULL));
				while (ci != NLL_NULL)
				{
					const struct contact *c = nll_Address(&pipeline->c_db.contact_net, ci);
					ds_Assert(ci >= pipeline->c_db.contacts_frame_usage.bit_count ||  BitVecGetBit(&pipeline->c_db.contacts_frame_usage, ci) == 1)
					
					const u32 neighbour_index = (body == c->cm.i1) ? c->cm.i2 : c->cm.i1;
					b = PoolAddress(&pipeline->body_pool, neighbour_index);
					const u32 neighbour_island = b->island_index;

					/* TODO: Make Thread-Safe */
					if (neighbour_island == island_to_split)
					{
						/* TODO: Make Thread-Safe */
						is_db_internal_add_body_to_island(pipeline, new_island, neighbour_index);
						body_stack[sc++] = neighbour_index;
						ds_Assert(sc < split->body_count);
					}
					
					ci = (body == CONTACT_KEY_TO_BODY_0(c->key))
						? c->nll_next[0] 
						: c->nll_next[1];
				}

				if (sc)
				{
					body = body_stack[--sc];
					continue;
				}
	
				break;
			}
		}
	}

	/* create contact lists of new islands */
	for (u32 i = split->contact_first; i != ISLAND_NULL;)
	{
		entry = array_list_address(pipeline->is_db.island_contact_lists, i);
		i = entry->next;
		const u32 index = entry->index;

		if (index >= pipeline->c_db.contacts_frame_usage.bit_count || BitVecGetBit(&pipeline->c_db.contacts_frame_usage, index) == 1)
		{
			const struct contact *c = nll_Address(&pipeline->c_db.contact_net, index);
			ds_Assert(PoolSlotAllocated(c));
			const struct rigid_body *b1 = PoolAddress(&pipeline->body_pool, c->cm.i1);
			const struct rigid_body *b2 = PoolAddress(&pipeline->body_pool, c->cm.i2);
			const u32 island1 = b1->island_index;
			const u32 island2 = b2->island_index;
			struct island *is = (island1 != ISLAND_STATIC)
				? array_list_address(pipeline->is_db.islands, island1)
				: array_list_address(pipeline->is_db.islands, island2);
			is_db_internal_add_contact_to_island(&pipeline->is_db, is, index);
		}
	}

	/* TODO: Make thread safe */
	/* Remove split island */
	is_db_island_remove(pipeline, split);
	ArenaPopRecord(mem_tmp);
}

static u32 *island_solve(struct arena *mem_frame, struct physics_pipeline *pipeline, struct island *is, const f32 timestep)
{
	u32 *bodies_simulated = ArenaPush(mem_frame, is->body_count*sizeof(u32));
	ArenaPushRecord(mem_frame);

	/* Important: Reserve extra space for static body defaults used in contact solver */
	is->bodies = ArenaPush(mem_frame, (is->body_count + 1) * sizeof(struct rigid_body *));
	is->contacts = ArenaPush(mem_frame, is->contact_count * sizeof(struct contact *));
	is->body_index_map = ArenaPush(mem_frame, pipeline->body_pool.count_max * sizeof(u32));

	/* init body and contact arrays */
	struct is_index_entry *entry = NULL;
	u32 k = is->body_first;
	for (u32 i = 0; i < is->body_count; ++i)
	{
		entry = array_list_address(pipeline->is_db.island_body_lists, k);
		struct rigid_body *b = PoolAddress(&pipeline->body_pool, entry->index);
		bodies_simulated[i] = entry->index;
		is->body_index_map[entry->index] = i;
		is->bodies[i] = b;
		k = entry->next;
	}

	if (g_solver_config->sleep_enabled && ISLAND_TRY_SLEEP_BIT(is))
	{
		is->flags = 0;
		for (u32 i = 0; i < is->body_count; ++i)
		{
			struct rigid_body *b = is->bodies[i];
			b->flags ^= RB_AWAKE;
		}
		PHYSICS_EVENT_ISLAND_ASLEEP(pipeline, array_list_index(pipeline->is_db.islands, is));	
	}
	/* Island low energy state was interrupted, or island is simply awake */
	else
	{
		k = is->contact_first;
		for (u32 i = 0; i < is->contact_count; ++i)
		{
			entry = array_list_address(pipeline->is_db.island_contact_lists, k);
			is->contacts[i] = nll_Address(&pipeline->c_db.contact_net, entry->index);
 			k = entry->next;
		}

		/* init solver and velocity constraints */
		struct contact_solver *solver = contact_solver_init_body_data(mem_frame, is, timestep);
		contact_solver_init_velocity_constraints(mem_frame, solver, pipeline, is);
		
		if (g_solver_config->warmup_solver)
		{
			contact_solver_warmup(solver, is);
		}

		for (u32 i = 0; i < g_solver_config->iteration_count; ++i)
		{
			contact_solver_iterate_velocity_constraints(solver);
		}

		contact_solver_cache_impulse_data(solver, is);

		/* integrate final solver velocities and update bodies and find lowest low_velocity time */
		if (g_solver_config->sleep_enabled)
		{
			f32 min_low_velocity_time = F32_MAX_POSITIVE_NORMAL;
			for (u32 i = 0; i < is->body_count; ++i)
			{
				struct rigid_body *b = is->bodies[i];
				vec3_translate_scaled(b->position, solver->linear_velocity[i], timestep);	
				vec3_copy(b->velocity, solver->linear_velocity[i]);	

				quat a_vel_quat, rot_delta;
				vec3_copy(b->angular_velocity, solver->angular_velocity[i]);	
				quat_set(a_vel_quat, 
						solver->angular_velocity[i][0], 
						solver->angular_velocity[i][1], 
						solver->angular_velocity[i][2],
					      	0.0f);
				quat_mult(rot_delta, a_vel_quat, b->rotation);
				quat_scale(rot_delta, timestep / 2.0f);
				quat_translate(b->rotation, rot_delta);
				quat_normalize(b->rotation);

				/* Always set RB_AWAKE, if island should sleep, we set it later,
				 * but the bodies may come in sleeping if island just woke up 
				 */
				b->flags |= RB_AWAKE;
				b->low_velocity_time = (1-ISLAND_SLEEP_RESET_BIT(is)) * b->low_velocity_time;
				const f32 lv_sq = vec3_dot(b->velocity, b->velocity);
				const f32 av_sq = vec3_dot(b->angular_velocity, b->angular_velocity);
				if (lv_sq <= g_solver_config->sleep_linear_velocity_sq_limit && av_sq <= g_solver_config->sleep_angular_velocity_sq_limit)
				{
					b->low_velocity_time += timestep;
				}
				min_low_velocity_time = f32_min(min_low_velocity_time, b->low_velocity_time);
			}

			is->flags &= ~ISLAND_SLEEP_RESET;
			if (g_solver_config->sleep_time_threshold <= min_low_velocity_time)
			{
				is->flags |= ISLAND_TRY_SLEEP;
			}
		}
		/* only integrate final solver velocities and update bodies  */
		else 
		{
			for (u32 i = 0; i < is->body_count; ++i)
			{
				struct rigid_body *b = is->bodies[i];
				vec3_translate_scaled(b->position, solver->linear_velocity[i], timestep);	
				vec3_copy(b->velocity, solver->linear_velocity[i]);	

				quat a_vel_quat, rot_delta;
				vec3_copy(b->angular_velocity, solver->angular_velocity[i]);	
				quat_set(a_vel_quat, 
						solver->angular_velocity[i][0], 
						solver->angular_velocity[i][1], 
						solver->angular_velocity[i][2],
					      	0.0f);
				quat_mult(rot_delta, a_vel_quat, b->rotation);
				quat_scale(rot_delta, timestep / 2.0f);
				quat_translate(b->rotation, rot_delta);
				quat_normalize(b->rotation);
			}
		}
	}

	ArenaPopRecord(mem_frame);
	return bodies_simulated;
}

void thread_island_solve(void *task_input)
{
	ProfZone;

	struct task *t_ctx = task_input;
	struct island_solve_input *args = t_ctx->input;
	args->out->body_count = args->is->body_count;
	args->out->bodies = island_solve(&t_ctx->executor->mem_frame, args->pipeline, args->is, args->timestep);

	ProfZoneEnd;
}
