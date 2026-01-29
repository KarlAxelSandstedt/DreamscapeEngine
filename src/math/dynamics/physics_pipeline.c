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

#include <stdlib.h>
#include <string.h>
#include <float.h>

#include "dynamics.h"
#include "float32.h"

const char *body_color_mode_str_buf[RB_COLOR_MODE_COUNT] = 
{
	"RB_COLOR_MODE_BODY",
	"RB_COLOR_MODE_COLLISION",
	"RB_COLOR_MODE_ISLAND",
	"RB_COLOR_MODE_SLEEP",
};

const char **body_color_mode_str = body_color_mode_str_buf;

dsThreadLocal struct collision_debug *tl_debug;

u32 g_a_thread_counter = 0;

static void thread_set_collision_debug(void *task_addr)
{
	struct task *task = task_addr;
	struct worker *worker = task->executor;
	const struct physics_pipeline *pipeline = task->input;
	tl_debug = pipeline->debug + ds_ThreadSelfIndex();

	AtomicFetchAddRel32(&g_a_thread_counter, 1);
	while (AtomicLoadAcq32(&g_a_thread_counter) != pipeline->debug_count);
}

struct physics_pipeline	physics_pipeline_alloc(struct arena *mem, const u32 initial_size, const u64 ns_tick, const u64 frame_memory, struct strdb *shape_db, struct strdb *prefab_db)
{
	struct physics_pipeline pipeline =
	{
		.gravity = { 0.0f, -GRAVITY_CONSTANT_DEFAULT, 0.0f },
		.ns_tick = ns_tick,
		.ns_elapsed = 0,
		.ns_start = 0,
		.frame = ArenaAlloc(frame_memory),
		.frames_completed = 0,
	};

	static u32 init_solver_once = 0;
	if (!init_solver_once)
	{
		init_solver_once = 1;
		const u32 iteration_count = 10;
		const u32 block_solver = 0; 
		const u32 warmup_solver = 1;
		const vec3 gravity = { 0.0f, -GRAVITY_CONSTANT_DEFAULT, 0.0f };
       		const f32 baumgarte_constant = 0.1f;
		const f32 max_condition = 1000.0f;
		const f32 linear_dampening = 0.1f;
		const f32 angular_dampening = 0.1f;
		const f32 linear_slop = 0.001f;
		const f32 restitution_threshold = 0.001f;
		const u32 sleep_enabled = 1;
		const f32 sleep_time_threshold = 0.5f;
		f32 sleep_linear_velocity_sq_limit = 0.001f*0.001f; 
		f32 sleep_angular_velocity_sq_limit = 0.01f*0.01f*2.0f*F32_PI;
		contact_solver_config_init(iteration_count, block_solver, warmup_solver, gravity, baumgarte_constant, max_condition, linear_dampening, angular_dampening, linear_slop, restitution_threshold, sleep_enabled, sleep_time_threshold, sleep_linear_velocity_sq_limit, sleep_angular_velocity_sq_limit);

	}

	ds_AssertString(PowerOfTwoCheck(initial_size), "For simplicity of future data structures, expect pipeline sizes to be powers of two");

	pipeline.body_pool = PoolAlloc(NULL, initial_size, struct rigid_body, GROWABLE);
	pipeline.body_marked_list = dll_Init(struct rigid_body);
	pipeline.body_non_marked_list = dll_Init(struct rigid_body);

	pipeline.event_pool = PoolAlloc(NULL, 256, struct physics_event, GROWABLE);
	pipeline.event_list = dll_Init(struct physics_event);

	pipeline.margin_on = 1;
	pipeline.margin = COLLISION_MARGIN_DEFAULT;

	pipeline.dynamic_tree = dbvh_alloc(NULL, 2*initial_size, 1);

	pipeline.c_db = c_db_alloc(mem, initial_size);
	pipeline.is_db = is_db_alloc(mem, initial_size);
	pipeline.shape_db = shape_db;

	pipeline.body_color_mode = RB_COLOR_MODE_BODY;
	pipeline.pending_body_color_mode = RB_COLOR_MODE_COLLISION;
	Vec4Set(pipeline.collision_color, 1.0f, 0.1f, 0.1f, 0.5f);
	Vec4Set(pipeline.static_color, 0.6f, 0.6f, 0.6f, 0.5f);
	Vec4Set(pipeline.sleep_color, 113.0f/256.0f, 241.0f/256.0f, 157.0f/256.0f, 0.7f);
	Vec4Set(pipeline.awake_color, 255.0f/256.0f, 36.0f/256.0f, 48.0f/256.0f, 0.7f);
	Vec4Set(pipeline.manifold_color, 0.6f, 0.6f, 0.9f, 1.0f);
	Vec4Set(pipeline.dbvh_color, 0.8f, 0.1f, 0.0f, 0.6f);
	Vec4Set(pipeline.sbvh_color, 0.0f, 0.8f, 0.1f, 0.6f);
	Vec4Set(pipeline.bounding_box_color, 0.8f, 0.1f, 0.6f, 1.0f);

	pipeline.draw_bounding_box = 0;
	pipeline.draw_dbvh = 0;
	pipeline.draw_sbvh = 1;
	pipeline.draw_manifold = 0;
	pipeline.draw_lines = 0;

	pipeline.debug_count = 0;
	pipeline.debug = NULL;
#ifdef DS_PHYSICS_DEBUG
	struct task_stream *stream = task_stream_init(&pipeline.frame);

	pipeline.debug_count = g_arch_config->Logical_core_count;
	pipeline.debug = malloc(g_arch_config->Logical_core_count * sizeof(struct collision_debug));
	for (u32 i = 0; i < pipeline.debug_count; ++i)
	{
		pipeline.debug[i].stack_segment = stack_visual_segment_alloc(NULL, 1024, GROWABLE);
		task_stream_dispatch(&pipeline.frame, stream, thread_set_collision_debug, &pipeline);
	}

	task_main_master_run_available_jobs();

	/* spin wait until last job completes */
	task_stream_spin_wait(stream);
	/* release any task resources */
	task_stream_cleanup(stream);		
#endif

	return pipeline;
}

void physics_pipeline_free(struct physics_pipeline *pipeline)
{
#ifdef DS_PHYSICS_DEBUG
	for (u32 i = 0; i < pipeline->debug_count; ++i)
	{
		stack_visual_segment_free(&pipeline->debug[i].stack_segment);
	}
#endif
	bvh_free(&pipeline->dynamic_tree);
	c_db_free(&pipeline->c_db);
	is_db_free(&pipeline->is_db);
	PoolDealloc(&pipeline->body_pool);
	PoolDealloc(&pipeline->event_pool);
}

static void internal_physics_pipeline_clear_frame(struct physics_pipeline *pipeline)
{
#ifdef DS_PHYSICS_DEBUG
	for (u32 i = 0; i < pipeline->debug_count; ++i)
	{
		stack_visual_segment_flush(&pipeline->debug[i].stack_segment);
	}
#endif
	pipeline->proxy_overlap_count = 0;
	pipeline->proxy_overlap = NULL;
	pipeline->cm_count = 0;
	pipeline->cm = NULL;

	is_db_clear_frame(&pipeline->is_db);
	c_db_clear_frame(&pipeline->c_db);
	ArenaFlush(&pipeline->frame);
}


void physics_pipeline_flush(struct physics_pipeline *pipeline)
{
#ifdef DS_PHYSICS_DEBUG
	for (u32 i = 0; i < pipeline->debug_count; ++i)
	{
		stack_visual_segment_flush(&pipeline->debug[i].stack_segment);
	}
#endif
	dbvh_flush(&pipeline->dynamic_tree);
	c_db_flush(&pipeline->c_db);
	is_db_flush(&pipeline->is_db);
	
	PoolFlush(&pipeline->body_pool);
	dll_Flush(&pipeline->body_marked_list);
	dll_Flush(&pipeline->body_non_marked_list);

	PoolFlush(&pipeline->event_pool);
	dll_Flush(&pipeline->event_list);

	ArenaFlush(&pipeline->frame);
	pipeline->frames_completed = 0;
	pipeline->ns_elapsed = 0;
}

void physics_pipeline_validate(const struct physics_pipeline *pipeline)
{
	ProfZone;

	c_db_validate(pipeline);
	is_db_validate(pipeline);

	ProfZoneEnd;
}

static void rigid_body_update_local_box(struct rigid_body *body, const struct collision_shape *shape)
{
	vec3 min = { FLT_MAX, FLT_MAX, FLT_MAX };
	vec3 max = { -FLT_MAX, -FLT_MAX, -FLT_MAX };

	vec3 v;
	mat3 rot;
	Mat3Quat(rot, body->rotation);

	if (body->shape_type == COLLISION_SHAPE_CONVEX_HULL)
	{
		for (u32 i = 0; i < shape->hull.v_count; ++i)
		{
			Mat3VecMul(v, rot, shape->hull.v[i]);
			min[0] = f32_min(min[0], v[0]); 
			min[1] = f32_min(min[1], v[1]);			
			min[2] = f32_min(min[2], v[2]);			
                                                   
			max[0] = f32_max(max[0], v[0]);			
			max[1] = f32_max(max[1], v[1]);			
			max[2] = f32_max(max[2], v[2]);			
		}
	}
	else if (body->shape_type == COLLISION_SHAPE_SPHERE)
	{
		const f32 r = shape->sphere.radius;
		Vec3Set(min, -r, -r, -r);
		Vec3Set(max, r, r, r);
	}
	else if (body->shape_type == COLLISION_SHAPE_CAPSULE)
	{
		v[0] = rot[1][0] * shape->capsule.half_height;	
		v[1] = rot[1][1] * shape->capsule.half_height;	
		v[2] = rot[1][2] * shape->capsule.half_height;	
		Vec3Set(max, 
			f32_max(-v[0], v[0]),
			f32_max(-v[1], v[1]),
			f32_max(-v[2], v[2]));
		Vec3AddConstant(max, shape->capsule.radius);
		Vec3Negate(min, max);
	}
	else if (body->shape_type == COLLISION_SHAPE_TRI_MESH)
	{
		const struct bvh_node *node = (struct bvh_node *) shape->mesh_bvh.bvh.tree.pool.buf;
		struct aabb bbox; 
		AabbRotate(&bbox, &node[shape->mesh_bvh.bvh.tree.root].bbox, rot);
		//Vec3Sub(min, bbox.center, bbox.hw);
		//Vec3Add(max, bbox.center, bbox.hw);
		Vec3Scale(min, bbox.hw, -1.0f);
		Vec3Scale(max, bbox.hw, 1.0f);
	}

	Vec3Sub(body->local_box.hw, max, min);
	Vec3ScaleSelf(body->local_box.hw, 0.5f);
	Vec3Add(body->local_box.center, min, body->local_box.hw);
}

struct slot physics_pipeline_rigid_body_alloc(struct physics_pipeline *pipeline, struct rigid_body_prefab *prefab, const vec3 position, const quat rotation, const u32 entity)
{
	struct slot slot = PoolAdd(&pipeline->body_pool);
	PHYSICS_EVENT_BODY_NEW(pipeline, slot.index);
	struct rigid_body *body = slot.address;
	dll_Append(&pipeline->body_non_marked_list, pipeline->body_pool.buf, slot.index);

	body->entity = entity;
	Vec3Copy(body->position, position);
	QuatCopy(body->rotation, rotation);
	Vec3Set(body->velocity, 0.0f, 0.0f, 0.0f);
	Vec3Set(body->angular_velocity, 0.0f, 0.0f, 0.0f);
	Vec3Set(body->linear_momentum, 0.0f, 0.0f, 0.0f);

	const u32 dynamic_flag = (prefab->dynamic) ? RB_DYNAMIC : 0;
	body->flags = RB_ACTIVE | (g_solver_config->sleep_enabled * RB_AWAKE) | dynamic_flag;
	body->margin = 0.25f;

	const struct collision_shape *shape = strdb_Address(pipeline->shape_db, prefab->shape);
	const struct slot shape_slot = strdb_Reference(pipeline->shape_db, shape->id);
	body->shape_handle = shape_slot.index;
	body->shape_type = shape->type;

	Mat3Copy(body->inertia_tensor, prefab->inertia_tensor);
	Mat3Copy(body->inv_inertia_tensor, prefab->inv_inertia_tensor);
	body->mass = prefab->mass;
	body->restitution = prefab->restitution;
	body->friction = prefab->friction;
	body->low_velocity_time = 0.0f;

	rigid_body_update_local_box(body, shape);
	struct aabb proxy;
	Vec3Add(proxy.center, body->local_box.center, body->position);
	if (body->shape_type == COLLISION_SHAPE_TRI_MESH)
	{
		Vec3Print("center", body->local_box.center);
		Vec3Print("center", body->position);
		Vec3Set(proxy.hw, 
			body->local_box.hw[0],
			body->local_box.hw[1],
			body->local_box.hw[2]);
	}
	else
	{
		Vec3Set(proxy.hw, 
			body->local_box.hw[0] + body->margin,
			body->local_box.hw[1] + body->margin,
			body->local_box.hw[2] + body->margin);
	}
	body->proxy = dbvh_insert(&pipeline->dynamic_tree, slot.index, &proxy);

	body->first_contact_index = NLL_NULL;
	if (body->flags & RB_DYNAMIC)
	{
		is_db_init_island_from_body(pipeline, slot.index);
	}
	else
	{
		body->island_index = ISLAND_STATIC;
	}
	
	return slot;
}

static void internal_update_dynamic_tree(struct physics_pipeline *pipeline)
{
	ProfZone;
	struct aabb world_AABB;

	const u32 flags = RB_ACTIVE | RB_DYNAMIC | (g_solver_config->sleep_enabled * RB_AWAKE);
	struct rigid_body *b = NULL;
	for (u32 i = pipeline->body_non_marked_list.first; i != DLL_NULL; i = dll_Next(b))
	{
		b = PoolAddress(&pipeline->body_pool, i);
		if ((b->flags & flags) == flags)
		{
			const struct collision_shape *shape = strdb_Address(pipeline->shape_db, b->shape_handle);
			rigid_body_update_local_box(b, shape);
			Vec3Add(world_AABB.center, b->local_box.center, b->position);
			Vec3Copy(world_AABB.hw, b->local_box.hw);
			const struct bvh_node *node = PoolAddress(&pipeline->dynamic_tree.tree.pool, b->proxy);
			const struct aabb *proxy = &node->bbox;
			if (!AabbContains(proxy, &world_AABB))
			{
				world_AABB.hw[0] += b->margin;
				world_AABB.hw[1] += b->margin;
				world_AABB.hw[2] += b->margin;
				dbvh_remove(&pipeline->dynamic_tree, b->proxy);
				b->proxy = dbvh_insert(&pipeline->dynamic_tree, i, &world_AABB);
			}
		}
	}
	ProfZoneEnd;
}

static void internal_push_proxy_overlaps(struct arena *mem_frame, struct physics_pipeline *pipeline)
{
	ProfZone;
	pipeline->proxy_overlap = dbvh_push_overlap_pairs(mem_frame, &pipeline->proxy_overlap_count, &pipeline->dynamic_tree);
	ProfZoneEnd;
}

struct tpc_output
{
	struct collision_result *result;
	u32 result_count;
};

static void thread_push_contacts(void *task_addr)
{
	ProfZone;

	struct task *task = task_addr;
	struct worker *worker = task->executor;
	const struct task_range *range = task->range;
	const struct physics_pipeline *pipeline = task->input;
	const struct dbvh_overlap *proxy_overlap = range->base;

	struct tpc_output *out = ArenaPush(&worker->mem_frame, sizeof(struct tpc_output));
	out->result_count = 0;
	out->result = ArenaPush(&worker->mem_frame, range->count * sizeof(struct collision_result));

	const f32 margin = (pipeline->margin_on) ? pipeline->margin : 0.0f;
	const struct rigid_body *b1, *b2;

	for (u64 i = 0; i < range->count; ++i)
	{
		b1 = PoolAddress(&pipeline->body_pool, proxy_overlap[i].id1);
		b2 = PoolAddress(&pipeline->body_pool, proxy_overlap[i].id2);

		if (body_body_contact_manifold(&worker->mem_frame, out->result + out->result_count, pipeline, b1, b2, margin))
		{
			out->result[out->result_count].manifold.i1 = proxy_overlap[i].id1;
			out->result[out->result_count].manifold.i2 = proxy_overlap[i].id2;

			//vec3 tmp;
			//Vec3Sub(tmp, b2->position, b1->position);

			//if (Vec3Dot(tmp, out->result[out->result_count].manifold.n) < 0)
			//{
			//	Vec3ScaleSelf(out->result[out->result_count].manifold.n, -1.0f);
			//}

			out->result_count += 1;
		}	
		else if (out->result[out->result_count].type == COLLISION_SAT_CACHE)
		{
			out->result_count += 1;
		}
	}

	ArenaPopPacked(&worker->mem_frame, (range->count - out->result_count) * sizeof(struct collision_result));

	task->output = out;
	ProfZoneEnd;
}

static void internal_parallel_push_contacts(struct arena *mem_frame, struct physics_pipeline *pipeline)
{
	struct task_bundle *bundle = task_bundle_split_range(
			mem_frame, 
			&thread_push_contacts, 
			g_task_ctx->worker_count, 
			pipeline->proxy_overlap, 
			pipeline->proxy_overlap_count, 
			sizeof(struct dbvh_overlap), 
			pipeline);

	ProfZone;
	pipeline->cm = (struct contact_manifold *) ArenaPush(mem_frame, pipeline->proxy_overlap_count * sizeof(struct contact_manifold));
	ArenaPushRecord(mem_frame);

	pipeline->cm_count = 0;
	if (bundle)
	{	
		task_main_master_run_available_jobs();
		task_bundle_wait(bundle);

		for (u32 i = 0; i < bundle->task_count; ++i)
		{
			struct tpc_output *out = (struct tpc_output *) AtomicLoadAcq64(&bundle->tasks[i].output);
			for (u32 j = 0; j < out->result_count; ++j)
			{
				if (out->result[j].type == COLLISION_SAT_CACHE)
				{
					sat_cache_add(&pipeline->c_db, &out->result[j].sat_cache);
					if (out->result[j].sat_cache.type != SAT_CACHE_SEPARATION)
					{
						pipeline->cm[pipeline->cm_count++] = out->result[j].manifold;
					}
				}
				else
				{
					pipeline->cm[pipeline->cm_count++] = out->result[j].manifold;
				}
			}
		}
	
		task_bundle_release(bundle);
	}
	
	ArenaPopRecord(mem_frame);
	ArenaPopPacked(mem_frame, (pipeline->proxy_overlap_count - pipeline->cm_count) * sizeof(struct contact_manifold));

	pipeline->c_db.contacts_frame_usage = BitVecAlloc(mem_frame, pipeline->c_db.contacts_persistent_usage.bit_count, 0, 0);
	ds_Assert(pipeline->c_db.contacts_frame_usage.block_count == pipeline->c_db.contacts_persistent_usage.block_count);
	ds_Assert(pipeline->c_db.contacts_frame_usage.bit_count == pipeline->c_db.contacts_persistent_usage.bit_count);

	pipeline->contact_new_count = 0;
	pipeline->contact_new = (u32 *) mem_frame->stack_ptr;
	//fprintf(stderr, "A: {");
	{
		ProfZoneNamed("internal_gather_real_contacts");
		for (u32 i = 0; i < pipeline->cm_count; ++i)
		{
			const struct contact *c = c_db_add_contact(pipeline, pipeline->cm + i, pipeline->cm[i].i1, pipeline->cm[i].i2);
			/* add to new links if needed */
			const u32 index = (u32) nll_Index(&pipeline->c_db.contact_net, c);
			if (index >= pipeline->c_db.contacts_persistent_usage.bit_count
				 || BitVecGetBit(&pipeline->c_db.contacts_persistent_usage, index) == 0)
			{
					pipeline->contact_new_count += 1;
					ArenaPushPackedMemcpy(mem_frame, &index, sizeof(index));
			}
			//fprintf(stderr, " %u", index);
		}
		ProfZoneEnd;
	}
	//fprintf(stderr, " } ");

	//if (bundle)
	//{
	//	//fprintf(stderr, "A: {");
	//	for (u32 i = 0; i < bundle->task_count; ++i)
	//	{
	//		struct tpc_output *out = (struct tpc_output *) AtomicLoadAcq64(&bundle->tasks[i].output);
	//		for (u32 j = 0; j < out->cm_count; ++j)
	//		{
	//			const struct contact *c = c_db_add_contact(pipeline, out->cm + j, out->cm[j].i1, out->cm[j].i2);
	//			/* add to new links if needed */
	//			const u32 index = (u32) nll_Index(&pipeline->c_db.contact_net, c);
	//			if (index >= pipeline->c_db.contacts_persistent_usage.bit_count
	//				 || BitVecGetBit(&pipeline->c_db.contacts_persistent_usage, index) == 0)
	//			{
	//					pipeline->contact_new_count += 1;
	//					ArenaPushPackedMemcpy(mem_frame, &index, sizeof(index));
	//			}
	//			//fprintf(stderr, " %u", index);
	//		}	
	//	}
	//	//fprintf(stderr, " } ");
	//}
	ProfZoneEnd;
}

static void internal_merge_islands(struct arena *mem_frame, struct physics_pipeline *pipeline)
{
	ProfZone;
	for (u32 i = 0; i < pipeline->contact_new_count; ++i)
	{
		struct contact *c = nll_Address(&pipeline->c_db.contact_net, pipeline->contact_new[i]);
		const struct rigid_body *body1 = PoolAddress(&pipeline->body_pool, c->cm.i1);
		const struct rigid_body *body2 = PoolAddress(&pipeline->body_pool, c->cm.i2);
		const u32 is1 = body1->island_index;
		const u32 is2 = body2->island_index;
		const u32 d1 = (is1 != ISLAND_STATIC) ? 0x2 : 0x0;
		const u32 d2 = (is2 != ISLAND_STATIC) ? 0x1 : 0x0;
		switch (d1 | d2)
		{
			/* dynamic-dynamic */
			case 0x3: 
			{
				is_db_merge_islands(pipeline, pipeline->contact_new[i], c->cm.i1, c->cm.i2);
			} break;

			/* dynamic-static */
			case 0x2:
			{
				is_db_add_contact_to_island(&pipeline->is_db, is1, pipeline->contact_new[i]);
			} break;

			/* static-dynamic */
			case 0x1:
			{
				is_db_add_contact_to_island(&pipeline->is_db, is2, pipeline->contact_new[i]);
			} break;
		}
	}
	ProfZoneEnd;
}

static void internal_remove_contacts_and_tag_split_islands(struct arena *mem_frame, struct physics_pipeline *pipeline)
{
	ProfZone;
	if (pipeline->c_db.contact_net.pool.count == 0) 
	{ 
	ProfZoneEnd;
		return; 
	}

	/* 
	 * For every removed contact
	 * (1)	if island is not tagged, tag island and push. 
	 * (2) 	remove contact
	 */

	/* Remove any contacts that were not persistent */
	//fprintf(stderr, " R: {");
	u32 bit = 0;
	is_db_reserve_splits_memory(mem_frame, &pipeline->is_db);
	for (u64 block = 0; block < pipeline->c_db.contacts_frame_usage.block_count; ++block)
	{
		u64 broken_link_block = 
				    pipeline->c_db.contacts_persistent_usage.bits[block]
				& (~pipeline->c_db.contacts_frame_usage.bits[block]);
		u32 b = 0;
		while (broken_link_block)
		{
			const u32 tzc = Ctz64(broken_link_block);
			b += tzc;
			const u32 ci = bit + b;
			b += 1;

			broken_link_block = (tzc < 63) 
				? broken_link_block >> (tzc + 1)
				: 0;
		
			//fprintf(stderr, " %lu", ci);
			struct contact *c = nll_Address(&pipeline->c_db.contact_net, ci);

			/* tag island, if any exist, to split */
			const u32 b1 = CONTACT_KEY_TO_BODY_0(c->key);
			const u32 b2 = CONTACT_KEY_TO_BODY_1(c->key);
			const struct rigid_body *body1 = PoolAddress(&pipeline->body_pool, b1);
			const struct rigid_body *body2 = PoolAddress(&pipeline->body_pool, b2);
			if (body1->island_index != ISLAND_STATIC)
			{
				struct island *is = is_db_body_to_island(pipeline, b1);
				ds_Assert(is->contact_count > 0);
				is_db_tag_for_splitting(pipeline, b1);
			}
			else if (body2->island_index != ISLAND_STATIC)
			{
				struct island *is = is_db_body_to_island(pipeline, b2);
				ds_Assert(is->contact_count > 0);
				is_db_tag_for_splitting(pipeline, b2);
			}

			c_db_remove_contact(pipeline, c->key, (u32) ci);
		}
		bit += 64;
	}	
	is_db_release_unused_splits_memory(mem_frame, &pipeline->is_db);
	//fprintf(stderr, " }\tcontacts: %u\n", pipeline->c_db.contacts->count-2);
	ProfZoneEnd;
}

static void internal_split_islands(struct arena *mem_frame, struct physics_pipeline *pipeline)
{
	ProfZone;
	/* TODO: Parallelize island splitting */

	for (u32 i = 0; i < pipeline->is_db.possible_splits_count; ++i)
	{
		is_db_split_island(mem_frame, pipeline, pipeline->is_db.possible_splits[i]);
	}

	c_db_update_persistent_contacts_usage(&pipeline->c_db);

	ProfZoneEnd;
}

static void internal_parallel_solve_islands(struct arena *mem_frame, struct physics_pipeline *pipeline, const f32 delta)
{
	ProfZone;

	/* acquire any task resources */
	struct task_stream *stream = task_stream_init(mem_frame);
	struct island_solve_output *output = NULL;
	struct island_solve_output **next = &output;
	u32 base = 0;
	u32 bodies = 0;

	for (u64 block = 0; block < pipeline->is_db.island_usage.block_count; ++block)
	{
		u64 island_block = pipeline->is_db.island_usage.bits[block];
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

			struct island *is = array_list_address(pipeline->is_db.islands, is_index);
			
			if (!g_solver_config->sleep_enabled || ISLAND_AWAKE_BIT(is))
			{
				bodies += is->body_count;
				struct island_solve_input *args = ArenaPush(mem_frame, sizeof(struct island_solve_input));
				*next = ArenaPush(mem_frame, sizeof(struct island_solve_output));
				(*next)->island = is_index;
				(*next)->island_asleep = 0;
				(*next)->next = NULL;
				args->out = *next;
				args->is = is;
				args->pipeline = pipeline;
				args->timestep = delta;
				task_stream_dispatch(mem_frame, stream, thread_island_solve, args);

				next = &(*next)->next;
			}
		}
		base += 64;
	}

	task_main_master_run_available_jobs();

	/* spin wait until last job completes */
	task_stream_spin_wait(stream);
	/* release any task resources */
	task_stream_cleanup(stream);		


	/*
	 * TODO:
	 * 	(1) pipeline->event_list sequential list of physics events
	 * 	(2) implement array_list_flush to clear whole list 
	 */
	for (; output; output = output->next)
	{
		if (output->island_asleep)
		{
			PHYSICS_EVENT_ISLAND_ASLEEP(pipeline, output->island);
		}

		for (u32 i = 0; i < output->body_count; ++i)
		{
			struct physics_event *event = physics_pipeline_event_push(pipeline);
			event->type = PHYSICS_EVENT_BODY_ORIENTATION;
			event->body = output->bodies[i];
		}
	}

	ProfZoneEnd;
}

void physics_pipeline_enable_sleeping(struct physics_pipeline *pipeline)
{
	ds_Assert(g_solver_config->sleep_enabled == 0);
	if (!g_solver_config->sleep_enabled)
	{
		g_solver_config->sleep_enabled = 1;
		const u32 body_flags = RB_ACTIVE | RB_DYNAMIC;
		struct rigid_body *body = NULL;
		for (u32 i = pipeline->body_non_marked_list.first; i != DLL_NULL; i = dll_Next(body))
		{
			body = PoolAddress(&pipeline->body_pool, i);
			if (body->flags & body_flags)
			{
				body->flags |= RB_AWAKE;
			}
		}

		for (u32 i = 0; i < pipeline->is_db.islands->length; ++i)
		{
			struct island *is = array_list_address(pipeline->is_db.islands, i);
			is->flags |= ISLAND_AWAKE | ISLAND_SLEEP_RESET;
			is->flags &= ~ISLAND_TRY_SLEEP;
		}
	}
}

void physics_pipeline_disable_sleeping(struct physics_pipeline *pipeline)
{
	ds_Assert(g_solver_config->sleep_enabled == 1);
	if (g_solver_config->sleep_enabled)
	{
		g_solver_config->sleep_enabled = 0;
		const u32 body_flags = RB_ACTIVE | RB_DYNAMIC;
		struct rigid_body *body = NULL;
		for (u32 i = pipeline->body_non_marked_list.first; i != DLL_NULL; i = dll_Next(body))
		{
			body = PoolAddress(&pipeline->body_pool, i);
			if (body->flags & body_flags)
			{
				body->flags |= RB_AWAKE;
			}
		}

		for (u32 i = 0; i < pipeline->is_db.islands->length; ++i)
		{
			struct island *is = array_list_address(pipeline->is_db.islands, i);
			is->flags |= ISLAND_AWAKE;
			is->flags &= ~(ISLAND_SLEEP_RESET | ISLAND_TRY_SLEEP);
		}
	}
}

static void internal_update_contact_solver_config(struct physics_pipeline *pipeline)
{
	g_solver_config->warmup_solver = g_solver_config->pending_warmup_solver;
	g_solver_config->block_solver = g_solver_config->pending_block_solver;
	g_solver_config->iteration_count = g_solver_config->pending_iteration_count;
	g_solver_config->linear_slop = g_solver_config->pending_linear_slop;
	g_solver_config->baumgarte_constant = g_solver_config->pending_baumgarte_constant;
	g_solver_config->restitution_threshold = g_solver_config->pending_restitution_threshold;
	g_solver_config->linear_dampening = g_solver_config->pending_linear_dampening;
	g_solver_config->angular_dampening = g_solver_config->pending_angular_dampening;

	if (g_solver_config->pending_sleep_enabled != g_solver_config->sleep_enabled)
	{
		(g_solver_config->pending_sleep_enabled)
			? physics_pipeline_enable_sleeping(pipeline)
			: physics_pipeline_disable_sleeping(pipeline);

		g_solver_config->sleep_enabled = g_solver_config->pending_sleep_enabled;
	}

}

static void physics_pipeline_rigid_body_dealloc(struct physics_pipeline *pipeline, const u32 handle)
{
	struct rigid_body *body = PoolAddress(&pipeline->body_pool, handle);
	ds_Assert(PoolSlotAllocated(body));

	strdb_Dereference(pipeline->shape_db, body->shape_handle);
	dbvh_remove(&pipeline->dynamic_tree, body->proxy);
	if (body->island_index != ISLAND_STATIC)
	{
		is_db_island_remove_body_resources(pipeline, body->island_index, handle);
		c_db_remove_body_contacts(pipeline, handle);
		if (BitVecGetBit(&pipeline->is_db.island_usage, body->island_index) && ((struct island *) array_list_address(pipeline->is_db.islands, body->island_index))->contact_count > 0)
		{
			is_db_split_island(&pipeline->frame, pipeline, body->island_index);
		}
	}
	else
	{
		ArenaPushRecord(&pipeline->frame);
		u32 island_count;
		u32 *island = c_db_remove_static_contacts_and_store_affected_islands(&pipeline->frame, &island_count, pipeline, handle);
		for (u32 i = 0; i < island_count; ++i)
		{
			struct island *is = array_list_address(pipeline->is_db.islands, island[i]);
			struct is_index_entry *prev = NULL;
			struct is_index_entry *entry;
			u32 prev_index = ISLAND_NULL;
			u32 index = is->contact_first;
			do 
			{
				entry = array_list_address(pipeline->is_db.island_contact_lists, index);
				const u32 next = entry->next;
				if (BitVecGetBit(&pipeline->c_db.contacts_persistent_usage, entry->index) == 0)
				{
					if (prev)
					{
						prev->next = entry->next;
					}
					else
					{
						is->contact_first = entry->next;
					}
					entry->next = ISLAND_NULL;
					entry->index = ISLAND_NULL;
					array_list_remove_index(pipeline->is_db.island_contact_lists, index);
					is->contact_count -= 1;
				}
				else
				{
					prev_index = index;
					prev = entry;
				}
				index = next;
			} 
			while (index != ISLAND_NULL);

			is->contact_last = prev_index;
			if (is->contact_count > 0)
			{
				is_db_split_island(&pipeline->frame, pipeline, island[i]);
			}
			else
			{
				is->flags &= ~(ISLAND_SPLIT);
				if (!(is->flags & ISLAND_AWAKE))
				{
					PHYSICS_EVENT_ISLAND_AWAKE(pipeline, island[i]);	
				}
				is->flags |= ISLAND_SLEEP_RESET | ISLAND_AWAKE;
			}
		}
		ArenaPopRecord(&pipeline->frame);
	}
	PoolRemove(&pipeline->body_pool, handle);
	PHYSICS_EVENT_BODY_REMOVED(pipeline, handle);
}

void physics_pipeline_rigid_body_tag_for_removal(struct physics_pipeline *pipeline, const u32 handle)
{
	struct rigid_body *b = PoolAddress(&pipeline->body_pool, handle);
	if (!RB_IS_MARKED(b))
	{
		b->flags |= RB_MARKED_FOR_REMOVAL;
		dll_Remove(&pipeline->body_non_marked_list, pipeline->body_pool.buf, handle);
		dll_Append(&pipeline->body_marked_list, pipeline->body_pool.buf, handle);
	}
}

static void internal_remove_marked_bodies(struct physics_pipeline *pipeline)
{
	struct rigid_body *b = NULL;
	for (u32 i = pipeline->body_marked_list.first; i != DLL_NULL; i = dll_Next(b))
	{
		b = PoolAddress(&pipeline->body_pool, i);
		physics_pipeline_rigid_body_dealloc(pipeline, i);
	}

	dll_Flush(&pipeline->body_marked_list);
}

void internal_physics_pipeline_simulate_frame(struct physics_pipeline *pipeline, const f32 delta)
{
	internal_remove_marked_bodies(pipeline);

	/* update, if possible, any pending values in contact solver config */
	internal_update_contact_solver_config(pipeline);

	/* broadphase => narrowphase => solve => integrate */
	internal_update_dynamic_tree(pipeline);
	internal_push_proxy_overlaps(&pipeline->frame, pipeline);
	internal_parallel_push_contacts(&pipeline->frame, pipeline);

	internal_merge_islands(&pipeline->frame, pipeline);
	internal_remove_contacts_and_tag_split_islands(&pipeline->frame, pipeline);
	internal_split_islands(&pipeline->frame, pipeline);
	internal_parallel_solve_islands(&pipeline->frame, pipeline, delta);

	PHYSICS_PIPELINE_VALIDATE(pipeline);
}

void physics_pipeline_tick(struct physics_pipeline *pipeline)
{
	ProfZone;

	if (pipeline->frames_completed > 0)
	{
		internal_physics_pipeline_clear_frame(pipeline);
	}
	pipeline->frames_completed += 1;
	const f32 delta = (f32) pipeline->ns_tick / NSEC_PER_SEC;
	internal_physics_pipeline_simulate_frame(pipeline, delta);

	ProfZoneEnd;
}

u32f32 physics_pipeline_raycast_parameter(struct arena *mem_tmp, const struct physics_pipeline *pipeline, const struct ray *ray)
{
	ArenaPushRecord(mem_tmp);

	struct bvh_raycast_info info = bvh_raycast_init(mem_tmp, &pipeline->dynamic_tree, ray);
	while (info.hit_queue.count)
	{
		const u32f32 tuple = MinQueueFixedPop(&info.hit_queue);
		if (info.hit.f < tuple.f)
		{
			break;	
		}

		if (bt_LeafCheck(info.node + tuple.u))
		{
			const u32 bi = info.node[tuple.u].bt_left;
			const struct rigid_body *body = (struct rigid_body *) pipeline->body_pool.buf + bi;
			const f32 t = body_raycast_parameter(pipeline, body, ray);
			if (t < info.hit.f)
			{
				info.hit = u32f32_inline(bi, t);
			}
		}
		else
		{
			bvh_raycast_test_and_push_children(&info, tuple);
		}
	}

	ArenaPopRecord(mem_tmp);

	return info.hit;
}

struct physics_event *physics_pipeline_event_push(struct physics_pipeline *pipeline)
{
	struct slot slot = PoolAdd(&pipeline->event_pool);
	dll_Append(&pipeline->event_list, pipeline->event_pool.buf, slot.index);
	struct physics_event *event = slot.address;
	event->ns = pipeline->ns_start + pipeline->frames_completed * pipeline->ns_tick;
	return event;
}

#define VOL	0 
#define T_X 	1
#define T_Y 	2
#define T_Z 	3
#define T_XX	4
#define T_YY	5
#define T_ZZ	6
#define T_XY	7
#define T_YZ	8
#define T_ZX	9
	    
//TODO: REPLACE using table
static u32 comb(const u32 o, const u32 u)
{
	ds_Assert(u <= o);

	u32 v1 = 1;
	u32 v2 = 1;
	u32 rep = (u <= o-u) ? u : o-u;

	for (u32 i = 0; i < rep; ++i)
	{
		v1 *= (o-i);
		v2 *= (i+1);
	}

	ds_Assert(v1 % v2 == 0);

	return v1 / v2;
}

static f32 statics_internal_line_integrals(const vec2 v0, const vec2 v1, const vec2 v2, const u32 p, const u32 q, const vec3 int_scalars)
{
       ds_Assert(p <= 4 && q <= 4);
       
       f32 sum = 0.0f;
       for (u32 i = 0; i <= p; ++i)
       {
               for (u32 j = 0; j <= q; ++j)
               {
                       sum += int_scalars[0] * comb(p, i) * comb(q, j) * f32_pow(v1[0], (f32) i) * f32_pow(v0[0], (f32) (p-i)) * f32_pow(v1[1], (f32) j) * f32_pow(v0[1], (f32) (q-j)) / comb(p+q, i+j);
                       sum += int_scalars[1] * comb(p, i) * comb(q, j) * f32_pow(v2[0], (f32) i) * f32_pow(v1[0], (f32) (p-i)) * f32_pow(v2[1], (f32) j) * f32_pow(v1[1], (f32) (q-j)) / comb(p+q, i+j);
                       sum += int_scalars[2] * comb(p, i) * comb(q, j) * f32_pow(v0[0], (f32) i) * f32_pow(v2[0], (f32) (p-i)) * f32_pow(v0[1], (f32) j) * f32_pow(v2[1], (f32) (q-j)) / comb(p+q, i+j);
               }
       }

       return sum / (p+q+1);
}

/*
 *  alpha beta gamma CCW
 */ 
static void statics_internal_calculate_face_integrals(f32 integrals[10], const struct collision_shape *shape, const u32 fi)
{
	f32 P_1   = 0.0f;
	f32 P_a   = 0.0f;
	f32 P_aa  = 0.0f;
	f32 P_aaa = 0.0f;
	f32 P_b   = 0.0f;
	f32 P_bb  = 0.0f;
	f32 P_bbb = 0.0f;
	f32 P_ab  = 0.0f;
	f32 P_aab = 0.0f;
	f32 P_abb = 0.0f;

	vec3 n, a, b;
	vec2 v0, v1, v2;

	vec3ptr v = shape->hull.v;
	struct dcelFace *f = shape->hull.f + fi;
	struct dcelEdge *e0 = shape->hull.e + f->first;
	struct dcelEdge *e1 = shape->hull.e + f->first + 1;
	struct dcelEdge *e2 = shape->hull.e + f->first + 2;

	Vec3Sub(a, v[e1->origin], v[e0->origin]);
	Vec3Sub(b, v[e2->origin], v[e0->origin]);
	Vec3Cross(n, a, b);
	Vec3ScaleSelf(n, 1.0f / Vec3Length(n));
	const f32 d = -Vec3Dot(n, v[e0->origin]);

	u32 max_index = 0;
	if (n[max_index]*n[max_index] < n[1]*n[1]) { max_index = 1; }
	if (n[max_index]*n[max_index] < n[2]*n[2]) { max_index = 2; }

	/* maxized normal direction determines projected surface integral axes (we maximse the projected surface area) */
	
	const u32 a_i = (1+max_index) % 3;
	const u32 b_i = (2+max_index) % 3;
	const u32 y_i = max_index % 3;

	//Vec3Set(n, n[a_i], n[b_i], n[y_i]);

	/* TODO: REPLACE */
	union { f32 f; u32 bits; } val = { .f = n[y_i] };
	const f32 n_sign = (val.bits >> 31) ? -1.0f : 1.0f;

	const u32 tri_count = f->count - 2;
	for (u32 i = 0; i < tri_count; ++i)
	{
		e0 = shape->hull.e + f->first;
		e1 = shape->hull.e + f->first + 1 + i;
		e2 = shape->hull.e + f->first + 2 + i;

		Vec2Set(v0, v[e0->origin][a_i], v[e0->origin][b_i]);
		Vec2Set(v1, v[e1->origin][a_i], v[e1->origin][b_i]);
		Vec2Set(v2, v[e2->origin][a_i], v[e2->origin][b_i]);
		
		const vec3 delta_a =
		{
			v1[0] - v0[0],
			v2[0] - v1[0],
			v0[0] - v2[0],
		};
		
		const vec3 delta_b = 
		{
			v1[1] - v0[1],
			v2[1] - v1[1],
			v0[1] - v2[1],
		};

		/* simplify cross product of v1-v0, v2-v0 to get this */
		P_1   += ((v0[0] + v1[0])*delta_b[0] + (v1[0] + v2[0])*delta_b[1] + (v0[0] + v2[0])*delta_b[2]) / 2.0f;
		P_a   +=  statics_internal_line_integrals(v0, v1, v2, 2, 0, delta_b);
		P_aa  +=  statics_internal_line_integrals(v0, v1, v2, 3, 0, delta_b);
		P_aaa +=  statics_internal_line_integrals(v0, v1, v2, 4, 0, delta_b);
		P_b   += -statics_internal_line_integrals(v0, v1, v2, 0, 2, delta_a);
		P_bb  += -statics_internal_line_integrals(v0, v1, v2, 0, 3, delta_a);
		P_bbb += -statics_internal_line_integrals(v0, v1, v2, 0, 4, delta_a);
		P_ab  +=  statics_internal_line_integrals(v0, v1, v2, 2, 1, delta_b);
		P_aab +=  statics_internal_line_integrals(v0, v1, v2, 3, 1, delta_b);
		P_abb +=  statics_internal_line_integrals(v0, v1, v2, 1, 3, delta_b);
	}

	P_1   *= n_sign;
	P_a   *= (n_sign / 2.0f); 
	P_aa  *= (n_sign / 3.0f); 
	P_aaa *= (n_sign / 4.0f); 
	P_b   *= (n_sign / 2.0f); 
	P_bb  *= (n_sign / 3.0f); 
	P_bbb *= (n_sign / 4.0f); 
	P_ab  *= (n_sign / 2.0f); 
	P_aab *= (n_sign / 3.0f); 
	P_abb *= (n_sign / 3.0f); 

	const f32 a_y_div = n_sign / n[y_i];
	const f32 n_y_div = 1.0f / n[y_i];

	/* surface integrals */
	const f32 S_a 	= a_y_div * P_a;
	const f32 S_aa 	= a_y_div * P_aa;
	const f32 S_aaa = a_y_div * P_aaa;
	const f32 S_aab = a_y_div * P_aab;
	const f32 S_b 	= a_y_div * P_b;
	const f32 S_bb 	= a_y_div * P_bb;
	const f32 S_bbb = a_y_div * P_bbb;
	const f32 S_bby = -a_y_div * n_y_div * (n[a_i]*P_abb + n[b_i]*P_bbb + d*P_bb);
	const f32 S_y 	= -a_y_div * n_y_div * (n[a_i]*P_a + n[b_i]*P_b + d*P_1);
	const f32 S_yy 	= a_y_div * n_y_div * n_y_div * (n[a_i]*n[a_i]*P_aa + 2.0f*n[a_i]*n[b_i]*P_ab + n[b_i]*n[b_i]*P_bb 
			+ 2.0f*d*n[a_i]*P_a + 2.0f*d*n[b_i]*P_b + d*d*P_1);	
	const f32 S_yyy = -a_y_div * n_y_div * n_y_div * n_y_div * (n[a_i]*n[a_i]*n[a_i]*P_aaa + 3.0f*n[a_i]*n[a_i]*n[b_i]*P_aab
			+ 3.0f*n[a_i]*n[b_i]*n[b_i]*P_abb + n[b_i]*n[b_i]*n[b_i]*P_bbb + 3.0f*d*n[a_i]*n[a_i]*P_aa 
			+ 6.0f*d*n[a_i]*n[b_i]*P_ab + 3.0f*d*n[b_i]*n[b_i]*P_bb + 3.0f*d*d*n[a_i]*P_a
		       	+ 3.0f*d*d*n[b_i]*P_b + d*d*d*P_1);
	const f32 S_yya = a_y_div * n_y_div * n_y_div * (n[a_i]*n[a_i]*P_aaa + 2.0f*n[a_i]*n[b_i]*P_aab + n[b_i]*n[b_i]*P_abb 
			+ 2.0f*d*n[a_i]*P_aa + 2.0f*d*n[b_i]*P_ab + d*d*P_a);	

	if (max_index == 2)
	{
		integrals[VOL] += S_a * n[0];
	}
	else if (max_index == 1)
	{
		integrals[VOL] += S_b * n[0];
	}
	else
	{
		integrals[VOL] += S_y * n[0];
	}

	integrals[T_X + a_i] += S_aa * n[a_i] / 2.0f;
	integrals[T_X + b_i] += S_bb * n[b_i] / 2.0f;
	integrals[T_X + y_i] += S_yy * n[y_i] / 2.0f;

	integrals[T_XX + a_i] += S_aaa * n[a_i] / 3.0f;
	integrals[T_XX + b_i] += S_bbb * n[b_i] / 3.0f;
	integrals[T_XX + y_i] += S_yyy * n[y_i] / 3.0f;

	integrals[T_XY + a_i] += S_aab * n[a_i] / 2.0f;
	integrals[T_XY + b_i] += S_bby * n[b_i] / 2.0f;
	integrals[T_XY + y_i] += S_yya * n[y_i] / 2.0f;
}

void prefab_statics_setup(struct rigid_body_prefab *prefab, struct collision_shape *shape, const f32 density)
{
	f32 I_xx = 0.0f;
	f32 I_yy = 0.0f;
	f32 I_zz = 0.0f;
	f32 I_xy = 0.0f;
	f32 I_xz = 0.0f;
	f32 I_yz = 0.0f;
	vec3 com = VEC3_ZERO;

	if (shape->type == COLLISION_SHAPE_CONVEX_HULL)
	{
		if (!shape->center_of_mass_localized)
		{
			f32 integrals[10] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }; 
			for (u32 fi = 0; fi < shape->hull.f_count; ++fi)
			{
				statics_internal_calculate_face_integrals(integrals, shape, fi);
			}

			const f32 mass = integrals[VOL] * density;
			ds_Assert(mass >= 0.0f);
			/* center of mass */
			Vec3Set(com,
				integrals[T_X] * density / mass,
			       	integrals[T_Y] * density / mass,
			       	integrals[T_Z] * density / mass
			);

			Vec3NegateSelf(com);
			for (u32 i = 0; i < shape->hull.v_count; ++i)
			{
				Vec3Translate(shape->hull.v[i], com);
			}
		}

		f32 integrals[10] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }; 
		for (u32 fi = 0; fi < shape->hull.f_count; ++fi)
		{
			statics_internal_calculate_face_integrals(integrals, shape, fi);
		}

//		fprintf(stderr, "c_hull Volume integrals: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
//				 integrals[VOL ],
//				 integrals[T_X ],
//				 integrals[T_Y ],
//				 integrals[T_Z ],
//				 integrals[T_XX],
//				 integrals[T_YY],
//				 integrals[T_ZZ],
//      	                   integrals[T_XY],
//      	                   integrals[T_YZ],
//      	                   integrals[T_ZX]);

		prefab->mass = integrals[VOL] * density;
		ds_Assert(prefab->mass >= 0.0f);

		I_xx = density * (integrals[T_YY] + integrals[T_ZZ]);
		I_yy = density * (integrals[T_XX] + integrals[T_ZZ]);
		I_zz = density * (integrals[T_XX] + integrals[T_YY]);
		I_xy = density * integrals[T_XY];
		I_xz = density * integrals[T_ZX];
		I_yz = density * integrals[T_YZ];
		Mat3Set(prefab->inertia_tensor, I_xx, -I_xy, -I_xz,
			       		 	 -I_xy,  I_yy, -I_yz,
						 -I_xz, -I_yz, I_zz);



		///* center of mass *//
		//Vec3Set(com,
		//	integrals[T_X] * density / prefab->mass,
		//       	integrals[T_Y] * density / prefab->mass,
		//       	integrals[T_Z] * density / prefab->mass
		//);

		//I_xx = density * (integrals[T_YY] + integrals[T_ZZ]) - prefab->mass * (com[1]*com[1] + com[2]*com[2]);
		//I_yy = density * (integrals[T_XX] + integrals[T_ZZ]) - prefab->mass * (com[0]*com[0] + com[2]*com[2]);
		//I_zz = density * (integrals[T_XX] + integrals[T_YY]) - prefab->mass * (com[0]*com[0] + com[1]*com[1]);
		//I_xy = density * integrals[T_XY] - prefab->mass * com[0] * com[1];
		//I_xz = density * integrals[T_ZX] - prefab->mass * com[0] * com[2];
		//I_yz = density * integrals[T_YZ] - prefab->mass * com[1] * com[2];
	
		/* set local frame coordinates */
		//Mat3Set(prefab->inertia_tensor, I_xx, -I_xy, -I_xz,
		//	       		 	 -I_xy,  I_yy, -I_yz,
		//				 -I_xz, -I_yz, I_zz);
	}
	else if (shape->type == COLLISION_SHAPE_SPHERE)
	{
		const f32 r = shape->sphere.radius;
		const f32 rr = r*r;
		const f32 rrr = rr*r;
		prefab->mass = density * 4.0f * F32_PI * rrr / 3.0f;
		I_xx = 2.0f * prefab->mass * rr / 5.0f;
		I_yy = I_xx;
		I_zz = I_xx;
		I_xy = 0.0f;
		I_yz = 0.0f;
		I_xz = 0.0f;

		Mat3Set(prefab->inertia_tensor, I_xx, -I_xy, -I_xz,
			       		 	 -I_xy,  I_yy, -I_yz,
						 -I_xz, -I_yz, I_zz);
	}
	else if (shape->type == COLLISION_SHAPE_CAPSULE)
	{
		const f32 r = shape->capsule.radius;
		const f32 h = shape->capsule.half_height;
		const f32 hpr = h+r;
		const f32 hmr = h-r;

		prefab->mass = density * 4.0f * F32_PI * r*r*r / 3.0f + density * 2.0f *h * F32_PI * r*r;

		const f32 I_xx_cap_up = (4.0f * F32_PI * r*r * h*h*h + 3.0f * F32_PI * r*r*r*r * h) / 6.0f;
		const f32 I_xx_sph_up = 2.0f * F32_PI * r*r * (hpr*hpr*hpr - hmr*hmr*hmr) / 3.0f + F32_PI * r*r*r*r*r;
		const f32 I_xx_up = I_xx_sph_up + I_xx_cap_up;
		const f32 I_zz_up = I_xx_up;

		const f32 I_yy_cap_up = F32_PI * r*r*r*r * h;
		const f32 I_yy_sph_up = 2.0f * F32_PI * r*r*r*r*r;
		const f32 I_yy_up = I_yy_cap_up + I_yy_sph_up;

		const f32 I_xy_up = 0;
		const f32 I_yz_up = 0;
		const f32 I_xz_up = 0;

		/* Derive */
		Mat3Set(prefab->inertia_tensor, I_xx_up, -I_xy_up, -I_xz_up,
			       		 	 -I_xy_up,  I_yy_up, -I_yz_up,
						 -I_xz_up, -I_yz_up,  I_zz_up);
	}

	shape->center_of_mass_localized = 1;
	Mat3Inverse(prefab->inv_inertia_tensor, prefab->inertia_tensor);
}
