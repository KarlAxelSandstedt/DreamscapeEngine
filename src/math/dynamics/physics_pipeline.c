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

#include "float32.h"
#include "dynamics.h"
#include "ds_job.h"

const char *body_color_mode_str_buf[RB_COLOR_MODE_COUNT] = 
{
	"RB_COLOR_MODE_BODY",
	"RB_COLOR_MODE_COLLISION",
	"RB_COLOR_MODE_ISLAND",
	"RB_COLOR_MODE_SLEEP",
};

const char **body_color_mode_str = body_color_mode_str_buf;

dsThreadLocal struct collisionDebug *tl_debug;

u32 g_a_thread_counter = 0;

static void ThreadSetCollisionDebug(void *task_addr)
{
	struct task *task = task_addr;
	struct worker *worker = task->executor;
	const struct ds_RigidBodyPipeline *pipeline = task->input;
	tl_debug = pipeline->debug + ds_ThreadSelfIndex();

	AtomicFetchAddRel32(&g_a_thread_counter, 1);
	while (AtomicLoadAcq32(&g_a_thread_counter) != pipeline->debug_count);
}

struct ds_RigidBodyPipeline PhysicsPipelineAlloc(struct arena *mem, const u32 initial_size, const u64 ns_tick, const u64 frame_memory, struct strdb *cshape_db, struct strdb *prefab_db)
{
	struct ds_RigidBodyPipeline pipeline =
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
		SolverConfigInit(iteration_count, block_solver, warmup_solver, gravity, baumgarte_constant, max_condition, linear_dampening, angular_dampening, linear_slop, restitution_threshold, sleep_enabled, sleep_time_threshold, sleep_linear_velocity_sq_limit, sleep_angular_velocity_sq_limit);

	}

	ds_AssertString(PowerOfTwoCheck(initial_size), "For simplicity of future data structures, expect pipeline sizes to be powers of two");

	pipeline.body_pool = PoolAlloc(NULL, initial_size, struct ds_RigidBody, GROWABLE);
	pipeline.body_marked_list = dll_Init(struct ds_RigidBody);
	pipeline.body_non_marked_list = dll_Init(struct ds_RigidBody);

	pipeline.shape_pool = PoolAlloc(NULL, initial_size, struct ds_Shape, GROWABLE);
	pipeline.shape_bvh = DbvhAlloc(NULL, 2*initial_size, 1);

	pipeline.event_pool = PoolAlloc(NULL, 256, struct physicsEvent, GROWABLE);
	pipeline.event_list = dll_Init(struct physicsEvent);

	pipeline.cshape_db = cshape_db;

	pipeline.cdb = cdb_Alloc(mem, initial_size);
	pipeline.is_db = isdb_Alloc(mem, initial_size);

    pipeline.margin_on = 0;
	pipeline.margin = COLLISION_DEFAULT_MARGIN;

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

	pipeline.debug_count = g_arch_config->logical_core_count;
	pipeline.debug = malloc(g_arch_config->logical_core_count * sizeof(struct collisionDebug));
	for (u32 i = 0; i < pipeline.debug_count; ++i)
	{
		pipeline.debug[i].stack_segment = stack_visualSegmentAlloc(NULL, 1024, GROWABLE);
		task_stream_dispatch(&pipeline.frame, stream, ThreadSetCollisionDebug, &pipeline);
	}

	task_main_master_run_available_jobs();

	/* spin wait until last job completes */
	task_stream_spin_wait(stream);
	/* release any task resources */
	task_stream_cleanup(stream);		
#endif

	return pipeline;
}

void PhysicsPipelineFree(struct ds_RigidBodyPipeline *pipeline)
{
#ifdef DS_PHYSICS_DEBUG
	for (u32 i = 0; i < pipeline->debug_count; ++i)
	{
		stack_visualSegmentFree(&pipeline->debug[i].stack_segment);
	}
	free(pipeline->debug);
#endif
	BvhFree(&pipeline->shape_bvh);
	cdb_Free(&pipeline->cdb);
	isdb_Dealloc(&pipeline->is_db);
	PoolDealloc(&pipeline->body_pool);
	PoolDealloc(&pipeline->event_pool);
	PoolDealloc(&pipeline->shape_pool);
}

static void InternalPhysicsPipelineClearFrame(struct ds_RigidBodyPipeline *pipeline)
{
#ifdef DS_PHYSICS_DEBUG
	for (u32 i = 0; i < pipeline->debug_count; ++i)
	{
		stack_visualSegmentFlush(&pipeline->debug[i].stack_segment);
	}
#endif
	pipeline->proxy_overlap_count = 0;
	pipeline->proxy_overlap = NULL;
	pipeline->cm_count = 0;
	pipeline->cm = NULL;

	isdb_ClearFrame(&pipeline->is_db);
	cdb_ClearFrame(&pipeline->cdb);
	ArenaFlush(&pipeline->frame);
}


void PhysicsPipelineFlush(struct ds_RigidBodyPipeline *pipeline)
{
#ifdef DS_PHYSICS_DEBUG
	for (u32 i = 0; i < pipeline->debug_count; ++i)
	{
		stack_visualSegmentFlush(&pipeline->debug[i].stack_segment);
	}
#endif
	cdb_Flush(&pipeline->cdb);
	isdb_Flush(&pipeline->is_db);
	
	PoolFlush(&pipeline->body_pool);
	dll_Flush(&pipeline->body_marked_list);
	dll_Flush(&pipeline->body_non_marked_list);

	DbvhFlush(&pipeline->shape_bvh);
	PoolFlush(&pipeline->shape_pool);

	PoolFlush(&pipeline->event_pool);
	dll_Flush(&pipeline->event_list);

	ArenaFlush(&pipeline->frame);
	pipeline->frames_completed = 0;
	pipeline->ns_elapsed = 0;
}

//void PhysicsPipelineValidate(const struct ds_RigidBodyPipeline *pipeline)
//{
//	ProfZone;
//
//	cdb_Validate(pipeline);
//	isdb_Validate(pipeline);
//
//	ProfZoneEnd;
//}

static void InternalUpdateShapeBvh(struct ds_RigidBodyPipeline *pipeline)
{
	ProfZone;

	const u32 flags = RB_ACTIVE | RB_DYNAMIC | (g_solver_config->sleep_enabled * RB_AWAKE);
	const struct ds_RigidBody *body = NULL;
	for (u32 i = pipeline->body_non_marked_list.first; i != DLL_NULL; i = dll_Next(body))
	{
		body = PoolAddress(&pipeline->body_pool, i);
		if ((body->flags & flags) == flags)
		{
            struct ds_Shape *shape = NULL;
            for (u32 j = body->shape_list.first; j != DLL_NULL; j = shape->dll_next)
            {
                shape = PoolAddress(&pipeline->shape_pool, j);
                struct aabb bbox = ds_ShapeWorldBbox(pipeline, shape);
                const struct bvhNode *node = PoolAddress(&pipeline->shape_bvh.tree.pool, shape->proxy);
			    const struct aabb *proxy = &node->bbox;
			    if (!AabbContains(proxy, &bbox))
			    {
			    	bbox.hw[0] += shape->margin;
			    	bbox.hw[1] += shape->margin;
			    	bbox.hw[2] += shape->margin;
			    	DbvhRemove(&pipeline->shape_bvh, shape->proxy);
			    	shape->proxy = DbvhInsert(&pipeline->shape_bvh, j, &bbox);
			    }
            }
		}
	}

	ProfZoneEnd;
}

static void InternalPushProxyOverlaps(struct arena *mem_frame, struct ds_RigidBodyPipeline *pipeline)
{
	ProfZone;
	pipeline->proxy_overlap = DbvhPushOverlapPairs(mem_frame, &pipeline->proxy_overlap_count, &pipeline->shape_bvh);
	ProfZoneEnd;
}

struct tpcOutput
{
	struct c_Manifold * manifold;
	u32                 manifold_count;
};

static void ThreadPushContacts(void *task_addr)
{
	ProfZone;

	struct task *task = task_addr;
	struct worker *worker = task->executor;
	struct ds_RigidBodyPipeline *pipeline = task->input;
	const struct task_range *range = task->range;
	const struct dbvhOverlap *proxy_overlap = range->base;

	struct tpcOutput *out = ArenaPush(&worker->mem_frame, range->count*sizeof(struct tpcOutput));
	out->manifold_count = 0;
	out->manifold = ArenaPush(&worker->mem_frame, range->count * sizeof(struct c_Manifold));

	const f32 margin = (pipeline->margin_on) ? pipeline->margin : 0.0f;
	const struct ds_Shape *s1, *s2;

	for (u64 i = 0; i < range->count; ++i)
	{
		s1 = PoolAddress(&pipeline->shape_pool, proxy_overlap[i].id1);
		s2 = PoolAddress(&pipeline->shape_pool, proxy_overlap[i].id2);

        if (s1->body == s2->body)
        {
            continue;
        }

		if (ds_ShapeContact(&worker->mem_frame, out->manifold + out->manifold_count, pipeline, s1, s2, margin))
		{
			out->manifold[out->manifold_count].i1 = proxy_overlap[i].id1;
			out->manifold[out->manifold_count].i2 = proxy_overlap[i].id2;
			out->manifold_count += 1;
		}	
	}

	ArenaPopPacked(&worker->mem_frame, (range->count - out->manifold_count)*sizeof(struct tpcOutput));

	task->output = out;
	ProfZoneEnd;
}

static void InternalParallelPushContacts(struct arena *mem_frame, struct ds_RigidBodyPipeline *pipeline)
{
//	struct task_bundle *bundle = task_bundle_split_range(
//			mem_frame, 
//			&ThreadPushContacts, 
//			g_task_ctx->worker_count, 
//			pipeline->proxy_overlap, 
//			pipeline->proxy_overlap_count, 
//			sizeof(struct dbvhOverlap), 
//			pipeline);
//
//	ProfZone;
//	pipeline->cm = (struct c_Manifold *) ArenaPush(mem_frame, pipeline->proxy_overlap_count * sizeof(struct c_Manifold));
//	ArenaPushRecord(mem_frame);
//
//	pipeline->cm_count = 0;
//	if (bundle)
//	{	
//		task_main_master_run_available_jobs();
//		task_bundle_wait(bundle);
//
//		for (u32 i = 0; i < bundle->task_count; ++i)
//		{
//			struct tpcOutput *out = (struct tpcOutput *) AtomicLoadAcq64(&bundle->tasks[i].output);
//			for (u32 j = 0; j < out->result_count; ++j)
//			{
//				if (out->result[j].type == COLLISION_SAT_CACHE)
//				{
//					sat_CacheAdd(&pipeline->cdb, &out->result[j].sat_cache);
//					if (out->result[j].sat_cache.type != SAT_CACHE_SEPARATION)
//					{
//						pipeline->cm[pipeline->cm_count++] = out->result[j].manifold;
//					}
//				}
//				else
//				{
//					pipeline->cm[pipeline->cm_count++] = out->result[j].manifold;
//				}
//			}
//		}
//	
//		task_bundle_release(bundle);
//	}
//	
//	ArenaPopRecord(mem_frame);
//	ArenaPopPacked(mem_frame, (pipeline->proxy_overlap_count - pipeline->cm_count) * sizeof(struct c_Manifold));
//
//	pipeline->cdb.contacts_frame_usage = BitVecAlloc(mem_frame, pipeline->cdb.contacts_persistent_usage.bit_count, 0, 0);
//	ds_Assert(pipeline->cdb.contacts_frame_usage.block_count == pipeline->cdb.contacts_persistent_usage.block_count);
//	ds_Assert(pipeline->cdb.contacts_frame_usage.bit_count == pipeline->cdb.contacts_persistent_usage.bit_count);
//
//	pipeline->contact_new_count = 0;
//	pipeline->contact_new = (u32 *) mem_frame->stack_ptr;
//	//fprintf(stderr, "A: {");
//	{
//		ProfZoneNamed("internal_gather_real_contacts");
//		for (u32 i = 0; i < pipeline->cm_count; ++i)
//		{
//			const struct ds_Contact *c = cdb_ContactAdd(pipeline, pipeline->cm + i, pipeline->cm[i].i1, pipeline->cm[i].i2);
//			/* add to new links if needed */
//			const u32 index = (u32) nll_Index(&pipeline->cdb.contact_net, c);
//			if (index >= pipeline->cdb.contacts_persistent_usage.bit_count
//				 || BitVecGetBit(&pipeline->cdb.contacts_persistent_usage, index) == 0)
//			{
//					pipeline->contact_new_count += 1;
//					ArenaPushPackedMemcpy(mem_frame, &index, sizeof(index));
//			}
//			//fprintf(stderr, " %u", index);
//		}
//		ProfZoneEnd;
//	}
//	//fprintf(stderr, " } ");
//
//	ProfZoneEnd;
}

//static void InternalMergeIslands(struct arena *mem_frame, struct ds_RigidBodyPipeline *pipeline)
//{
//	ProfZone;
//	for (u32 i = 0; i < pipeline->contact_new_count; ++i)
//	{
//		struct ds_Contact *c = nll_Address(&pipeline->cdb.contact_net, pipeline->contact_new[i]);
//		const struct ds_RigidBody *body1 = PoolAddress(&pipeline->body_pool, c->cm.i1);
//		const struct ds_RigidBody *body2 = PoolAddress(&pipeline->body_pool, c->cm.i2);
//		const u32 is1 = body1->island_index;
//		const u32 is2 = body2->island_index;
//		const u32 d1 = (is1 != ISLAND_STATIC) ? 0x2 : 0x0;
//		const u32 d2 = (is2 != ISLAND_STATIC) ? 0x1 : 0x0;
//		switch (d1 | d2)
//		{
//			/* dynamic-dynamic */
//			case 0x3: 
//			{
//				isdb_MergeIslands(pipeline, pipeline->contact_new[i], c->cm.i1, c->cm.i2);
//			} break;
//
//			/* dynamic-static */
//			case 0x2:
//			{
//				struct island *is = PoolAddress(&pipeline->is_db.island_pool, is1);
//				dll_Append(&is->contact_list, pipeline->cdb.contact_net.pool.buf, pipeline->contact_new[i]);
//			} break;
//
//			/* static-dynamic */
//			case 0x1:
//			{
//				struct island *is = PoolAddress(&pipeline->is_db.island_pool, is2);
//				dll_Append(&is->contact_list, pipeline->cdb.contact_net.pool.buf, pipeline->contact_new[i]);
//			} break;
//		}
//	}
//	ProfZoneEnd;
//}
//
//static void InternalRemoveContactsAndTagSplitIslands(struct arena *mem_frame, struct ds_RigidBodyPipeline *pipeline)
//{
//	ProfZone;
//	if (pipeline->cdb.contact_net.pool.count == 0) 
//	{ 
//		ProfZoneEnd;
//		return; 
//	}
//
//	/* 
//	 * For every removed contact
//	 * (1)	if island is not tagged, tag island and push. 
//	 * (2) 	remove contact
//	 */
//
//	/* Remove any contacts that were not persistent */
//	//fprintf(stderr, " R: {");
//	u32 bit = 0;
//	isdb_ReserveSplitsMemory(mem_frame, &pipeline->is_db);
//	for (u64 block = 0; block < pipeline->cdb.contacts_frame_usage.block_count; ++block)
//	{
//		u64 broken_link_block = 
//				    pipeline->cdb.contacts_persistent_usage.bits[block]
//				& (~pipeline->cdb.contacts_frame_usage.bits[block]);
//		u32 b = 0;
//		while (broken_link_block)
//		{
//			const u32 tzc = Ctz64(broken_link_block);
//			b += tzc;
//			const u32 ci = bit + b;
//			b += 1;
//
//			broken_link_block = (tzc < 63) 
//				? broken_link_block >> (tzc + 1)
//				: 0;
//		
//			//fprintf(stderr, " %lu", ci);
//			struct ds_Contact *c = nll_Address(&pipeline->cdb.contact_net, ci);
//
//			/* tag island, if any exist, to split */
//			const u32 b1 = CONTACT_KEY_TO_BODY_0(c->key);
//			const u32 b2 = CONTACT_KEY_TO_BODY_1(c->key);
//			const struct ds_RigidBody *body1 = PoolAddress(&pipeline->body_pool, b1);
//			const struct ds_RigidBody *body2 = PoolAddress(&pipeline->body_pool, b2);
//			ds_Assert(body1->island_index != ISLAND_STATIC || body2->island_index != ISLAND_STATIC);
//
//			struct island *is;
//			if (body1->island_index != ISLAND_STATIC)
//			{
//				is = isdb_BodyToIsland(pipeline, b1);
//				if (body2->island_index != ISLAND_STATIC)
//				{
//					isdb_TagForSplitting(pipeline, b1);
//				}
//			}
//			else
//			{
//				is = isdb_BodyToIsland(pipeline, b2);
//			}
//
//			ds_Assert(is->contact_list.count > 0);
//			dll_Remove(&is->contact_list, pipeline->cdb.contact_net.pool.buf, ci);
//			cdb_ContactRemove(pipeline, c->key, (u32) ci);
//		}
//		bit += 64;
//	}	
//	isdb_ReleaseUnusedSplitsMemory(mem_frame, &pipeline->is_db);
//	//fprintf(stderr, " }\tcontacts: %u\n", pipeline->cdb.contacts->count-2);
//	ProfZoneEnd;
//}
//
//static void InternalSplitIslands(struct arena *mem_frame, struct ds_RigidBodyPipeline *pipeline)
//{
//	ProfZone;
//	/* TODO: Parallelize island splitting */
//
//	for (u32 i = 0; i < pipeline->is_db.possible_splits_count; ++i)
//	{
//		isdb_SplitIsland(mem_frame, pipeline, pipeline->is_db.possible_splits[i]);
//	}
//
//	cdb_UpdatePersistentContactsUsage(&pipeline->cdb);
//
//	ProfZoneEnd;
//}
//
//static void InternalParallelSolveIslands(struct arena *mem_frame, struct ds_RigidBodyPipeline *pipeline, const f32 delta) 
//{
//	ProfZone;
//
//	/* acquire any task resources */
//	struct task_stream *stream = task_stream_init(mem_frame);
//	struct islandSolveOutput *output = NULL;
//	struct islandSolveOutput **next = &output;
//
//	struct island *is = NULL;
//	for (u32 i = pipeline->is_db.island_list.first; i != DLL_NULL; i = dll_Next(is))
//	{
//		is = PoolAddress(&pipeline->is_db.island_pool, i);
//		if (!g_solver_config->sleep_enabled || ISLAND_AWAKE_BIT(is))
//		{
//			struct islandSolveInput *args = ArenaPush(mem_frame, sizeof(struct islandSolveInput));
//			*next = ArenaPush(mem_frame, sizeof(struct islandSolveOutput));
//			(*next)->island = i;
//			(*next)->island_asleep = 0;
//			(*next)->next = NULL;
//			args->out = *next;
//			args->is = is;
//			args->pipeline = pipeline;
//			args->timestep = delta;
//			task_stream_dispatch(mem_frame, stream, ThreadIslandSolve, args);
//
//			next = &(*next)->next;
//		}
//	}
//
//	task_main_master_run_available_jobs();
//
//	/* spin wait until last job completes */
//	task_stream_spin_wait(stream);
//	/* release any task resources */
//	task_stream_cleanup(stream);		
//
//
//	/*
//	 * TODO:
//	 * 	(1) pipeline->event_list sequential list of physics events
//	 * 	(2) implement array_list_flush to clear whole list 
//	 */
//	for (; output; output = output->next)
//	{
//		if (output->island_asleep)
//		{
//			PhysicsEventIslandAsleep(pipeline, output->island);
//		}
//
//		for (u32 i = 0; i < output->body_count; ++i)
//		{
//			struct physicsEvent *event = PhysicsPipelineEventPush(pipeline);
//			event->type = PHYSICS_EVENT_BODY_ORIENTATION;
//			event->body = output->bodies[i];
//		}
//	}
//
//	ProfZoneEnd;
//}

void PhysicsPipelineSleepEnable(struct ds_RigidBodyPipeline *pipeline)
{
	ds_Assert(g_solver_config->sleep_enabled == 0);
	if (!g_solver_config->sleep_enabled)
	{
		g_solver_config->sleep_enabled = 1;
		const u32 body_flags = RB_ACTIVE | RB_DYNAMIC;
		struct ds_RigidBody *body = NULL;
		for (u32 i = pipeline->body_non_marked_list.first; i != DLL_NULL; i = dll_Next(body))
		{
			body = PoolAddress(&pipeline->body_pool, i);
			if (body->flags & body_flags)
			{
				body->flags |= RB_AWAKE;
			}
		}

		struct island *is = NULL;
		for (u32 i = pipeline->is_db.island_list.first; i != DLL_NULL; i = dll_Next(is))
		{
			is = PoolAddress(&pipeline->is_db.island_pool, i);
			is->flags |= ISLAND_AWAKE | ISLAND_SLEEP_RESET;
			is->flags &= ~ISLAND_TRY_SLEEP;
		}
	}
}

void PhysicsPipelineSleepDisable(struct ds_RigidBodyPipeline *pipeline)
{
	ds_Assert(g_solver_config->sleep_enabled == 1);
	if (g_solver_config->sleep_enabled)
	{
		g_solver_config->sleep_enabled = 0;
		const u32 body_flags = RB_ACTIVE | RB_DYNAMIC;
		struct ds_RigidBody *body = NULL;
		for (u32 i = pipeline->body_non_marked_list.first; i != DLL_NULL; i = dll_Next(body))
		{
			body = PoolAddress(&pipeline->body_pool, i);
			if (body->flags & body_flags)
			{
				body->flags |= RB_AWAKE;
			}
		}

		struct island *is = NULL;
		for (u32 i = pipeline->is_db.island_list.first; i != DLL_NULL; i = dll_Next(is))
		{
			is = PoolAddress(&pipeline->is_db.island_pool, i);
			is->flags |= ISLAND_AWAKE;
			is->flags &= ~(ISLAND_SLEEP_RESET | ISLAND_TRY_SLEEP);
		}
	}
}

static void InternalUpdateSolverConfig(struct ds_RigidBodyPipeline *pipeline)
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
			? PhysicsPipelineSleepEnable(pipeline)
			: PhysicsPipelineSleepDisable(pipeline);

		g_solver_config->sleep_enabled = g_solver_config->pending_sleep_enabled;
	}
}

void PhysicsPipelineRigidBodyTagForRemoval(struct ds_RigidBodyPipeline *pipeline, const u32 handle)
{
	struct ds_RigidBody *b = PoolAddress(&pipeline->body_pool, handle);
	if (!RB_IS_MARKED(b))
	{
		b->flags |= RB_MARKED_FOR_REMOVAL;
		dll_Remove(&pipeline->body_non_marked_list, pipeline->body_pool.buf, handle);
		dll_Append(&pipeline->body_marked_list, pipeline->body_pool.buf, handle);
	}
}

static void InternalRemoveMarkedBodies(struct ds_RigidBodyPipeline *pipeline)
{
	struct ds_RigidBody *b = NULL;
	for (u32 i = pipeline->body_marked_list.first; i != DLL_NULL; i = dll_Next(b))
	{
		b = PoolAddress(&pipeline->body_pool, i);
		ds_RigidBodyRemove(pipeline, i);
	}

	dll_Flush(&pipeline->body_marked_list);
}

void InternalPhysicsPipelineSimulateFrame(struct ds_RigidBodyPipeline *pipeline, const f32 delta)
{
	InternalRemoveMarkedBodies(pipeline);

	/* update, if possible, any pending values in contact solver config */
	InternalUpdateSolverConfig(pipeline);

	/* broadphase => narrowphase => solve => integrate */
	InternalUpdateShapeBvh(pipeline);
	InternalPushProxyOverlaps(&pipeline->frame, pipeline);
	InternalParallelPushContacts(&pipeline->frame, pipeline);

	//InternalMergeIslands(&pipeline->frame, pipeline);
	//InternalRemoveContactsAndTagSplitIslands(&pipeline->frame, pipeline);
	//InternalSplitIslands(&pipeline->frame, pipeline);
	//InternalParallelSolveIslands(&pipeline->frame, pipeline, delta);

	//PHYSICS_PIPELINE_VALIDATE(pipeline);
}

void PhysicsPipelineTick(struct ds_RigidBodyPipeline *pipeline)
{
	ProfZone;

	if (pipeline->frames_completed > 0)
	{
		InternalPhysicsPipelineClearFrame(pipeline);
	}
	pipeline->frames_completed += 1;
	const f32 delta = (f32) pipeline->ns_tick / NSEC_PER_SEC;
	InternalPhysicsPipelineSimulateFrame(pipeline, delta);

	ProfZoneEnd;
}

u32f32 PhysicsPipelineRaycastParameter(struct arena *mem_tmp1, struct arena *mem_tmp2, const struct ds_RigidBodyPipeline *pipeline, const struct ray *ray)
{
	ArenaPushRecord(mem_tmp1);
	ArenaPushRecord(mem_tmp2);

	struct bvhRaycastInfo info = BvhRaycastInit(mem_tmp1, &pipeline->shape_bvh, ray);
	while (info.hit_queue.count)
	{
		const u32f32 tuple = MinQueueFixedPop(&info.hit_queue);
		if (info.hit.f < tuple.f)
		{
			break;	
		}

		if (bt_LeafCheck(info.node + tuple.u))
		{
			const u32 si = info.node[tuple.u].bt_left;
			const struct ds_Shape *shape = (struct ds_Shape *) pipeline->shape_pool.buf + si;
			const f32 t = ds_ShapeRaycastParameter(mem_tmp2, pipeline, shape, ray);
			if (t < info.hit.f)
			{
				info.hit = u32f32_inline(si, t);
			}
		}
		else
		{
			BvhRaycastTestAndPushChildren(&info, tuple);
		}
	}

	ArenaPopRecord(mem_tmp1);
	ArenaPopRecord(mem_tmp2);

	return info.hit;
}

struct physicsEvent *PhysicsPipelineEventPush(struct ds_RigidBodyPipeline *pipeline)
{
	struct slot slot = PoolAdd(&pipeline->event_pool);
	dll_Append(&pipeline->event_list, pipeline->event_pool.buf, slot.index);
	struct physicsEvent *event = slot.address;
	event->ns = pipeline->ns_start + pipeline->frames_completed * pipeline->ns_tick;
	return event;
}
