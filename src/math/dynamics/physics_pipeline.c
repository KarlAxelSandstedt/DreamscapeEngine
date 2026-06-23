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

struct collisionDebug *g_collision_debug;

void ds_DynamicsStaticAssert(void)
{
    ds_StaticAssert(sizeof(struct ds_NarrowPhaseJob) == 2*DS_CACHE_LINE, "");
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
        const f32 max_linear_velocity_magnitude = 400.0f;
        const f32 max_angular_velocity_magnitude = 10.0f * F32_PI;

		init_solver_once = 1;
		const u32 pgs_iteration_count = 8;
		const u32 ngs_iteration_count = 3;
		const u32 warmup_solver = 1;
		const vec3 gravity = { 0.0f, -GRAVITY_CONSTANT_DEFAULT, 0.0f };
       	const f32 baumgarte_constant = 0.1f;
        const f32 max_linear_correction = 0.2f;
		const f32 linear_dampening = 0.1f;
		const f32 angular_dampening = 0.1f;
		const f32 linear_slop = 0.005f;
		const f32 restitution_threshold = 0.001f;
		const u32 sleep_enabled = 1;
		const f32 sleep_time_threshold = 0.5f;
		f32 sleep_linear_velocity_sq_limit = 0.005f*0.005f; 
		f32 sleep_angular_velocity_sq_limit = 0.01f*0.01f*2.0f*F32_PI;
		SolverConfigInit(pgs_iteration_count, ngs_iteration_count, warmup_solver, gravity, baumgarte_constant, max_linear_correction, max_linear_velocity_magnitude, max_angular_velocity_magnitude, linear_dampening, angular_dampening, linear_slop, restitution_threshold, sleep_enabled, sleep_time_threshold, sleep_linear_velocity_sq_limit, sleep_angular_velocity_sq_limit);
	}

	ds_AssertString(PowerOfTwoCheck(initial_size), "For simplicity of future data structures, expect pipeline sizes to be powers of two");

	pipeline.body_pool = ds_PoolAlloc(NULL, initial_size, struct ds_RigidBody, GROWABLE);
	pipeline.body_marked_list = dll_Init(struct ds_RigidBody);
	pipeline.body_non_marked_list = dll_Init(struct ds_RigidBody);

	pipeline.shape_pool = ds_PoolAlloc(NULL, initial_size, struct ds_Shape, GROWABLE);
	pipeline.shape_bvh = DbvhAlloc(NULL, 2*initial_size, 1);

	pipeline.event_pool = ds_PoolAlloc(NULL, 256, struct physicsEvent, GROWABLE);
	pipeline.event_list = dll_Init(struct physicsEvent);

	pipeline.cshape_db = cshape_db;

	pipeline.cdb = cdb_Alloc(mem, initial_size);
	pipeline.is_db = isdb_Alloc(mem, initial_size);

    pipeline.margin_on = 0;
	pipeline.margin = COLLISION_DEFAULT_MARGIN;

	pipeline.debug_count = 0;
	pipeline.debug = NULL;

    pipeline.cd_jobs = ArenaPushAligned(mem, sizeof(struct ds_CollisionJobPhase), DS_CACHE_LINE);
    ds_JobPhaseAlloc(mem, &pipeline.cd_jobs->phase, COLLISION_JOB_COUNT);
#ifdef DS_PHYSICS_DEBUG
	pipeline.debug_count = g_arch_config->logical_core_count;
	pipeline.debug = malloc(g_arch_config->logical_core_count * sizeof(struct collisionDebug));
    g_collision_debug = pipeline.debug;
	for (u32 i = 0; i < pipeline.debug_count; ++i)
	{
		pipeline.debug[i].stack_segment = stack_visualSegmentAlloc(NULL, 1024, GROWABLE);
	}
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
    ds_JobPhaseDealloc(&pipeline->cd_jobs->phase);
	BvhFree(&pipeline->shape_bvh);
	cdb_Free(pipeline->cdb);
	isdb_Dealloc(&pipeline->is_db);
	ds_PoolDealloc(&pipeline->body_pool);
	ds_PoolDealloc(&pipeline->event_pool);
	ds_PoolDealloc(&pipeline->shape_pool);
}

static void PhysicsPipelineClearFrame(struct ds_RigidBodyPipeline *pipeline)
{
#ifdef DS_PHYSICS_DEBUG
	for (u32 i = 0; i < pipeline->debug_count; ++i)
	{
		stack_visualSegmentFlush(&pipeline->debug[i].stack_segment);
	}
#endif
	isdb_ClearFrame(&pipeline->is_db);
	cdb_ClearFrame(pipeline->cdb);
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
	cdb_Flush(pipeline->cdb);
	isdb_Flush(&pipeline->is_db);
	
	ds_PoolFlush(&pipeline->body_pool);
	dll_Flush(&pipeline->body_marked_list);
	dll_Flush(&pipeline->body_non_marked_list);

	DbvhFlush(&pipeline->shape_bvh);
	ds_PoolFlush(&pipeline->shape_pool);

	ds_PoolFlush(&pipeline->event_pool);
	dll_Flush(&pipeline->event_list);

	ArenaFlush(&pipeline->frame);
	pipeline->frames_completed = 0;
	pipeline->ns_elapsed = 0;
}

void PhysicsPipelineValidate(const struct ds_RigidBodyPipeline *pipeline)
{
	ProfZone;

	cdb_Validate(pipeline);
	isdb_Validate(pipeline);

	ProfZoneEnd;
}

struct tcc_Output
{
    struct tcc_Output *     next;
    struct sat_Cache *      cache;
	struct c_Manifold       manifold;
    struct ds_ContactKey    key;
    u32                     collision;
    u32                     cache_index;
};

struct tcc_Input
{
    struct tcc_Output *             out;
    struct ds_RigidBodyPipeline *   pipeline;
    struct ds_Shape *               s1;
    struct ds_Shape *               s2;
};

static void NarrowPhaseSeedJobExecute(struct ds_CollisionJobPhase *phase, struct ds_NarrowPhaseSeedJob *job)
{
	ProfZone;

    LogString(T_SYSTEM, S_NOTE, "Seeding...");

    const struct ds_RigidBodyPipeline *pipeline = phase->pipeline;

    const u32 count = job->high - job->low;
    const u32 base = ds_JobPhaseReserve(&phase->phase, COLLISION_JOB_NARROWPHASE, count);
    ds_Assert(base + count <= phase->narrowphase_count_max);

    u32 job_count = 0;
    for (u32 i = 0; i < count; ++i)
    {
        const u32 index = base + i;
        const struct dbvhOverlap *overlap = phase->overlap + job->low + i;
        struct ds_Shape *s1 = (struct ds_Shape *) pipeline->shape_pool.buf + overlap->id1;
        struct ds_Shape *s2 = (struct ds_Shape *) pipeline->shape_pool.buf + overlap->id2;
        if (s1->body == s2->body)
        {
            phase->narrowphase_jobs[index].valid = 0;
            continue;
        }

        job_count += 1;
        phase->narrowphase_jobs[index].valid = 1;
        phase->narrowphase_jobs[index].key = ds_ContactKeyCanonical(s1->body, overlap->id1, s2->body, overlap->id2);
    
        ds_WSDequePushBottom(g_scheduler->deque + ds_ThreadSelfIndex(), ds_JobIdInit(COLLISION_JOB_NARROWPHASE, index));
    }

    ds_JobPhaseFetchAddRemaining(&phase->phase, job_count);
    ds_JobPhaseFetchDecrementRemaining(&phase->phase);

	ProfZoneEnd;
}

static void NarrowPhaseJobExecute(struct ds_CollisionJobPhase *phase, struct ds_NarrowPhaseJob *job)
{
	ProfZone;

    const struct ds_RigidBodyPipeline *pipeline = phase->pipeline;
    job->cache = NULL;
    job->cache_index = U32_MAX;

    const struct ds_RigidBody *b0 = (struct ds_RigidBody *) pipeline->body_pool.buf + job->key.body0;
    const struct ds_RigidBody *b1 = (struct ds_RigidBody *) pipeline->body_pool.buf + job->key.body1;
    const struct ds_Shape *s0 = (struct ds_Shape *) pipeline->shape_pool.buf + job->key.shape0;
    const struct ds_Shape *s1 = (struct ds_Shape *) pipeline->shape_pool.buf + job->key.shape1;

    if (s0->cshape_type == C_SHAPE_CONVEX_HULL && s1->cshape_type == C_SHAPE_CONVEX_HULL)
    {
        const struct sat_CacheKey key = sat_CacheKeyCanonical(
            ((u64) b0->tag << 32) | job->key.body0,
            ((u64) s0->tag << 32) | job->key.shape0,
            ((u64) b1->tag << 32) | job->key.body1,
            ((u64) s1->tag << 32) | job->key.shape1);
 
        struct slot slot = sat_CacheLookup(pipeline->cdb, &key);
        if (!slot.address)
        {
            slot = sat_CacheAdd(pipeline->cdb, &key);
        }
        job->cache_index = slot.index;
        job->cache = slot.address;
    }

    ds_Assert(s0->body != s1->body);
    job->collision = ds_ShapeContact(&g_scheduler->worker[ds_ThreadSelfIndex()].mem_frame, &job->manifold, job->cache, pipeline, s0, s1);

    ds_JobPhaseFetchDecrementRemaining(&phase->phase);

	ProfZoneEnd;
}

void ds_CollisionJobPhaseDispatch(const ds_JobId job)
{
    struct ds_CollisionJobPhase *phase = (struct ds_CollisionJobPhase *) g_scheduler->phase;
 
    const u32 index = ds_JobIdIndex(job);
    const enum ds_CollisionJobType type = ds_JobIdTag(job);

    switch (type)
    {
        case COLLISION_JOB_SEED: { NarrowPhaseSeedJobExecute(phase, phase->seed_jobs + index); } break;
        case COLLISION_JOB_NARROWPHASE: { NarrowPhaseJobExecute(phase, phase->narrowphase_jobs + index); } break;
        default: { ds_AssertString(0, "Should not be possible"); } break;
    };
}

static void CollisionDetection(struct ds_RigidBodyPipeline *pipeline)
{
	ProfZone;
    struct cdb *cdb = pipeline->cdb;

    {
    	ProfZoneNamed("DbvhUpdate");
    
    	const u32 flags = RB_ACTIVE | RB_DYNAMIC | (g_solver_config->sleep_enabled * RB_AWAKE);
    	const struct ds_RigidBody *body = NULL;
    	for (u32 i = pipeline->body_non_marked_list.first; i != DLL_NULL; i = dll_Next(body))
    	{
    		body = ds_PoolAddress(&pipeline->body_pool, i);
    		if ((body->flags & flags) == flags)
    		{
                struct ds_Shape *shape = NULL;
                for (u32 j = body->shape_list.first; j != DLL_NULL; j = shape->dll_next)
                {
                    shape = ds_PoolAddress(&pipeline->shape_pool, j);
                    struct aabb bbox = ds_ShapeWorldBbox(pipeline, shape);
                    const struct bvhNode *node = ds_PoolAddress(&pipeline->shape_bvh.tree.pool, shape->proxy);
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

	struct dbvhOverlap *proxy_overlap = NULL;
	u32 proxy_overlap_count = 0;
    {
    	ProfZoneNamed("Broadphase");
    	proxy_overlap = DbvhPushOverlapPairs(&pipeline->frame, &proxy_overlap_count, &pipeline->shape_bvh);
    	ProfZoneEnd;
    }

    struct ds_CollisionJobPhase *cd_jobs = pipeline->cd_jobs;
    ds_JobPhaseBegin(&cd_jobs->phase);
    {
    	ProfZoneNamed("NarrowPhase");
  
        cd_jobs->phase.dispatch = &ds_CollisionJobPhaseDispatch;
        cd_jobs->pipeline = pipeline;
        cd_jobs->overlap = proxy_overlap;

        cd_jobs->seed_count_max = g_arch_config->logical_core_count;
        cd_jobs->seed_jobs = ArenaPush(&pipeline->frame, cd_jobs->seed_count_max*sizeof(struct ds_NarrowPhaseSeedJob));

        cd_jobs->narrowphase_count_max = proxy_overlap_count;
        cd_jobs->narrowphase_jobs = ArenaPushAligned(&pipeline->frame, cd_jobs->narrowphase_count_max*sizeof(struct ds_NarrowPhaseJob), DS_CACHE_LINE);

        const u32 jobs_per_seed = proxy_overlap_count / cd_jobs->seed_count_max;
        u32 extra = proxy_overlap_count % cd_jobs->seed_count_max;
        u32 low = 0;
        for (u32 i = 0; i < cd_jobs->seed_count_max; ++i)
        {
            cd_jobs->seed_jobs[i].low = low;
            cd_jobs->seed_jobs[i].high = low + jobs_per_seed;
            if (extra)
            {
                cd_jobs->seed_jobs[i].high += 1;
                extra -= 1;
            }

            low = cd_jobs->seed_jobs[i].high;
        }
        ds_Assert(cd_jobs->seed_jobs[cd_jobs->seed_count_max - 1].high == proxy_overlap_count);

        ds_JobPhaseFetchAddRemaining(&cd_jobs->phase, cd_jobs->seed_count_max);
        for (u32 i = 0; i < cd_jobs->seed_count_max; ++i)
        {
            ds_WSDequePushBottom(g_scheduler->deque + 0, ds_JobIdInit(COLLISION_JOB_SEED, i));
        }
        
        for (u32 i = 1; i < g_arch_config->logical_core_count; ++i)
        {
            SemaphorePost(&g_scheduler->jobs_are_available);
        }

        ds_MasterRunAvailableJobs();
    }
    ds_JobPhaseEnd(&cd_jobs->phase);

    {
    	ProfZoneNamed("ContactManagement");

	    cdb->sat_cache_frame_usage = BitVecAlloc(&pipeline->frame, cdb->sat_cache_persistent_usage.bit_count, 0, 0);
	    cdb->contact_frame_usage = BitVecAlloc(&pipeline->frame, cdb->contact_persistent_usage.bit_count, 0, 0);

        const u32 narrowphase_count = AtomicLoadRlx32(&cd_jobs->phase.next[COLLISION_JOB_NARROWPHASE].a_counter);
        struct memArray arr = ArenaPushAlignedAll(&pipeline->frame, sizeof(u32), sizeof(u32));
        cdb->contact_new = arr.addr;
        //fprintf(stderr, "A: {");
        for (u32 i = 0; i < narrowphase_count; ++i)
        {
            const struct ds_NarrowPhaseJob *job = cd_jobs->narrowphase_jobs + i;
            if (!job->valid)
            {
                continue;
            }

            if (job->cache)
            {
                cdb->sat_cache_count += 1;
                if (job->cache_index < cdb->sat_cache_persistent_usage.bit_count)
                {
                    BitVecSetBit(&cdb->sat_cache_frame_usage, job->cache_index, 1);   
                }

                if (job->collision)
                {
                    cdb->contact_count += 1;
                    struct slot slot = ds_ContactKeyLookup(pipeline, &job->key);
                    if (!slot.address)
                    {
                        slot = ds_ContactAdd(pipeline, &job->manifold, &job->key);
                    }
                    else
                    {
                        ds_ContactUpdate(pipeline, slot, &job->manifold);
                    }
                     
			        /* add to new links if needed */
			        if (slot.index >= cdb->contact_persistent_usage.bit_count
			        	 || BitVecGetBit(&cdb->contact_persistent_usage, slot.index) == 0)
			        {
                            if (cdb->contact_new_count >= arr.len)
                            {
                                LogString(T_PHYSICS, S_FATAL, "Frame arena OOM in Broadphase, increase size!");
                                FatalCleanupAndExit();
                            }
                            cdb->contact_new[ cdb->contact_new_count ] = slot.index;
			        		cdb->contact_new_count += 1;
			        }
			        //fprintf(stderr, " %u", index);
                }
            }
        }
        //fprintf(stderr, " } ");
        ArenaPopPacked(&pipeline->frame, sizeof(u32)*(arr.len - cdb->contact_new_count));

        /* Remove stale sat_Caches */
	    u32 bit = 0;
	    for (u64 block = 0; block < cdb->sat_cache_frame_usage.block_count; ++block)
	    {
	    	u64 broken_link_block = 
	    			    cdb->sat_cache_persistent_usage.bits[block]
	    			& (~cdb->sat_cache_frame_usage.bits[block]);
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

	    		sat_CacheRemove(cdb, ci);
	    	}
	    	bit += 64;
	    }	
    
        /* Update sat_cache_persistent_usage */
        for (u64 i = 0; i < cdb->sat_cache_frame_usage.block_count; ++i)
        {
        	cdb->sat_cache_persistent_usage.bits[i] = cdb->sat_cache_frame_usage.bits[i];	
        }

        const u32 count_max = AtomicLoadRlx32(&cdb->sat_cache_pool.a_count_max);
        const u32 length = AtomicLoadRlx32(&cdb->sat_cache_pool.a_length);
        if (cdb->sat_cache_persistent_usage.bit_count < count_max)
        {
        	const u64 low_bit = cdb->sat_cache_persistent_usage.bit_count;
        	const u64 high_bit = count_max;
        	BitVecIncreaseSize(&cdb->sat_cache_persistent_usage, length, 0);
        	/* any new sat_caches that is in the appended region must now be set */
        	for (u64 bit = low_bit; bit < high_bit; ++bit)
        	{
        		BitVecSetBit(&cdb->sat_cache_persistent_usage, bit, 1);
        	}
        }

    	ProfZoneEnd;
    }

	ProfZoneEnd;
}

static void MergeIslands(struct ds_RigidBodyPipeline *pipeline)
{
	ProfZone;
	for (u32 i = 0; i < pipeline->cdb->contact_new_count; ++i)
	{
		struct ds_Contact *c = nll_Address(&pipeline->cdb->contact_net, pipeline->cdb->contact_new[i]);
		const struct ds_RigidBody *body0 = ds_PoolAddress(&pipeline->body_pool, c->key.body0);
		const struct ds_RigidBody *body1 = ds_PoolAddress(&pipeline->body_pool, c->key.body1);
		const u32 is0 = body0->island_index;
		const u32 is1 = body1->island_index;
		const u32 d0 = (is0 != ISLAND_STATIC) ? 0x2 : 0x0;
		const u32 d1 = (is1 != ISLAND_STATIC) ? 0x1 : 0x0;
		switch (d0 | d1)
		{
			/* dynamic-dynamic */
			case 0x3: 
			{
				isdb_MergeIslands(pipeline, pipeline->cdb->contact_new[i], c->key.body0, c->key.body1);
			} break;

			/* dynamic-static */
			case 0x2:
			{
				struct ds_Island *is = ds_PoolAddress(&pipeline->is_db.island_pool, is0);
				dll_Append(&is->contact_list, pipeline->cdb->contact_net.pool.buf, pipeline->cdb->contact_new[i]);
			} break;

			/* static-dynamic */
			case 0x1:
			{
				struct ds_Island *is = ds_PoolAddress(&pipeline->is_db.island_pool, is1);
				dll_Append(&is->contact_list, pipeline->cdb->contact_net.pool.buf, pipeline->cdb->contact_new[i]);
			} break;
		}
	}
	ProfZoneEnd;
}

static void SplitIslandsAndRemoveContacts(struct ds_RigidBodyPipeline *pipeline)
{
	ProfZone;

    struct cdb *cdb = pipeline->cdb;

	if (cdb->contact_net.pool.count == 0) 
	{ 
		ProfZoneEnd;
		return; 
	}
    
	u32 *split = ArenaPush(&pipeline->frame, pipeline->is_db.island_pool.count*sizeof(u32));
    u32 split_count = 0;
	u32 bit = 0;
	//fprintf(stderr, " R: {");
	for (u64 block = 0; block < cdb->contact_frame_usage.block_count; ++block)
	{
		u64 broken_link_block = 
				    cdb->contact_persistent_usage.bits[block]
				& (~cdb->contact_frame_usage.bits[block]);
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
			struct ds_Contact *c = nll_Address(&cdb->contact_net, ci);

			const u32 b0 = c->key.body0;
			const u32 b1 = c->key.body1;
			const struct ds_RigidBody *body0 = ds_PoolAddress(&pipeline->body_pool, b0);
			const struct ds_RigidBody *body1 = ds_PoolAddress(&pipeline->body_pool, b1);
			ds_Assert(body0->island_index != ISLAND_STATIC || body1->island_index != ISLAND_STATIC);

			struct ds_Island *is;
			if (body0->island_index != ISLAND_STATIC)
			{
				is = isdb_BodyToIsland(pipeline, b0);
				if (body1->island_index != ISLAND_STATIC)
				{
                    if (!(is->flags & ISLAND_SPLIT))
                    {
                    	is->flags |= ISLAND_SPLIT;
                    	split[split_count++] = body0->island_index;
                        ds_Assert(split_count <= pipeline->is_db.island_pool.count);
                    }
				}
			}
			else
			{
				is = isdb_BodyToIsland(pipeline, b1);
			}

			ds_Assert(is->contact_list.count > 0);
			dll_Remove(&is->contact_list, cdb->contact_net.pool.buf, ci);
			ds_ContactRemove(pipeline, ci);
		}
		bit += 64;
	}	
	ArenaPopPacked(&pipeline->frame, (pipeline->is_db.island_pool.count - split_count)*sizeof(u32));

    struct arena tmp = ArenaAlloc1MB();
	/* TODO: Parallelize island splitting */
	for (u32 i = 0; i < split_count; ++i)
	{
		isdb_SplitIsland(&tmp, pipeline, split[i]);
	}

    /* Update contact_persistent_usage */
    {
        for (u64 i = 0; i < cdb->contact_frame_usage.block_count; ++i)
        {
        	cdb->contact_persistent_usage.bits[i] = cdb->contact_frame_usage.bits[i];	
        }

        if (cdb->contact_persistent_usage.bit_count < cdb->contact_net.pool.count_max)
        {
        	const u64 low_bit = cdb->contact_persistent_usage.bit_count;
        	const u64 high_bit = cdb->contact_net.pool.count_max;
        	BitVecIncreaseSize(&cdb->contact_persistent_usage, cdb->contact_net.pool.length, 0);
        	/* any new contacts that is in the appended region must now be set */
        	for (u64 bit = low_bit; bit < high_bit; ++bit)
        	{
        		BitVecSetBit(&cdb->contact_persistent_usage, bit, 1);
        	}
        }
    } 
	ArenaFree1MB(&tmp);

	ProfZoneEnd;
}

static void SolveIslands(struct ds_RigidBodyPipeline *pipeline, const f32 delta) 
{
	ProfZone;

	/* acquire any task resources */
	struct task_stream *stream = task_stream_init(&pipeline->frame);
	struct ds_IslandSolveOutput *output = NULL;
	struct ds_IslandSolveOutput **next = &output;

	struct ds_Island *is = NULL;
	for (u32 i = pipeline->is_db.island_list.first; i != DLL_NULL; i = dll_Next(is))
	{
		is = ds_PoolAddress(&pipeline->is_db.island_pool, i);
		if (!g_solver_config->sleep_enabled || ISLAND_AWAKE_BIT(is))
		{
			struct ds_IslandSolveInput *args = ArenaPush(&pipeline->frame, sizeof(struct ds_IslandSolveInput));
			*next = ArenaPush(&pipeline->frame, sizeof(struct ds_IslandSolveOutput));
			(*next)->island = i;
			(*next)->island_asleep = 0;
			(*next)->next = NULL;
			args->out = *next;
			args->is = is;
			args->pipeline = pipeline;
			args->timestep = delta;
			task_stream_dispatch(&pipeline->frame, stream, ThreadIslandSolve, args);

			next = &(*next)->next;
		}
	}

	task_main_master_run_available_jobs();
	/* spin wait until last job completes */
	task_stream_spin_wait(stream);
	/* release any task resources */
	task_stream_cleanup(stream);		

	for (; output; output = output->next)
	{
		if (output->island_asleep)
		{
			PhysicsEventIslandAsleep(pipeline, output->island);
		}

		for (u32 i = 0; i < output->body_count; ++i)
		{
			struct physicsEvent *event = PhysicsPipelineEventPush(pipeline);
			event->type = PHYSICS_EVENT_BODY_ORIENTATION;
            const struct ds_RigidBody *body = ds_PoolAddress(&pipeline->body_pool, output->bodies[i]);
			event->body = ((u64) body->tag << 32) | output->bodies[i];
		}
	}

	ProfZoneEnd;
}

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
			body = ds_PoolAddress(&pipeline->body_pool, i);
			if (body->flags & body_flags)
			{
				body->flags |= RB_AWAKE;
			}
		}

		struct ds_Island *is = NULL;
		for (u32 i = pipeline->is_db.island_list.first; i != DLL_NULL; i = dll_Next(is))
		{
			is = ds_PoolAddress(&pipeline->is_db.island_pool, i);
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
			body = ds_PoolAddress(&pipeline->body_pool, i);
			if (body->flags & body_flags)
			{
				body->flags |= RB_AWAKE;
			}
		}

		struct ds_Island *is = NULL;
		for (u32 i = pipeline->is_db.island_list.first; i != DLL_NULL; i = dll_Next(is))
		{
			is = ds_PoolAddress(&pipeline->is_db.island_pool, i);
			is->flags |= ISLAND_AWAKE;
			is->flags &= ~(ISLAND_SLEEP_RESET | ISLAND_TRY_SLEEP);
		}
	}
}

static void UpdateSolverConfig(struct ds_RigidBodyPipeline *pipeline)
{
	g_solver_config->warmup_solver = g_solver_config->pending_warmup_solver;
	g_solver_config->pgs_iteration_count = g_solver_config->pending_pgs_iteration_count;
	g_solver_config->ngs_iteration_count = g_solver_config->pending_ngs_iteration_count;
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
	struct ds_RigidBody *b = ds_PoolAddress(&pipeline->body_pool, handle);
	if (!RB_IS_MARKED(b))
	{
		b->flags |= RB_MARKED_FOR_REMOVAL;
		dll_Remove(&pipeline->body_non_marked_list, pipeline->body_pool.buf, handle);
		dll_Append(&pipeline->body_marked_list, pipeline->body_pool.buf, handle);
	}
}

static void RemoveMarkedBodies(struct ds_RigidBodyPipeline *pipeline)
{
    struct arena tmp = ArenaAlloc1MB();
    u32 next;
	for (u32 i = pipeline->body_marked_list.first; i != DLL_NULL; i = next)
	{
		struct ds_RigidBody *b = ds_PoolAddress(&pipeline->body_pool, i);
        next = dll_Next(b);
		ds_RigidBodyRemove(&tmp, pipeline, i);
	}

	dll_Flush(&pipeline->body_marked_list);
    ArenaFree1MB(&tmp);
}

void PhysicsPipelineSimulateFrame(struct ds_RigidBodyPipeline *pipeline, const f32 delta)
{
	RemoveMarkedBodies(pipeline);

	/* update, if possible, any pending values in contact solver config */
	UpdateSolverConfig(pipeline);

	/* broadphase => narrowphase => solve => integrate */
    CollisionDetection(pipeline);

	MergeIslands(pipeline);
	SplitIslandsAndRemoveContacts(pipeline);
	SolveIslands(pipeline, delta);

	PHYSICS_PIPELINE_VALIDATE(pipeline);
}

void PhysicsPipelineTick(struct ds_RigidBodyPipeline *pipeline)
{
	ProfZone;

	if (pipeline->frames_completed > 0)
	{
		PhysicsPipelineClearFrame(pipeline);
	}
	pipeline->frames_completed += 1;
	const f32 delta = (f32) pipeline->ns_tick / NSEC_PER_SEC;
	PhysicsPipelineSimulateFrame(pipeline, delta);

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
	struct slot slot = ds_PoolAdd(&pipeline->event_pool);
	dll_Append(&pipeline->event_list, pipeline->event_pool.buf, slot.index);
	struct physicsEvent *event = slot.address;
	event->ns = pipeline->ns_start + pipeline->frames_completed * pipeline->ns_tick;
	return event;
}

void PhysicsPipelinePrintUsage(const struct ds_RigidBodyPipeline *pipeline)
{
    fprintf(stderr, "Physics:\n");
    fprintf(stderr, "\tbodies:                      %u\n", pipeline->body_pool.count);
    fprintf(stderr, "\tshapes:                      %u\n", pipeline->shape_pool.count);
    fprintf(stderr, "\tshape_bvh nodes:             %u\n", pipeline->shape_bvh.tree.pool.count);
    fprintf(stderr, "\tevents:                      %u\n", pipeline->event_pool.count);
    fprintf(stderr, "\tislands:                     %u\n", pipeline->is_db.island_pool.count);
    fprintf(stderr, "\tcontacts:                    %u\n", pipeline->cdb->contact_net.pool.count);
    fprintf(stderr, "\tsat caches (max):            %u\n", AtomicLoadRlx32(&pipeline->cdb->sat_cache_pool.a_count_max));
    fprintf(stderr, "\tcontact bitvector size:      %lu\n", pipeline->cdb->contact_persistent_usage.block_count*sizeof(u64));
    fprintf(stderr, "\tsat cache bitvector size:    %lu\n", pipeline->cdb->sat_cache_persistent_usage.block_count*sizeof(u64));
}
