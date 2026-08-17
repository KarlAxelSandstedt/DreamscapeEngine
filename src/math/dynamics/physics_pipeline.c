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
#include "ds_job.h"

POOL_DEFINE(ds_PhysicsEvent);

struct collisionDebug *g_collision_debug;

void ds_DynamicsStaticAssert(void)
{
    ds_StaticAssert(sizeof(struct ds_NarrowPhaseJob) == DS_CACHE_LINE, "");
    ds_StaticAssert(sizeof(struct ds_RigidBodyCompute) == DS_CACHE_LINE, "");
}

struct ds_RigidBodyPipeline PhysicsPipelineAlloc(struct arena *mem, const u32 initial_size, const u64 ns_tick, const u64 frame_memory, struct strdb *cshape_db, struct strdb *prefab_db)
{
	struct ds_RigidBodyPipeline pipeline =
	{
		.gravity = { 0.0f, -GRAVITY_CONSTANT_DEFAULT, 0.0f },
		.ns_tick = ns_tick,
		.ns_elapsed = 0,
		.ns_start = 0,
		.frame = ArenaAlloc(mem, frame_memory),
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
		f32 sleep_linear_velocity_sq_limit = 0.01f*0.01f; 
		f32 sleep_angular_velocity_sq_limit = 0.05f*0.05f;
		SolverConfigInit(pgs_iteration_count, ngs_iteration_count, warmup_solver, gravity, baumgarte_constant, max_linear_correction, max_linear_velocity_magnitude, max_angular_velocity_magnitude, linear_dampening, angular_dampening, linear_slop, restitution_threshold, sleep_enabled, sleep_time_threshold, sleep_linear_velocity_sq_limit, sleep_angular_velocity_sq_limit);
	}

	ds_AssertString(PowerOfTwoCheck(initial_size), "For simplicity of future data structures, expect pipeline sizes to be powers of two");

	pipeline.body_pool = ds_RigidBodyPoolAlloc(NULL, initial_size, GROWABLE);
    pipeline.body_usage_set = ds_BitSetAlloc(NULL, initial_size, 0, GROWABLE);

    pipeline.joint_pool = ds_JointPoolAlloc(NULL, initial_size, GROWABLE);

	pipeline.shape_pool = ds_ShapePoolAlloc(NULL, initial_size, GROWABLE);
	pipeline.shape_bvh = DbvhAlloc(NULL, 2*initial_size, GROWABLE);

	pipeline.event_pool = ds_PhysicsEventPoolAlloc(NULL, 256, GROWABLE);
	ds_DLLFlush(&pipeline.event_list);

	pipeline.cshape_db = cshape_db;

	pipeline.cdb = cdb_Alloc(mem, initial_size);
	pipeline.island_pool = ds_IslandPoolAlloc(NULL, initial_size, GROWABLE);
    pipeline.island_high_energy_set = ds_BitSetAlloc(NULL, initial_size, 0, GROWABLE);

    pipeline.margin_on = 0;
	pipeline.margin = COLLISION_DEFAULT_MARGIN;

	pipeline.debug_count = 0;
	pipeline.debug = NULL;

    pipeline.cd_jobs = ArenaPushAligned(mem, sizeof(struct ds_CollisionJobPhase), DS_CACHE_LINE);
    pipeline.solver_phase = ArenaPushAligned(mem, sizeof(struct ds_SolverJobPhase), DS_CACHE_LINE);
    ds_JobPhaseAlloc(mem, &pipeline.cd_jobs->phase, COLLISION_JOB_COUNT, ds_CollisionJobPhaseDispatch);
    ds_JobPhaseAlloc(mem, &pipeline.solver_phase->phase, SOLVER_JOB_COUNT, ds_SolverJobPhaseDispatch);
#ifdef DS_PHYSICS_DEBUG
	pipeline.debug_count = g_scheduler->worker_count;
	pipeline.debug = malloc(g_scheduler->worker_count * sizeof(struct collisionDebug));
    g_collision_debug = pipeline.debug;
	for (u32 i = 0; i < pipeline.debug_count; ++i)
	{
		ds_CPoolAlloc(NULL, pipeline.debug[i].stack_segment, 1024, GROWABLE);
	}
#endif

    ds_CGraphAlloc(&pipeline, 4096);
    pipeline.numerics_config = ds_NumericsConfigDefault();

    pipeline.solver_set_pool = ds_SolverSetPoolAlloc(NULL, 4096, GROWABLE);
    const struct slot set_disabled = ds_SolverSetAdd(&pipeline, 256, 0, 0, 4096, 4096);
    const struct slot set_static = ds_SolverSetAdd(&pipeline, 256, 0, 0, 0, 0);
    const struct slot set_active = ds_SolverSetAdd(&pipeline, 4096, 4096, 0, 0, 4096);
    ds_Assert(set_disabled.index == SOLVER_SET_DISABLED);
    ds_Assert(set_static.index == SOLVER_SET_STATIC);
    ds_Assert(set_active.index == SOLVER_SET_ACTIVE);

    ds_SolverSetSetupDummies(&pipeline);

	return pipeline;
}

void PhysicsPipelineFree(struct ds_RigidBodyPipeline *pipeline)
{
#ifdef DS_PHYSICS_DEBUG
	for (u32 i = 0; i < pipeline->debug_count; ++i)
	{
		ds_CPoolDealloc(pipeline->debug[i].stack_segment);
	}
	free(pipeline->debug);
#endif
	BvhFree(&pipeline->shape_bvh);
	cdb_Free(pipeline->cdb);
	ds_IslandPoolDealloc(&pipeline->island_pool);
    ds_BitSetDealloc(&pipeline->island_high_energy_set);
	ds_RigidBodyPoolDealloc(&pipeline->body_pool);
	ds_PhysicsEventPoolDealloc(&pipeline->event_pool);
	ds_ShapePoolDealloc(&pipeline->shape_pool);
    ds_JointPoolDealloc(&pipeline->joint_pool);
    ds_CGraphDealloc(pipeline);
    ds_BitSetDealloc(&pipeline->body_usage_set);
    
    for (u32 i = 0; i < pipeline->solver_set_pool.count_max; ++i)
    {
        const struct ds_SolverSet *set = pipeline->solver_set_pool.buf + i;
        if (ds_PoolSlotAllocated(set))
        {
            ds_SolverSetRemove(pipeline, i);
        }
    }
    ds_SolverSetPoolDealloc(&pipeline->solver_set_pool);
}

static void PhysicsPipelineClearFrame(struct ds_RigidBodyPipeline *pipeline)
{
#ifdef DS_PHYSICS_DEBUG
	for (u32 i = 0; i < pipeline->debug_count; ++i)
	{
		ds_CPoolFlush(pipeline->debug[i].stack_segment);
	}
#endif
	cdb_ClearFrame(pipeline->cdb);
	ArenaFlush(&pipeline->frame);
    ds_CGraphFramePrepare(pipeline);
    ds_BitSetClear(&pipeline->island_high_energy_set, 0);
}


void PhysicsPipelineFlush(struct ds_RigidBodyPipeline *pipeline)
{
#ifdef DS_PHYSICS_DEBUG
	for (u32 i = 0; i < pipeline->debug_count; ++i)
	{
		ds_CPoolFlush(pipeline->debug[i].stack_segment);
	}
#endif

    ds_SolverSetFlush(pipeline, 0);
    ds_SolverSetFlush(pipeline, 1);
    ds_SolverSetFlush(pipeline, 2);
    for (u32 i = SOLVER_SET_SLEEPING_FIRST; i < pipeline->solver_set_pool.count_max; ++i)
    {
        const struct ds_SolverSet *set = pipeline->solver_set_pool.buf + i;
        if (ds_PoolSlotAllocated(set))
        {
            ds_SolverSetRemove(pipeline, i);
        }
    }

	cdb_Flush(pipeline->cdb);

	ds_IslandPoolFlush(&pipeline->island_pool);
	
	ds_RigidBodyPoolFlush(&pipeline->body_pool);
    ds_BitSetClear(&pipeline->body_usage_set, 0);

	DbvhFlush(&pipeline->shape_bvh);
	ds_ShapePoolFlush(&pipeline->shape_pool);

	ds_PhysicsEventPoolFlush(&pipeline->event_pool);
	ds_DLLFlush(&pipeline->event_list);

    ds_JointPoolFlush(&pipeline->joint_pool);
    ds_CGraphFlush(pipeline);

	ArenaFlush(&pipeline->frame);
	pipeline->frames_completed = 0;
	pipeline->ns_elapsed = 0;

    ds_SolverSetSetupDummies(pipeline);
}

void PhysicsPipelineValidate(const struct ds_RigidBodyPipeline *pipeline)
{
	ProfZone;

    for (u32 i = 0; i < pipeline->solver_set_pool.count_max; ++i)
    {
        const struct ds_SolverSet *set = pipeline->solver_set_pool.buf + i;
        if (ds_PoolSlotAllocated(set))
        {
            ds_SolverSetValidate(pipeline, i);
        }
    }

    ds_CGraphValidate(pipeline);
	cdb_Validate(pipeline);
	ds_IslandValidateAll(pipeline);

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

static u32 NarrowPhaseSeedJob(struct ds_CollisionJobPhase *phase, struct ds_NarrowPhaseSeedJob *job)
{
	ProfZone;

    const struct ds_RigidBodyPipeline *pipeline = phase->pipeline;

    const u32 count = job->high - job->low;
    const u32 base = ds_JobPhaseReserve(&phase->phase, COLLISION_JOB_NARROWPHASE, count);
    ds_Assert(base + count <= phase->narrowphase_count_max);

    u32 job_count = 0;
    for (u32 i = 0; i < count; ++i)
    {
        const u32 index = base + i;
        const struct dbvhOverlap *overlap = phase->overlap + job->low + i;
        struct ds_Shape *s1 = pipeline->shape_pool.buf + overlap->id1;
        struct ds_Shape *s2 = pipeline->shape_pool.buf + overlap->id2;
        struct ds_RigidBody *b1 = pipeline->body_pool.buf + s1->body; 
        struct ds_RigidBody *b2 = pipeline->body_pool.buf + s2->body; 
        if (s1->body == s2->body || ((!RB_IS_DYNAMIC(b1)) && (!RB_IS_DYNAMIC(b2))) )
        {
            phase->narrowphase_jobs[index].valid = 0;
            continue;
        }

        job_count += 1;
        phase->narrowphase_jobs[index].valid = 1;
        phase->narrowphase_jobs[index].key_in = ds_ContactKeyCanonical(s1->body, overlap->id1, s2->body, overlap->id2);
    
        ds_WSDequePushBottom(g_scheduler->deque + ds_ThreadSelfIndex(), ds_JobIdInit(COLLISION_JOB_NARROWPHASE, index));
    }

	ProfZoneEnd;

    return job_count - 1;
}

static u32 NarrowPhaseJob(struct ds_CollisionJobPhase *phase, struct ds_NarrowPhaseJob *job)
{
	ProfZone;

    const struct ds_RigidBodyPipeline *pipeline = phase->pipeline;
    job->cache = NULL;
    job->cache_index = U32_MAX;
    job->key = &job->key_in;

    const struct ds_RigidBody *b0 = pipeline->body_pool.buf + job->key_in.body0;
    const struct ds_RigidBody *b1 = pipeline->body_pool.buf + job->key_in.body1;
    const struct ds_Shape *s0 = pipeline->shape_pool.buf + job->key_in.shape0;
    const struct ds_Shape *s1 = pipeline->shape_pool.buf + job->key_in.shape1;

    
    ds_Assert(s0->body != s1->body);

    //TODO simplify with table lookups based on cshape_type...
    //TODO repetition between the two cases...
    if (s0->cshape_type == C_SHAPE_TRI_MESH || s1->cshape_type == C_SHAPE_TRI_MESH)
    {
        if (s0->cshape_type == C_SHAPE_CONVEX_HULL || s1->cshape_type == C_SHAPE_CONVEX_HULL)
        {
            const struct sat_CacheKey key = sat_CacheKeyCanonical(b0->id, s0->id, b1->id, s1->id);
 
            struct slot slot = sat_CacheLookup(pipeline->cdb, &key);
            if (!slot.address)
            {
                slot = sat_CacheAdd(pipeline->cdb, &key);
            }
            job->cache_index = slot.index;
            job->cache = slot.address;
        }

        u32 *tri;
        job->collision_count = ds_ShapeMeshContact(g_tl_self->frame, &job->manifold, &tri, job->cache, pipeline, s0, s1);
        job->key = ArenaPush(g_tl_self->frame, job->collision_count*sizeof(struct ds_ContactKey));
        if (s0->cshape_type == C_SHAPE_TRI_MESH)
        {
            for (u32 i = 0; i < job->collision_count; ++i)
            {
                job->key[i] = ds_ContactKeyCanonical(job->key_in.body0, INDIRECT_SHAPE_INIT(tri[i]), job->key_in.body1, job->key_in.shape1);
            }
        }
        else
        {
            for (u32 i = 0; i < job->collision_count; ++i)
            {
                job->key[i] = ds_ContactKeyCanonical(job->key_in.body0, job->key_in.shape0, job->key_in.body1, INDIRECT_SHAPE_INIT(tri[i]));
            }
        }

        //TODO
        for (u32 i = 0; i < job->collision_count; ++i)
        {
            if (!c_ManifoldCheck(job->manifold + i))
            {
                Breakpoint(1);
            }
        }
    }
    else
    {
        if (s0->cshape_type == C_SHAPE_CONVEX_HULL && s1->cshape_type == C_SHAPE_CONVEX_HULL)
        {
            const struct sat_CacheKey key = sat_CacheKeyCanonical(b0->id, s0->id, b1->id, s1->id);
            struct slot slot = sat_CacheLookup(pipeline->cdb, &key);
            if (!slot.address)
            {
                slot = sat_CacheAdd(pipeline->cdb, &key);
            }
            else
            {
                struct sat_Cache *cache = slot.address;
                /* Quick and dirty cache invalidation for fast moving objects; NOTE: not size invariant!  */
                if (cache->type != SAT_CACHE_SEPARATION)
                {
                    struct ds_SolverSet *active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;
                    struct ds_RigidBodyCompute *compute[2] =
                    {
                        active->body_compute_pool.buf + (RB_IS_DYNAMIC(b0) ? b0->sim : 0),
                        active->body_compute_pool.buf + (RB_IS_DYNAMIC(b1) ? b1->sim : 0),
                    };

                    vec3 diff;
                    Vec3Sub(diff, compute[0]->linear_velocity, compute[1]->linear_velocity);
                    const f32 linear_vel_abs_diff = f32_abs(Vec3Dot(diff, cache->normal));
                    if (linear_vel_abs_diff >= g_numerics_config->manifold_cache_linear_velocity_max_diff_allowed)
                    {
                        cache->type = SAT_CACHE_NOT_SET;
                    }
                }
            }
            
            job->cache_index = slot.index;
            job->cache = slot.address;
        }

        struct c_Manifold manifold;
        job->collision_count = ds_ShapeContact(&manifold, job->cache, pipeline, s0, s1);

        if (job->collision_count)
        {
            job->manifold = ArenaPushAlignedMemcpy(g_tl_self->frame, &manifold, sizeof(struct c_Manifold), 4);

            //TODO
            if (!c_ManifoldCheck(job->manifold))
            {
                Breakpoint(1);
                ds_ShapeContact(&manifold, job->cache, pipeline, s0, s1);
            }
        }
    }


	ProfZoneEnd;

    return U32_MAX;
}

u32 ds_CollisionJobPhaseDispatch(const ds_JobId job)
{
    struct ds_CollisionJobPhase *phase = (struct ds_CollisionJobPhase *) g_scheduler->phase;
 
    const u32 index = ds_JobIdIndex(job);
    const enum ds_CollisionJobType type = ds_JobIdTag(job);

    u32 job_diff = 0;
    switch (type)
    {
        case COLLISION_JOB_SEED: { job_diff = NarrowPhaseSeedJob(phase, phase->seed_jobs + index); } break;
        case COLLISION_JOB_NARROWPHASE: { job_diff = NarrowPhaseJob(phase, phase->narrowphase_jobs + index); } break;
        default: { ds_AssertString(0, "Should not be possible"); } break;
    };

    return job_diff;
}

static void CollisionDetection(struct ds_RigidBodyPipeline *pipeline)
{
	ProfZone;
    struct cdb *cdb = pipeline->cdb;

    {
    	ProfZoneNamed("DbvhUpdate");
        struct ds_SolverSet *active_set = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;
        for (u32 i = 0; i < active_set->body_sim_pool.count; ++i)
        {
            struct ds_RigidBodySim *sim = active_set->body_sim_pool.buf + i;
            struct ds_RigidBody *body = pipeline->body_pool.buf + sim->body;

            struct ds_Shape *shape = NULL;
            for (u32 j = body->shape_list.first; (i32) j != DLL_SENTINEL; j = shape->body_shape.next)
            {
                shape = pipeline->shape_pool.buf + j;
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
    {
    	ProfZoneNamed("JobPhase (NarrowPhase)");

        ds_JobPhaseBegin(&cd_jobs->phase);
  
        cd_jobs->pipeline = pipeline;
        cd_jobs->overlap = proxy_overlap;

        cd_jobs->seed_count_max = 3*g_scheduler->worker_count;
        cd_jobs->seed_jobs = ArenaPush(&pipeline->frame, cd_jobs->seed_count_max*sizeof(struct ds_NarrowPhaseSeedJob));
        ds_JobPhaseReserve(&cd_jobs->phase, COLLISION_JOB_SEED, cd_jobs->seed_count_max);

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
            ds_WSDequePushBottom(g_scheduler->seed_deque, ds_JobIdInit(COLLISION_JOB_SEED, i));
        }
        ds_Assert(cd_jobs->seed_jobs[cd_jobs->seed_count_max - 1].high == proxy_overlap_count);

        AtomicStoreRlx32(&g_scheduler->a_seeds_remaining, cd_jobs->seed_count_max);
        ds_JobPhaseAddFetchRemaining(&cd_jobs->phase, cd_jobs->seed_count_max);
        ds_WSDequePublish(g_scheduler->seed_deque);
        
        for (u32 i = 1; i < g_scheduler->worker_count; ++i)
        {
            SemaphorePost(&g_scheduler->jobs_are_available);
        }

        ds_MasterRunAvailableJobs();

        ds_JobPhaseEnd();

        ProfZoneEnd;
    }

    {
    	ProfZoneNamed("ContactManagement");

	    cdb->sat_cache_frame_usage = ds_BitSetAlloc(&pipeline->frame, cdb->sat_cache_persistent_usage.bit_count, 0, 0);
	    cdb->contact_frame_usage = ds_BitSetAlloc(&pipeline->frame, cdb->contact_persistent_usage.bit_count, 0, 0);

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
                    ds_BitSetSet(&cdb->sat_cache_frame_usage, job->cache_index, 1);   
                } 
            }

            for (u32 c = 0; c < job->collision_count; ++c)
            {
                cdb->contact_count += 1;
                struct slot slot = ds_ContactKeyLookup(pipeline, job->key + c);
                if (!slot.address)
                {
                    slot = ds_ContactAdd(pipeline, job->manifold + c, job->key + c);
                }
                else
                {
                    ds_ContactUpdate(pipeline, slot, job->manifold + c);
                }
                 
			    /* add to new links if needed */
			    if (slot.index >= cdb->contact_persistent_usage.bit_count
			    	 || ds_BitSetGet(&cdb->contact_persistent_usage, slot.index) == 0)
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
        //fprintf(stderr, " } ");
        ArenaPopPacked(&pipeline->frame, sizeof(u32)*(arr.len - cdb->contact_new_count));

        /* Remove stale sat_Caches */
	    for (u64 block = 0; block < cdb->sat_cache_frame_usage.block_count; ++block)
	    {
	    	const u64 broken_link_block = 
	    			    cdb->sat_cache_persistent_usage.bits[block]
	    			& (~cdb->sat_cache_frame_usage.bits[block]);
            struct ds_BitBlock it = ds_BitBlockInit(broken_link_block, block);
	    	while (ds_BitBlockHasNext(&it))
	    	{
	    	    sat_CacheRemove(cdb, ds_BitBlockNext(&it));
	    	}
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
        	ds_BitSetIncreaseSize(&cdb->sat_cache_persistent_usage, length, 0);
        	/* any new sat_caches that is in the appended region must now be set */
        	for (u64 bit = low_bit; bit < high_bit; ++bit)
        	{
        		ds_BitSetSet(&cdb->sat_cache_persistent_usage, bit, 1);
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
		struct ds_Contact *c = pipeline->cdb->contact_pool.buf + pipeline->cdb->contact_new[i];
		const struct ds_RigidBody *body0 = pipeline->body_pool.buf + c->key.body0;
		const struct ds_RigidBody *body1 = pipeline->body_pool.buf + c->key.body1;
		const u32 is0 = body0->island;
		const u32 is1 = body1->island;
		const u32 d0 = RB_IS_DYNAMIC(body0) ? 0x2 : 0x0;
		const u32 d1 = RB_IS_DYNAMIC(body1) ? 0x1 : 0x0;
		switch (d0 | d1)
		{
			/* dynamic-dynamic */
			case 0x3: 
			{
				ds_IslandMerge(pipeline, body0->island, body1->island, pipeline->cdb->contact_new[i]);
			} break;

			/* dynamic-static */
			case 0x2:
			{
				struct ds_Island *is = pipeline->island_pool.buf + is0;
				ds_DLLAppend(is->contact_list, pipeline->cdb->contact_pool.buf, pipeline->cdb->contact_new[i], island_contact);
                c->island = is0;
                /*
                 * TODO: This feels bad and dangerous; we've found a new contact of the island
                 * which is in the Constraint Graph while the rest of the island's contacts are
                 * in the sleeper set; it should be fine to wake up the set and move all 
                 * sleeping constraints to the Constraint Graph without messing up links, but
                 * it becomes very nasty to reason about
                 */
                if (is->set >= SOLVER_SET_SLEEPING_FIRST)
                {
                    ds_SolverSetWakeUp(pipeline, is->set);
	                PhysicsEventIslandAwake(pipeline, is0);	
                }
			} break;

			/* static-dynamic */
			case 0x1:
			{
				struct ds_Island *is = pipeline->island_pool.buf + is1;
				ds_DLLAppend(is->contact_list, pipeline->cdb->contact_pool.buf, pipeline->cdb->contact_new[i], island_contact);
                c->island = is1;
                /*
                 * TODO: This feels bad and dangerous; we've found a new contact of the island
                 * which is in the Constraint Graph while the rest of the island's contacts are
                 * in the sleeper set; it should be fine to wake up the set and move all 
                 * sleeping constraints to the Constraint Graph without messing up links, but
                 * it becomes very nasty to reason about
                 */
                if (is->set >= SOLVER_SET_SLEEPING_FIRST)
                {
                    ds_SolverSetWakeUp(pipeline, is->set);
	                PhysicsEventIslandAwake(pipeline, is1);	
                }
			} break;
		}
	}
	ProfZoneEnd;
}

static void SplitIslandsAndRemoveContacts(struct ds_RigidBodyPipeline *pipeline)
{
	ProfZone;

    struct cdb *cdb = pipeline->cdb;

	//fprintf(stderr, " R: {");
	for (u64 block = 0; block < cdb->contact_frame_usage.block_count; ++block)
	{
		const u64 broken_link_block = 
				    cdb->contact_persistent_usage.bits[block]
				& (~cdb->contact_frame_usage.bits[block]);

        struct ds_BitBlock it = ds_BitBlockInit(broken_link_block, block);
	    while (ds_BitBlockHasNext(&it))
	    {
            const u64 ci = ds_BitBlockNext(&it);
			struct ds_Contact *c = cdb->contact_pool.buf + ci;
			//fprintf(stderr, " %lu", ci);

			const u32 b0 = c->key.body0;
			const u32 b1 = c->key.body1;
			const struct ds_RigidBody *body0 = pipeline->body_pool.buf + b0;
			const struct ds_RigidBody *body1 = pipeline->body_pool.buf + b1;
			ds_Assert(RB_IS_DYNAMIC(body0) || RB_IS_DYNAMIC(body1));

			if (body0->set != SOLVER_SET_STATIC && body1->set != SOLVER_SET_STATIC)
			{
			    struct ds_Island *is = pipeline->island_pool.buf + body0->island;
                is->constraint_remove_count += 1;
			}

			ds_ContactRemove(pipeline, ci);
		}
	}	

    /* Update contact_persistent_usage */
    {
        for (u64 i = 0; i < cdb->contact_frame_usage.block_count; ++i)
        {
        	cdb->contact_persistent_usage.bits[i] = cdb->contact_frame_usage.bits[i];	
        }

        if (cdb->contact_persistent_usage.bit_count < cdb->contact_pool.count_max)
        {
        	const u64 low_bit = cdb->contact_persistent_usage.bit_count;
        	const u64 high_bit = cdb->contact_pool.count_max;
        	ds_BitSetIncreaseSize(&cdb->contact_persistent_usage, cdb->contact_pool.length, 0);
        	/* any new contacts that is in the appended region must now be set */
        	for (u64 bit = low_bit; bit < high_bit; ++bit)
        	{
        		ds_BitSetSet(&cdb->contact_persistent_usage, bit, 1);
        	}
        }
    }
    
    if (pipeline->island_to_split != DS_ID_NULL)
    {
        const u32 split_index = ds_IdIndex(pipeline->island_to_split);
        if (ds_PoolSlotAllocated(pipeline->island_pool.buf + split_index) 
                && pipeline->island_to_split == pipeline->island_pool.buf[split_index].id)
        {
            ds_IslandSplit(pipeline, split_index);
        }
    }

	ProfZoneEnd;
}

u32 ds_SolverJobPhaseDispatch(const ds_JobId job)
{
    ProfZone;

    struct ds_SolverJobPhase *phase = (struct ds_SolverJobPhase *) g_scheduler->phase;
    struct ds_RigidBodyPipeline *pipeline = phase->pipeline;

    const u32 ranges_per_color = 3*g_scheduler->worker_count;
    /* Serial color only has a single range */
    const u32 ranges_per_iteration = (CG_COLOR_COUNT-1)*ranges_per_color + 1;
    const u32 range_velocity_count = ranges_per_iteration * g_solver_config->pgs_iteration_count;
    
#include <sched.h>
    u32 spin_count = 0;
    u32 range = 0;
    u64 local_dependency = 0;
    u64 range_dependency = 0;
    while (1)
    {
    RETRY:
        if (range >= range_velocity_count)
        {
            break;
        }

        const u32 iteration = range / ranges_per_iteration;
        const u32 iteration_range = range % ranges_per_iteration;
        const u32 color_index = iteration_range / ranges_per_color;
        const u32 color_range = iteration_range % ranges_per_color;

        const struct ds_CGraphColor *color = pipeline->cgraph.color + color_index; 

        const u32 cc_per_range = color->contact_constraint_pool.count / ranges_per_color;
        const u32 cc_extra = color->contact_constraint_pool.count % ranges_per_color;
       
        const u32 cc_high = (color_range < cc_extra)
                        ? (color_range + 1)*cc_per_range + color_range 
                        : (color_range + 1)*cc_per_range + cc_extra;
        const u32 cc_low = (color_range < cc_extra)
                        ? cc_high - cc_per_range - 1
                        : cc_high - cc_per_range;
    
        range_dependency = ((u64) iteration << 32) + color_index;
        while (local_dependency < range_dependency)
        {
            local_dependency = AtomicLoadRlx64(&phase->a_range_dependency);
            if (local_dependency >= range_dependency)
            {
                break;
            }
            
            if (spin_count < 64)
            {
                __asm__ __volatile__("pause");
                spin_count += 1;
            }
            else
            {
                sched_yield();
                spin_count = 0;
                goto RETRY;
            }
        }

        if (AtomicCompareExchangeRlxRlx32(&phase->a_range_next, &range, range+1))
        {
            AtomicLoadAcq32(&phase->a_range_completed);
            ds_ContactConstraintColorIterate(pipeline, color_index, cc_low, cc_high); 
            AtomicAddFetchRel32(&phase->a_range_completed, 1);
            range += 1;

            const u32 new_iteration = range / ranges_per_iteration;
            const u32 new_iteration_range = range % ranges_per_iteration;
            const u32 new_color_index = new_iteration_range / ranges_per_color;
            const u64 new_dependency = ((u64) new_iteration << 32) | new_color_index;
            if (range_dependency < new_dependency)
            {
                AtomicCompareExchangeRlxRlx64(&phase->a_range_dependency, &range_dependency, new_dependency);
            }
        }
    }
    
    ProfZoneEnd;

    return U32_MAX;
}

static void SolveConstraints(struct ds_RigidBodyPipeline *pipeline) 
{
	ProfZone;

    ds_RigidBodyUpdateSolverDataAll(pipeline);
    ds_ContactConstraintInitAll(pipeline);

    if (g_solver_config->warmup_solver)
    {
        ds_ContactConstraintWarmupAll(pipeline);
    }

	//for (u32 i = 0; i < g_solver_config->pgs_iteration_count; ++i)
	//{
    //    for (u32 c = CG_STATIC_COLOR_FIRST; c <= CG_STATIC_COLOR_LAST; ++c)
    //    {
    //        ds_ContactConstraintColorIterate(pipeline, c);
    //    }

    //    for (u32 c = CG_DYNAMIC_COLOR_FIRST; c <= CG_DYNAMIC_COLOR_LAST; ++c)
    //    {
    //        ds_ContactConstraintColorIterate(pipeline, c);
    //    }

    //    ds_ContactConstraintColorIterate(pipeline, CG_SERIAL_COLOR);
	//}

    struct ds_SolverJobPhase *solver_phase = pipeline->solver_phase;
    {
    	ProfZoneNamed("JobPhase(Solve)");

        ds_JobPhaseBegin(&solver_phase->phase);

        f32 cc_parallel = 0;
        const f32 cc_serial = pipeline->cgraph.color[CG_SERIAL_COLOR].contact_constraint_pool.count;
        for (u32 color_index = 0; color_index < CG_SERIAL_COLOR; ++color_index)
        {
            cc_parallel += pipeline->cgraph.color[color_index].contact_constraint_pool.count;
        }

        fprintf(stderr, "Solver serial percentage: %f\n", 100.0f * cc_serial / (cc_serial+cc_parallel));

        solver_phase->pipeline = pipeline;
        AtomicStoreRel32(&solver_phase->a_range_next, 0);
        AtomicStoreRel32(&solver_phase->a_range_completed, 0);
        AtomicStoreRel64(&solver_phase->a_range_dependency, 0);

        solver_phase->job_count = g_scheduler->worker_count;
        solver_phase->job = ArenaPushZero(&pipeline->frame, solver_phase->job_count*sizeof(struct ds_SolverJob));
        ds_JobPhaseReserve(&solver_phase->phase, SOLVER_JOB_SEED, solver_phase->job_count);

        for (u32 i = 0; i < solver_phase->job_count; ++i)
        {
            ds_WSDequePushBottom(g_scheduler->seed_deque, ds_JobIdInit(SOLVER_JOB_SEED, i));
        }

        AtomicStoreRlx32(&g_scheduler->a_seeds_remaining, solver_phase->job_count);
        ds_JobPhaseAddFetchRemaining(&solver_phase->phase, solver_phase->job_count);
        ds_WSDequePublish(g_scheduler->seed_deque);
        for (u32 i = 1; i < g_scheduler->worker_count; ++i)
        {
            SemaphorePost(&g_scheduler->jobs_are_available);
        }

	    ds_MasterRunAvailableJobs();
        
        ds_JobPhaseEnd();

    	ProfZoneEnd;
    }

    ds_ContactConstraintCacheImpulse(pipeline);

    /* integrate final solver velocities and update bodies  */
    ds_RigidBodyIntegrateVelocitiesAll(pipeline);

    ds_PositionConstraintColorInitAll(pipeline); 

    for (u32 i = 0; i < g_solver_config->ngs_iteration_count; ++i)
	{
		//const u32 contacts_okay = SolverIteratePositionConstraints(solver);
        //if (contacts_okay)
        //{
        //    break;
        //}

        for (u32 c = CG_STATIC_COLOR_FIRST; c <= CG_STATIC_COLOR_LAST; ++c)
        {
            ds_PositionConstraintColorIterate(pipeline, c);
        }

        for (u32 c = CG_DYNAMIC_COLOR_FIRST; c <= CG_DYNAMIC_COLOR_LAST; ++c)
        {
            ds_PositionConstraintColorIterate(pipeline, c);
        }

        ds_PositionConstraintColorIterate(pipeline, CG_SERIAL_COLOR);
	}

    ds_RigidBodyUpdateOrientationAll(pipeline);
        
    struct ds_SolverSet *active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;
    ds_BitSetClear(&pipeline->island_high_energy_set, 0);
    pipeline->island_to_split = DS_ID_NULL; 
    f32 global_max_low_velocity_time = F32_MIN_NEGATIVE_NORMAL;
    for (u32 i = 0; i < active->island_pool.count; ++i)
    {
        const u32 isi = active->island_pool.buf[i];
        const struct ds_Island *island = pipeline->island_pool.buf + isi; 
		f32 min_low_velocity_time = F32_MAX_POSITIVE_NORMAL;

        struct ds_RigidBody *body;
        for (i32 bi = island->body_list.first; bi != DLL_SENTINEL; bi = body->island_body.next)
        {
            body = pipeline->body_pool.buf + bi;
            const struct ds_RigidBodyCompute *compute = active->body_compute_pool.buf + body->sim;
			const f32 lv_sq = Vec3Dot(compute->linear_velocity, compute->linear_velocity);
			const f32 av_sq = Vec3Dot(compute->angular_velocity, compute->angular_velocity);
			if (lv_sq <= g_solver_config->sleep_linear_velocity_sq_limit && av_sq <= g_solver_config->sleep_angular_velocity_sq_limit)
			{
				body->low_velocity_time += pipeline->timestep;
			}
            else
            {
                body->low_velocity_time = 0.0f;
            }
			min_low_velocity_time = f32_min(min_low_velocity_time, body->low_velocity_time);
        }

        /* integrate final solver velocities and update bodies and find lowest low_velocity time */
	    if (min_low_velocity_time < g_solver_config->sleep_time_threshold)
	    {
            ds_BitSetSet(&pipeline->island_high_energy_set, i, 1);
	    }
        else if (global_max_low_velocity_time < min_low_velocity_time && island->constraint_remove_count)
        {
            global_max_low_velocity_time = min_low_velocity_time;
            pipeline->island_to_split = island->id; 
        }
    }

    for (u64 block = 0; block < pipeline->island_high_energy_set.block_count; ++block)
	{
        const u64 sleep_block = ~pipeline->island_high_energy_set.bits[block];
        struct ds_BitBlock it = ds_BitBlockInit(sleep_block, block);
		while (ds_BitBlockHasNext(&it))
		{

            const u32 i = ds_BitBlockPeekNext(&it);
            if (i >= active->island_pool.count)
            {
                goto DONE;
            }

            u32 pop = 1;
            const u32 isi = active->island_pool.buf[i];
            const struct ds_Island *island = pipeline->island_pool.buf + isi;
            if (island->constraint_remove_count == 0)
            {
                if (i != active->island_pool.count-1 && ds_BitSetGet(&pipeline->island_high_energy_set, active->island_pool.count-1) == 0)
                {
                    pop = 0;
                }
                ds_SolverSetSleep(pipeline, isi);
	            PhysicsEventIslandAsleep(pipeline, isi);
                active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;
            }

            if (pop)
            {
                ds_BitBlockNext(&it);
            }
		}
	}
DONE:

	ProfZoneEnd;
}

void PhysicsPipelineSleepEnable(struct ds_RigidBodyPipeline *pipeline)
{
	ds_Assert(g_solver_config->sleep_enabled == 0);
	if (g_solver_config->sleep_enabled)
	{
        return;
    }

	g_solver_config->sleep_enabled = 1;
    for (u32 set_index = 0; set_index < pipeline->solver_set_pool.count_max; ++set_index)
    {
        struct ds_SolverSet *set = pipeline->solver_set_pool.buf + set_index;
        if (ds_PoolSlotAllocated(set))
        {
            for (u32 k = 0; k < set->island_pool.count; ++k)
            {
                const u32 island_index = set->island_pool.buf[k];
        	    struct ds_Island *is = pipeline->island_pool.buf + k;
            }

            if (set_index >= SOLVER_SET_SLEEPING_FIRST)
            {
                ds_SolverSetWakeUp(pipeline, set_index);
            }
        }
    }
}

void PhysicsPipelineSleepDisable(struct ds_RigidBodyPipeline *pipeline)
{
	ds_Assert(g_solver_config->sleep_enabled == 1);
	if (!g_solver_config->sleep_enabled)
    {
        return;
    }
	
	g_solver_config->sleep_enabled = 0;
    for (u32 set_index = 0; set_index < pipeline->solver_set_pool.count_max; ++set_index)
    {
        struct ds_SolverSet *set = pipeline->solver_set_pool.buf + set_index;
        if (ds_PoolSlotAllocated(set))
        {
            for (u32 k = 0; k < set->island_pool.count; ++k)
            {
                const u32 island_index = set->island_pool.buf[k];
	    	    struct ds_Island *is = pipeline->island_pool.buf + k;
            }

            if (set_index >= SOLVER_SET_SLEEPING_FIRST)
            {
                ds_SolverSetWakeUp(pipeline, set_index);
            }
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

void PhysicsPipelineSimulateFrame(struct ds_RigidBodyPipeline *pipeline)
{
    pipeline->timestep = (f32) pipeline->ns_tick / NSEC_PER_SEC;
	/* update, if possible, any pending values in contact solver config */
	UpdateSolverConfig(pipeline);

	/* broadphase => narrowphase => solve => integrate */
    CollisionDetection(pipeline);

	MergeIslands(pipeline);
	SplitIslandsAndRemoveContacts(pipeline);
	SolveConstraints(pipeline);

	PHYSICS_PIPELINE_VALIDATE(pipeline);
}

void PhysicsPipelineTick(struct ds_RigidBodyPipeline *pipeline)
{
	ProfZone;

    ds_NumericsConfigPush(&pipeline->numerics_config);

	if (pipeline->frames_completed > 0)
	{
		PhysicsPipelineClearFrame(pipeline);
	}
	pipeline->frames_completed += 1;
	PhysicsPipelineSimulateFrame(pipeline);

    ds_NumericsConfigPop();

	ProfZoneEnd;
}

u32f32 PhysicsPipelineRaycastParameter(struct arena *mem_tmp1, struct arena *mem_tmp2, const struct ds_RigidBodyPipeline *pipeline, const struct ray *ray)
{
	ArenaPushRecord(mem_tmp1);

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
			const struct ds_Shape *shape = pipeline->shape_pool.buf + si;
			const f32 t = ds_ShapeRaycastParameter(pipeline, shape, ray);
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

	return info.hit;
}

struct ds_PhysicsEvent *ds_PhysicsEventPush(struct ds_RigidBodyPipeline *pipeline)
{
	struct slot slot = ds_PhysicsEventPoolAdd(&pipeline->event_pool);
    ds_DLLAppend(pipeline->event_list, pipeline->event_pool.buf, slot.index, node);
	struct ds_PhysicsEvent *event = slot.address;
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
    fprintf(stderr, "\tislands:                     %u\n", pipeline->island_pool.count);
    fprintf(stderr, "\tcontacts:                    %u\n", pipeline->cdb->contact_pool.count);
    fprintf(stderr, "\tsat caches (max):            %u\n", AtomicLoadRlx32(&pipeline->cdb->sat_cache_pool.a_count_max));
    fprintf(stderr, "\tcontact bitvector size:      %lu\n", (long unsigned) pipeline->cdb->contact_persistent_usage.block_count*sizeof(u64));
    fprintf(stderr, "\tsat cache bitvector size:    %lu\n", (long unsigned) pipeline->cdb->sat_cache_persistent_usage.block_count*sizeof(u64));
}
