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
	pipeline.dynamic_bvh = DbvhAlloc(NULL, 2*initial_size, GROWABLE);
	pipeline.static_bvh = DbvhAlloc(NULL, 2*initial_size, GROWABLE);
    pipeline.dirty_shape_set = ds_BitSetAlloc(NULL, initial_size, 0, GROWABLE);
    ds_CPoolAlloc(NULL, pipeline.dirty_shape_query, initial_size, GROWABLE);

	pipeline.event_pool = ds_PhysicsEventPoolAlloc(NULL, 256, GROWABLE);
	ds_DLLFlush(&pipeline.event_list);

	pipeline.cshape_db = cshape_db;

    pipeline.contact_pool = ds_ContactPoolAlloc(NULL, initial_size, GROWABLE);
    pipeline.contact_map = ds_HashMapAlloc(NULL, initial_size, initial_size, GROWABLE);
	pipeline.contact_persistent_usage = ds_BitSetAlloc(NULL, initial_size, 0, GROWABLE);

	pipeline.island_pool = ds_IslandPoolAlloc(NULL, initial_size, GROWABLE);
    pipeline.island_high_energy_set = ds_BitSetAlloc(NULL, initial_size, 0, GROWABLE);

    pipeline.margin_on = 0;
	pipeline.margin = COLLISION_DEFAULT_MARGIN;

	pipeline.debug_count = 0;
	pipeline.debug = NULL;

    pipeline.broad_phase = ArenaPushAligned(mem, sizeof(struct ds_BroadJobPhase), DS_CACHE_LINE);
    pipeline.narrow_phase = ArenaPushAligned(mem, sizeof(struct ds_NarrowJobPhase), DS_CACHE_LINE);
    pipeline.solver_phase = ArenaPushAligned(mem, sizeof(struct ds_SolverJobPhase), DS_CACHE_LINE);
    ds_JobPhaseAlloc(mem, &pipeline.broad_phase->phase, BROAD_JOB_COUNT, ds_BroadJobPhaseDispatch);
    ds_JobPhaseAlloc(mem, &pipeline.narrow_phase->phase, NARROW_JOB_COUNT, ds_NarrowJobPhaseDispatch);
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

    ds_BitSetDealloc(&pipeline->dirty_shape_set);
    ds_CPoolDealloc(pipeline->dirty_shape_query);
	BvhFree(&pipeline->dynamic_bvh);
	BvhFree(&pipeline->static_bvh);
    ds_ContactPoolDealloc(&pipeline->contact_pool);
	ds_BitSetDealloc(&pipeline->contact_persistent_usage);
    ds_HashMapDealloc(&pipeline->contact_map);
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

	pipeline->contact_frame_usage.bits = NULL;
	pipeline->contact_frame_usage.bit_count = 0;
	pipeline->contact_frame_usage.block_count = 0;	
    pipeline->contact_count = 0;
    pipeline->contact_new_count = 0;

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

	ds_BitSetClear(&pipeline->contact_persistent_usage, 0);
    ds_ContactPoolFlush(&pipeline->contact_pool);
    ds_HashMapFlush(&pipeline->contact_map);
	ds_IslandPoolFlush(&pipeline->island_pool);
	
	ds_RigidBodyPoolFlush(&pipeline->body_pool);
    ds_BitSetClear(&pipeline->body_usage_set, 0);

    ds_BitSetClear(&pipeline->dirty_shape_set, 0);
    ds_CPoolFlush(pipeline->dirty_shape_query);
	DbvhFlush(&pipeline->dynamic_bvh);
	DbvhFlush(&pipeline->static_bvh);
	ds_ShapePoolFlush(&pipeline->shape_pool);

	ds_PhysicsEventPoolFlush(&pipeline->event_pool);
	ds_DLLFlush(&pipeline->event_list);

    ds_JointPoolFlush(&pipeline->joint_pool);
    ds_CGraphFlush(pipeline);

	ArenaFlush(&pipeline->frame);
	pipeline->frames_completed = 0;
	pipeline->ns_elapsed = 0;
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
	ds_ContactValidateAll(pipeline);
	ds_IslandValidateAll(pipeline);

	ProfZoneEnd;
}

u32 ds_BroadJobPhaseDispatch(const ds_JobId job)
{
    struct arena *frame = g_tl_self->frame;
    struct ds_BroadJobPhase *phase = (struct ds_BroadJobPhase *) g_scheduler->phase;
    struct ds_RigidBodyPipeline *pipeline = phase->pipeline;
    struct ds_ParallelForChain *chain = &phase->pf;
    struct ds_ParallelFor *pf = chain->parallel_for + 0;
    struct ds_BitSet *dirty = &pipeline->dirty_shape_set;
    struct ds_ProxyQuery *query = pipeline->dirty_shape_query.buf;
    u32 low, high;

    ds_ParallelFor(pf, range_index)
    {
        ds_ParallelForRange(&low, &high, pf, range_index);
        for (u64 block = low; block < high; ++block)
        {
            struct ds_BitBlock it = ds_BitBlockInit(dirty->bits[block], block);
		    while (ds_BitBlockHasNext(&it))
		    {
                const u32 si = ds_BitBlockNext(&it);
                const struct ds_Shape *shape = pipeline->shape_pool.buf + si;
                const struct bvhNode *node = (const struct bvhNode *) pipeline->dynamic_bvh.tree.pool.buf + shape->proxy;
                const struct bvh_QuerySet dynamic_query = BvhQueryAndFilterOnBody(frame, &pipeline->dynamic_bvh, node);
                const struct bvh_QuerySet static_query = BvhQuery(frame, &pipeline->static_bvh, node);

                query[si].dynamic_count = dynamic_query.count;
                query[si].dynamic_query = dynamic_query.query;
                query[si].static_count = static_query.count;
                query[si].static_query = static_query.query;
		    }
        }
    }

    return U32_MAX;
}

u32 ds_NarrowJobPhaseDispatch(const ds_JobId job)
{
    struct arena *frame = g_tl_self->frame;
    struct ds_NarrowJobPhase *phase = (struct ds_NarrowJobPhase *) g_scheduler->phase;
    struct ds_RigidBodyPipeline *pipeline = phase->pipeline;
    struct ds_ParallelForChain *chain;
    struct ds_ParallelFor *pf;
    u32 low, high;

    for (u32 i = 0; i < CG_COLOR_COUNT; ++i)
    {
        ProfZoneNamed("Color");
        struct ds_CGraphColor *color = pipeline->cgraph.color + i;
        chain = phase->pf + i;
        {
            pf = chain->parallel_for + 0;
            ds_ParallelFor(pf, range_index)
            {
                ds_ParallelForRange(&low, &high, pf, range_index);
                for (u32 si = low; si < high; ++si)
                {
                    const u32 ci = color->contact_pool.buf[si];
                    ds_ShapeContact(frame, pipeline, ci);
                }
            }
        }

        ProfZoneEnd;
    }

    chain = phase->pf + CG_COLOR_COUNT;
    {
        struct ds_SolverSet *active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;
        pf = chain->parallel_for + 0;
        ds_ParallelFor(pf, range_index)
        {
            ds_ParallelForRange(&low, &high, pf, range_index);
            for (u32 si = low; si < high; ++si)
            {
                const u32 ci = active->contact_pool.buf[si];
                ds_ShapeContact(frame, pipeline, ci);
            }
        }
    }

    return U32_MAX;
}

static void CollisionDetection(struct ds_RigidBodyPipeline *pipeline)
{
    /*
     * Achieving Determinism and Parallelization in the BroadPhase
     * ===========================================================
     *
     * (0) Assume, at this point, that we have a compact buffer of all dirty proxies. A proxy
     *     is dirty if it was newly inserted, enlarged or moved (re-inserted). As this buffer
     *     is added to according to what work is thread did in the solver phase, it is out-of
     *     -order.
     *
     * (1) In order to get determinism without sorting, we store, separate to the dynamic tree
     *     a compact pool with the number of tree leaves.
     *
     *          dirty_shape_results[] 
     *
     * (2) the parallel broadphase becomes a parallel-for in which we query each dirty proxy 
     *     against the dynamic tree, reporting all overlaps with shapes not attached to its 
     *     body, or any moving bodies with a lower index. If a thread process proxy p, it
     *     writes the result to
     *
     *          dirty_shape_results[p]    
     *
     * (3) The master thread may now process the results in index order; this can of course be
     *     done efficiently with the use of bit-sets. 
     *
     *
     * Preliminary Documentation 
     * =========================
     *
     * Any time a proxy is created, re-inserted or enlarged, its bit in dirty_shape_set is set.
     * All set bits are then processed in parallel in the broadphase; the thread processing 
     * dirty bit B writes the proxy's query set (stored locally in the thread's frame arena)
     * to dirty_query.buf[B]: 
     *
     *      dirty_query.buf[B].count = proxy.overlap_count;
     *      dirty_query.buf[B].query = proxy.overlap;       (frame-arena backed)
     *
     * After the broadphase, the master thread process all written queries, processing each
     * query in each query set, from the lower to higher index. 
     *
     *
     *
     * () TODO: how to we locate removed contacts?
     */

    // TODO Remove 
	//struct dbvhOverlap *proxy_overlap = NULL;
	//u32 proxy_overlap_count = 0;
    //{
    //	proxy_overlap = DbvhPushOverlapPairs(&pipeline->frame, &proxy_overlap_count, &pipeline->dynamic_bvh);
    //}

    //for (u32 oi = 0; oi < proxy_overlap_count; ++oi)
    //{
    //    const struct ds_ContactKey key = ds_ContactKeyCanonical(proxy_overlap[oi].id1, proxy_overlap[oi].id2);
    //    const struct slot slot = ds_ContactKeyLookup(pipeline, key);
    //    if (!slot.address)
    //    {
    //        ds_ContactAdd(pipeline, key);
    //    }
    //}

    struct ds_BroadJobPhase *broad_phase = pipeline->broad_phase;
    {
    	ProfZoneNamed("JobPhase(Broadphase)");

        ds_JobPhaseBegin(&broad_phase->phase);

        broad_phase->pf = ds_ParallelForChainAlloc(&pipeline->frame, 1); 
        
        //TODO: U32_MAX => moved count
        //TODO: this range size is random hardcoded value, change
        ds_ParallelForInit(broad_phase->pf.parallel_for, U32_MAX, 8);

        broad_phase->pipeline = pipeline;
        broad_phase->job_count = g_scheduler->worker_count;
        broad_phase->job = ArenaPushZero(&pipeline->frame, broad_phase->job_count*sizeof(struct ds_BroadJob));
        ds_JobPhaseReserve(&broad_phase->phase, BROAD_JOB_SEED, broad_phase->job_count);

        for (u32 i = 0; i < broad_phase->job_count; ++i)
        {
            ds_WSDequePushBottom(g_scheduler->seed_deque, ds_JobIdInit(BROAD_JOB_SEED, i));
        }

        AtomicStoreRlx32(&g_scheduler->a_seeds_remaining, broad_phase->job_count);
        ds_JobPhaseAddFetchRemaining(&broad_phase->phase, broad_phase->job_count);
        ds_WSDequePublish(g_scheduler->seed_deque);
        for (u32 i = 1; i < g_scheduler->worker_count; ++i)
        {
            SemaphorePost(&g_scheduler->jobs_are_available);
        }

	    ds_MasterRunAvailableJobs();
        
        ds_JobPhaseEnd();

        ProfZoneEnd;
    }

    struct ds_NarrowJobPhase *narrow_phase = pipeline->narrow_phase;
    {
    	ProfZoneNamed("JobPhase(Narrowphase)");

        ds_JobPhaseBegin(&narrow_phase->phase);

        for (u32 i = 0; i < CG_COLOR_COUNT + 1; ++i)
        {
            narrow_phase->pf[i] = ds_ParallelForChainAlloc(&pipeline->frame, 1); 
        }

        struct ds_ParallelFor *pf;
        for (u32 i = 0; i < CG_COLOR_COUNT; ++i)
        {
            const struct ds_CGraphColor *color = pipeline->cgraph.color + i;
            pf = narrow_phase->pf[i].parallel_for;
            //TODO: this range size is random hardcoded value, change
            ds_ParallelForInit(pf + 0, color->contact_pool.count, 8);
        }

        pf = narrow_phase->pf[CG_COLOR_COUNT].parallel_for;
        //TODO: this range size is random hardcoded value, change
        ds_ParallelForInit(pf + 0, pipeline->solver_set_pool.buf[SOLVER_SET_ACTIVE].contact_pool.count, 8);

        narrow_phase->pipeline = pipeline;
        narrow_phase->job_count = g_scheduler->worker_count;
        narrow_phase->job = ArenaPushZero(&pipeline->frame, narrow_phase->job_count*sizeof(struct ds_NarrowJob));
        ds_JobPhaseReserve(&narrow_phase->phase, NARROW_JOB_SEED, narrow_phase->job_count);

        for (u32 i = 0; i < narrow_phase->job_count; ++i)
        {
            ds_WSDequePushBottom(g_scheduler->seed_deque, ds_JobIdInit(NARROW_JOB_SEED, i));
        }

        AtomicStoreRlx32(&g_scheduler->a_seeds_remaining, narrow_phase->job_count);
        ds_JobPhaseAddFetchRemaining(&narrow_phase->phase, narrow_phase->job_count);
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

        /*
            (0) We need to promote any non-touching contacts (SOLVER_SET_ACTIVE) to touching 
            contacts (CG_COLOR_***) if the contact was found to be touching. Similarly we need to
            demote touching contacts to non-touching if we found it to be separating.

            (1) We need to move contacts between the active set and the constraint graph 
            deterministically, so we may as well use the deterministic ordering of contacts we
            established after the broadphase.
         */

	//    cdb->sat_cache_frame_usage = ds_BitSetAlloc(&pipeline->frame, cdb->sat_cache_persistent_usage.bit_count, 0, 0);
	//    cdb->contact_frame_usage = ds_BitSetAlloc(&pipeline->frame, cdb->contact_persistent_usage.bit_count, 0, 0);

    //    const u32 narrowphase_count = AtomicLoadRlx32(&cd_jobs->phase.next[COLLISION_JOB_NARROWPHASE].a_counter);
    //    struct memArray arr = ArenaPushAlignedAll(&pipeline->frame, sizeof(u32), sizeof(u32));
    //    cdb->contact_new = arr.addr;
    //    //fprintf(stderr, "A: {");
    //    for (u32 i = 0; i < narrowphase_count; ++i)
    //    {
    //        const struct ds_NarrowPhaseJob *job = cd_jobs->narrowphase_jobs + i;
    //        if (!job->valid)
    //        {
    //            continue;
    //        }

    //        if (job->cache)
    //        {
    //            cdb->sat_cache_count += 1;
    //            if (job->cache_index < cdb->sat_cache_persistent_usage.bit_count)
    //            {
    //                ds_BitSetSet(&cdb->sat_cache_frame_usage, job->cache_index, 1);   
    //            } 
    //        }

    //        for (u32 c = 0; c < job->collision_count; ++c)
    //        {
    //            cdb->contact_count += 1;
    //            struct slot slot = ds_ContactKeyLookup(pipeline, job->key + c);
    //            if (!slot.address)
    //            {
    //                slot = ds_ContactAdd(pipeline, job->manifold + c, job->key + c);
    //            }
    //            else
    //            {
    //                ds_ContactUpdate(pipeline, slot, job->manifold + c);
    //            }
    //             
	//		    /* add to new links if needed */
	//		    if (slot.index >= cdb->contact_persistent_usage.bit_count
	//		    	 || ds_BitSetGet(&cdb->contact_persistent_usage, slot.index) == 0)
	//		    {
    //                    if (cdb->contact_new_count >= arr.len)
    //                    {
    //                        LogString(T_PHYSICS, S_FATAL, "Frame arena OOM in Broadphase, increase size!");
    //                        FatalCleanupAndExit();
    //                    }
    //                    cdb->contact_new[ cdb->contact_new_count ] = slot.index;
	//		    		cdb->contact_new_count += 1;
	//		    }
	//		    //fprintf(stderr, " %u", index);
    //        }
    //    }
    //    //fprintf(stderr, " } ");
    //    ArenaPopPacked(&pipeline->frame, sizeof(u32)*(arr.len - cdb->contact_new_count));

    //    /* Remove stale sat_Caches */
	//    for (u64 block = 0; block < cdb->sat_cache_frame_usage.block_count; ++block)
	//    {
	//    	const u64 broken_link_block = 
	//    			    cdb->sat_cache_persistent_usage.bits[block]
	//    			& (~cdb->sat_cache_frame_usage.bits[block]);
    //        struct ds_BitBlock it = ds_BitBlockInit(broken_link_block, block);
	//    	while (ds_BitBlockHasNext(&it))
	//    	{
	//    	    sat_CacheRemove(cdb, ds_BitBlockNext(&it));
	//    	}
	//    }	
    //
    //    /* Update sat_cache_persistent_usage */
    //    for (u64 i = 0; i < cdb->sat_cache_frame_usage.block_count; ++i)
    //    {
    //    	cdb->sat_cache_persistent_usage.bits[i] = cdb->sat_cache_frame_usage.bits[i];	
    //    }

    //    const u32 count_max = AtomicLoadRlx32(&cdb->sat_cache_pool.a_count_max);
    //    const u32 length = AtomicLoadRlx32(&cdb->sat_cache_pool.a_length);
    //    if (cdb->sat_cache_persistent_usage.bit_count < count_max)
    //    {
    //    	const u64 low_bit = cdb->sat_cache_persistent_usage.bit_count;
    //    	const u64 high_bit = count_max;
    //    	ds_BitSetIncreaseSize(&cdb->sat_cache_persistent_usage, length, 0);
    //    	/* any new sat_caches that is in the appended region must now be set */
    //    	for (u64 bit = low_bit; bit < high_bit; ++bit)
    //    	{
    //    		ds_BitSetSet(&cdb->sat_cache_persistent_usage, bit, 1);
    //    	}
    //    }

    	ProfZoneEnd;
    }
}

static void MergeIslands(struct ds_RigidBodyPipeline *pipeline)
{
	ProfZone;
	for (u32 i = 0; i < pipeline->contact_new_count; ++i)
	{
		struct ds_Contact *c = pipeline->contact_pool.buf + pipeline->contact_new[i];

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

        const u32 dynamic[2] = { RB_IS_DYNAMIC(body[0]), RB_IS_DYNAMIC(body[1]) };
        if (dynamic[0] && dynamic[1])
        {
			ds_IslandMerge(pipeline, body[0]->island, body[1]->island, pipeline->contact_new[i]);
        }
        else
        {
            ds_Assert(dynamic[0] || dynamic[1]);
            c->island = body[ dynamic[1] ]->island;
			struct ds_Island *island = pipeline->island_pool.buf + c->island;
			ds_DLLAppend(island->contact_list, pipeline->contact_pool.buf, pipeline->contact_new[i], island_contact);
            /*
             * TODO: is this even relevant anymore? 
             *
             * TODO: This feels bad and dangerous; we've found a new contact of the island
             * which is in the Constraint Graph while the rest of the island's contacts are
             * in the sleeper set; it should be fine to wake up the set and move all 
             * sleeping constraints to the Constraint Graph without messing up links, but
             * it becomes very nasty to reason about
             */
            if (island->set >= SOLVER_SET_SLEEPING_FIRST)
            {
                ds_SolverSetWakeUp(pipeline, island->set);
	            PhysicsEventIslandAwake(pipeline, island->id);	
            }
	        PhysicsEventIslandExpanded(pipeline, island->id);
        }
	}
	ProfZoneEnd;
}

static void SplitIslandsAndRemoveContacts(struct ds_RigidBodyPipeline *pipeline)
{
	ProfZone;

	//fprintf(stderr, " R: {");
	for (u64 block = 0; block < pipeline->contact_frame_usage.block_count; ++block)
	{
		const u64 broken_link_block = 
				    pipeline->contact_persistent_usage.bits[block]
				& (~pipeline->contact_frame_usage.bits[block]);

        struct ds_BitBlock it = ds_BitBlockInit(broken_link_block, block);
	    while (ds_BitBlockHasNext(&it))
	    {
            const u64 ci = ds_BitBlockNext(&it);
			struct ds_Contact *c = pipeline->contact_pool.buf + ci;
			//fprintf(stderr, " %lu", ci);

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
			ds_Assert(RB_IS_DYNAMIC(body[0]) || RB_IS_DYNAMIC(body[1]));

			if (body[0]->set != SOLVER_SET_STATIC && body[1]->set != SOLVER_SET_STATIC)
			{
			    struct ds_Island *is = pipeline->island_pool.buf + body[0]->island;
                is->constraint_remove_count += 1;
			}

			ds_ContactRemove(pipeline, ci);
		}
	}	

    /* Update contact_persistent_usage */
    {
        for (u64 i = 0; i < pipeline->contact_frame_usage.block_count; ++i)
        {
        	pipeline->contact_persistent_usage.bits[i] = pipeline->contact_frame_usage.bits[i];	
        }

        if (pipeline->contact_persistent_usage.bit_count < pipeline->contact_pool.count_max)
        {
        	const u64 low_bit = pipeline->contact_persistent_usage.bit_count;
        	const u64 high_bit = pipeline->contact_pool.count_max;
        	ds_BitSetIncreaseSize(&pipeline->contact_persistent_usage, pipeline->contact_pool.length, 0);
        	/* any new contacts that is in the appended region must now be set */
        	for (u64 bit = low_bit; bit < high_bit; ++bit)
        	{
        		ds_BitSetSet(&pipeline->contact_persistent_usage, bit, 1);
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
    struct ds_ParallelForChain *chain;
    struct ds_ParallelFor *pf;
    u32 low, high;

    chain = &phase->pf_body_update;
    {
        ProfZoneNamed("Body Update");
        pf = chain->parallel_for + 0;
        ds_ParallelFor(pf, range_index)
        {
            ds_ParallelForRange(&low, &high, pf, range_index);
            ds_RigidBodyUpdateSolverDataRange(pipeline, low, high);

        }
        ProfZoneEnd;
    }
    ds_ParallelForChainWait(chain);

    chain = &phase->pf_contact_init;
    {
        ProfZoneNamed("Constraint Initialization");
        for (u32 ci = 0; ci < CG_COLOR_COUNT; ++ci)
        {
            pf = chain->parallel_for + ci;
            ds_ParallelFor(pf, range_index)
            {
                ds_ParallelForRange(&low, &high, pf, range_index);
                ds_ContactConstraintInitRange(pipeline, ci, low, high);
            }
        }
        ProfZoneEnd;
    }
    ds_ParallelForChainWait(chain);

    if (g_solver_config->warmup_solver)
    {
        chain = &phase->pf_contact_warmup;
        {
            ProfZoneNamed("Constraint Warmup");
            for (u32 ci = 0; ci < CG_COLOR_COUNT; ++ci)
            {
                pf = chain->parallel_for + ci;
                ds_ParallelFor(pf, range_index)
                {
                    ds_ParallelForRange(&low, &high, pf, range_index);
                    ds_ContactConstraintWarmupRange(pipeline, ci, low, high);
                }
            }
            ProfZoneEnd;
        }
        ds_ParallelForChainWait(chain);
    }

    chain = &phase->pf_velocity_solve;
    {
        ProfZoneNamed("Velocity Solve")
        for (u32 i = 0; i < g_solver_config->pgs_iteration_count; ++i)
        {
            ProfZoneNamed("Iteration");
            for (u32 ci = 0; ci < CG_COLOR_COUNT; ++ci)
            {
                ProfZoneNamed("Color");

                struct ds_CGraphColor *color = pipeline->cgraph.color + ci;
                const u32 pfi = i*CG_COLOR_COUNT + ci;
                pf = chain->parallel_for + pfi;
                ds_ParallelFor(pf, range_index)
                {
                    ds_ParallelForRange(&low, &high, pf, range_index);
                    ds_ContactConstraintIterateRange(pipeline, ci, low, high); 
                }
                ProfZoneEnd;
            }
            ProfZoneEnd;
        }
        ProfZoneEnd;
    }
    ds_ParallelForChainWait(chain);

    chain = &phase->pf_integrate;
    {
        ProfZoneNamed("Velocity Integrate");
        pf = chain->parallel_for + 0;
        ds_ParallelFor(pf, range_index)
        {
            ds_ParallelForRange(&low, &high, pf, range_index);
            ds_RigidBodyIntegrateVelocitiesRange(pipeline, low, high);

        }
        ProfZoneEnd;
    }
    ds_ParallelForChainWait(chain);

    chain = &phase->pf_cache_impulse_and_position_init;
    {
        ProfZoneNamed("Cache Impulses and Initalize Position Constraints");
        for (u32 ci = 0; ci < CG_COLOR_COUNT; ++ci)
        {
            pf = chain->parallel_for + ci;
            ds_ParallelFor(pf, range_index)
            {
                ds_ParallelForRange(&low, &high, pf, range_index);
                ds_PositionConstraintInitAndCacheImpulsesRange(pipeline, ci, low, high);
            }
        }
        ProfZoneEnd;
    }
    ds_ParallelForChainWait(chain);

    chain = &phase->pf_position_solve;
    {
        ProfZoneNamed("Position Solve")
        for (u32 i = 0; i < g_solver_config->ngs_iteration_count; ++i)
        {
            ProfZoneNamed("Iteration");
            for (u32 ci = 0; ci < CG_COLOR_COUNT; ++ci)
            {
                ProfZoneNamed("Color");

                struct ds_CGraphColor *color = pipeline->cgraph.color + ci;
                const u32 pfi = i*CG_COLOR_COUNT + ci;
                pf = chain->parallel_for + pfi;
                ds_ParallelFor(pf, range_index)
                {
                    ds_ParallelForRange(&low, &high, pf, range_index);
                    ds_PositionConstraintIterateRange(pipeline, ci, low, high); 
                }
                ProfZoneEnd;
            }
            ProfZoneEnd;
        }
        ProfZoneEnd;
    }
    ds_ParallelForChainWait(chain);

    chain = &phase->pf_orientation;
    {
        ProfZoneNamed("Position Update");
        pf = chain->parallel_for + 0;
        ds_ParallelFor(pf, range_index)
        {
            struct ds_ProxyRange *proxy_range = phase->proxy_range + range_index;
            ds_ParallelForRange(&low, &high, pf, range_index);
            ds_RigidBodyUpdateOrientationRange(pipeline, proxy_range, low, high);
        }
        ProfZoneEnd;
    }
    ds_ParallelForChainWait(chain);

    ProfZoneEnd;

    return U32_MAX;
}

static void SolveConstraints(struct ds_RigidBodyPipeline *pipeline) 
{
    struct ds_SolverJobPhase *solver_phase = pipeline->solver_phase;
    {
    	ProfZoneNamed("JobPhase(Solve)");

        ds_JobPhaseBegin(&solver_phase->phase);

        solver_phase->pf_body_update = ds_ParallelForChainAlloc(&pipeline->frame, 1);
        solver_phase->pf_contact_init = ds_ParallelForChainAlloc(&pipeline->frame, CG_COLOR_COUNT);
        solver_phase->pf_contact_warmup = ds_ParallelForChainAlloc(&pipeline->frame, CG_COLOR_COUNT);
        solver_phase->pf_velocity_solve = ds_ParallelForChainAlloc(&pipeline->frame, g_solver_config->pgs_iteration_count*CG_COLOR_COUNT);
        solver_phase->pf_integrate = ds_ParallelForChainAlloc(&pipeline->frame, 1);
        solver_phase->pf_cache_impulse_and_position_init = ds_ParallelForChainAlloc(&pipeline->frame, CG_COLOR_COUNT);
        solver_phase->pf_position_solve = ds_ParallelForChainAlloc(&pipeline->frame, g_solver_config->ngs_iteration_count*CG_COLOR_COUNT);
        solver_phase->pf_orientation = ds_ParallelForChainAlloc(&pipeline->frame, 1);

        struct ds_ParallelFor *pfb = solver_phase->pf_body_update.parallel_for;
        struct ds_ParallelFor *pfin = solver_phase->pf_integrate.parallel_for;
        struct ds_ParallelFor *pfo = solver_phase->pf_orientation.parallel_for;
        //TODO: this range size is random hardcoded value, change
        ds_ParallelForInit(pfb + 0, pipeline->solver_set_pool.buf[SOLVER_SET_ACTIVE].body_compute_pool.count, 32);
        ds_ParallelForInit(pfin + 0, pipeline->solver_set_pool.buf[SOLVER_SET_ACTIVE].body_compute_pool.count, 32);
        ds_ParallelForInit(pfo + 0, pipeline->solver_set_pool.buf[SOLVER_SET_ACTIVE].body_compute_pool.count, 32);
        solver_phase->proxy_range = ArenaPush(&pipeline->frame, pfo->range_count*sizeof(struct ds_ProxyRange));

        struct ds_ParallelFor *pfci = solver_phase->pf_contact_init.parallel_for;
        struct ds_ParallelFor *pfcw = solver_phase->pf_contact_warmup.parallel_for;
        struct ds_ParallelFor *pfvs = solver_phase->pf_velocity_solve.parallel_for;
        struct ds_ParallelFor *pfcp = solver_phase->pf_cache_impulse_and_position_init.parallel_for;
        struct ds_ParallelFor *pfps = solver_phase->pf_position_solve.parallel_for;
        for (u32 ci = 0; ci < CG_COLOR_COUNT; ++ci)
        {
            struct ds_CGraphColor *color = pipeline->cgraph.color + ci;
            const u32 cc_count = color->contact_pool.count;
            //TODO: this range size is random hardcoded value, change
            u32 cc_per_range = (ci == CG_SERIAL_COLOR)
                ? cc_count + 1
                : 8;

            ds_ParallelForInit(pfci + ci, cc_count, cc_per_range);
            ds_ParallelForInit(pfcw + ci, cc_count, cc_per_range);
            ds_ParallelForInit(pfcp + ci, cc_count, cc_per_range);

            //const f32 d = (f32) cc_count / (cc_per_range * g_scheduler->worker_count);
            //if (ci == 0)
            //  fprintf(stderr, "range distribution: \n {");

            //(ci+1 < CG_COLOR_COUNT)
            //    ? fprintf(stderr, " %f,", d)
            //    : fprintf(stderr, " %f }\n", d);

            for (u32 pgsi = 0; pgsi < g_solver_config->pgs_iteration_count; ++pgsi)
            {
                const u32 pfi = pgsi*CG_COLOR_COUNT + ci;
                ds_ParallelForInit(pfvs + pfi, cc_count, cc_per_range);
            }

            for (u32 pgsi = 0; pgsi < g_solver_config->ngs_iteration_count; ++pgsi)
            {
                const u32 pfi = pgsi*CG_COLOR_COUNT + ci;
                ds_ParallelForInit(pfps + pfi, cc_count, cc_per_range);
            }
        }

        solver_phase->pipeline = pipeline;
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

    {
        struct ds_ParallelFor *pf = solver_phase->pf_orientation.parallel_for + 0;
        f32 dirty_count = 0; 
        f32 reinsert_count = 0;
        for (u32 ri = 0; ri < pf->range_count; ++ri)
        {
            const struct ds_ProxyRange *range = solver_phase->proxy_range + ri;
            dirty_count += range->count;
            reinsert_count += range->reinsert_count;
        }

        if (dirty_count)
        {
            const f32 reinsert_fraction = reinsert_count / dirty_count;
            if (reinsert_fraction > g_numerics_config->dbvh_reinsert_threshold)
            {
                ProfZoneNamed("DBVH Rebuild");
                ds_Assert(0);
                ProfZoneEnd;
            }
            else
            {
                ProfZoneNamed("DBVH Update");
                for (u32 ri = 0; ri < pf->range_count; ++ri)
                {
                    const struct ds_ProxyRange *range = solver_phase->proxy_range + ri;
                    for (u32 pi = 0; pi < range->count; ++pi)
                    {
                        const struct ds_ProxyDirty *dirty = range->proxy + pi;
                        ds_BitSetSet(&pipeline->dirty_shape_set, dirty->shape, 1);
                        if (dirty->reinsert)
                        {
                            ProfZoneNamed("Reinsert");
                            struct ds_Shape *shape = pipeline->shape_pool.buf + dirty->shape;
                    	    DbvhRemove(&pipeline->dynamic_bvh, shape->proxy);
                    	    shape->proxy = DbvhInsert(&pipeline->dynamic_bvh, shape->body, dirty->shape, &dirty->bbox);
                            ProfZoneEnd;
                        }
                    }
                }
                ProfZoneEnd;
            }
        }
    }
        
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
	            PhysicsEventIslandAsleep(pipeline, island->id);
                active = pipeline->solver_set_pool.buf + SOLVER_SET_ACTIVE;
            }

            if (pop)
            {
                ds_BitBlockNext(&it);
            }
		}
	}
DONE:
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

	struct bvhRaycastInfo info = BvhRaycastInit(mem_tmp1, &pipeline->dynamic_bvh, ray);
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
    fprintf(stderr, "\tdynamic_bvh nodes:             %u\n", pipeline->dynamic_bvh.tree.pool.count);
    fprintf(stderr, "\tevents:                      %u\n", pipeline->event_pool.count);
    fprintf(stderr, "\tislands:                     %u\n", pipeline->island_pool.count);
    fprintf(stderr, "\tcontacts:                    %u\n", pipeline->contact_pool.count);
    fprintf(stderr, "\tcontact bitvector size:      %lu\n", (long unsigned) pipeline->contact_persistent_usage.block_count*sizeof(u64));
}
