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

#include "ds_collision.h"


//TODO can play around with these
#define COST_TRAVERSAL  1.0f	/* Overhead of internal node traversal (AABB testing of children) */
#define COST_INTERNAL 	1.5f	/* Overhead of triangle intersection tests */
#define COST_QUEUE_INITIAL_COUNT 	64


struct BVH DBVHAlloc(struct arena *mem, const u32 initial_length, const u32 growable)
{
	ds_Assert(!mem || !growable);
	struct BVH bvh =
	{
        .leaf_set = ds_BitSetAlloc(mem, initial_length, 0, growable),
        .internal_set = ds_BitSetAlloc(mem, initial_length, 0, growable),
		.cost_queue = MQAAlloc(NULL, COST_QUEUE_INITIAL_COUNT, growable),
		.heap_allocated = !mem,
	};
	ds_PoolAlloc(mem, bvh.pool, initial_length, growable);

    BTFlush(bvh.bt);

	return bvh;
}

void BVHFree(struct BVH *bvh)
{
    ds_BitSetDealloc(&bvh->internal_set);
    ds_BitSetDealloc(&bvh->leaf_set);
    ds_PoolDealloc(bvh->pool);
	MQADealloc(&bvh->cost_queue);
}

u64 BVHSerializeSize(const struct BVH *bvh)
{
	return 2*sizeof(u32) 
		+ ds_BitSetSerializeSize(&bvh->leaf_set) 
		+ ds_BitSetSerializeSize(&bvh->internal_set) 
		+ ds_PoolSerializeSize(bvh->pool);
}

void BVHSerialize(struct ss *ss, const struct BVH *bvh)
{
	ds_Assert(BVHSerializeSize(bvh) <= ss_BytesLeft(ss));

	ss_WriteU32Le(ss, bvh->bt.root);
	ss_WriteU32Le(ss, bvh->bt.count);
	ds_BitSetSerialize(ss, &bvh->leaf_set);
	ds_BitSetSerialize(ss, &bvh->internal_set);
	ds_PoolSerialize(ss, bvh->pool);
}

u32 BVHTryDeserialize(struct arena *mem, struct ss *ss, struct BVH *bvh, const u32 growable)
{
	const u64 bit_index = ss->bit_index;
	const u64 mem_left = (mem) ? mem->mem_left : 0;
	*bvh = (struct BVH) { .heap_allocated = !mem };
	BTFlush(bvh->bt);
	if (ss_BytesLeft(ss) < 2*sizeof(u32))
	{
		return 0;
	}

	bvh->bt.root = ss_ReadU32Le(ss);
	bvh->bt.count = ss_ReadU32Le(ss);
	if (!ds_BitSetTryDeserialize(mem, ss, &bvh->leaf_set, growable)
		|| !ds_BitSetTryDeserialize(mem, ss, &bvh->internal_set, growable)
		|| !ds_PoolTryDeserialize(mem, ss, bvh->pool, growable))
	{
		if (mem)
		{
			ArenaPopPacked(mem, mem_left - mem->mem_left);
		}
		else
		{
			ds_BitSetDealloc(&bvh->leaf_set);
			ds_BitSetDealloc(&bvh->internal_set);
			ds_PoolDealloc(bvh->pool);
		}
		*bvh = (struct BVH) { .heap_allocated = !mem };
		BTFlush(bvh->bt);
		ss->bit_index = bit_index;
		return 0;
	}

	bvh->cost_queue = MQAAlloc(NULL, COST_QUEUE_INITIAL_COUNT, growable);
	return 1;
}

static f32 BodySah(const struct aabb *box)
{
	const v3 hw = V3Scale(V3Sub(box->max, box->min), 0.5f);
	return hw.x*(hw.y + hw.z) + hw.y*hw.z;
}

static f32 BVHCostRecursive(const struct BVH *bvh, const u32 index)
{
	f32 cost;
	
	const struct BVHNode *node = bvh->pool.buf;
	if (BTLeafCheck(node + index))
	{
		cost = node[index].bt_child[1] * COST_INTERNAL;
	}
	else
	{
		const f32 cost_left = BVHCostRecursive(bvh, node[index].bt_child[0]);
		const f32 cost_right = BVHCostRecursive(bvh, node[index].bt_child[1]);

		const f32 probability_left = BodySah(&node[node[index].bt_child[0]].bbox) /  BodySah(&node[index].bbox);
		const f32 probability_right = BodySah(&node[node[index].bt_child[1]].bbox) / BodySah(&node[index].bbox);

		cost = COST_TRAVERSAL + probability_left*cost_left + probability_right*cost_right; 
	}

	return cost;
}

f32 BVHCost(const struct BVH *bvh)
{
	return BVHCostRecursive(bvh, bvh->bt.root);
}

void DBVHFlush(struct BVH *bvh)
{
    ds_BitSetClear(&bvh->internal_set, 0);
    ds_BitSetClear(&bvh->leaf_set, 0);
	BTFlush(bvh->bt);
    ds_PoolFlush(bvh->pool);
	MQAFlush(&bvh->cost_queue);
}

static void DBVHInternalBalanceNode(struct BVH *bvh, const u32 node)
{
	struct BVHNode *nodes = bvh->pool.buf;
	/* (1) find best rotation */
	u32 left = nodes[node].bt_child[0];
	u32 right = nodes[node].bt_child[1];
	struct aabb box_union;
	f32 cost_rotation, cost_original, cost_best = F32_INFINITY;
			
	u32 upper_rotation = BT_INDEX_NULL; /* child to rotate */
	u32 best_rotation = BT_INDEX_NULL; /* best grandchild to rotate */
	if (!BTLeafCheck(nodes + left))
	{
		box_union = BboxUnion(nodes[nodes[left].bt_child[0]].bbox, nodes[right].bbox);
		cost_original = BodySah(&nodes[left].bbox);	
		cost_rotation = BodySah(&box_union);
		if (cost_rotation < cost_original)
		{
			upper_rotation = right;
			best_rotation = nodes[left].bt_child[1];
			cost_best = cost_rotation;
		}

		box_union = BboxUnion(nodes[nodes[left].bt_child[1]].bbox, nodes[right].bbox);
		cost_rotation = BodySah(&box_union);
		if (cost_rotation < cost_original && cost_rotation < cost_best)
		{
			upper_rotation = right;
			best_rotation = nodes[left].bt_child[0];
			cost_best = cost_rotation;
		}
	}

	if (!BTLeafCheck(nodes + right))
	{
		box_union = BboxUnion(nodes[nodes[right].bt_child[0]].bbox, nodes[left].bbox);
		cost_original = BodySah(&nodes[right].bbox);
		cost_rotation = BodySah(&box_union);
		if (cost_rotation < cost_best && cost_rotation < cost_original)
		{
			upper_rotation = left;
			best_rotation = nodes[right].bt_child[1];
			cost_best = cost_rotation;
		}

		box_union = BboxUnion(nodes[nodes[right].bt_child[1]].bbox, nodes[left].bbox);
		cost_rotation = BodySah(&box_union);
		if (cost_rotation < cost_best && cost_rotation < cost_original)
		{
			upper_rotation = left;
			best_rotation = nodes[right].bt_child[0];
			cost_best = cost_rotation;
		}
	}

	/* (2) apply rotation */
	if (best_rotation != BT_INDEX_NULL)
	{
		nodes[best_rotation].bt_parent = (nodes[best_rotation].bt_parent & BT_LEAF_MASK) | node;
		if (upper_rotation == left)
		{
			nodes[upper_rotation].bt_parent = (nodes[upper_rotation].bt_parent & BT_LEAF_MASK) | right;
			nodes[node].bt_child[0] = best_rotation;
			if (best_rotation == nodes[right].bt_child[0])
			{
				nodes[right].bbox = BboxUnion(nodes[nodes[right].bt_child[1]].bbox, nodes[upper_rotation].bbox);
				nodes[right].bt_child[0] = upper_rotation;
			}
			else
			{
				nodes[right].bbox = BboxUnion(nodes[nodes[right].bt_child[0]].bbox, nodes[upper_rotation].bbox);
				nodes[right].bt_child[1] = upper_rotation;
			}
			left = best_rotation;
		}
		else
		{
			nodes[upper_rotation].bt_parent = (nodes[upper_rotation].bt_parent & BT_LEAF_MASK) | left;
			nodes[node].bt_child[1] = best_rotation;
			if (best_rotation == nodes[left].bt_child[0])
			{
				nodes[left].bbox = BboxUnion(nodes[nodes[left].bt_child[1]].bbox, nodes[upper_rotation].bbox);
				nodes[left].bt_child[0] = upper_rotation;
			}
			else
			{
				nodes[left].bbox = BboxUnion(nodes[nodes[left].bt_child[0]].bbox, nodes[upper_rotation].bbox);
				nodes[left].bt_child[1] = upper_rotation;
			}
			right = best_rotation;
		}
	}

	/* (3) refit node's box */
	nodes[node].bbox = BboxUnion(nodes[left].bbox, nodes[right].bbox);
}

u32 DBVHInsert(struct BVH *bvh, const u32 body, const u32 shape, const struct aabb *bbox)
{
    struct slot leaf = ds_PoolAdd(bvh->pool);
    struct slot internal = { .address = NULL, .index = U32_MAX };
	if (bvh->bt.root == BT_INDEX_NULL)
	{
	    struct BVHNode *nodes = bvh->pool.buf;
        BTAddRoot(bvh->bt, nodes, leaf.index);

		/* Store external id's in bt_child[0] of leaves */
		nodes[leaf.index].bt_child[0] = shape;
		nodes[leaf.index].bt_child[1] = body;
		nodes[leaf.index].bbox = *bbox;
	}
	else
	{
		internal = ds_PoolAdd(bvh->pool);
	    struct BVHNode *nodes = bvh->pool.buf;
		nodes[leaf.index].bbox = *bbox;
		nodes[leaf.index].bt_parent = BT_LEAF_MASK | internal.index;
		nodes[leaf.index].bt_child[0] = shape;
		nodes[leaf.index].bt_child[1] = body;
        bvh->bt.count += 2;

		/**
		 * (1) Find best sibling using the minimum surface area hueristic + branch and bound algorithm.
		 * The idea is that every node in the hierarchy is a potential sibling to the new node, and we find
		 * the best suitable one by continuously delve deeper into the hierarchy as long as the some 
		 * new potential node gives a better cost than previous ones. We keep track of the best score and 
		 * the node achieving it. When no node achieves a better score, we are done and set the best scoring
		 * one as the sibling.
		 */
		u32 best_index = bvh->bt.root;
		f32 best_cost = F32_INFINITY;
		f32 node_cost = 0.0f; 
	
		MQAPush(&bvh->cost_queue, node_cost, bvh->bt.root);

		u32 node;
		f32 inherited_cost, cost;
	
		while (bvh->cost_queue.object_pool.count > 0)
		{
			/* (i) Get cost of node */
			inherited_cost = bvh->cost_queue.elements[0].priority; 
			node = MQAPop(&bvh->cost_queue);
			const struct aabb box_union = BboxUnion(nodes[leaf.index].bbox, nodes[node].bbox);
			/* Inherited area cost + expanded node area cost */
			cost = inherited_cost + BodySah(&box_union);

			if (cost < best_cost)
			{
				best_cost = cost;
				best_index = node;
			}

			/**
			 * The current difference in area produced by the node's path + the new box's area
			 * is a lower bound on the node's descendants' cost. If the lower bound is not less
			 * than the best cost, we can prune the children's trees. Otherwise, we must still
			 * consider them as viable siblings. Their priorities become the increase in cost 
			 * to node's path when adding the new box (the inherited cost).
			 */
			cost -= BodySah(&nodes[node].bbox);

			if (!BTLeafCheck(nodes + node) && cost + BodySah(&nodes[leaf.index].bbox) < best_cost)
			{
				MQAPush(&bvh->cost_queue, cost, nodes[node].bt_child[0]);
				MQAPush(&bvh->cost_queue, cost, nodes[node].bt_child[1]);
			}
		}

		/* (2) Setup a new parent node for the new node and its sibling */
		const u32 best_parent = nodes[best_index].bt_parent & BT_INDEX_MASK;
		if (BTRootCheck(nodes + best_index))
		{
			bvh->bt.root = internal.index;
		}
		else
		{
			if (nodes[best_parent].bt_child[0] == best_index)
			{
				nodes[best_parent].bt_child[0] = internal.index;
			}
			else
			{
				nodes[best_parent].bt_child[1] = internal.index;
			}
		}

		nodes[internal.index].bt_parent = best_parent;
		nodes[internal.index].bt_child[0] = best_index;
		nodes[internal.index].bt_child[1] = leaf.index;
		nodes[internal.index].bbox = BboxUnion(nodes[leaf.index].bbox, nodes[best_index].bbox);
		nodes[best_index].bt_parent = (nodes[best_index].bt_parent & BT_LEAF_MASK) | internal.index;

		node = nodes[internal.index].bt_parent;
		/* (3) Traverse from grandparent of leaf, refitting and rotating node up to the root */
		while (node != BT_INDEX_NULL)
		{
			DBVHInternalBalanceNode(bvh, node);
			node = nodes[node].bt_parent;
		}
	}

	//struct arena *tmp = ArenaPushScratch();
	//BVHValidate(bvh);
	//ArenaPopScratch();

    if (bvh->leaf_set.bit_count < bvh->pool.length)
    {
        ds_BitSetIncreaseSize(&bvh->leaf_set, bvh->pool.length, 0);
        ds_BitSetIncreaseSize(&bvh->internal_set, bvh->pool.length, 0);
    }

    if (internal.address)
    {
        ds_BitSetSet(&bvh->internal_set, internal.index, 1);
    }
    ds_BitSetSet(&bvh->leaf_set, leaf.index, 1);

	return leaf.index;
}

void DBVHRemove(struct BVH *bvh, const u32 index)
{
	struct BVHNode *nodes = bvh->pool.buf;
	ds_Assert(BTLeafCheck(nodes + index));
    ds_BitSetSet(&bvh->leaf_set, index, 0);

	u32 parent = nodes[index].bt_parent & BT_INDEX_MASK;
	if (parent == BT_INDEX_NULL)
	{
		bvh->bt.root = BT_INDEX_NULL;
        bvh->bt.count -= 1;
        ds_PoolRemove(bvh->pool, index);
	}
	else
	{
        ds_BitSetSet(&bvh->internal_set, parent, 0);
		const u32 sibling = (nodes[parent].bt_child[0] == index)
			? nodes[parent].bt_child[1]
			: nodes[parent].bt_child[0];

		const u32 grand_parent = nodes[parent].bt_parent;
		nodes[sibling].bt_parent = (nodes[sibling].bt_parent & BT_LEAF_MASK) | grand_parent;

        ds_PoolRemove(bvh->pool, parent);
        ds_PoolRemove(bvh->pool, index);
        bvh->bt.count -= 2;

		/* set new root */
		if (grand_parent == BT_INDEX_NULL)
		{
			bvh->bt.root = sibling;
		}
		else
		{
			if (nodes[grand_parent].bt_child[0] == parent)
			{
				nodes[grand_parent].bt_child[0] = sibling;
			}
			else
			{
				nodes[grand_parent].bt_child[1] = sibling;
			}

			nodes[grand_parent].bbox = BboxUnion(nodes[nodes[grand_parent].bt_child[0]].bbox, nodes[nodes[grand_parent].bt_child[1]].bbox);
			parent = nodes[grand_parent].bt_parent;
			while (parent != BT_INDEX_NULL)
			{
				DBVHInternalBalanceNode(bvh, parent);
				parent = nodes[parent].bt_parent;
			}
		}
	}
}

struct BVHQuerySet BVHQuery(struct arena *mem, const struct BVH *bvh, const struct BVHNode *node)
{
    if (BTLeafCount(bvh->bt) < 1) { return (struct BVHQuerySet) { 0 }; }
	const struct BVHNode *nodes = bvh->pool.buf;

    struct memArray mem_arr = ArenaPushAlignedAll(mem, sizeof(u32), sizeof(u32));
    struct BVHQuerySet query = 
    { 
        .count = 0,
        .shape = mem_arr.addr,
    };

    struct arena *tmp = ArenaPushScratch();
    struct memArray stack_arr = ArenaPushAlignedAll(tmp, sizeof(void *), sizeof(void *));
    u32 *node_stack = stack_arr.addr;
    u32 sc = 0;

    if (AabbTest(&node->bbox, &nodes[bvh->bt.root].bbox))
    {
       node_stack[ sc++ ] = bvh->bt.root;
    }

    while (sc--)
    {
        const struct BVHNode *n = nodes + node_stack[ sc ];
        if (BTLeafCheck(n))
        {
            if (query.count == mem_arr.len)
            {
				LogString(T_PHYSICS, S_FATAL, "out-of-memory in query set, increase arena size!");		
				FatalCleanupAndExit();
            }
            query.shape[ query.count++ ] = n->bt_child[0];
        }
        else
        {
            /* room for both children before either is pushed */
            if (sc + 2 > stack_arr.len)
            {
				LogString(T_PHYSICS, S_FATAL, "out-of-memory in query stack, increase arena size!");		
				FatalCleanupAndExit();
            }
        	const struct BVHNode *left = nodes + n->bt_child[0];
        	const struct BVHNode *right = nodes + n->bt_child[1];
        	if (AabbTest(&node->bbox, &right->bbox))
        	{
        		node_stack[ sc++ ] = n->bt_child[1];
        	}
        	if (AabbTest(&node->bbox, &left->bbox))
            {
        		node_stack[ sc++ ] = n->bt_child[0];
            }
        }
    }

    ArenaPopScratch();

    ArenaPopPacked(mem, (mem_arr.len - query.count)*sizeof(u32));

    return query;
}

struct BVHQuerySet BVHQueryAndFilterOnBody(struct arena *mem, const struct BVH *bvh, const struct BVHNode *node, const struct ds_BitSet *dirty)
{
	if (BTLeafCount(bvh->bt) < 1) { return (struct BVHQuerySet) { 0 }; }
	const struct BVHNode *nodes = bvh->pool.buf;

    struct memArray mem_arr = ArenaPushAlignedAll(mem, sizeof(u32), sizeof(u32));
    struct BVHQuerySet query = 
    { 
        .count = 0,
        .shape = mem_arr.addr,
    };

    struct arena *tmp = ArenaPushScratch();
    struct memArray stack_arr = ArenaPushAlignedAll(tmp, sizeof(void *), sizeof(void *));
    u32 *node_stack = stack_arr.addr;
    u32 sc = 0;

    if (AabbTest(&node->bbox, &nodes[bvh->bt.root].bbox))
    {
       node_stack[ sc++ ] = bvh->bt.root;
    }

    while (sc--)
    {
        const struct BVHNode *n = nodes + node_stack[ sc ];
        /* bt_child[1] == ds_Body index */
        if (BTLeafCheck(n))
        {
            if (node->bt_child[1] < n->bt_child[1] || (node->bt_child[1] != n->bt_child[1] && !ds_BitSetGet(dirty, n->bt_child[0])))
            {
                if (query.count == mem_arr.len)
                {
			    	LogString(T_PHYSICS, S_FATAL, "out-of-memory in query set, increase arena size!");		
			    	FatalCleanupAndExit();
                }
                query.shape[ query.count++ ] = n->bt_child[0];
            }
        }
        else
        {
            /* room for both children before either is pushed */
            if (sc + 2 > stack_arr.len)
            {
				LogString(T_PHYSICS, S_FATAL, "out-of-memory in query stack, increase arena size!");		
				FatalCleanupAndExit();
            }
        	const struct BVHNode *left = nodes + n->bt_child[0];
        	const struct BVHNode *right = nodes + n->bt_child[1];
        	if (AabbTest(&node->bbox, &right->bbox))
        	{
        		node_stack[ sc++ ] = n->bt_child[1];
        	}
        	if (AabbTest(&node->bbox, &left->bbox))
            {
        		node_stack[ sc++ ] = n->bt_child[0];
            }
        }
    }

    ArenaPopScratch();

    ArenaPopPacked(mem, (mem_arr.len - query.count)*sizeof(u32));

    return query;
}

void BVHPropagateBoundingBoxesFromLeaves(struct BVH *bvh)
{
    ProfZone;

    struct BVHNode *n = bvh->pool.buf;
    BTI it;
    BTLRInit(it, bvh->pool.buf, bvh->bt.root);
    while (it.at != BT_INDEX_NULL)
    {
        struct BVHNode *p = n + it.at;
        if (!BTLeafCheck(p))
        {
            const struct BVHNode *c[2] =
            {
                n + p->bt_child[0],
                n + p->bt_child[1],
            };
            p->bbox = BboxUnion(c[0]->bbox, c[1]->bbox);
        }
        BTLRAdvance(it, bvh->pool.buf);
    }

    ProfZoneEnd;
}

void BVHValidate(const struct BVH *bvh)
{
	BTValidate(bvh->bt, bvh->pool.buf);
	if (bvh->bt.root == BT_INDEX_NULL) { return; }

	const struct BVHNode *node = bvh->pool.buf;
    BTI it;
    BTDFInit(it, bvh->pool.buf, bvh->bt.root);
    do
    {
        const u32 i = it.at;
        if (!BTRootCheck(node + i))
        {
			const u32 parent = node[i].bt_parent & BT_INDEX_MASK;
			ds_Assert(AabbContainsMargin(&node[parent].bbox, &node[i].bbox, 0.001f));
        }

        BTDFAdvance(it, bvh->pool.buf);
    }
    while (it.at != it.root);
}

struct triMeshBVH TriMeshBVHConstruct(struct arena *mem, const struct triMesh *mesh, const u32 bin_count)
{
	ds_Assert(bin_count && bin_count <= 255);	/* bin indices are bytes */
	if (!mesh->tri_count)
	{
		struct triMeshBVH none = { 0 };
		BTFlush(none.bvh.bt);
		return none;
	}

	ProfZone;

	ArenaPushRecord(mem);
	const u32 max_node_count_required = 2*mesh->tri_count - 1;

	struct triMeshBVH mesh_bvh = 
	{
		.mesh = mesh,
		.bvh = 
		{ 
            .internal_set = { 0 },
            .leaf_set = { 0 },
			.heap_allocated = 0,
		},
		.tri = ArenaPush(mem, mesh->tri_count*sizeof(u32)),
		.tri_count = mesh->tri_count,
        .depth = 0,
	};
	ds_PoolAlloc(mem, mesh_bvh.bvh.pool, max_node_count_required, NOT_GROWABLE);
    BTFlush(mesh_bvh.bvh.bt);

    struct arena *tmp1 = ArenaPushScratch();
    struct arena *tmp2 = ArenaPushScratch();
	struct aabb *axis_bin_bbox[3];
	u32 *axis_bin_tri_count[3];
	u8 *centroid_bin_map[3];
	centroid_bin_map[0] = ArenaPush(tmp1, mesh->tri_count*sizeof(u8));
	centroid_bin_map[1] = ArenaPush(tmp1, mesh->tri_count*sizeof(u8));
	centroid_bin_map[2] = ArenaPush(tmp1, mesh->tri_count*sizeof(u8));
	axis_bin_bbox[0] = ArenaPush(tmp1, bin_count*sizeof(struct aabb));
	axis_bin_bbox[1] = ArenaPush(tmp1, bin_count*sizeof(struct aabb));
	axis_bin_bbox[2] = ArenaPush(tmp1, bin_count*sizeof(struct aabb));
	axis_bin_tri_count[0] = ArenaPush(tmp1, bin_count*sizeof(u32));
	axis_bin_tri_count[1] = ArenaPush(tmp1, bin_count*sizeof(u32));
	axis_bin_tri_count[2] = ArenaPush(tmp1, bin_count*sizeof(u32));
	struct aabb *bbox_tri = ArenaPush(tmp1, mesh->tri_count*sizeof(struct aabb));
	struct memArray arr = ArenaPushAlignedAll(tmp1, sizeof(u32), 4);
    struct memArray depth_arr = ArenaPushAlignedAll(tmp2, sizeof(u32), 4);

	u32 success = 1;
	if (!mesh_bvh.bvh.pool.length 
			|| !mesh_bvh.tri 
			|| !centroid_bin_map[2] 
			|| !axis_bin_tri_count[2] 
			|| !axis_bin_bbox[2] 
			|| !arr.len 
			|| !bbox_tri)
	{
		success = 0;
		goto end;
	}

	u32 *node_stack = arr.addr;	
    u32 *depth_stack = depth_arr.addr;
	u32 node_stack_size = (u32) arr.len;
	u32 sc = 1;
    struct slot root = ds_PoolAdd(mesh_bvh.bvh.pool);
	BTAddRoot(mesh_bvh.bvh.bt, mesh_bvh.bvh.pool.buf, root.index);
	struct BVHNode *node = root.address;
	/* bt_child[0] = tri_first,
	 * bt_child[1] = tri_count */
	node->bt_child[0] = 0;
	node->bt_child[1] = mesh->tri_count;
	node->bbox = BboxTriangle(
				mesh->v[mesh->tri[0].buf[0]],
				mesh->v[mesh->tri[0].buf[1]],
				mesh->v[mesh->tri[0].buf[2]]);
	node_stack[0] = root.index;
    depth_stack[0] = 0;

	for (u32 i = 0; i < mesh->tri_count; ++i)
	{
		mesh_bvh.tri[i] = i;
		bbox_tri[i] = BboxTriangle(
				mesh->v[mesh->tri[i].buf[0]],
				mesh->v[mesh->tri[i].buf[1]],
				mesh->v[mesh->tri[i].buf[2]]);
		node->bbox = BboxUnion(node->bbox, bbox_tri[i]);
	}

	/* Process triangles from left to right, depth-first. */
	while (sc--)
	{
        if (mesh_bvh.depth < depth_stack[sc])
        {
            mesh_bvh.depth = depth_stack[sc];
        }
		node = mesh_bvh.bvh.pool.buf + node_stack[sc];
		const u32 tri_first = node->bt_child[0];
		const u32 tri_count = node->bt_child[1];
		if (tri_count == 1)
		{
			continue;
		}

		ProfZoneNamed("mesh_bvh.bvh construction iteration");
		const v3 bbox_max = node->bbox.max;
		const v3 bbox_min = node->bbox.min;

		u32 best_axis = U32_MAX;
		u32 best_split = U32_MAX;
		u32 best_left_count = 0;
		u32 best_right_count = 0;
		struct aabb best_bbox_left = { 0 };
		struct aabb best_bbox_right = { 0 };
		const f32 parent_sah = BodySah(&node->bbox);
		f32 best_score = F32_INFINITY;
		for (u32 axis = 0; axis < 3; axis++)
		{
			/* flat along this axis: no split possible, and the binning below would be 0/0 */
			const f32 extent = bbox_max.buf[axis] - bbox_min.buf[axis];
			ds_Assert(extent == extent);
			if (!(extent > 0.0f))
			{
				continue;
			}

			for (u32 bi = 0; bi < bin_count; ++bi)
			{
				axis_bin_tri_count[axis][bi] = 0;
			}

			for (u32 i = tri_first; i < tri_first + tri_count; ++i)
			{
				const u32 tri = mesh_bvh.tri[i];
				const f32 val = bin_count * (0.5f*(bbox_tri[tri].min.buf[axis] + bbox_tri[tri].max.buf[axis]) - bbox_min.buf[axis]) / (bbox_max.buf[axis] - bbox_min.buf[axis]);
				const u8 bi = (u8) F32Clamp(val, 0.0f, bin_count - 0.01f);
				centroid_bin_map[axis][tri] = bi;
				axis_bin_bbox[axis][bi] = (axis_bin_tri_count[axis][bi] > 0)
					? BboxUnion(axis_bin_bbox[axis][bi], bbox_tri[tri])
					: bbox_tri[tri];
				axis_bin_tri_count[axis][bi] += 1;
			}

			//TODO simplify bbox constructing by creating bbox array before loop so we can easily just bbox_left = [], bbox_right = [] 
			struct aabb bbox_left = { 0 };
			u32 left_count = 0;
			for (u32 split = 0; split < bin_count-1; ++split)
			{
				if (axis_bin_tri_count[axis][split] == 0)
				{
					continue;
				}

				bbox_left = (left_count == 0)
					? axis_bin_bbox[axis][split]
					: BboxUnion(bbox_left, axis_bin_bbox[axis][split]);
				left_count += axis_bin_tri_count[axis][split];

				const u32 right_count = node->bt_child[1] - left_count;
				if (right_count == 0)
				{
					break;
				}

				u32 bi = split + 1;
				for (; axis_bin_tri_count[axis][bi] == 0; ++bi);

				struct aabb bbox_right = axis_bin_bbox[axis][bi++];
				for (; bi < bin_count; bi++)
				{
					if (axis_bin_tri_count[axis][bi])
					{
						bbox_right = BboxUnion(bbox_right, axis_bin_bbox[axis][bi]);
					}
				}

				const f32 cost_traversal = COST_TRAVERSAL;
				const f32 cost_internal = COST_INTERNAL;
				const f32 left_cost = left_count*BodySah(&bbox_left)/parent_sah;
				const f32 right_cost = right_count*BodySah(&bbox_right)/parent_sah;
				const f32 score = cost_traversal + cost_internal*(left_cost + right_cost);
				if (score < best_score)
				{
					best_score = score;
					best_axis = axis;
					best_split = split;
					best_bbox_left = bbox_left;
					best_bbox_right = bbox_right;
					best_left_count = left_count;
					best_right_count = right_count;
				}
			}
		}

		if (best_left_count && best_right_count)
		{
			if (sc + 2 <= node_stack_size)
			{
				u32 left = tri_first;
				u32 right = tri_first + tri_count - 1;
				while (left < right)
				{
					const u32 tri = mesh_bvh.tri[left];
					if (centroid_bin_map[best_axis][tri] <= best_split)
					{
						left += 1;
					}
					else
					{
						mesh_bvh.tri[left] = mesh_bvh.tri[right];
						mesh_bvh.tri[right] = tri;
						right -= 1;
					}
				}


				const struct slot slot_left = ds_PoolAdd(mesh_bvh.bvh.pool);
                const struct slot slot_right = ds_PoolAdd(mesh_bvh.bvh.pool);

				BTAddChildren(mesh_bvh.bvh.bt, mesh_bvh.bvh.pool.buf, node_stack[sc], slot_left.index, slot_right.index);
				ds_Assert(slot_left.address && slot_right.address);

				struct BVHNode *child_left = slot_left.address;
				struct BVHNode *child_right = slot_right.address;

				child_left->bbox = best_bbox_left;
				child_left->bt_child[0] = tri_first;
				child_left->bt_child[1] = best_left_count;

				child_right->bbox = best_bbox_right;
				child_right->bt_child[0] = tri_first + best_left_count;
				child_right->bt_child[1] = best_right_count;

				node_stack[sc] = slot_right.index;
				node_stack[sc+1] = slot_left.index;

                const u32 new_depth = depth_stack[sc] + 1;
                depth_stack[sc] = new_depth;
                depth_stack[sc+1] = new_depth;
				sc += 2;
			}
			else
			{
				sc = 0;
				success = 0;	
			}
		}
		
		ProfZoneEnd;
	}
	
end:
	if (success)
	{
		ArenaRemoveRecord(mem);
	}
	else
	{
		ArenaPopRecord(mem);
		const u64 size_required = max_node_count_required*sizeof(struct BVHNode) 
			+ mesh->tri_count*sizeof(u32) 
			+ mesh->tri_count*sizeof(struct aabb)
			+ 3*mesh->tri_count*sizeof(u8) 
			+ 3*bin_count*(sizeof(struct aabb) + sizeof(u32));
		Log(T_SYSTEM, S_ERROR, "Failed to allocate bvh from triangle mesh, minimum size required: %lu", size_required);
		mesh_bvh = (struct triMeshBVH) { 0 };
	}

    ArenaPopScratch();
    ArenaPopScratch();

	BVHValidate(&mesh_bvh.bvh);

	ProfZoneEnd;
	return mesh_bvh;
}

/* the box's queue key: the entry t into its volume (0 if the origin is inside), F32_INFINITY on a miss */
static f32 BVHNodeRaycastParameter(const struct rayAabb *ray, const struct aabb *bbox)
{
	const intv t = AabbRaycastInterval(ray, bbox);
	const f32 t_entry = F32Max(t.low, 0.0f);
	return (t_entry <= t.high)
		? t_entry
		: F32_INFINITY;
}

struct BVHRaycastInfo BVHRaycastInit(struct arena *mem, const struct BVH *bvh, const struct ray *ray)
{
	struct BVHRaycastInfo info =
	{
		.hit = u32f32_inline(U32_MAX, F32_INFINITY),
		.node = (struct BVHNode *) bvh->pool.buf,
		.ray = RayAabb(ray),
		.bvh = bvh,
	};

	if (bvh->bt.count) 
	{
		const f32 root_hit_param = BVHNodeRaycastParameter(&info.ray, &info.node[info.bvh->bt.root].bbox);
		if (root_hit_param < F32_INFINITY) 
		{
			info.hit_queue = MQAllocAll(mem);
			MQPush(&info.hit_queue, bvh->bt.root, root_hit_param);
		}
	}

	return info;
}

void BVHRaycastTestAndPushChildren(struct BVHRaycastInfo *info, const u32f32 popped_tuple)
{
	const struct BVHNode *node = info->node;
	const f32 distance_left = BVHNodeRaycastParameter(&info->ray, &node[node[popped_tuple.u].bt_child[0]].bbox);
	const f32 distance_right = BVHNodeRaycastParameter(&info->ray, &node[node[popped_tuple.u].bt_child[1]].bbox);

	if (distance_left < F32_INFINITY)
	{
		MQPush(&info->hit_queue, info->node[popped_tuple.u].bt_child[0], distance_left);
	}

	if (distance_right < F32_INFINITY)
	{
		if (info->hit_queue.count == info->hit_queue.length)
		{
			LogString(T_SYSTEM, S_FATAL, "distance queue in bvh_raycast OOM, aborting");
			FatalCleanupAndExit();
		}
		MQPush(&info->hit_queue, info->node[popped_tuple.u].bt_child[1], distance_right);
	}
}
