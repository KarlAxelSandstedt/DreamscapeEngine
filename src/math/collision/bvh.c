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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "collision.h"

POOL_DEFINE(bvhNode);

//TODO can play around with these
#define COST_TRAVERSAL  1.0f	/* Overhead of internal node traversal (AABB testing of children) */
#define COST_INTERNAL 	1.5f	/* Overhead of triangle intersection tests */


struct bvh DbvhAlloc(struct arena *mem, const u32 initial_length, const u32 growable)
{
	ds_Assert(!mem || !growable);
	struct bvh bvh =
	{
		.pool = bvhNodePoolAlloc(mem, initial_length, growable),
		.cost_queue = MinQueueAlloc(NULL, COST_QUEUE_INITIAL_COUNT, growable),
		.heap_allocated = !mem,	
	};

    ds_BTFlush(bvh.bt);

	return bvh;
}

void BvhFree(struct bvh *bvh)
{
    bvhNodePoolDealloc(&bvh->pool);
	MinQueueDealloc(&bvh->cost_queue);
}

static f32 BodySah(const struct aabb *box)
{
	return box->hw[0]*(box->hw[1] + box->hw[2]) + box->hw[1]*box->hw[2];
}

static f32 BvhCostRecursive(const struct bvh *bvh, const u32 index)
{
	f32 cost;
	
	const struct bvhNode *node = bvh->pool.buf;
	if (ds_BTLeafCheck(node + index))
	{
		cost = node[index].bt_child[1] * COST_INTERNAL;
	}
	else
	{
		const f32 cost_left = BvhCostRecursive(bvh, node[index].bt_child[0]);
		const f32 cost_right = BvhCostRecursive(bvh, node[index].bt_child[1]);

		const f32 probability_left = BodySah(&node[node[index].bt_child[0]].bbox) /  BodySah(&node[index].bbox);
		const f32 probability_right = BodySah(&node[node[index].bt_child[1]].bbox) / BodySah(&node[index].bbox);

		cost = COST_TRAVERSAL + probability_left*cost_left + probability_right*cost_right; 
	}

	return cost;
}

f32 BvhCost(const struct bvh *bvh)
{
	return BvhCostRecursive(bvh, bvh->bt.root);
}

void DbvhFlush(struct bvh *bvh)
{
	ds_BTFlush(bvh->bt);
    bvhNodePoolFlush(&bvh->pool);
	MinQueueFlush(&bvh->cost_queue);
}

static void DbvhInternalBalanceNode(struct bvh *bvh, const u32 node)
{
	struct bvhNode *nodes = bvh->pool.buf;
	/* (1) find best rotation */
	u32 left = nodes[node].bt_child[0];
	u32 right = nodes[node].bt_child[1];
	struct aabb box_union;
	f32 cost_rotation, cost_original, cost_best = F32_INFINITY;
			
	u32 upper_rotation; /* child to rotate */
	u32 best_rotation = BT_INDEX_NULL; /* best grandchild to rotate */
	if (!ds_BTLeafCheck(nodes + left))
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

	if (!ds_BTLeafCheck(nodes + right))
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

u32 DbvhInsert(struct bvh *bvh, const u32 body, const u32 shape, const struct aabb *bbox)
{
    struct slot leaf = bvhNodePoolAdd(&bvh->pool);
	if (bvh->bt.root == BT_INDEX_NULL)
	{
	    struct bvhNode *nodes = bvh->pool.buf;
        ds_BTAddRoot(bvh->bt, nodes, leaf.index);

		/* Store external id's in bt_child[0] of leaves */
		nodes[leaf.index].bt_child[0] = shape;
		nodes[leaf.index].bt_child[1] = body;
		nodes[leaf.index].bbox = *bbox;
	}
	else
	{
		struct slot internal = bvhNodePoolAdd(&bvh->pool);
	    struct bvhNode *nodes = bvh->pool.buf;
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
	
		MinQueuePush(&bvh->cost_queue, node_cost, bvh->bt.root);

		u32 node;
		f32 inherited_cost, cost;
	
		while (bvh->cost_queue.object_pool.count > 0)
		{
			/* (i) Get cost of node */
			inherited_cost = bvh->cost_queue.elements[0].priority; 
			node = MinQueuePop(&bvh->cost_queue);
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

			if (!ds_BTLeafCheck(nodes + node) && cost + BodySah(&nodes[leaf.index].bbox) < best_cost)
			{
				MinQueuePush(&bvh->cost_queue, cost, nodes[node].bt_child[0]);
				MinQueuePush(&bvh->cost_queue, cost, nodes[node].bt_child[1]);
			}
		}

		/* (2) Setup a new parent node for the new node and its sibling */
		const u32 best_parent = nodes[best_index].bt_parent & BT_INDEX_MASK;
		if (ds_BTRootCheck(nodes + best_index))
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
			DbvhInternalBalanceNode(bvh, node);
			node = nodes[node].bt_parent;
		}
	}

	//struct arena *tmp = ArenaPushScratch();
	//BvhValidate(tmp, bvh);
	//ArenaPopScratch();

	return leaf.index;
}

void DbvhRemove(struct bvh *bvh, const u32 index)
{
	struct bvhNode *nodes = bvh->pool.buf;
	ds_Assert(ds_BTLeafCheck(nodes + index));

	u32 parent = nodes[index].bt_parent & BT_INDEX_MASK;
	if (parent == BT_INDEX_NULL)
	{
		bvh->bt.root = BT_INDEX_NULL;
        bvh->bt.count -= 1;
        bvhNodePoolRemove(&bvh->pool, parent);
	}
	else
	{
		const u32 sibling = (nodes[parent].bt_child[0] == index)
			? nodes[parent].bt_child[1]
			: nodes[parent].bt_child[0];

		const u32 grand_parent = nodes[parent].bt_parent;
		nodes[sibling].bt_parent = (nodes[sibling].bt_parent & BT_LEAF_MASK) | grand_parent;

        bvhNodePoolRemove(&bvh->pool, parent);
        bvhNodePoolRemove(&bvh->pool, index);
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
				DbvhInternalBalanceNode(bvh, parent);
				parent = nodes[parent].bt_parent;
			}
		}
	}
}

struct bvh_QuerySet BvhQuery(struct arena *mem, const struct bvh *bvh, const struct bvhNode *node)
{
    if (ds_BTLeafCount(bvh->bt) < 1) { return (struct bvh_QuerySet) { 0 }; }
	const struct bvhNode *nodes = bvh->pool.buf;

    struct memArray mem_arr = ArenaPushAlignedAll(mem, sizeof(u32), sizeof(u32));
    struct bvh_QuerySet query = 
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
        const struct bvhNode *n = nodes + node_stack[ sc ];
        if (ds_BTLeafCheck(n))
        {
            query.shape[ query.count ] = n->bt_child[0];
            query.count += 1;
            if (query.count >= mem_arr.len)
            {
				LogString(T_PHYSICS, S_FATAL, "out-of-memory in query set, increase arena size!");		
				FatalCleanupAndExit();
            }
        }
        else
        {
        	const struct bvhNode *left = nodes + n->bt_child[0];
        	const struct bvhNode *right = nodes + n->bt_child[1];
        	if (AabbTest(&node->bbox, &right->bbox))
        	{
        		node_stack[ sc++ ] = n->bt_child[1];
        	}
        
        	if (AabbTest(&node->bbox, &left->bbox))
            {
        		node_stack[ sc++ ] = n->bt_child[0];
                if (sc >= stack_arr.len)
                {
					LogString(T_PHYSICS, S_FATAL, "out-of-memory in query stack, increase arena size!");		
					FatalCleanupAndExit();
                }
            }
        }
    }

    ArenaPopScratch();

    ArenaPopPacked(mem, (mem_arr.len - query.count)*sizeof(u32));

    return query;
}

struct bvh_QuerySet BvhQueryAndFilterOnBody(struct arena *mem, const struct bvh *bvh, const struct bvhNode *node)
{
	if (ds_BTLeafCount(bvh->bt) < 1) { return (struct bvh_QuerySet) { 0 }; }
	const struct bvhNode *nodes = bvh->pool.buf;

    struct memArray mem_arr = ArenaPushAlignedAll(mem, sizeof(u32), sizeof(u32));
    struct bvh_QuerySet query = 
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
        const struct bvhNode *n = nodes + node_stack[ sc ];
        /* bt_child[1] == ds_RigidBody index */
        if (ds_BTLeafCheck(n))
        {
            if (node->bt_child[1] < n->bt_child[1])
            {
                query.shape[ query.count ] = n->bt_child[0];
                query.count += 1;
                if (query.count >= mem_arr.len)
                {
			    	LogString(T_PHYSICS, S_FATAL, "out-of-memory in query set, increase arena size!");		
			    	FatalCleanupAndExit();
                }
            }
        }
        else
        {
        	const struct bvhNode *left = nodes + n->bt_child[0];
        	const struct bvhNode *right = nodes + n->bt_child[1];
        	if (AabbTest(&node->bbox, &right->bbox))
        	{
        		node_stack[ sc++ ] = n->bt_child[1];
        	}
        
        	if (AabbTest(&node->bbox, &left->bbox))
            {
        		node_stack[ sc++ ] = n->bt_child[0];
                if (sc >= stack_arr.len)
                {
					LogString(T_PHYSICS, S_FATAL, "out-of-memory in query stack, increase arena size!");		
					FatalCleanupAndExit();
                }
            }
        }
    }

    ArenaPopScratch();

    ArenaPopPacked(mem, (mem_arr.len - query.count)*sizeof(u32));

    return query;
}


u32 DbvhInternalPushSubtreeOverlapPairs(struct arena *mem, struct dbvhOverlap *stack, const u64 stack_len, const struct bvh *bvh, u32 subA, u32 subB)
{
	struct bvhNode *nodes = bvh->pool.buf;
	u32 overlap_count = 0;
	struct dbvhOverlap overlap;
	u32 q = U32_MAX;

	while (1)
	{
		if (AabbTest(&nodes[subA].bbox, &nodes[subB].bbox))
		{
			if (ds_BTLeafCheck(nodes + subA) && ds_BTLeafCheck(nodes + subB))
			{
				overlap_count += 1;
				/* id's */
				if (nodes[subA].bt_child[0] < nodes[subB].bt_child[0])
				{
					overlap.id1 = nodes[subA].bt_child[0];	
					overlap.id2 = nodes[subB].bt_child[0];	
				}
				else
				{
					overlap.id1 = nodes[subB].bt_child[0];	
					overlap.id2 = nodes[subA].bt_child[0];	
				}
				ArenaPushPackedMemcpy(mem, &overlap, sizeof(overlap));
			}
			else
			{
				/* if a is larger than b, descend into a first  */
				if (ds_BTLeafCheck(nodes + subB) || (!ds_BTLeafCheck(nodes + subA) && BodySah(&nodes[subB].bbox) < BodySah(&nodes[subA].bbox)))
				{
					stack[++q].id1 = nodes[subA].bt_child[0];
					stack[q].id2 = subB;
					subA = nodes[subA].bt_child[1];
				}
				else
				{
					stack[++q].id1 = nodes[subB].bt_child[0];
					stack[q].id2 = subA;
					subB = nodes[subB].bt_child[1];
				}

				if (q+1 >= stack_len)
				{
					LogString(T_PHYSICS, S_FATAL, "out-of-memory in arena based stack, increase arena size!");		
					FatalCleanupAndExit();
				}
				continue;
			}
		}

		if (q != U32_MAX)
		{
			subA = stack[q].id1;
			subB = stack[q--].id2;
		}
		else
		{
			break;
		}
	}

	return overlap_count;
}

struct dbvhOverlap *DbvhPushOverlapPairs(struct arena *mem, u32 *count, const struct bvh *bvh)
{
	if (ds_BTLeafCount(bvh->bt) < 2) { return 0; }
	const struct bvhNode *nodes = bvh->pool.buf;

	*count = 0;
	u32 a = nodes[bvh->bt.root].bt_child[0];
	u32 b = nodes[bvh->bt.root].bt_child[1];
	u32 q = U32_MAX;

	struct arena *tmp1 = ArenaPushScratch();
	struct arena *tmp2 = ArenaPushScratch();

	struct memArray arr1 = ArenaPushAlignedAll(tmp1, sizeof(struct dbvhOverlap), 4); 
	struct memArray arr2 = ArenaPushAlignedAll(tmp2, sizeof(struct dbvhOverlap), 4); 

	struct dbvhOverlap *stack1 = arr1.addr;
	struct dbvhOverlap *stack2 = arr2.addr;
	struct dbvhOverlap *overlaps = (struct dbvhOverlap *) mem->stack_ptr; 

	while (1)
	{
		*count += DbvhInternalPushSubtreeOverlapPairs(mem, stack2, arr2.len, bvh, a, b);

		if (!ds_BTLeafCheck(nodes + a))
		{
			stack1[++q].id1 = nodes[a].bt_child[0];
			stack1[q].id2 = nodes[a].bt_child[1];	
			if (q >= arr1.len)
			{
				LogString(T_PHYSICS, S_FATAL, "out-of-memory in arena based stack, increase arena size!");		
				FatalCleanupAndExit();
			}
		}

		if (!ds_BTLeafCheck(nodes + b))
		{
			 a = nodes[b].bt_child[0];	
			 b = nodes[b].bt_child[1];	
			 continue;
		}

		if (q != U32_MAX)
		{
			a = stack1[q].id1;
			b = stack1[q--].id2;
		}
		else
		{
			break;
		}
	}

    ArenaPopScratch();
    ArenaPopScratch();

	return (*count) ? overlaps : NULL;
}

void BvhValidate(struct arena *tmp, const struct bvh *bvh)
{
	ds_BTValidate(bvh->bt, bvh->pool.buf);
	if (bvh->bt.root == BT_INDEX_NULL) { return; }

	ArenaPushRecord(tmp);
	const struct bvhNode *node = bvh->pool.buf;
	struct memArray arr = ArenaPushAlignedAll(tmp, sizeof(u32), 4);
	u32 *stack = arr.addr;
	stack[0] = bvh->bt.root;
	u32 sc = 1;
	while (sc--)
	{
		const u32 i = stack[sc];
		if (!ds_BTRootCheck(node + i))
		{
			const u32 parent = node[i].bt_parent & BT_INDEX_MASK;
			ds_Assert(AabbContainsMargin(&node[parent].bbox, &node[i].bbox, 0.001f));
		}

		if (!ds_BTLeafCheck(node + stack[sc]))
		{
			stack[sc + 0] = node[i].bt_child[0];
			stack[sc + 1] = node[i].bt_child[1];
			sc += 2;
		}
	}
	ArenaPopRecord(tmp);
}

struct triMeshBvh TriMeshBvhConstruct(struct arena *mem, const struct triMesh *mesh, const u32 bin_count)
{
	ds_Assert(bin_count);
	if (!mesh->tri_count)
	{
		return (struct triMeshBvh) { 0 };
	}

	ProfZone;

	ArenaPushRecord(mem);
	const u32 max_node_count_required = 2*mesh->tri_count - 1;

	struct triMeshBvh mesh_bvh = 
	{
		.mesh = mesh,
		.bvh = 
		{ 
			.pool = bvhNodePoolAlloc(mem, max_node_count_required, NOT_GROWABLE),
			.heap_allocated = 0,
		},
		.tri = ArenaPush(mem, mesh->tri_count*sizeof(u32)),
		.tri_count = mesh->tri_count,
        .depth = 0,
	};
    ds_BTFlush(mesh_bvh.bvh.bt);

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
	u32 node_stack_size = arr.len;
	u32 sc = 1;
    struct slot root = bvhNodePoolAdd(&mesh_bvh.bvh.pool);
	ds_BTAddRoot(mesh_bvh.bvh.bt, mesh_bvh.bvh.pool.buf, root.index);
	struct bvhNode *node = root.address;
	/* bt_child[0] = tri_first,
	 * bt_child[1] = tri_count */
	node->bt_child[0] = 0;
	node->bt_child[1] = mesh->tri_count;
	node->bbox = BboxTriangle(
				mesh->v[mesh->tri[0][0]],
				mesh->v[mesh->tri[0][1]],
				mesh->v[mesh->tri[0][2]]);
	node_stack[0] = root.index;
    depth_stack[0] = 0;

	for (u32 i = 0; i < mesh->tri_count; ++i)
	{
		mesh_bvh.tri[i] = i;
		bbox_tri[i] = BboxTriangle(
				mesh->v[mesh->tri[i][0]],
				mesh->v[mesh->tri[i][1]],
				mesh->v[mesh->tri[i][2]]);
		node->bbox = BboxUnion(node->bbox, bbox_tri[i]);
	}

	ds_AssertString(Vec3Length(node->bbox.center) < 0.0001f, "Center should most likely be 0.0, so the root box center defines a local origin!");
	
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
		vec3 bbox_min, bbox_max;
		Vec3Add(bbox_max, node->bbox.center, node->bbox.hw);
		Vec3Sub(bbox_min, node->bbox.center, node->bbox.hw);

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
			for (u32 bi = 0; bi < bin_count; ++bi)
			{
				axis_bin_tri_count[axis][bi] = 0;
			}

			for (u32 i = tri_first; i < tri_first + tri_count; ++i)
			{
				const u32 tri = mesh_bvh.tri[i];
				const f32 val = bin_count * (bbox_tri[tri].center[axis] - bbox_min[axis]) / (bbox_max[axis] - bbox_min[axis]);
				const u8 bi = (u8) f32_clamp(val, 0.0f, bin_count - 0.01f);
				centroid_bin_map[axis][tri] = bi;
				axis_bin_bbox[axis][bi] = (axis_bin_tri_count[axis][bi] > 0)
					? BboxUnion(axis_bin_bbox[axis][bi], bbox_tri[tri])
					: bbox_tri[tri];
				axis_bin_tri_count[axis][bi] += 1;
			}

			//TODO simplify bbox constructing by creating bbox array before loop so we can easily just bbox_left = [], bbox_right = [] 
			struct aabb bbox_left;
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


				const struct slot slot_left = bvhNodePoolAdd(&mesh_bvh.bvh.pool);
                const struct slot slot_right = bvhNodePoolAdd(&mesh_bvh.bvh.pool);

				ds_BTAddChildren(mesh_bvh.bvh.bt, mesh_bvh.bvh.pool.buf, node_stack[sc], slot_left.index, slot_right.index);
				ds_Assert(slot_left.address && slot_right.address);

				struct bvhNode *child_left = slot_left.address;
				struct bvhNode *child_right = slot_right.address;

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
		const u64 size_required = max_node_count_required*sizeof(struct bvhNode) 
			+ mesh->tri_count*sizeof(u32) 
			+ mesh->tri_count*sizeof(struct aabb)
			+ 3*mesh->tri_count*sizeof(u8) 
			+ 3*bin_count*(sizeof(struct aabb) + sizeof(u32));
		Log(T_SYSTEM, S_ERROR, "Failed to allocate bvh from triangle mesh, minimum size required: %lu\n", size_required);
		mesh_bvh = (struct triMeshBvh) { 0 };
	}

    ArenaPopScratch();
    ArenaPopScratch();

	BvhValidate(mem, &mesh_bvh.bvh);

	ProfZoneEnd;
	return mesh_bvh;
}

struct bvhRaycastInfo BvhRaycastInit(struct arena *mem, const struct bvh *bvh, const struct ray *ray)
{
	struct bvhRaycastInfo info =
	{
		.hit = u32f32_inline(U32_MAX, F32_INFINITY),
		.node = (struct bvhNode *) bvh->pool.buf,
		.ray = ray,
		.bvh = bvh,
	};

	if (bvh->bt.count) 
	{
		AabbRaycastParameterExSetup(info.multiplier, info.dir_sign_bit, info.ray);
		const f32 root_hit_param = AabbRaycastParameterEx(&info.node[info.bvh->bt.root].bbox, info.ray, info.multiplier, info.dir_sign_bit);
		if (root_hit_param < F32_INFINITY) 
		{
			info.hit_queue = MinQueueFixedAllocAll(mem);
			MinQueueFixedPush(&info.hit_queue, bvh->bt.root, root_hit_param);
		}
	}

	return info;
}

void BvhRaycastTestAndPushChildren(struct bvhRaycastInfo *info, const u32f32 popped_tuple)
{
	const struct bvhNode *node = info->node;
	const f32 distance_left = AabbRaycastParameterEx(&node[node[popped_tuple.u].bt_child[0]].bbox, info->ray, info->multiplier, info->dir_sign_bit);
	const f32 distance_right = AabbRaycastParameterEx(&node[node[popped_tuple.u].bt_child[1]].bbox, info->ray, info->multiplier, info->dir_sign_bit);

	if (distance_left < F32_INFINITY)
	{
		MinQueueFixedPush(&info->hit_queue, info->node[popped_tuple.u].bt_child[0], distance_left);
	}

	if (distance_right < F32_INFINITY)
	{
		if (info->hit_queue.count == info->hit_queue.length)
		{
			LogString(T_SYSTEM, S_FATAL, "distance queue in bvh_raycast OOM, aborting");
			FatalCleanupAndExit();
		}
		MinQueueFixedPush(&info->hit_queue, info->node[popped_tuple.u].bt_child[1], distance_right);
	}
}

u32f32 TriMeshBvhRaycast(struct arena *tmp, const struct triMeshBvh *mesh_bvh, const struct ray *ray)
{
	ProfZone;
	ArenaPushRecord(tmp);

	const struct bvh *bvh = &mesh_bvh->bvh;
	struct bvhRaycastInfo info = BvhRaycastInit(tmp, bvh, ray);
	while (info.hit_queue.count)
	{
		const u32f32 tuple = MinQueueFixedPop(&info.hit_queue);
		if (info.hit.f < tuple.f)
		{
			break;	
		}

		if (ds_BTLeafCheck(info.node + tuple.u))
		{
			const u32 tri_first = info.node[tuple.u].bt_child[0];
			const u32 tri_last = tri_first + info.node[tuple.u].bt_child[1] - 1;
			for (u32 i = tri_first; i <= tri_last; ++i)
			{
				const f32 distance = TriMeshRaycastParameter(mesh_bvh->mesh, mesh_bvh->tri[i], ray);
				if (distance < info.hit.f)
				{
					info.hit = u32f32_inline(mesh_bvh->tri[i], distance);
				}
			}
		}
		else
		{
			BvhRaycastTestAndPushChildren(&info, tuple);
		}
	}

	ArenaPopRecord(tmp);

	ProfZoneEnd;
	return info.hit;
}
