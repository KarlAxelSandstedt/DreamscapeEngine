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

#include "ds_base.h"
#include "bit_vector.h"
#include "tree.h"

struct bt bt_AllocInternal(struct arena *mem, const u32 initial_length, const u64 slot_size, const u64 parent_offset, const u64 left_offset, const u64 right_offset, const u64 pool_slot_offset, const u32 growable)
{
	ds_Assert(!growable || !mem);

	struct bt bt =
	{
		.parent_offset = parent_offset,
		.left_offset = left_offset,
		.right_offset = right_offset,
		.root = BT_PARENT_INDEX_MASK,
		.pool = PoolAllocInternal(mem, initial_length, slot_size, pool_slot_offset, U64_MAX, growable),
	};

	return bt;
}

void bt_Dealloc(struct bt *tree)
{
	PoolDealloc(&tree->pool);
}

void bt_Flush(struct bt *tree)
{
	PoolFlush(&tree->pool);
	tree->root = BT_PARENT_INDEX_MASK;
}

void bt_Validate(struct arena *tmp, const struct bt *tree)
{
	if (tree->root == BT_PARENT_INDEX_MASK)
	{
		ds_Assert(0 == bt_NodeCount(tree));
		return;
	}

	ArenaPushRecord(tmp);
	struct bitVec traversed = BitVecAlloc(tmp, tree->pool.length, 0, NOT_GROWABLE);
	struct memArray arr = ArenaPushAlignedAll(tmp, sizeof(u32), 4);
	u32 *stack = arr.addr;
	stack[0] = tree->root;
	u32 sc = 1;

	u32 node_count = 0;
	u32 leaf_count = 0;
	while (sc--)
	{
		node_count += 1;
		u8 *addr = PoolAddress(&tree->pool, stack[sc]);
		u32 *alloc = (u32 *) (addr + tree->pool.slot_allocation_offset);
		u32 *p = (u32 *) (addr + tree->parent_offset);
		u32 *l = (u32 *) (addr + tree->left_offset);
		u32 *r = (u32 *) (addr + tree->right_offset);

		ds_Assert((*alloc & POOL_ALLOCATION_MASK) == 0);
		ds_Assert(BitVecGetBit(&traversed, stack[sc]) == 0);
		BitVecSetBit(&traversed, stack[sc], 1);

		if ((BT_PARENT_INDEX_MASK & (*p)) != BT_PARENT_INDEX_MASK)
		{
			u8 *parent = PoolAddress(&tree->pool, BT_PARENT_INDEX_MASK & (*p));
			u32 *p_alloc = (u32 *) (parent + tree->pool.slot_allocation_offset);
			u32 *p_p = (u32 *) (parent + tree->parent_offset);
			u32 *p_l = (u32 *) (parent + tree->left_offset);
			u32 *p_r = (u32 *) (parent + tree->right_offset);

			ds_Assert((*p_alloc & POOL_ALLOCATION_MASK) == 0);
			ds_Assert(!(BT_PARENT_LEAF_MASK & (*p_p)));
			ds_Assert(*p_l == stack[sc] || *p_r == stack[sc]);
		}

		if (BT_PARENT_LEAF_MASK & (*p))
		{
			leaf_count += 1;
		}
		else
		{
			stack[sc + 0] = *l;
			stack[sc + 1] = *r;
			sc += 2;
		}
	};

	const u32 actual_leaf_count = bt_LeafCount(tree);
	const u32 actual_node_count = bt_NodeCount(tree);
	ds_Assert(leaf_count == actual_leaf_count);
	ds_Assert(node_count == actual_node_count);

	ArenaPopRecord(tmp);
}

struct slot bt_NodeAdd(struct bt *tree)
{
	return PoolAdd(&tree->pool);
}

void bt_NodeRemove(struct bt *tree, const u32 index)
{
	PoolRemove(&tree->pool, BT_PARENT_INDEX_MASK & index);
}

struct slot bt_NodeAddRoot(struct bt *tree)
{
	struct slot slot = PoolAdd(&tree->pool);
	if (slot.index != POOL_NULL)
	{
		ds_Assert(tree->root == BT_PARENT_INDEX_MASK);
		tree->root = slot.index;
		u32 *parent = (u32 *) (((u8 *) slot.address) + tree->parent_offset);	
		*parent = BT_PARENT_LEAF_MASK | BT_PARENT_INDEX_MASK;
	}
	return slot;
}

void bt_NodeAddChildren(struct bt *tree, struct slot *left, struct slot *right, const u32 parent)
{
	*left = PoolAdd(&tree->pool);
	*right = PoolAdd(&tree->pool);

	if (!right->address)
	{
		if (left->address)
		{
			PoolRemove(&tree->pool, left->index);
			*left = empty_slot;
		}
	}
	else
	{
		u32 *bt_parent = (u32 *)(((u8 *) tree->pool.buf) + tree->pool.slot_size*parent + tree->parent_offset);	
		u32 *bt_left   = (u32 *)(((u8 *) tree->pool.buf) + tree->pool.slot_size*parent + tree->left_offset);	
		u32 *bt_right  = (u32 *)(((u8 *) tree->pool.buf) + tree->pool.slot_size*parent + tree->right_offset);	

		ds_Assert(*bt_parent & BT_PARENT_LEAF_MASK);
		*bt_parent &= BT_PARENT_INDEX_MASK;
		*bt_left = left->index;
		*bt_right = right->index;

		bt_parent = (u32 *)(((u8 *) left->address) + tree->parent_offset);	
		*bt_parent = BT_PARENT_LEAF_MASK | parent;

		bt_parent = (u32 *)(((u8 *) right->address) + tree->parent_offset);	
		*bt_parent = BT_PARENT_LEAF_MASK | parent;
	}
}

u32 bt_NodeCount(const struct bt *tree)
{
	ds_Assert(tree->pool.count == 0 || (tree->pool.count & 0x1));
	return tree->pool.count;
}

u32 bt_LeafCount(const struct bt *tree)
{
	ds_Assert(tree->pool.count == 0 || (tree->pool.count & 0x1));
	return (tree->pool.count)
		? (tree->pool.count >> 1) + 1
		: 0;
}


