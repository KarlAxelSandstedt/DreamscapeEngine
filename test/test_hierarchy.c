/*
==========================================================================
    Copyright (C) 2026 Axel Sandstedt

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

#include <string.h>

#include "ds_test.h"
#include "ds_hierarchy.h"

/*
 * Hierarchy Performance: the hierarchy operations the editor, UI, renderer and windows use, on a 4096-node
 * tree of 128-byte nodes (about a ui_Node's weight). The trees come from a fixed-seed generator, not the
 * engine RNG (seeded from entropy), so every run and every build does the same work. size = operations.
 */

#define HI_PERF_NODES	4096

typedef struct perf_HINode
{
	HI_NODE;
	u32	value;
	u8	payload[92];
} perf_HINode;
DEFINE_HI(perf_HINode);

struct hi_PerfInput
{
	HI(perf_HINode)	hi;
	struct arena	tmp;
	u32		parent[HI_PERF_NODES];	/* the i-th add's parent: HI_ROOT or an earlier add */
	u32		order[HI_PERF_NODES];	/* a fixed permutation of the adds, for adopts */
	u64		sum;
};

static u64 hi_PerfRandom(u64 *state)
{
	/* xorshift64*: the same sequence on every compiler */
	*state ^= *state >> 12;
	*state ^= *state << 25;
	*state ^= *state >> 27;
	return *state * 0x2545F4914F6CDD1Dull;
}

static void *hi_PerfInit(void)
{
	struct hi_PerfInput *in = ds_Alloc(sizeof(struct hi_PerfInput), DS_CACHE_LINE);
	memset(in, 0, sizeof(*in));
	HIAlloc(NULL, in->hi, HI_PERF_NODES, NOT_GROWABLE);
	in->tmp = ArenaAlloc(NULL, 1024*1024);

	u64 state = 0x9E3779B97F4A7C15ull;
	for (u32 i = 0; i < HI_PERF_NODES; ++i)
	{
		/* after a flush the adds take indices 2, 3, ...: add i may hang under the root or any earlier add */
		const u32 r = (u32) (hi_PerfRandom(&state) % (i + 1));
		in->parent[i] = (r == i) ? HI_ROOT : 2 + r;
		in->order[i] = 2 + i;
	}
	for (u32 i = HI_PERF_NODES - 1; i > 0; --i)
	{
		const u32 j = (u32) (hi_PerfRandom(&state) % (i + 1));
		const u32 t = in->order[i];
		in->order[i] = in->order[j];
		in->order[j] = t;
	}
	return in;
}

static void hi_PerfFree(void *args)
{
	struct hi_PerfInput *in = args;
	HIDealloc(in->hi);
	ArenaFree(&in->tmp);
	ds_Free(in);
}

static void hi_PerfBuild(struct hi_PerfInput *in)
{
	HIFlush(in->hi);
	for (u32 i = 0; i < HI_PERF_NODES; ++i)
	{
		const struct slot slot = HIAdd(in->hi, in->parent[i]);
		in->hi.pool.buf[slot.index].value = slot.index;
	}
}

/* reset: an empty hierarchy; test: build the tree */
static void hi_PerfResetEmpty(void *args)
{
	struct hi_PerfInput *in = args;
	HIFlush(in->hi);
}

/* reset: the full tree */
static void hi_PerfResetBuilt(void *args)
{
	hi_PerfBuild(args);
}

static void hi_add(void *args)
{
	struct hi_PerfInput *in = args;
	for (u32 i = 0; i < HI_PERF_NODES; ++i)
	{
		in->sum += HIAdd(in->hi, in->parent[i]).index;
	}
}

static void hi_remove(void *args)
{
	struct hi_PerfInput *in = args;
	const struct perf_HINode *root = in->hi.pool.buf + HI_ROOT;
	while (root->hi_first != HI_NULL)
	{
		HIRemove(&in->tmp, in->hi, (u32) root->hi_first);
	}
}

static void hi_PerfCustomFree(void *node, const u32 index, void *data)
{
	struct hi_PerfInput *in = data;
	in->sum += ((const struct perf_HINode *) node)->value + index;
}

static void hi_remove_custom_free(void *args)
{
	struct hi_PerfInput *in = args;
	const struct perf_HINode *root = in->hi.pool.buf + HI_ROOT;
	while (root->hi_first != HI_NULL)
	{
		HIApplyCustomFreeAndRemove(&in->tmp, in->hi, (u32) root->hi_first, hi_PerfCustomFree, in);
	}
}

/* every node to the orphan and back to the root: the fixed nodes are in no node's sub-hierarchy */
static void hi_adopt(void *args)
{
	struct hi_PerfInput *in = args;
	for (u32 i = 0; i < HI_PERF_NODES; ++i)
	{
		HIAdoptNode(in->hi, in->order[i], HI_ORPHAN);
		HIAdoptNode(in->hi, in->order[i], HI_ROOT);
	}
}

/* every node alone to the orphan (its children take its place), then back to the root */
static void hi_adopt_exclusive(void *args)
{
	struct hi_PerfInput *in = args;
	for (u32 i = 0; i < HI_PERF_NODES; ++i)
	{
		HIAdoptNodeExclusive(in->hi, in->order[i], HI_ORPHAN);
		HIAdoptNode(in->hi, in->order[i], HI_ROOT);
	}
}

/* control: the iterator macros are the same in both APIs */
static void hi_iterate(void *args)
{
	struct hi_PerfInput *in = args;
	HII it;
	HIIInit(it, in->hi, HI_ROOT);
	do
	{
		in->sum += in->hi.pool.buf[it.at].value;
		HIIAdvance(it, in->hi);
	} while (it.at != HI_ROOT);
}

struct test_PerformanceSerial hierarchy_serial_test[] =
{
	{ .id = "hi_add",                .size = HI_PERF_NODES,   .test = &hi_add,                .test_init = &hi_PerfInit, .test_reset = &hi_PerfResetEmpty, .test_free = &hi_PerfFree, },
	{ .id = "hi_remove",             .size = HI_PERF_NODES,   .test = &hi_remove,             .test_init = &hi_PerfInit, .test_reset = &hi_PerfResetBuilt, .test_free = &hi_PerfFree, },
	{ .id = "hi_remove_custom_free", .size = HI_PERF_NODES,   .test = &hi_remove_custom_free, .test_init = &hi_PerfInit, .test_reset = &hi_PerfResetBuilt, .test_free = &hi_PerfFree, },
	{ .id = "hi_adopt",              .size = 2*HI_PERF_NODES, .test = &hi_adopt,              .test_init = &hi_PerfInit, .test_reset = &hi_PerfResetBuilt, .test_free = &hi_PerfFree, },
	{ .id = "hi_adopt_exclusive",    .size = 2*HI_PERF_NODES, .test = &hi_adopt_exclusive,    .test_init = &hi_PerfInit, .test_reset = &hi_PerfResetBuilt, .test_free = &hi_PerfFree, },
	{ .id = "hi_iterate",            .size = HI_PERF_NODES,   .test = &hi_iterate,            .test_init = &hi_PerfInit, .test_reset = &hi_PerfResetBuilt, .test_free = &hi_PerfFree, },
};

struct suite_Performance storage_hierarchy_performance_suite =
{
	.id = "Hierarchy Performance",
	.serial_test = hierarchy_serial_test,
	.serial_test_count = sizeof(hierarchy_serial_test) / sizeof(hierarchy_serial_test[0]),
};

struct suite_Performance *hierarchy_performance_suite = &storage_hierarchy_performance_suite;
