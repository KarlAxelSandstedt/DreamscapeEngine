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

#include <stdio.h>

#include "ds_test.h"
#include "ds_geometry.h"
#include "ds_collision.h"
#include "ds_dynamics.h"

/*
Bounding volume hierarchies (bvh.c): the dynamic BVH's queries against brute force over the leaves through
inserts, removals and re-inserts, its invariants (BVHValidate), its serialization, and the mesh BVH's leaves.
*/

#define BVH_TEST_LEAF_MAX 256

struct BVHTestLeaf
{
    struct aabb box;
    u32 body;
    u32 node;       /* DBVHInsert's handle, BT_INDEX_NULL when not in the tree */
};

static struct aabb BVHTestRandomBox(const f32 range, const f32 size_max)
{
    const v3 c = V3(RngF32Range(-range, range), RngF32Range(-range, range), RngF32Range(-range, range));
    const v3 hw = V3(RngF32Range(0.01f, size_max), RngF32Range(0.01f, size_max), RngF32Range(0.01f, size_max));
    return (struct aabb) { .min = V3Sub(c, hw), .max = V3Add(c, hw) };
}

/*
A query for leaf q (its box, its body) must return exactly the shapes whose leaf box overlaps it; with the body
filter, only those of a larger body index. The engine's query set is compared as a set against brute force over
the leaves in the tree (shape i is in the tree iff leaf[i].node != BT_INDEX_NULL).
*/
static u32 BVHTestQueriesAgree(struct arena *mem, const struct BVH *bvh, const struct BVHTestLeaf *leaf, const u32 count, const u32 q, const u32 filter)
{
    ArenaPushRecord(mem);
    const struct BVHNode probe = { .bbox = leaf[q].box, .bt_child = { q, leaf[q].body } };
    const struct BVHQuerySet set = (filter)
        ? BVHQueryAndFilterOnBody(mem, bvh, &probe)
        : BVHQuery(mem, bvh, &probe);

    u8 found[BVH_TEST_LEAF_MAX] = { 0 };
    u32 ok = 1;
    for (u32 i = 0; i < set.count; ++i)
    {
        if (set.shape[i] >= count || found[set.shape[i]])
        {
            ok = 0;	/* out of range, or twice */
            break;
        }
        found[set.shape[i]] = 1;
    }
    for (u32 i = 0; ok && i < count; ++i)
    {
        const u32 expected = (leaf[i].node != BT_INDEX_NULL) && AabbTest(&leaf[q].box, &leaf[i].box)
                             && (!filter || leaf[q].body < leaf[i].body);
        if (expected != found[i])
        {
            fprintf(stderr, "query %u (filter %u): shape %u %s, expected %s\n", q, filter, i, found[i] ? "returned" : "missing", expected ? "returned" : "absent");
            ok = 0;
        }
    }
    ArenaPopRecord(mem);
    return ok;
}

/* Insert random leaves, query every leaf, remove half, query again, re-insert, flush: brute force agrees throughout */
struct test_Output DBVHQueryTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    struct BVH bvh = DBVHAlloc(NULL, 16, GROWABLE);	/* small: the pool and the sets grow */
    struct BVHTestLeaf leaf[BVH_TEST_LEAF_MAX];
    const u32 count = BVH_TEST_LEAF_MAX;
    for (u32 i = 0; i < count; ++i)
    {
        leaf[i].box = BVHTestRandomBox(10.0f, (i % 8 == 0) ? 4.0f : 0.5f);
        leaf[i].body = RngU64Range(0, 63);	/* several shapes per body */
        leaf[i].node = DBVHInsert(&bvh, leaf[i].body, i, &leaf[i].box);
    }
    BVHValidate(&bvh);
    TEST_NOT_ZERO(BTLeafCount(bvh.bt) == count);

    for (u32 q = 0; q < count; ++q)
    {
        if (!BVHTestQueriesAgree(env->mem_1, &bvh, leaf, count, q, 0) || !BVHTestQueriesAgree(env->mem_1, &bvh, leaf, count, q, 1))
        {
            BVHFree(&bvh);
            TEST_FAILURE;
        }
    }

    /* remove every other leaf, in a scrambled order */
    for (u32 k = 0; k < count; k += 2)
    {
        const u32 i = (k * 37) % count;
        if (leaf[i].node != BT_INDEX_NULL)
        {
            DBVHRemove(&bvh, leaf[i].node);
            leaf[i].node = BT_INDEX_NULL;
        }
    }
    BVHValidate(&bvh);
    for (u32 q = 0; q < count; ++q)
    {
        if (!BVHTestQueriesAgree(env->mem_1, &bvh, leaf, count, q, 0) || !BVHTestQueriesAgree(env->mem_1, &bvh, leaf, count, q, 1))
        {
            BVHFree(&bvh);
            TEST_FAILURE;
        }
    }

    /* re-insert them moved */
    for (u32 i = 0; i < count; ++i)
    {
        if (leaf[i].node == BT_INDEX_NULL)
        {
            leaf[i].box = BVHTestRandomBox(10.0f, 1.0f);
            leaf[i].node = DBVHInsert(&bvh, leaf[i].body, i, &leaf[i].box);
        }
    }
    BVHValidate(&bvh);
    TEST_NOT_ZERO(BTLeafCount(bvh.bt) == count);
    for (u32 q = 0; q < count; ++q)
    {
        if (!BVHTestQueriesAgree(env->mem_1, &bvh, leaf, count, q, 0))
        {
            BVHFree(&bvh);
            TEST_FAILURE;
        }
    }

    /* remove everything: an empty tree answers nothing and takes a leaf again */
    for (u32 i = 0; i < count; ++i)
    {
        DBVHRemove(&bvh, leaf[i].node);
        leaf[i].node = BT_INDEX_NULL;
    }
    TEST_NOT_ZERO(bvh.bt.root == BT_INDEX_NULL && BTLeafCount(bvh.bt) == 0);
    TEST_NOT_ZERO(BVHTestQueriesAgree(env->mem_1, &bvh, leaf, count, 0, 0));
    leaf[0].node = DBVHInsert(&bvh, leaf[0].body, 0, &leaf[0].box);
    BVHValidate(&bvh);
    TEST_NOT_ZERO(BTLeafCount(bvh.bt) == 1 && BVHTestQueriesAgree(env->mem_1, &bvh, leaf, count, 0, 0));

    DBVHFlush(&bvh);
    TEST_NOT_ZERO(bvh.bt.root == BT_INDEX_NULL);
    BVHFree(&bvh);

    return output;
}

/* A serialized dynamic BVH deserializes to a tree with the same shape, boxes and query answers */
struct test_Output DBVHSerializeTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    struct BVH bvh = DBVHAlloc(NULL, 64, GROWABLE);
    struct BVHTestLeaf leaf[64];
    const u32 count = 64;
    for (u32 i = 0; i < count; ++i)
    {
        leaf[i].box = BVHTestRandomBox(5.0f, 1.0f);
        leaf[i].body = i / 2;
        leaf[i].node = DBVHInsert(&bvh, leaf[i].body, i, &leaf[i].box);
    }

    const u64 size = BVHSerializeSize(&bvh);
    u8 *buf = ArenaPush(env->mem_1, size);
    struct ss out = ss_Buffered(buf, size);
    BVHSerialize(&out, &bvh);
    TEST_NOT_ZERO(out.bit_index == 8*size);

    struct ss in = ss_Buffered(buf, size);
    struct BVH copy;
    TEST_NOT_ZERO(BVHTryDeserialize(NULL, &in, &copy, GROWABLE));
    TEST_NOT_ZERO(in.bit_index == 8*size);
    BVHValidate(&copy);
    TEST_NOT_ZERO(copy.bt.root == bvh.bt.root && copy.bt.count == bvh.bt.count);
    for (u32 i = 0; i < count; ++i)
    {
        const struct BVHNode *a = (const struct BVHNode *) bvh.pool.buf + leaf[i].node;
        const struct BVHNode *b = (const struct BVHNode *) copy.pool.buf + leaf[i].node;
        TEST_NOT_ZERO(a->bt_parent == b->bt_parent && a->bt_child[0] == b->bt_child[0] && a->bt_child[1] == b->bt_child[1]);
        TEST_NOT_ZERO(a->bbox.min.x == b->bbox.min.x && a->bbox.min.y == b->bbox.min.y && a->bbox.min.z == b->bbox.min.z);
        TEST_NOT_ZERO(a->bbox.max.x == b->bbox.max.x && a->bbox.max.y == b->bbox.max.y && a->bbox.max.z == b->bbox.max.z);
        TEST_NOT_ZERO(ds_BitSetGet(&copy.leaf_set, leaf[i].node) == ds_BitSetGet(&bvh.leaf_set, leaf[i].node));
    }
    for (u32 q = 0; q < count; ++q)
    {
        if (!BVHTestQueriesAgree(env->mem_1, &copy, leaf, count, q, 1))
        {
            BVHFree(&copy);
            BVHFree(&bvh);
            TEST_FAILURE;
        }
    }

    /* the copy keeps working: a removal and an insert */
    DBVHRemove(&copy, leaf[3].node);
    leaf[3].node = BT_INDEX_NULL;
    BVHValidate(&copy);
    TEST_NOT_ZERO(BVHTestQueriesAgree(env->mem_1, &copy, leaf, count, 5, 0));
    leaf[3].node = DBVHInsert(&copy, leaf[3].body, 3, &leaf[3].box);
    BVHValidate(&copy);

    /* a truncated stream fails and leaves nothing behind */
    struct ss truncated = ss_Buffered(buf, size / 2);
    struct BVH none;
    TEST_NOT_ZERO(!BVHTryDeserialize(NULL, &truncated, &none, GROWABLE));
    TEST_NOT_ZERO(none.bt.root == BT_INDEX_NULL && truncated.bit_index == 0);

    BVHFree(&copy);
    BVHFree(&bvh);
    return output;
}

/*
The mesh BVH's leaves partition the triangle order [0, tri_count), each leaf's box contains its triangles, every
triangle index appears once in tri, the depth is the deepest leaf's, and BVHValidate holds. An empty mesh gives
the empty hierarchy.
*/
struct test_Output TriMeshBVHTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    for (u32 round = 0; round < 6; ++round)
    {
        ArenaPushRecord(env->mem_1);
        const u32 tri_count = (round == 0) ? 1 : (round == 1) ? 2 : 50 + round * 137;
        struct triMesh *mesh = ArenaPush(env->mem_1, sizeof(struct triMesh));
        mesh->v_count = 3 * tri_count;
        mesh->tri_count = tri_count;
        mesh->v = ArenaPush(env->mem_1, mesh->v_count*sizeof(v3));
        mesh->tri = ArenaPush(env->mem_1, tri_count*sizeof(v3u32));
        for (u32 i = 0; i < tri_count; ++i)
        {
            const v3 c = V3(RngF32Range(-20.0f, 20.0f), RngF32Range(-2.0f, 2.0f), RngF32Range(-20.0f, 20.0f));
            for (u32 k = 0; k < 3; ++k)
            {
                mesh->v[3*i + k] = V3Add(c, V3(RngF32Range(-1.0f, 1.0f), RngF32Range(-1.0f, 1.0f), RngF32Range(-1.0f, 1.0f)));
            }
            mesh->tri[i] = (v3u32) { .buf = { 3*i, 3*i + 1, 3*i + 2 } };
        }

        const struct triMeshBVH mesh_bvh = TriMeshBVHConstruct(env->mem_1, mesh, 8);
        TEST_NOT_ZERO(mesh_bvh.tri_count == tri_count && mesh_bvh.mesh == mesh);
        BVHValidate(&mesh_bvh.bvh);

        u8 *seen = ArenaPush(env->mem_1, tri_count);
        for (u32 i = 0; i < tri_count; ++i) { seen[i] = 0; }
        u32 covered = 0;
        u32 depth_max = 0;
        const struct BVHNode *node = mesh_bvh.bvh.pool.buf;
        BTI it;
        BTDFInit(it, mesh_bvh.bvh.pool.buf, mesh_bvh.bvh.bt.root);
        do
        {
            const struct BVHNode *n = node + it.at;
            u32 depth = 0;
            for (u32 p = n->bt_parent & BT_INDEX_MASK; p != BT_INDEX_NULL; p = node[p].bt_parent & BT_INDEX_MASK) { depth += 1; }
            depth_max = (depth > depth_max) ? depth : depth_max;
            if (BTLeafCheck(n))
            {
                const u32 first = n->bt_child[0], leaf_count = n->bt_child[1];
                TEST_NOT_ZERO(leaf_count >= 1 && first + leaf_count <= tri_count);
                for (u32 i = first; i < first + leaf_count; ++i)
                {
                    const u32 tri = mesh_bvh.tri[i];
                    TEST_NOT_ZERO(tri < tri_count && !seen[tri]);
                    seen[tri] = 1;
                    covered += 1;
                    const struct aabb box = BboxTriangle(mesh->v[mesh->tri[tri].x], mesh->v[mesh->tri[tri].y], mesh->v[mesh->tri[tri].z]);
                    TEST_NOT_ZERO(AabbContainsMargin(&n->bbox, &box, 0.0f));
                }
            }
            BTDFAdvance(it, mesh_bvh.bvh.pool.buf);
        }
        while (it.at != it.root);
        TEST_NOT_ZERO(covered == tri_count && depth_max == mesh_bvh.depth);
        TEST_NOT_ZERO(BVHCost(&mesh_bvh.bvh) > 0.0f);
        ArenaPopRecord(env->mem_1);
    }

    const struct triMesh empty = { 0 };
    const struct triMeshBVH none = TriMeshBVHConstruct(env->mem_1, &empty, 8);
    TEST_NOT_ZERO(none.tri_count == 0 && none.bvh.bt.count == 0 && none.bvh.bt.root == BT_INDEX_NULL);
    BVHValidate(&none.bvh);

    return output;
}

static struct test_Output (*bvh_tests[])(struct test_Environment *) =
{
    DBVHQueryTest,
    DBVHSerializeTest,
    TriMeshBVHTest,
};

struct suite_Correctness m_bvh_suite =
{
    .id = "BVH",
    .unit_test = bvh_tests,
    .unit_test_count = sizeof(bvh_tests) / sizeof(bvh_tests[0]),
};

struct suite_Correctness *bvh_correctness_suite = &m_bvh_suite;
