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
#include "ds_asset.h"
#include "ds_renderer.h"

/*
The renderer's GL-free parts.

Command keys (RenderCommandKeyTest): every field round-trips through r_CommandKey and the R_CMD_*_GET macros at
its maximum, materials through r_MaterialConstruct; the key orders by screen layer first, then transparency, then
depth, then material.

Meshes (RenderMeshTest): the stub box, a sphere, a capsule and a box hull have index counts that are multiples
of three, every index below the vertex count and the maximum index used equal to the last vertex; sphere and
capsule vertices lie on their surfaces with unit normals along them; every triangle of every mesh is wound
counter-clockwise seen from outside (its normal points away from the centroid), as GL_CCW / GL_BACK culling needs.

Camera (RenderCameraTest): with yaw and pitch 0 the window's bottom-left, centre and top-right pixels map to the
near plane's corners and centre (left is +x, up is +y, forward +z); the 2d transform maps the view centre to the
origin and the view's top-right corner to (1, 1).

Scene (RenderSceneTest): three immediate mesh instances sort descending by key, equal materials share a bucket
and the buffer sums their vertex bytes; a frame that draws nothing prunes them all. The cached proxy path
(r_InstanceAdd) needs r_Init's proxy hierarchy and mesh database, which only the editor's GL setup provides, so
it is not covered here.
*/

static u32 RendererTestNear(const f32 a, const f32 b, const f32 tol)
{
    return F32Abs(a - b) <= tol;
}

static u32 RendererTestV3Near(const v3 a, const v3 b, const f32 tol)
{
    return RendererTestNear(a.x, b.x, tol) && RendererTestNear(a.y, b.y, tol) && RendererTestNear(a.z, b.z, tol);
}

static u32 RendererTestBytesEqual(const void *a, const void *b, const u64 size)
{
    const u8 *x = a;
    const u8 *y = b;
    for (u64 i = 0; i < size; ++i)
    {
        if (x[i] != y[i])
        {
            return 0;
        }
    }
    return 1;
}

static struct test_Output RenderCommandKeyTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    const u64 screen_max = ((u64) 1 << R_CMD_SCREEN_LAYER_BITS) - 1;
    const u64 depth_max = ((u64) 1 << R_CMD_DEPTH_BITS) - 1;
    const u64 transparency_max = ((u64) 1 << R_CMD_TRANSPARENCY_BITS) - 1;
    const u64 material_max = ((u64) 1 << R_CMD_MATERIAL_BITS) - 1;
    const u64 key = r_CommandKey(screen_max, depth_max, transparency_max, material_max, R_CMD_PRIMITIVE_LINE, R_CMD_INSTANCED, R_CMD_ELEMENTS);
    TEST_TRUE(R_CMD_SCREEN_LAYER_GET(key) == screen_max && R_CMD_DEPTH_GET(key) == depth_max && R_CMD_TRANSPARENCY_GET(key) == transparency_max);
    TEST_TRUE(R_CMD_MATERIAL_GET(key) == material_max && R_CMD_PRIMITIVE_GET(key) == R_CMD_PRIMITIVE_LINE && R_CMD_INSTANCED_GET(key) == R_CMD_INSTANCED && R_CMD_ELEMENTS_GET(key) == R_CMD_ELEMENTS);
    TEST_TRUE(R_CMD_UNUSED_GET(key) == 0);

    struct rng rng;
    RngLocalInit(&rng, env->seed);
    for (u32 n = 0; n < 1000; ++n)
    {
        const u64 screen = RngLocalU64Range(&rng, 0, screen_max);
        const u64 depth = RngLocalU64Range(&rng, 0, depth_max);
        const u64 transparency = RngLocalU64Range(&rng, 0, transparency_max);
        const u64 program = RngLocalU64Range(&rng, 0, (MATERIAL_PROGRAM_MASK >> MATERIAL_PROGRAM_LOW_BIT));
        const u64 mesh = RngLocalU64Range(&rng, 0, (MATERIAL_MESH_MASK >> MATERIAL_MESH_LOW_BIT));
        const u64 texture = RngLocalU64Range(&rng, 0, (MATERIAL_TEXTURE_MASK >> MATERIAL_TEXTURE_LOW_BIT));
        const u64 material = r_MaterialConstruct(program, mesh, texture);
        TEST_TRUE(MATERIAL_PROGRAM_GET(material) == program && MATERIAL_MESH_GET(material) == mesh && MATERIAL_TEXTURE_GET(material) == texture);
        const u64 k = r_CommandKey(screen, depth, transparency, material, R_CMD_PRIMITIVE_TRIANGLE, R_CMD_NON_INSTANCED, R_CMD_ARRAYS);
        TEST_TRUE(R_CMD_SCREEN_LAYER_GET(k) == screen && R_CMD_DEPTH_GET(k) == depth && R_CMD_TRANSPARENCY_GET(k) == transparency && R_CMD_MATERIAL_GET(k) == material);
    }

    /* the order of the fields: a higher layer beats any transparency, a higher transparency any depth, a higher depth any material */
    const u64 m0 = r_MaterialConstruct(0, 0, 0);
    const u64 m1 = r_MaterialConstruct(1, 1, 1);
    TEST_TRUE(r_CommandKey(1, 0, 0, m0, 0, 0, 0) > r_CommandKey(0, depth_max, transparency_max, material_max, 1, 1, 1));
    TEST_TRUE(r_CommandKey(0, 0, 1, m0, 0, 0, 0) > r_CommandKey(0, depth_max, 0, material_max, 1, 1, 1));
    TEST_TRUE(r_CommandKey(0, 1, 0, m0, 0, 0, 0) > r_CommandKey(0, 0, 0, material_max, 1, 1, 1));
    TEST_TRUE(r_CommandKey(0, 0, 0, m1, 0, 0, 0) > r_CommandKey(0, 0, 0, m0, 1, 1, 1));

    return output;
}

/* indices in range, the maximum used equal to the last vertex, every triangle CCW seen from outside (centroid inside) */
static u32 RendererTestMeshCheck(const struct r_Mesh *mesh, const v3 inside)
{
    if (mesh->index_count % 3 != 0 || mesh->vertex_count == 0)
    {
        return 0;
    }
    const u8 *vertex = mesh->vertex_data;
    if (mesh->index_count)
    {
        u32 max_used = 0;
        for (u32 i = 0; i < mesh->index_count; ++i)
        {
            if (mesh->index_data[i] >= mesh->vertex_count)
            {
                return 0;
            }
            max_used = (mesh->index_data[i] > max_used) ? mesh->index_data[i] : max_used;
        }
        if (max_used != mesh->index_max_used || max_used != mesh->vertex_count - 1)
        {
            return 0;
        }
    }

    const u32 tri_count = (mesh->index_count) ? mesh->index_count / 3 : mesh->vertex_count / 3;
    for (u32 t = 0; t < tri_count; ++t)
    {
        v3 p[3];
        for (u32 k = 0; k < 3; ++k)
        {
            const u32 vi = (mesh->index_count) ? mesh->index_data[3*t + k] : 3*t + k;
            memcpy(p + k, vertex + vi*mesh->local_stride, sizeof(v3));
        }
        const v3 n = V3Cross(V3Sub(p[1], p[0]), V3Sub(p[2], p[0]));
        const v3 centroid = V3Scale(V3Add(V3Add(p[0], p[1]), p[2]), 1.0f / 3.0f);
        if (V3Dot(n, V3Sub(centroid, inside)) <= 0.0f)
        {
            return 0;
        }
    }

    return 1;
}

static struct test_Output RenderMeshTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    struct arena *mem = env->mem_1;
    ArenaPushRecord(mem);

    struct r_Mesh stub = { 0 };
    r_MeshStubBox(&stub);
    TEST_TRUE(stub.vertex_count == 24 && stub.index_count == 36 && RendererTestMeshCheck(&stub, V3Zero()));

    /* sphere: 2 + (refinement - 1) * 2 refinement vertices on the radius, normals along them */
    const f32 radius = 2.0f;
    struct r_Mesh sphere = { 0 };
    r_MeshSphere(mem, &sphere, radius, 6);
    TEST_TRUE(sphere.vertex_count == 2 + 5*12 && sphere.index_count == 360 && RendererTestMeshCheck(&sphere, V3Zero()));
    for (u32 i = 0; i < sphere.vertex_count; ++i)
    {
        v3 p, n;
        memcpy(&p, (u8 *) sphere.vertex_data + i*sphere.local_stride, sizeof(v3));
        memcpy(&n, (u8 *) sphere.vertex_data + i*sphere.local_stride + sizeof(v3), sizeof(v3));
        TEST_TRUE(RendererTestNear(V3Length(p), radius, 1e-5f) && RendererTestV3Near(V3Scale(p, 1.0f / radius), n, 1e-5f));
    }

    /* capsule: every vertex 0.5 from the core segment, the normal radial from the nearest core point */
    const f32 half_height = 1.0f;
    const f32 cap_radius = 0.5f;
    struct r_Mesh capsule = { 0 };
    r_MeshCapsule(mem, &capsule, half_height, cap_radius, 4);
    TEST_TRUE(capsule.vertex_count == 2 + 8*8 && RendererTestMeshCheck(&capsule, V3Zero()));
    for (u32 i = 0; i < capsule.vertex_count; ++i)
    {
        v3 p, n;
        memcpy(&p, (u8 *) capsule.vertex_data + i*capsule.local_stride, sizeof(v3));
        memcpy(&n, (u8 *) capsule.vertex_data + i*capsule.local_stride + sizeof(v3), sizeof(v3));
        const v3 core = V3(0.0f, F32Clamp(p.y, -half_height, half_height), 0.0f);
        const v3 radial = V3Sub(p, core);
        TEST_TRUE(RendererTestNear(V3Length(radial), cap_radius, 1e-5f) && RendererTestV3Near(V3Scale(radial, 1.0f / cap_radius), n, 1e-5f));
    }

    /* a box hull: 6 faces of 4 vertices, 12 triangles */
    const struct dcel box = DcelBox(mem, V3(1.0f, 2.0f, 3.0f));
    struct r_Mesh hull = { 0 };
    r_MeshHull(mem, &hull, &box);
    TEST_TRUE(hull.vertex_count == 24 && hull.index_count == 36 && RendererTestMeshCheck(&hull, V3Zero()));

    ArenaPopRecord(mem);
    return output;
}

static struct test_Output RenderCameraTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    /* fov_x 90 degrees, near 1: the near plane is 2 wide; aspect 2: 1 high */
    const struct r_Camera cam = r_CameraInit(V3F64(0.0, 0.0, 0.0), V3(0.0f, 0.0f, 1.0f), 1.0f, 100.0f, 2.0f, F32_PI / 2.0f);
    TEST_TRUE(cam.yaw == 0.0f && cam.pitch == 0.0f);
    f32 width, height;
    FrustumProjectionPlaneSides(&width, &height, cam.fz_near, cam.fov_x, cam.aspect_ratio);
    TEST_TRUE(RendererTestNear(width, 2.0f, 1e-5f) && RendererTestNear(height, 1.0f, 1e-5f));

    const v2 win = V2(800.0f, 400.0f);
    const v3 bottom_left = V3F64Demote(WindowSpaceToWorldSpace(V2(0.0f, 0.0f), win, &cam));
    const v3 centre = V3F64Demote(WindowSpaceToWorldSpace(V2(400.0f, 200.0f), win, &cam));
    const v3 top_right = V3F64Demote(WindowSpaceToWorldSpace(V2(800.0f, 400.0f), win, &cam));
    TEST_TRUE(RendererTestV3Near(bottom_left, V3(1.0f, -0.5f, 1.0f), 1e-5f));
    TEST_TRUE(RendererTestV3Near(centre, V3(0.0f, 0.0f, 1.0f), 1e-5f));
    TEST_TRUE(RendererTestV3Near(top_right, V3(-1.0f, 0.5f, 1.0f), 1e-5f));

    const m3 to_screen = r_Camera2dTransform(V2(10.0f, 20.0f), 4.0f, 2.0f);
    const v3 origin = M3V3Mul(to_screen, V3(10.0f, 20.0f, 1.0f));
    const v3 corner = M3V3Mul(to_screen, V3(14.0f, 22.0f, 1.0f));
    TEST_TRUE(RendererTestV3Near(origin, V3(0.0f, 0.0f, 1.0f), 1e-5f) && RendererTestV3Near(corner, V3(1.0f, 1.0f, 1.0f), 1e-5f));

    return output;
}

static struct test_Output RenderSceneTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    struct r_Scene *scene = r_SceneAlloc();
    r_SceneSetGlobal(scene);

    struct r_Mesh box = { 0 };
    r_MeshStubBox(&box);
    const u64 vertex_bytes = box.vertex_count * box.local_stride;
    const u64 colour = r_MaterialConstruct(PROGRAM_COLOR, MESH_NONE, TEXTURE_NONE);
    const u64 k_near = r_CommandKey(R_CMD_SCREEN_LAYER_GAME, 5, R_CMD_TRANSPARENCY_OPAQUE, colour, R_CMD_PRIMITIVE_TRIANGLE, R_CMD_NON_INSTANCED, R_CMD_ARRAYS);
    const u64 k_far = r_CommandKey(R_CMD_SCREEN_LAYER_GAME, 9, R_CMD_TRANSPARENCY_OPAQUE, colour, R_CMD_PRIMITIVE_TRIANGLE, R_CMD_NON_INSTANCED, R_CMD_ARRAYS);
    const u64 k_hud = r_CommandKey(R_CMD_SCREEN_LAYER_HUD, 1, R_CMD_TRANSPARENCY_ADDITIVE, colour, R_CMD_PRIMITIVE_LINE, R_CMD_NON_INSTANCED, R_CMD_ARRAYS);

    /* frame 1: three immediate mesh instances, added out of order */
    r_SceneFrameBegin();
    const u64 keys[3] = { k_near, k_hud, k_far };
    for (u32 i = 0; i < 3; ++i)
    {
        struct r_Instance *instance = r_InstanceAddNonCached(keys[i]);
        instance->type = R_INSTANCE_MESH;
        instance->mesh = &box;
    }
    r_SceneFrameEnd();
    TEST_TRUE(scene->cmd_frame_count == 3 && scene->cmd_frame[0].key == k_far && scene->cmd_frame[1].key == k_near && scene->cmd_frame[2].key == k_hud);
    for (u32 i = 0; i < 3; ++i)
    {
        const struct r_Instance *instance = scene->instance_pool.buf + scene->cmd_frame[i].instance;
        TEST_TRUE(instance->cmd == scene->cmd_frame + i && instance->frame_last_touched == scene->frame);
    }
    const struct r_Bucket *b0 = scene->frame_bucket_list;
    TEST_TRUE(b0 && b0->next && b0->next->next == NULL);
    TEST_TRUE(b0->c_l == 0 && b0->c_h == 1 && b0->buffer_count == 1 && b0->buffer_array[0]->local_size == 2*vertex_bytes);
    TEST_TRUE(b0->next->c_l == 2 && b0->next->c_h == 2 && b0->next->primitive == R_CMD_PRIMITIVE_LINE && b0->next->screen_layer == R_CMD_SCREEN_LAYER_HUD);
    TEST_TRUE(RendererTestBytesEqual(b0->buffer_array[0]->local_data, box.vertex_data, vertex_bytes) && RendererTestBytesEqual(b0->buffer_array[0]->local_data + vertex_bytes, box.vertex_data, vertex_bytes));

    /* frame 2 draws nothing: the immediate instances are pruned, nothing is left to bucket */
    r_SceneFrameBegin();
    r_SceneFrameEnd();
    TEST_TRUE(scene->cmd_frame_count == 0 && scene->instance_pool.count == 0 && scene->frame_bucket_list == NULL);

    r_SceneSetGlobal(NULL);
    r_SceneDealloc(scene);
    return output;
}

static struct test_Output (*renderer_tests[])(struct test_Environment *) =
{
    RenderCommandKeyTest,
    RenderMeshTest,
    RenderCameraTest,
    RenderSceneTest,
};

struct suite_Correctness m_renderer_suite =
{
    .id = "renderer",
    .unit_test = renderer_tests,
    .unit_test_count = sizeof(renderer_tests) / sizeof(renderer_tests[0]),
};

struct suite_Correctness *renderer_correctness_suite = &m_renderer_suite;
