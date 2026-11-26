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
#include <stddef.h>
#include <string.h>

#include "ds_base.h"
#include "ds_math.h"
#include "ds_geometry.h"
#include "ds_collision.h"
#include "ds_dynamics.h"

/*
Assembly test box (DreamscapeAsm; script/linux/asm_run.sh, asm_debug.sh)
========================================================================
Hand-rolled NASM versions of engine functions (the .asm files in test/asm; Linux x86-64, System V calling
convention) run against the C functions on a table of hand-made cases. The run first prints the struct offsets
the assembly reads and writes, then one line per case:

    PASS:   both return the same count and, for a contact, v_count and id[0] are equal and n, depth[0] and v[0]
            agree within tolerance; "(bit-identical)" when every compared float has the same bits.
    ERROR:  the first value that differs, from both.

Tolerance: n per component 1e-5 (a unit vector); depth[0] and v[0] per component 1e-5 * max(r0 + r1, |p|), with
p = centre 1 - centre 0: the size of the numbers the results are computed from (depth cancels near touching).
Both manifolds are filled with 0xff bytes before the calls, so a value the assembly never wrote reads as NaN
and fails. No case sits on the contact limit |p| = r0 + r1 + speculative_distance, where a last-bit difference
changes the count.
*/

u32 c_SphereContactHandrolled(struct c_Manifold *out, struct ds_ConvexCache *cache, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1);

struct asm_SphereCase
{
    const char *        id;
    f32                 radius[2];
    ds_TransformWorld   t[2];
};

#define ASM_TOLERANCE           1.0e-5f
#define ASM_SPHERE_FLOAT_COUNT  7

/* Print the case's line; return 1 on PASS, 0 on ERROR. bit_identical: 1 if every compared float has the same bits. */
static u32 asm_SphereCaseCheck(u32 *bit_identical, const struct asm_SphereCase *c)
{
    *bit_identical = 0;

    struct c_Shape s[2];
    memset(s, 0, sizeof(s));
    s[0].type = C_SHAPE_SPHERE;
    s[0].sphere.radius = c->radius[0];
    s[1].type = C_SHAPE_SPHERE;
    s[1].sphere.radius = c->radius[1];

    struct ds_ConvexCache cache_c = { .sat.type = SAT_CACHE_NONE, .gjk.count = 0 };
    struct ds_ConvexCache cache_asm = { .sat.type = SAT_CACHE_NONE, .gjk.count = 0 };
    struct c_Manifold m_c;
    struct c_Manifold m_asm;
    memset(&m_c, 0xff, sizeof(m_c));
    memset(&m_asm, 0xff, sizeof(m_asm));

    const u32 count_c = c_SphereContact(&m_c, &cache_c, s + 0, c->t + 0, s + 1, c->t + 1);
    const u32 count_asm = c_SphereContactHandrolled(&m_asm, &cache_asm, s + 0, c->t + 0, s + 1, c->t + 1);

    if (count_c != count_asm)
    {
        printf("ERROR: %s: return value: C %u, asm %u\n", c->id, count_c, count_asm);
        return 0;
    }

    if (count_c == 0)
    {
        *bit_identical = 1;
        printf("PASS:  %s (no contact)\n", c->id);
        return 1;
    }

    if (m_c.v_count != m_asm.v_count)
    {
        printf("ERROR: %s: v_count: C %u, asm %u\n", c->id, m_c.v_count, m_asm.v_count);
        return 0;
    }

    if (m_c.id[0].val != m_asm.id[0].val)
    {
        printf("ERROR: %s: id[0]: C 0x%08x, asm 0x%08x\n", c->id, m_c.id[0].val, m_asm.id[0].val);
        return 0;
    }

    const f32 r_sum = c->radius[0] + c->radius[1];
    const f32 size = F32Max(r_sum, V3Length(V3F64Demote(V3F64Sub(c->t[1].position, c->t[0].position))));
    const f32 tol_length = ASM_TOLERANCE * size;

    const char *name[ASM_SPHERE_FLOAT_COUNT] = { "n.x", "n.y", "n.z", "depth[0]", "v[0].x", "v[0].y", "v[0].z" };
    const f32 f_c[ASM_SPHERE_FLOAT_COUNT] = { m_c.n.x, m_c.n.y, m_c.n.z, m_c.depth[0], m_c.v[0].x, m_c.v[0].y, m_c.v[0].z };
    const f32 f_asm[ASM_SPHERE_FLOAT_COUNT] = { m_asm.n.x, m_asm.n.y, m_asm.n.z, m_asm.depth[0], m_asm.v[0].x, m_asm.v[0].y, m_asm.v[0].z };
    const f32 tol[ASM_SPHERE_FLOAT_COUNT] = { ASM_TOLERANCE, ASM_TOLERANCE, ASM_TOLERANCE, tol_length, tol_length, tol_length, tol_length };

    u32 identical = 1;
    u32 diff_max_index = 0;
    f32 diff_max = 0.0f;
    for (u32 i = 0; i < ASM_SPHERE_FLOAT_COUNT; ++i)
    {
        /* written so that a NaN on either side fails */
        const f32 diff = F32Abs(f_c[i] - f_asm[i]);
        if (!(diff <= tol[i]))
        {
            printf("ERROR: %s: %s: C %.9g, asm %.9g (difference %.3g, tolerance %.3g)\n", c->id, name[i], (f64) f_c[i], (f64) f_asm[i], (f64) diff, (f64) tol[i]);
            return 0;
        }

        if ((b32) { .f = f_c[i] }.u != (b32) { .f = f_asm[i] }.u)
        {
            identical = 0;
        }

        if (diff > diff_max)
        {
            diff_max = diff;
            diff_max_index = i;
        }
    }

    *bit_identical = identical;
    if (identical)
    {
        printf("PASS:  %s (bit-identical)\n", c->id);
    }
    else
    {
        printf("PASS:  %s (within tolerance: largest difference %.3g in %s)\n", c->id, (f64) diff_max, name[diff_max_index]);
    }

    return 1;
}

int main(void)
{
    struct ds_SolverConfig solver_config = ds_SolverConfigDefault();
    ds_SolverConfigPush(&solver_config);

    printf("layout (byte offsets):\n");
    printf("    c_Shape             sphere.radius %zu\n", offsetof(struct c_Shape, sphere.radius));
    printf("    ds_TransformWorld   rotation %zu (x y z w, f32), position %zu (x y z, f64), size %zu\n", offsetof(ds_TransformWorld, rotation), offsetof(ds_TransformWorld, position), sizeof(ds_TransformWorld));
    printf("    c_Manifold          v %zu, depth %zu, n %zu, v_count %zu, id %zu, tri %zu, size %zu\n", offsetof(struct c_Manifold, v), offsetof(struct c_Manifold, depth), offsetof(struct c_Manifold, n), offsetof(struct c_Manifold, v_count), offsetof(struct c_Manifold, id), offsetof(struct c_Manifold, tri), sizeof(struct c_Manifold));
    printf("    ds_SolverConfig     speculative_distance %zu (g_solver_config points at the struct; value %.9g)\n", offsetof(struct ds_SolverConfig, speculative_distance), (f64) solver_config.speculative_distance);
    printf("    c_PointIdConstant(0) = 0x%08x\n\n", c_PointIdConstant(0).val);

    const q identity = QIdentity();
    const q quarter_z = QUnitAxisAngle(V3(0.0f, 0.0f, 1.0f), F32_PI / 2.0f);
    const q turned0 = QNormalize(Q(0.1f, 0.2f, 0.3f, 0.9f));
    const q turned1 = QNormalize(Q(-0.5f, 0.1f, 0.7f, 0.2f));

    /* speculative_distance is 0.02 m: radii 0.5 and 0.5 touch at 1 m and make a contact up to 1.02 m */
    const struct asm_SphereCase sphere_case[] =
    {
        { .id = "separated (1 m apart)",                    .radius = { 0.5f, 0.5f },       .t = { { .rotation = identity,  .position = V3F64(0.0, 0.0, 0.0) },                 { .rotation = identity, .position = V3F64(2.0, 0.0, 0.0) } } },
        { .id = "beyond the band (3 cm apart)",             .radius = { 0.5f, 0.5f },       .t = { { .rotation = identity,  .position = V3F64(0.0, 0.0, 0.0) },                 { .rotation = identity, .position = V3F64(1.03, 0.0, 0.0) } } },
        { .id = "speculative (1 cm apart)",                 .radius = { 0.5f, 0.5f },       .t = { { .rotation = identity,  .position = V3F64(0.0, 0.0, 0.0) },                 { .rotation = identity, .position = V3F64(1.01, 0.0, 0.0) } } },
        { .id = "touching",                                 .radius = { 0.5f, 0.5f },       .t = { { .rotation = identity,  .position = V3F64(0.0, 0.0, 0.0) },                 { .rotation = identity, .position = V3F64(1.0, 0.0, 0.0) } } },
        { .id = "overlapping (25 cm)",                      .radius = { 0.5f, 0.5f },       .t = { { .rotation = identity,  .position = V3F64(0.0, 0.0, 0.0) },                 { .rotation = identity, .position = V3F64(0.75, 0.0, 0.0) } } },
        { .id = "deep (centres 10 cm apart along y)",       .radius = { 0.5f, 0.5f },       .t = { { .rotation = identity,  .position = V3F64(0.0, 0.0, 0.0) },                 { .rotation = identity, .position = V3F64(0.0, 0.1, 0.0) } } },
        { .id = "coincident centres",                       .radius = { 0.5f, 0.5f },       .t = { { .rotation = identity,  .position = V3F64(1.0, 2.0, 3.0) },                 { .rotation = identity, .position = V3F64(1.0, 2.0, 3.0) } } },
        { .id = "coincident centres, sphere 0 turned",      .radius = { 0.5f, 0.25f },      .t = { { .rotation = turned0,   .position = V3F64(1.0, 2.0, 3.0) },                 { .rotation = identity, .position = V3F64(1.0, 2.0, 3.0) } } },
        { .id = "diagonal offset",                          .radius = { 0.5f, 0.5f },       .t = { { .rotation = identity,  .position = V3F64(1.0, 2.0, 3.0) },                 { .rotation = identity, .position = V3F64(1.3, 2.4, 3.5) } } },
        { .id = "negative offset",                          .radius = { 0.5f, 0.5f },       .t = { { .rotation = identity,  .position = V3F64(0.0, 0.0, 0.0) },                 { .rotation = identity, .position = V3F64(-0.4, -0.5, -0.6) } } },
        { .id = "sphere 0 turned 90 degrees about z",       .radius = { 0.5f, 0.5f },       .t = { { .rotation = quarter_z, .position = V3F64(0.0, 0.0, 0.0) },                 { .rotation = identity, .position = V3F64(0.75, 0.0, 0.0) } } },
        { .id = "both turned",                              .radius = { 0.5f, 0.5f },       .t = { { .rotation = turned0,   .position = V3F64(-1.0, 0.5, 2.0) },                { .rotation = turned1,  .position = V3F64(-0.6, 0.9, 1.7) } } },
        { .id = "radii 2 m and 0.25 m",                     .radius = { 2.0f, 0.25f },      .t = { { .rotation = turned0,   .position = V3F64(0.0, 0.0, 0.0) },                 { .rotation = identity, .position = V3F64(0.0, -2.2, 0.0) } } },
        { .id = "point against sphere (radius 0)",          .radius = { 0.0f, 0.5f },       .t = { { .rotation = identity,  .position = V3F64(0.0, 0.0, 0.0) },                 { .rotation = identity, .position = V3F64(0.3, 0.0, 0.2) } } },
        { .id = "small (radii 1 mm)",                       .radius = { 0.001f, 0.001f },   .t = { { .rotation = turned1,   .position = V3F64(0.0, 0.0, 0.0) },                 { .rotation = identity, .position = V3F64(0.0015, 0.0, 0.0) } } },
        { .id = "large (radii 500 m and 300 m)",            .radius = { 500.0f, 300.0f },   .t = { { .rotation = turned0,   .position = V3F64(0.0, 0.0, 0.0) },                 { .rotation = identity, .position = V3F64(0.0, 0.0, 790.0) } } },
        { .id = "10 km from the origin",                    .radius = { 0.5f, 0.5f },       .t = { { .rotation = turned0,   .position = V3F64(10000.25, -5000.5, 20000.125) },  { .rotation = identity, .position = V3F64(10000.75, -5000.0, 20000.625) } } },
        { .id = "10 000 km from the origin (f64 positions)", .radius = { 0.5f, 0.5f },      .t = { { .rotation = turned1,   .position = V3F64(1.0e7 + 0.1, -1.0e7 - 0.3, 1.0e7 + 0.7) }, { .rotation = identity, .position = V3F64(1.0e7 + 0.7, -1.0e7 - 0.6, 1.0e7 + 0.9) } } },
    };
    const u32 case_count = sizeof(sphere_case) / sizeof(sphere_case[0]);

    printf("c_SphereContact against c_SphereContactHandrolled:\n");
    u32 error_count = 0;
    u32 identical_count = 0;
    for (u32 i = 0; i < case_count; ++i)
    {
        u32 bit_identical;
        const u32 pass = asm_SphereCaseCheck(&bit_identical, sphere_case + i);
        if (!pass)
        {
            error_count += 1;
        }
        else if (bit_identical)
        {
            identical_count += 1;
        }
    }

    printf("\n%u cases: %u bit-identical, %u within tolerance, %u errors\n", case_count, identical_count, case_count - identical_count - error_count, error_count);

    ds_SolverConfigPop();

    return (error_count) ? 1 : 0;
}
