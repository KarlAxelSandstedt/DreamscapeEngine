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
Shapes (c_shape.c, c_test.c): mass properties against closed forms and an f64 reference, and the overlap tests
against the distance functions.
*/

/* relative tolerance of the hull integrals (f32 sums of powers over the faces) */
#define SHAPE_TEST_TOLERANCE 1e-4f

static u32 ShapeTestNear(const f32 a, const f32 b, const f32 scale)
{
    const f32 d = a - b;
    return (d < 0.0f ? -d : d) <= SHAPE_TEST_TOLERANCE * scale;
}

static u32 ShapeTestV3Near(const v3 a, const v3 b, const f32 scale)
{
    return ShapeTestNear(a.x, b.x, scale) && ShapeTestNear(a.y, b.y, scale) && ShapeTestNear(a.z, b.z, scale);
}

/*
 * every entry of the inertia tensor within the tolerance of scale: the largest expected diagonal, or larger when
 * the integrals are taken far from the origin (the tensor about the center of mass is their difference)
 */
static u32 ShapeTestM3Near(const m3 a, const m3 b, const f32 scale_min)
{
    f32 scale = scale_min;
    scale = (b.a11 > scale) ? b.a11 : scale;
    scale = (b.a22 > scale) ? b.a22 : scale;
    scale = (b.a33 > scale) ? b.a33 : scale;
    for (u32 c = 0; c < 3; ++c)
    {
        if (!ShapeTestV3Near(a.col[c], b.col[c], scale))
        {
            return 0;
        }
    }
    return 1;
}

static void ShapeTestPrintM3(const char *name, const m3 m)
{
    fprintf(stderr, "\t%s: [%.6g %.6g %.6g; %.6g %.6g %.6g; %.6g %.6g %.6g]\n", name,
        m.a11, m.a12, m.a13, m.a21, m.a22, m.a23, m.a31, m.a32, m.a33);
}

/* a box of half widths hw, centered, axis aligned: volume 8 hx hy hz, I = V/3 diag(hy^2+hz^2, hx^2+hz^2, hx^2+hy^2) */
static m3 ShapeTestBoxInertia(const v3 hw)
{
    const f32 v = 8.0f * hw.x * hw.y * hw.z;
    m3 i = M3Identity();
    i.a11 = v * (hw.y*hw.y + hw.z*hw.z) / 3.0f;
    i.a22 = v * (hw.x*hw.x + hw.z*hw.z) / 3.0f;
    i.a33 = v * (hw.x*hw.x + hw.y*hw.y) / 3.0f;
    return i;
}

/* DcelBox's hull: the closed forms */
struct test_Output ShapeMassBoxTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    const v3 hw = V3(0.5f, 1.0f, 1.5f);
    struct c_Shape box = { .type = C_SHAPE_CONVEX_HULL };
    box.hull = DcelBox(env->mem_1, hw);
    c_ShapeUpdateMassProperties(&box);

    const f32 volume = 8.0f * hw.x * hw.y * hw.z;
    const m3 inertia = ShapeTestBoxInertia(hw);
    if (!ShapeTestNear(box.volume, volume, volume) || !ShapeTestV3Near(box.center_of_mass, V3Zero(), hw.z) || !ShapeTestM3Near(box.inertia_tensor, inertia, 0.0f))
    {
        fprintf(stderr, "box: volume %.6g (expected %.6g), center (%.6g %.6g %.6g)\n", box.volume, volume, box.center_of_mass.x, box.center_of_mass.y, box.center_of_mass.z);
        ShapeTestPrintM3("inertia", box.inertia_tensor);
        ShapeTestPrintM3("expected", inertia);
        TEST_FAILURE;
    }

    return output;
}

/*
A box rotated by R and moved to t, built with DcelConvexHull from its corners: the volume is unchanged, the center
of mass is t, and the tensor (about the center of mass) is R I R^T.
*/
struct test_Output ShapeMassHullPlacedTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    for (u32 round = 0; round < 8; ++round)
    {
        const v3 hw = V3(RngF32Range(0.2f, 2.0f), RngF32Range(0.2f, 2.0f), RngF32Range(0.2f, 2.0f));
        const v3 t = V3(RngF32Range(-3.0f, 3.0f), RngF32Range(-3.0f, 3.0f), RngF32Range(-3.0f, 3.0f));
        const q rotation = QNormalize(Q(RngF32Range(-1.0f, 1.0f), RngF32Range(-1.0f, 1.0f), RngF32Range(-1.0f, 1.0f), RngF32Range(-1.0f, 1.0f)));
        const m3 r = M3QUnit(rotation);

        v3 corner[8];
        for (u32 i = 0; i < 8; ++i)
        {
            const v3 c = V3((i & 1) ? hw.x : -hw.x, (i & 2) ? hw.y : -hw.y, (i & 4) ? hw.z : -hw.z);
            corner[i] = V3Add(QUnitRotate(rotation, c), t);
        }

        struct c_Shape box = { .type = C_SHAPE_CONVEX_HULL };
        box.hull = DcelConvexHull(env->mem_1, corner, 8);
        TEST_NOT_ZERO(box.hull.f_count == 6);
        c_ShapeUpdateMassProperties(&box);

        const f32 volume = 8.0f * hw.x * hw.y * hw.z;
        const m3 inertia = M3Mul(r, M3Mul(ShapeTestBoxInertia(hw), M3Transpose(r)));
        const f32 extent = V3Length(t) + V3Length(hw);
        /* the integrals are about the origin, so the tensor's error scales with V |t|^2, not with the tensor */
        if (!ShapeTestNear(box.volume, volume, volume) || !ShapeTestV3Near(box.center_of_mass, t, extent) || !ShapeTestM3Near(box.inertia_tensor, inertia, volume * extent * extent))
        {
            fprintf(stderr, "placed box %u: volume %.6g (expected %.6g), center (%.6g %.6g %.6g) (expected %.6g %.6g %.6g)\n", round,
                box.volume, volume, box.center_of_mass.x, box.center_of_mass.y, box.center_of_mass.z, t.x, t.y, t.z);
            ShapeTestPrintM3("inertia", box.inertia_tensor);
            ShapeTestPrintM3("expected", inertia);
            TEST_FAILURE;
        }
    }

    return output;
}

/* a sphere: V = 4/3 pi r^3, I = 2/5 V r^2 */
struct test_Output ShapeMassSphereTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    (void) env;

    const f32 r = 0.7f;
    struct c_Shape sphere = { .type = C_SHAPE_SPHERE, .sphere.radius = r };
    c_ShapeUpdateMassProperties(&sphere);

    const f32 volume = 4.0f * F32_PI * r*r*r / 3.0f;
    const m3 inertia = M3Scale(M3Identity(), 2.0f * volume * r*r / 5.0f);
    if (!ShapeTestNear(sphere.volume, volume, volume) || !ShapeTestV3Near(sphere.center_of_mass, V3Zero(), r) || !ShapeTestM3Near(sphere.inertia_tensor, inertia, 0.0f))
    {
        fprintf(stderr, "sphere: volume %.6g (expected %.6g)\n", sphere.volume, volume);
        ShapeTestPrintM3("inertia", sphere.inertia_tensor);
        ShapeTestPrintM3("expected", inertia);
        TEST_FAILURE;
    }

    return output;
}

static f64 ShapeTestF64Sqrt(const f64 a)
{
    f64 x = (f64) F32Sqrt((f32) a);
    if (x > 0.0)
    {
        x = 0.5 * (x + a / x);
        x = 0.5 * (x + a / x);
    }
    return x;
}

/*
Capsule along y (half height h, radius r) at density 1, by slices: a slice at y is a disk of radius rr with
area A = pi rr^2 and second moment pi rr^4 / 4 about a diameter, so
    V = int A dy,  I_yy = int pi rr^4 / 2 dy,  I_xx = I_zz = int (pi rr^4 / 4 + A y^2) dy.
Closed form, for the record: the cylinder's I_xx = m (r^2/4 + h^2/3), I_yy = m r^2 / 2 with m = 2 pi r^2 h; each
cap I_xx = m_h (2/5 r^2 + h^2 + 3 h r / 4), I_yy = 2/5 m_h r^2 with m_h = 2/3 pi r^3.
*/
static void ShapeTestCapsuleReference(f64 *volume, f64 *i_xx, f64 *i_yy, const f64 r, const f64 h)
{
    const f64 pi = 3.14159265358979323846;
    const u32 n = 20000;
    const f64 dy = 2.0 * (h + r) / n;
    f64 v = 0.0, xx = 0.0, yy = 0.0;
    for (u32 i = 0; i < n; ++i)
    {
        const f64 y = -(h + r) + (i + 0.5) * dy;
        const f64 ay = (y < 0.0) ? -y : y;
        f64 rr = r;
        if (ay > h)
        {
            const f64 d = ay - h;
            rr = (r*r > d*d) ? ShapeTestF64Sqrt(r*r - d*d) : 0.0;
        }
        const f64 area = pi * rr * rr;
        const f64 second = pi * rr*rr*rr*rr / 4.0;
        v += area * dy;
        yy += 2.0 * second * dy;
        xx += (second + area * y*y) * dy;
    }
    *volume = v;
    *i_xx = xx;
    *i_yy = yy;
}

/* a capsule against the f64 slice integration */
struct test_Output ShapeMassCapsuleTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    (void) env;

    const f32 r[] = { 0.5f, 1.0f, 0.3f };
    const f32 h[] = { 1.0f, 0.25f, 2.0f };
    for (u32 i = 0; i < 3; ++i)
    {
        struct c_Shape capsule = { .type = C_SHAPE_CAPSULE, .capsule.radius = r[i], .capsule.half_height = h[i] };
        c_ShapeUpdateMassProperties(&capsule);

        f64 volume, i_xx, i_yy;
        ShapeTestCapsuleReference(&volume, &i_xx, &i_yy, r[i], h[i]);
        m3 inertia = M3Identity();
        inertia.a11 = (f32) i_xx;
        inertia.a22 = (f32) i_yy;
        inertia.a33 = (f32) i_xx;
        if (!ShapeTestNear(capsule.volume, (f32) volume, (f32) volume) || !ShapeTestV3Near(capsule.center_of_mass, V3Zero(), r[i]) || !ShapeTestM3Near(capsule.inertia_tensor, inertia, 0.0f))
        {
            fprintf(stderr, "capsule r %g h %g: volume %.6g (expected %.6g)\n", r[i], h[i], capsule.volume, volume);
            ShapeTestPrintM3("inertia", capsule.inertia_tensor);
            ShapeTestPrintM3("expected", inertia);
            TEST_FAILURE;
        }
    }

    return output;
}

/********************************** overlap tests **********************************/

static ds_TransformWorld ShapeTestPlacement(const f32 range)
{
    return (ds_TransformWorld)
    {
        .rotation = QNormalize(Q(RngF32Range(-1.0f, 1.0f), RngF32Range(-1.0f, 1.0f), RngF32Range(-1.0f, 1.0f), RngF32Range(-1.0f, 1.0f))),
        .position = V3F64(RngF32Range(-range, range), RngF32Range(-range, range), RngF32Range(-range, range)),
    };
}

static struct c_Shape ShapeTestRandom(struct arena *mem, const enum c_ShapeType type)
{
    struct c_Shape s = { .type = type };
    switch (type)
    {
        case C_SHAPE_SPHERE:
        {
            s.sphere.radius = RngF32Range(0.1f, 1.5f);
        } break;
        case C_SHAPE_CAPSULE:
        {
            s.capsule.radius = RngF32Range(0.1f, 1.0f);
            s.capsule.half_height = RngF32Range(0.0f, 1.5f);
        } break;
        default:
        {
            s.hull = DcelBox(mem, V3(RngF32Range(0.1f, 1.5f), RngF32Range(0.1f, 1.5f), RngF32Range(0.1f, 1.5f)));
        } break;
    }
    return s;
}

/*
Each c_<S0><S1>Test agrees with c_<S0><S1>Distance: overlapping or touching iff the distance is 0. The placements
make about half of the pairs overlap.
*/
struct test_Output ShapeOverlapTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    struct ds_NumericsConfig config = ds_NumericsConfigDefault();
    ds_NumericsConfigPush(&config);

    typedef u32 (*test_func)(const struct c_Shape *, const ds_TransformWorld *, const struct c_Shape *, const ds_TransformWorld *);
    typedef f32 (*distance_func)(v3f64 *, v3f64 *, const struct c_Shape *, const ds_TransformWorld *, const struct c_Shape *, const ds_TransformWorld *);
    const struct { const char *id; enum c_ShapeType t0, t1; test_func test; distance_func distance; } pair[] =
    {
        { "sphere-sphere",   C_SHAPE_SPHERE,      C_SHAPE_SPHERE,  c_SphereTest,        c_SphereDistance },
        { "capsule-sphere",  C_SHAPE_CAPSULE,     C_SHAPE_SPHERE,  c_CapsuleSphereTest, c_CapsuleSphereDistance },
        { "capsule-capsule", C_SHAPE_CAPSULE,     C_SHAPE_CAPSULE, c_CapsuleTest,       c_CapsuleDistance },
        { "hull-sphere",     C_SHAPE_CONVEX_HULL, C_SHAPE_SPHERE,  c_HullSphereTest,    c_HullSphereDistance },
        { "hull-capsule",    C_SHAPE_CONVEX_HULL, C_SHAPE_CAPSULE, c_HullCapsuleTest,   c_HullCapsuleDistance },
        { "hull-hull",       C_SHAPE_CONVEX_HULL, C_SHAPE_CONVEX_HULL, c_HullTest,      c_HullDistance },
    };

    for (u32 p = 0; p < sizeof(pair) / sizeof(pair[0]); ++p)
    {
        u32 overlaps = 0;
        const u32 rounds = 400;
        for (u32 round = 0; round < rounds; ++round)
        {
            ArenaPushRecord(env->mem_1);
            const struct c_Shape s0 = ShapeTestRandom(env->mem_1, pair[p].t0);
            const struct c_Shape s1 = ShapeTestRandom(env->mem_1, pair[p].t1);
            const ds_TransformWorld t0 = ShapeTestPlacement(1.0f);
            const ds_TransformWorld t1 = ShapeTestPlacement(2.0f);

            v3f64 c0, c1;
            const f32 d = pair[p].distance(&c0, &c1, &s0, &t0, &s1, &t1);
            const u32 overlap = pair[p].test(&s0, &t0, &s1, &t1);
            overlaps += overlap;
            if (overlap != (d == 0.0f))
            {
                fprintf(stderr, "%s round %u: test %u, distance %.9g\n", pair[p].id, round, overlap, d);
                ArenaPopRecord(env->mem_1);
                ds_NumericsConfigPop();
                TEST_FAILURE;
            }
            ArenaPopRecord(env->mem_1);
        }
        fprintf(stdout, "\t%s: %u of %u overlap\n", pair[p].id, overlaps, rounds);
    }

    ds_NumericsConfigPop();
    return output;
}

static struct test_Output (*shape_tests[])(struct test_Environment *) =
{
    ShapeMassBoxTest,
    ShapeMassHullPlacedTest,
    ShapeMassSphereTest,
    ShapeMassCapsuleTest,
    ShapeOverlapTest,
};

struct suite_Correctness m_shape_suite =
{
    .id = "Shape",
    .unit_test = shape_tests,
    .unit_test_count = sizeof(shape_tests) / sizeof(shape_tests[0]),
};

struct suite_Correctness *shape_correctness_suite = &m_shape_suite;
