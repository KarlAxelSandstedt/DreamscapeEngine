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
    if (!ShapeTestNear(box.volume, volume, volume) || !ShapeTestV3Near(box.center_of_mass, V3Zero(), hw.z) || !ShapeTestM3Near(box.inertia_tensor, inertia, 0.0f) || !ShapeTestNear(box.center_of_mass_surface_distance, hw.x, hw.z))
    {
        fprintf(stderr, "box: volume %.6g (expected %.6g), center (%.6g %.6g %.6g), surface distance %.6g (expected %.6g)\n", box.volume, volume, box.center_of_mass.x, box.center_of_mass.y, box.center_of_mass.z, box.center_of_mass_surface_distance, hw.x);
        ShapeTestPrintM3("inertia", box.inertia_tensor);
        ShapeTestPrintM3("expected", inertia);
        TEST_FAILURE;
    }

    /* the stub: the unit cube */
    struct c_Shape stub = { .type = C_SHAPE_CONVEX_HULL };
    stub.hull = DcelBoxStub();
    c_ShapeUpdateMassProperties(&stub);
    if (!ShapeTestNear(stub.volume, 1.0f, 1.0f) || !ShapeTestNear(stub.center_of_mass_surface_distance, 0.5f, 0.5f))
    {
        fprintf(stderr, "stub: volume %.6g (expected 1), surface distance %.6g (expected 0.5)\n", stub.volume, stub.center_of_mass_surface_distance);
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
        const f32 surface_distance = F32Min(F32Min(hw.x, hw.y), hw.z);
        if (!ShapeTestNear(box.volume, volume, volume) || !ShapeTestV3Near(box.center_of_mass, t, extent) || !ShapeTestM3Near(box.inertia_tensor, inertia, volume * extent * extent) || !ShapeTestNear(box.center_of_mass_surface_distance, surface_distance, extent))
        {
            fprintf(stderr, "placed box %u: volume %.6g (expected %.6g), center (%.6g %.6g %.6g) (expected %.6g %.6g %.6g), surface distance %.6g (expected %.6g)\n", round,
                box.volume, volume, box.center_of_mass.x, box.center_of_mass.y, box.center_of_mass.z, t.x, t.y, t.z, box.center_of_mass_surface_distance, surface_distance);
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
    if (!ShapeTestNear(sphere.volume, volume, volume) || !ShapeTestV3Near(sphere.center_of_mass, V3Zero(), r) || !ShapeTestM3Near(sphere.inertia_tensor, inertia, 0.0f) || sphere.center_of_mass_surface_distance != r)
    {
        fprintf(stderr, "sphere: volume %.6g (expected %.6g), surface distance %.6g (expected %.6g)\n", sphere.volume, volume, sphere.center_of_mass_surface_distance, r);
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
        if (!ShapeTestNear(capsule.volume, (f32) volume, (f32) volume) || !ShapeTestV3Near(capsule.center_of_mass, V3Zero(), r[i]) || !ShapeTestM3Near(capsule.inertia_tensor, inertia, 0.0f) || capsule.center_of_mass_surface_distance != r[i])
        {
            fprintf(stderr, "capsule r %g h %g: volume %.6g (expected %.6g), surface distance %.6g (expected %.6g)\n", r[i], h[i], capsule.volume, volume, capsule.center_of_mass_surface_distance, r[i]);
            ShapeTestPrintM3("inertia", capsule.inertia_tensor);
            ShapeTestPrintM3("expected", inertia);
            TEST_FAILURE;
        }
    }

    return output;
}

/*
center_of_mass_surface_distance of random hulls against an f64 reference: the smallest distance from the center of
mass to the faces' planes, each plane from the face's vertices (Newell's normal through their centroid), not from
the stored planes.
*/
struct test_Output ShapeSurfaceDistanceHullTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    for (u32 round = 0; round < 200; ++round)
    {
        const struct arena record = *env->mem_1;
        const v3 hw = V3(RngF32Range(0.1f, 3.0f), RngF32Range(0.1f, 3.0f), RngF32Range(0.1f, 3.0f));
        const v3 t = V3(RngF32Range(-5.0f, 5.0f), RngF32Range(-5.0f, 5.0f), RngF32Range(-5.0f, 5.0f));
        v3 point[24];
        for (u32 i = 0; i < 24; ++i)
        {
            const f32 x = RngF32Range(-hw.x, hw.x);
            const f32 y = RngF32Range(-hw.y, hw.y);
            const f32 z = RngF32Range(-hw.z, hw.z);
            point[i] = V3Add(V3(x, y, z), t);
        }

        struct c_Shape hull = { .type = C_SHAPE_CONVEX_HULL };
        hull.hull = DcelConvexHull(env->mem_1, point, 24);
        c_ShapeUpdateMassProperties(&hull);

        f64 reference = 1e30;
        for (u32 fi = 0; fi < hull.hull.f_count; ++fi)
        {
            const struct dcelFace *face = hull.hull.f + fi;
            f64 n[3] = { 0.0, 0.0, 0.0 };
            f64 c[3] = { 0.0, 0.0, 0.0 };
            for (u32 k = 0; k < face->count; ++k)
            {
                const v3 a = hull.hull.v[hull.hull.e[face->first + k].origin];
                const v3 b = hull.hull.v[hull.hull.e[face->first + (k + 1) % face->count].origin];
                n[0] += ((f64) a.y - (f64) b.y) * ((f64) a.z + (f64) b.z);
                n[1] += ((f64) a.z - (f64) b.z) * ((f64) a.x + (f64) b.x);
                n[2] += ((f64) a.x - (f64) b.x) * ((f64) a.y + (f64) b.y);
                c[0] += (f64) a.x;
                c[1] += (f64) a.y;
                c[2] += (f64) a.z;
            }
            const f64 length = ShapeTestF64Sqrt(n[0]*n[0] + n[1]*n[1] + n[2]*n[2]);
            const f64 d = ((f64) hull.center_of_mass.x - c[0] / face->count) * n[0] + ((f64) hull.center_of_mass.y - c[1] / face->count) * n[1] + ((f64) hull.center_of_mass.z - c[2] / face->count) * n[2];
            const f64 distance = ((d < 0.0) ? -d : d) / length;
            reference = (distance < reference) ? distance : reference;
        }

        const f32 scale = V3Length(t) + V3Length(hw);
        if (!(hull.center_of_mass_surface_distance > 0.0f) || !ShapeTestNear(hull.center_of_mass_surface_distance, (f32) reference, scale))
        {
            fprintf(stderr, "hull %u (%u faces): surface distance %.9g (expected %.9g)\n", round, hull.hull.f_count, hull.center_of_mass_surface_distance, reference);
            TEST_FAILURE;
        }

        *env->mem_1 = record;
    }

    return output;
}

/* a random unit vector, components drawn one statement at a time */
static v3 ShapeTestUnit(void)
{
    while (1)
    {
        const f32 x = RngF32Range(-1.0f, 1.0f);
        const f32 y = RngF32Range(-1.0f, 1.0f);
        const f32 z = RngF32Range(-1.0f, 1.0f);
        const v3 u = V3(x, y, z);
        if (V3Length(u) >= 0.1f)
        {
            return V3Normalize(u);
        }
    }
}

/*
c_ShapeBbox at random transforms: a hull's box is the box of its vertices rotated by QUnitRotate and moved; a
sphere's is p +- r; a capsule's the box of its segment's ends +- r; random surface points of the sphere and the
capsule lie inside their boxes.
*/
struct test_Output ShapeBboxTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    for (u32 round = 0; round < 200; ++round)
    {
        const struct arena record = *env->mem_1;
        const f32 qx = RngF32Range(-1.0f, 1.0f);
        const f32 qy = RngF32Range(-1.0f, 1.0f);
        const f32 qz = RngF32Range(-1.0f, 1.0f);
        const f32 qw = RngF32Range(-1.0f, 1.0f);
        const q rotation = QNormalize(Q(qx, qy, qz, qw));
        const v3 position = V3(RngF32Range(-10.0f, 10.0f), RngF32Range(-10.0f, 10.0f), RngF32Range(-10.0f, 10.0f));
        const ds_Transform transform = { .rotation = rotation, .position = position };

        v3 point[16];
        for (u32 i = 0; i < 16; ++i)
        {
            const f32 x = RngF32Range(-2.0f, 2.0f);
            const f32 y = RngF32Range(-2.0f, 2.0f);
            const f32 z = RngF32Range(-2.0f, 2.0f);
            point[i] = V3(x, y, z);
        }
        struct c_Shape hull = { .type = C_SHAPE_CONVEX_HULL };
        hull.hull = DcelConvexHull(env->mem_1, point, 16);
        v3 hull_min = V3(F32_INFINITY, F32_INFINITY, F32_INFINITY);
        v3 hull_max = V3(-F32_INFINITY, -F32_INFINITY, -F32_INFINITY);
        for (u32 i = 0; i < hull.hull.v_count; ++i)
        {
            const v3 v = V3Add(QUnitRotate(rotation, hull.hull.v[i]), position);
            hull_min = V3Min(hull_min, v);
            hull_max = V3Max(hull_max, v);
        }
        const struct aabb hull_box = c_ShapeBbox(&hull, &transform);
        const f32 scale = V3Length(position) + 4.0f;
        if (!ShapeTestV3Near(hull_box.min, hull_min, scale) || !ShapeTestV3Near(hull_box.max, hull_max, scale))
        {
            fprintf(stderr, "hull %u: box (%.6g %.6g %.6g) (%.6g %.6g %.6g), expected (%.6g %.6g %.6g) (%.6g %.6g %.6g)\n", round, hull_box.min.x, hull_box.min.y, hull_box.min.z, hull_box.max.x, hull_box.max.y, hull_box.max.z, hull_min.x, hull_min.y, hull_min.z, hull_max.x, hull_max.y, hull_max.z);
            TEST_FAILURE;
        }

        const f32 r = RngF32Range(0.05f, 2.0f);
        const f32 h = RngF32Range(0.05f, 2.0f);
        const struct c_Shape sphere = { .type = C_SHAPE_SPHERE, .sphere.radius = r };
        const struct c_Shape capsule = { .type = C_SHAPE_CAPSULE, .capsule.radius = r, .capsule.half_height = h };
        const struct aabb sphere_box = c_ShapeBbox(&sphere, &transform);
        const struct aabb capsule_box = c_ShapeBbox(&capsule, &transform);

        const v3 radius = V3(r, r, r);
        const v3 end_0 = V3Add(QUnitRotate(rotation, V3(0.0f, h, 0.0f)), position);
        const v3 end_1 = V3Add(QUnitRotate(rotation, V3(0.0f, -h, 0.0f)), position);
        TEST_TRUE(ShapeTestV3Near(sphere_box.min, V3Sub(position, radius), scale) && ShapeTestV3Near(sphere_box.max, V3Add(position, radius), scale));
        TEST_TRUE(ShapeTestV3Near(capsule_box.min, V3Sub(V3Min(end_0, end_1), radius), scale) && ShapeTestV3Near(capsule_box.max, V3Add(V3Max(end_0, end_1), radius), scale));

        const f32 tolerance = SHAPE_TEST_TOLERANCE * scale;
        for (u32 k = 0; k < 32; ++k)
        {
            const v3 u = ShapeTestUnit();
            const f32 s = RngF32Range(-1.0f, 1.0f);
            const v3 on_sphere = V3AddScaled(position, u, r);
            const v3 on_capsule = V3AddScaled(V3Add(QUnitRotate(rotation, V3(0.0f, s * h, 0.0f)), position), u, r);
            for (u32 c = 0; c < 3; ++c)
            {
                TEST_TRUE(sphere_box.min.buf[c] - tolerance <= on_sphere.buf[c] && on_sphere.buf[c] <= sphere_box.max.buf[c] + tolerance);
                TEST_TRUE(capsule_box.min.buf[c] - tolerance <= on_capsule.buf[c] && on_capsule.buf[c] <= capsule_box.max.buf[c] + tolerance);
            }
        }

        *env->mem_1 = record;
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


/********************************** distances **********************************/

struct st_d3 { f64 x, y, z; };
static struct st_d3 StD3(const v3 v) { return (struct st_d3) { v.x, v.y, v.z }; }
static struct st_d3 StD3F64(const v3f64 v) { return (struct st_d3) { v.x, v.y, v.z }; }
static struct st_d3 StSub(const struct st_d3 a, const struct st_d3 b) { return (struct st_d3) { a.x - b.x, a.y - b.y, a.z - b.z }; }
static struct st_d3 StAdd(const struct st_d3 a, const struct st_d3 b) { return (struct st_d3) { a.x + b.x, a.y + b.y, a.z + b.z }; }
static struct st_d3 StScale(const struct st_d3 a, const f64 s) { return (struct st_d3) { a.x*s, a.y*s, a.z*s }; }
static f64 StDot(const struct st_d3 a, const struct st_d3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
static f64 StLength(const struct st_d3 a) { return ShapeTestF64Sqrt(StDot(a, a)); }
static f64 StClamp01(const f64 a) { return (a < 0.0) ? 0.0 : (a > 1.0) ? 1.0 : a; }

/* f64 closest points of two segments (Ericson 5.1.9); returns their distance */
static f64 StSegmentDistance(const struct st_d3 p1, const struct st_d3 q1, const struct st_d3 p2, const struct st_d3 q2)
{
    const struct st_d3 d1 = StSub(q1, p1), d2 = StSub(q2, p2), r = StSub(p1, p2);
    const f64 a = StDot(d1, d1), e = StDot(d2, d2), f = StDot(d2, r);
    f64 s = 0.0, t = 0.0;
    if (a <= 1e-300 && e <= 1e-300)
    {
        return StLength(r);
    }
    if (a <= 1e-300)
    {
        t = StClamp01(f / e);
    }
    else
    {
        const f64 c = StDot(d1, r);
        if (e <= 1e-300)
        {
            s = StClamp01(-c / a);
        }
        else
        {
            const f64 b = StDot(d1, d2), denom = a*e - b*b;
            s = (denom != 0.0) ? StClamp01((b*f - c*e) / denom) : 0.0;
            t = (b*s + f) / e;
            if (t < 0.0) { t = 0.0; s = StClamp01(-c / a); }
            else if (t > 1.0) { t = 1.0; s = StClamp01((b - c) / a); }
        }
    }
    return StLength(StSub(StAdd(p1, StScale(d1, s)), StAdd(p2, StScale(d2, t))));
}

/* the axis segment of a capsule in world f64 */
static void StCapsuleSegment(struct st_d3 *p, struct st_d3 *q, const struct c_Shape *s, const ds_TransformWorld *t)
{
    const v3 axis = QUnitRotate(t->rotation, V3(0.0f, s->capsule.half_height, 0.0f));
    *p = StAdd(StD3F64(t->position), StD3(axis));
    *q = StSub(StD3F64(t->position), StD3(axis));
}

/* f64 distance between the surfaces of two spheres or capsules (negative when they overlap) */
static f64 StCoreDistance(const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1)
{
    struct st_d3 p0, q0, p1, q1;
    f64 r0, r1;
    if (s0->type == C_SHAPE_SPHERE) { p0 = q0 = StD3F64(t0->position); r0 = s0->sphere.radius; }
    else { StCapsuleSegment(&p0, &q0, s0, t0); r0 = s0->capsule.radius; }
    if (s1->type == C_SHAPE_SPHERE) { p1 = q1 = StD3F64(t1->position); r1 = s1->sphere.radius; }
    else { StCapsuleSegment(&p1, &q1, s1, t1); r1 = s1->capsule.radius; }
    return StSegmentDistance(p0, q0, p1, q1) - r0 - r1;
}

/* how far a world point is from a shape's surface, in its local frame: 0 on the surface, < 0 inside */
static f32 StSurfaceDistance(const struct c_Shape *s, const ds_TransformWorld *t, const v3f64 p_world)
{
    const v3 p = V3F64InverseTransform(p_world, t);
    switch (s->type)
    {
        case C_SHAPE_SPHERE:
        {
            return V3Length(p) - s->sphere.radius;
        }
        case C_SHAPE_CAPSULE:
        {
            const f32 y = F32Clamp(p.y, -s->capsule.half_height, s->capsule.half_height);
            return V3Length(V3Sub(p, V3(0.0f, y, 0.0f))) - s->capsule.radius;
        }
        default:
        {
            f32 best = -F32_INFINITY;
            for (u32 fi = 0; fi < s->hull.f_count; ++fi)
            {
                const f32 d = V3Dot(s->hull.plane[fi].normal, p) - s->hull.plane[fi].signed_distance;
                best = (d > best) ? d : best;
            }
            return best;
        }
    }
}

/*
Each c_<S0><S1>Distance: the sphere and capsule pairs against an f64 reference, and for every separated pair the
witness points lie on their shapes' surfaces and are the distance apart.
*/
struct test_Output ShapeDistanceTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    struct ds_NumericsConfig config = ds_NumericsConfigDefault();
    ds_NumericsConfigPush(&config);

    typedef f32 (*distance_func)(v3f64 *, v3f64 *, const struct c_Shape *, const ds_TransformWorld *, const struct c_Shape *, const ds_TransformWorld *);
    const struct { const char *id; enum c_ShapeType t0, t1; distance_func distance; } pair[] =
    {
        { "sphere-sphere",   C_SHAPE_SPHERE,      C_SHAPE_SPHERE,  c_SphereDistance },
        { "capsule-sphere",  C_SHAPE_CAPSULE,     C_SHAPE_SPHERE,  c_CapsuleSphereDistance },
        { "capsule-capsule", C_SHAPE_CAPSULE,     C_SHAPE_CAPSULE, c_CapsuleDistance },
        { "hull-sphere",     C_SHAPE_CONVEX_HULL, C_SHAPE_SPHERE,  c_HullSphereDistance },
        { "hull-capsule",    C_SHAPE_CONVEX_HULL, C_SHAPE_CAPSULE, c_HullCapsuleDistance },
        { "hull-hull",       C_SHAPE_CONVEX_HULL, C_SHAPE_CONVEX_HULL, c_HullDistance },
    };

    for (u32 p = 0; p < sizeof(pair) / sizeof(pair[0]); ++p)
    {
        u32 separated = 0;
        f32 worst_surface = 0.0f, worst_length = 0.0f;
        const u32 rounds = 400;
        for (u32 round = 0; round < rounds; ++round)
        {
            ArenaPushRecord(env->mem_1);
            const struct c_Shape s0 = ShapeTestRandom(env->mem_1, pair[p].t0);
            const struct c_Shape s1 = ShapeTestRandom(env->mem_1, pair[p].t1);
            const ds_TransformWorld t0 = ShapeTestPlacement(1.0f);
            const ds_TransformWorld t1 = ShapeTestPlacement(2.0f);
            const f32 scale = 8.0f;	/* positions to 2, extents to 1.5 */
            const f32 tol = 1e-4f * scale;

            v3f64 c0, c1;
            const f32 d = pair[p].distance(&c0, &c1, &s0, &t0, &s1, &t1);
            u32 ok = (d >= 0.0f && d == d);
            if (pair[p].t0 != C_SHAPE_CONVEX_HULL && pair[p].t1 != C_SHAPE_CONVEX_HULL)
            {
                const f64 reference = StCoreDistance(&s0, &t0, &s1, &t1);
                ok = ok && ((reference <= 0.0) ? (d == 0.0f) : (F32Abs((f32) reference - d) <= tol));
                if (!ok)
                {
                    fprintf(stderr, "%s round %u: distance %.9g, reference %.9g\n", pair[p].id, round, d, reference);
                }
            }
            if (ok && d > 0.0f)
            {
                separated += 1;
                const f32 on0 = StSurfaceDistance(&s0, &t0, c0);
                const f32 on1 = StSurfaceDistance(&s1, &t1, c1);
                const f32 length = (f32) StLength(StSub(StD3F64(c1), StD3F64(c0)));
                worst_surface = F32Max(worst_surface, F32Max(F32Abs(on0), F32Abs(on1)));
                worst_length = F32Max(worst_length, F32Abs(length - d));
                if (F32Abs(on0) > tol || F32Abs(on1) > tol || F32Abs(length - d) > tol)
                {
                    fprintf(stderr, "%s round %u: distance %.9g, witness on surfaces %.3g %.3g, witnesses %.9g apart\n", pair[p].id, round, d, on0, on1, length);
                    ok = 0;
                }
            }
            ArenaPopRecord(env->mem_1);
            if (!ok)
            {
                ds_NumericsConfigPop();
                TEST_FAILURE;
            }
        }
        fprintf(stdout, "\t%s: %u of %u separated; witness error %.3g on the surfaces, %.3g in length\n", pair[p].id, separated, rounds, worst_surface, worst_length);
    }

    ds_NumericsConfigPop();
    return output;
}

/*
 * c_ShapeDB: the stub is the unit box; Add copies the shape, computes mass properties and returns the slot, the
 * stub's slot for an empty, present or over-long id; Remove refuses a referenced shape and removes it once
 * dereferenced; the copies survive the database growing and the sources dying.
 */
static struct test_Output ShapeDatabaseTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    struct c_ShapeDB db = { 0 };
    c_ShapeDBAlloc(NULL, &db, 4, GROWABLE);
    const struct c_Shape *stub = c_ShapeDBShape(&db, SDB_STUB);
    TEST_EQUAL(C_SHAPE_CONVEX_HULL, stub->type);
    TEST_TRUE(ShapeTestNear(stub->volume, 1.0f, 1.0f));
    TEST_EQUAL(SDB_STUB, c_ShapeDBLookup(&db, Utf8Inline("sphere")).index);

    const f32 r = 0.5f;
    const struct c_Shape sphere = { .type = C_SHAPE_SPHERE, .sphere.radius = r };
    const struct slot s = c_ShapeDBAdd(&db, Utf8Inline("sphere"), &sphere);
    TEST_TRUE(s.index != SDB_STUB && s.index != U32_MAX);
    TEST_EQUAL(s.index, c_ShapeDBLookup(&db, Utf8Inline("sphere")).index);
    const struct c_Shape *added = c_ShapeDBShape(&db, s.index);
    TEST_EQUAL(C_SHAPE_SPHERE, added->type);
    TEST_TRUE(Utf8Equivalence(Utf8Inline("sphere"), added->id));
    const f32 volume = 4.0f / 3.0f * F32_PI * r * r * r;
    TEST_TRUE(ShapeTestNear(added->volume, volume, volume));

    /* the stub's slot for a bad id */
    TEST_EQUAL(SDB_STUB, c_ShapeDBAdd(&db, Utf8Empty(), &sphere).index);
    TEST_EQUAL(SDB_STUB, c_ShapeDBAdd(&db, Utf8Inline("sphere"), &sphere).index);
    u8 long_buf[C_SHAPE_ID_SIZE + 8];
    memset(long_buf, 'x', sizeof(long_buf));
    const utf8 long_id = { .buf = long_buf, .size = sizeof(long_buf), .len = sizeof(long_buf) };
    TEST_EQUAL(SDB_STUB, c_ShapeDBAdd(&db, long_id, &sphere).index);
    TEST_EQUAL(2, db.sdb.pool.count);

    /* the stub's slot for a shape c_ShapeValidate rejects: no volume, or a mesh without triangles */
    const struct c_Shape flat_sphere = { .type = C_SHAPE_SPHERE, .sphere.radius = 0.0f };
    TEST_EQUAL(SDB_STUB, c_ShapeDBAdd(&db, Utf8Inline("flat_sphere"), &flat_sphere).index);
    const struct c_Shape nan_sphere = { .type = C_SHAPE_SPHERE, .sphere.radius = F32Nan() };
    TEST_EQUAL(SDB_STUB, c_ShapeDBAdd(&db, Utf8Inline("nan_sphere"), &nan_sphere).index);
    const struct c_Shape thin_capsule = { .type = C_SHAPE_CAPSULE, .capsule = { .half_height = 1.0f, .radius = 0.0f } };
    TEST_EQUAL(SDB_STUB, c_ShapeDBAdd(&db, Utf8Inline("thin_capsule"), &thin_capsule).index);
    const struct c_Shape empty_hull = { .type = C_SHAPE_CONVEX_HULL, .hull = DcelEmpty() };
    TEST_EQUAL(SDB_STUB, c_ShapeDBAdd(&db, Utf8Inline("empty_hull"), &empty_hull).index);
    struct c_Shape empty_mesh = { .type = C_SHAPE_TRI_MESH };
    MeshConstruct(env->mem_1, &empty_mesh.mesh, 8);
    TEST_EQUAL(SDB_STUB, c_ShapeDBAdd(&db, Utf8Inline("empty_mesh"), &empty_mesh).index);
    TEST_EQUAL(2, db.sdb.pool.count);

    /* referenced: not removed; dereferenced: removed */
    TEST_EQUAL(s.index, c_ShapeDBReference(&db, Utf8Inline("sphere")).index);
    TEST_EQUAL(0, c_ShapeDBRemove(&db, Utf8Inline("sphere")));
    TEST_EQUAL(s.index, c_ShapeDBLookup(&db, Utf8Inline("sphere")).index);
    c_ShapeDBDereference(&db, s.index);
    TEST_EQUAL(1, c_ShapeDBRemove(&db, Utf8Inline("sphere")));
    TEST_EQUAL(SDB_STUB, c_ShapeDBLookup(&db, Utf8Inline("sphere")).index);
    TEST_EQUAL(1, db.sdb.pool.count);

    /* the slots own their copies: grow past the initial length, then clobber the sources and look everything up */
    const u32 count = 12;
    u32 index[12];
    for (u32 i = 0; i < count; ++i)
    {
        const v3 hw = V3(0.5f + (f32) i, 0.5f, 0.25f);
        struct c_Shape box = { .type = C_SHAPE_CONVEX_HULL, .hull = DcelBox(env->mem_1, hw) };
        index[i] = c_ShapeDBAdd(&db, Utf8Format(env->mem_1, "box_%u", i), &box).index;
        TEST_TRUE(index[i] != SDB_STUB);
        memset(box.hull.v, 0xff, box.hull.v_count*sizeof(v3));
        memset(box.hull.plane, 0xff, box.hull.f_count*sizeof(struct plane));
    }
    for (u32 i = 0; i < count; ++i)
    {
        const utf8 id = Utf8Format(env->mem_1, "box_%u", i);
        const struct slot found = c_ShapeDBLookup(&db, id);
        TEST_EQUAL(index[i], found.index);
        const struct c_Shape *shape = c_ShapeDBShape(&db, found.index);
        TEST_TRUE(Utf8Equivalence(id, shape->id));
        TEST_EQUAL(8, shape->hull.v_count);
        TEST_TRUE(ShapeTestNear(shape->volume, 8.0f * (0.5f + (f32) i) * 0.5f * 0.25f, 1.0f));
        TEST_TRUE(ShapeTestNear(shape->hull.bbox.max.x, 0.5f + (f32) i, 1.0f));
        TEST_TRUE(shape->render_mesh == NULL);
    }
    TEST_EQUAL(1, c_ShapeDBRemove(&db, Utf8Inline("box_3")));
    TEST_EQUAL(SDB_STUB, c_ShapeDBLookup(&db, Utf8Inline("box_3")).index);
    TEST_EQUAL(index[4], c_ShapeDBLookup(&db, Utf8Inline("box_4")).index);

    /* a mesh's copy owns its vertices, triangles, leaf order and nodes */
    const v3 quad_v[4] = { V3(0.0f, 0.0f, 0.0f), V3(0.0f, 0.0f, 1.0f), V3(1.0f, 0.0f, 0.0f), V3(1.0f, 0.0f, 1.0f) };
    const v3u32 quad_tri[2] = { { .buf = { 0, 1, 2 } }, { .buf = { 2, 1, 3 } } };
    struct c_Shape quad = { .type = C_SHAPE_TRI_MESH, .mesh = { .v_count = 4, .tri_count = 2 } };
    quad.mesh.v = ArenaPush(env->mem_1, sizeof(quad_v));
    quad.mesh.tri = ArenaPush(env->mem_1, sizeof(quad_tri));
    memcpy(quad.mesh.v, quad_v, sizeof(quad_v));
    memcpy(quad.mesh.tri, quad_tri, sizeof(quad_tri));
    MeshConstruct(env->mem_1, &quad.mesh, 8);
    const u32 quad_index = c_ShapeDBAdd(&db, Utf8Inline("quad"), &quad).index;
    TEST_TRUE(quad_index != SDB_STUB);
    memset(quad.mesh.v, 0xff, sizeof(quad_v));
    memset(quad.mesh.tri, 0xff, sizeof(quad_tri));
    memset(quad.mesh.leaf_tri, 0xff, quad.mesh.tri_count*sizeof(u32));
    memset(quad.mesh.bvh.pool.buf, 0xff, quad.mesh.bvh.pool.count_max*sizeof(struct BVHNode));
    const struct mesh *copy = &c_ShapeDBShape(&db, quad_index)->mesh;
    TEST_EQUAL(4, copy->v_count);
    TEST_EQUAL(2, copy->tri_count);
    for (u32 i = 0; i < 4; ++i)
    {
        TEST_TRUE(copy->v[i].x == quad_v[i].x && copy->v[i].y == quad_v[i].y && copy->v[i].z == quad_v[i].z);
    }
    for (u32 i = 0; i < 2; ++i)
    {
        TEST_TRUE(copy->tri[i].x == quad_tri[i].x && copy->tri[i].y == quad_tri[i].y && copy->tri[i].z == quad_tri[i].z);
    }
    TEST_TRUE(copy->leaf_tri[0] < 2 && copy->leaf_tri[0] + copy->leaf_tri[1] == 1);
    BVHValidate(&copy->bvh);
    const struct aabb root = copy->bvh.pool.buf[copy->bvh.bt.root].bbox;
    TEST_TRUE(root.min.x == 0.0f && root.min.z == 0.0f && root.max.x == 1.0f && root.max.z == 1.0f);

    c_ShapeDBDealloc(&db);
	return output;
}

static struct test_Output (*shape_tests[])(struct test_Environment *) =
{
    ShapeDatabaseTest,
    ShapeMassBoxTest,
    ShapeMassHullPlacedTest,
    ShapeMassSphereTest,
    ShapeMassCapsuleTest,
    ShapeSurfaceDistanceHullTest,
    ShapeBboxTest,
    ShapeOverlapTest,
    ShapeDistanceTest,
};

struct suite_Correctness m_shape_suite =
{
    .id = "Shape",
    .unit_test = shape_tests,
    .unit_test_count = sizeof(shape_tests) / sizeof(shape_tests[0]),
};

struct suite_Correctness *shape_correctness_suite = &m_shape_suite;
