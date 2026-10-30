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
Raycasts (c_raycast.c) against f64 references and against each other: sphere and capsule against f64 root
finding, triangles against f64 Moeller-Trumbore, hulls against their own triangulated faces, the mesh BVH against
brute force over its triangles, closed meshes for leaks, and the world forms against the local ones.

A hit is compared with tolerance RAYCAST_TEST_TOLERANCE times the scale of the configuration; a grazing ray, whose
line passes within that tolerance of the surface's silhouette, may legitimately differ in hit or miss and is
skipped (counted).
*/

#define RAYCAST_TEST_TOLERANCE 2e-5
#define RC_F64_INFINITY ((f64) F32_INFINITY)

struct rc_d3 { f64 x, y, z; };

static struct rc_d3 RcD3(const v3 v) { return (struct rc_d3) { v.x, v.y, v.z }; }
static struct rc_d3 RcSub(const struct rc_d3 a, const struct rc_d3 b) { return (struct rc_d3) { a.x - b.x, a.y - b.y, a.z - b.z }; }
static struct rc_d3 RcAdd(const struct rc_d3 a, const struct rc_d3 b) { return (struct rc_d3) { a.x + b.x, a.y + b.y, a.z + b.z }; }
static struct rc_d3 RcScale(const struct rc_d3 a, const f64 s) { return (struct rc_d3) { a.x*s, a.y*s, a.z*s }; }
static f64 RcDot(const struct rc_d3 a, const struct rc_d3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
static struct rc_d3 RcCross(const struct rc_d3 a, const struct rc_d3 b) { return (struct rc_d3) { a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x }; }
static f64 RcAbs(const f64 a) { return (a < 0.0) ? -a : a; }

static f64 RcSqrt(const f64 a)
{
    f64 x = (f64) F32Sqrt((f32) a);
    if (x > 0.0)
    {
        x = 0.5 * (x + a / x);
        x = 0.5 * (x + a / x);
        x = 0.5 * (x + a / x);
    }
    return x;
}

static f64 RcLength(const struct rc_d3 a) { return RcSqrt(RcDot(a, a)); }

/* a ray with a random origin in [-range, range]^3 and a random direction of length in [0.5, 2] */
static struct ray RcRandomRay(const f32 range)
{
    const f32 ox = RngF32Range(-range, range), oy = RngF32Range(-range, range), oz = RngF32Range(-range, range);
    f32 dx, dy, dz;
    do
    {
        dx = RngF32Range(-1.0f, 1.0f);
        dy = RngF32Range(-1.0f, 1.0f);
        dz = RngF32Range(-1.0f, 1.0f);
    } while (V3LengthSquared(V3(dx, dy, dz)) < 0.25f);
    const f32 scale = RngF32Range(0.5f, 2.0f);
    return Ray(V3(ox, oy, oz), V3Scale(V3(dx, dy, dz), scale));
}

/* t and t_ref agree: both infinite, or both finite within tol of each other */
static u32 RcAgree(const f32 t, const f64 t_ref, const f64 tol)
{
    if (t == F32_INFINITY || t_ref == RC_F64_INFINITY)
    {
        return (t == F32_INFINITY) && (t_ref == RC_F64_INFINITY);
    }
    return RcAbs((f64) t - t_ref) <= tol;
}

/********************************** sphere **********************************/

/*
f64 reference: the quadratic |o + t d|^2 = r^2 in its plain form (the engine uses Hearn's discriminant); the
smallest root >= 0. Sets *graze to the line's distance to the center minus r (near 0: a tangent ray).
*/
static f64 RcSphereReference(f64 *graze, const f64 r, const struct ray *ray)
{
    const struct rc_d3 o = RcD3(ray->origin), d = RcD3(ray->dir);
    const f64 a = RcDot(d, d), b = 2.0 * RcDot(o, d), c = RcDot(o, o) - r*r;
    const f64 disc = b*b - 4.0*a*c;
    /* distance of the line to the center */
    const f64 line_distance = RcLength(RcCross(o, d)) / RcSqrt(a);
    *graze = line_distance - r;
    if (disc < 0.0)
    {
        return RC_F64_INFINITY;
    }
    const f64 s = RcSqrt(disc);
    const f64 t0 = (-b - s) / (2.0*a), t1 = (-b + s) / (2.0*a);
    return (t0 >= 0.0) ? t0 : (t1 >= 0.0) ? t1 : RC_F64_INFINITY;
}

struct test_Output RaycastSphereTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    (void) env;

    u32 grazing = 0, hits = 0;
    const u32 rounds = 20000;
    for (u32 round = 0; round < rounds; ++round)
    {
        const struct sphere sphere = { .radius = RngF32Range(0.05f, 3.0f) };
        const struct ray ray = RcRandomRay(4.0f);
        const f32 t = SphereRaycastParameter(&sphere, &ray);
        f64 graze;
        const f64 t_ref = RcSphereReference(&graze, sphere.radius, &ray);
        const f64 scale = 4.0 + sphere.radius;
        if (RcAbs(graze) <= RAYCAST_TEST_TOLERANCE * scale)
        {
            grazing += 1;
            continue;
        }
        hits += (t != F32_INFINITY);
        if (t < 0.0f || t != t || !RcAgree(t, t_ref, RAYCAST_TEST_TOLERANCE * scale / V3Length(ray.dir)))
        {
            fprintf(stderr, "sphere r %g, ray o (%g %g %g) d (%g %g %g): t %.9g, reference %.9g\n", sphere.radius,
                ray.origin.x, ray.origin.y, ray.origin.z, ray.dir.x, ray.dir.y, ray.dir.z, t, t_ref);
            TEST_FAILURE;
        }
    }
    fprintf(stdout, "\tsphere: %u rays, %u hits, %u grazing skipped\n", rounds, hits, grazing);

    return output;
}

/********************************** capsule **********************************/

/*
f64 reference by regions: the roots of the infinite cylinder count where |y| <= h, the roots of each cap's sphere
where y is beyond that cap's plane; the smallest such root >= 0. Independent of the engine's entry/exit logic.
*graze: the line's distance to the axis segment minus r.
*/
static f64 RcCapsuleReference(f64 *graze, const f64 r, const f64 h, const struct ray *ray)
{
    const struct rc_d3 o = RcD3(ray->origin), d = RcD3(ray->dir);
    f64 best = RC_F64_INFINITY;

    /* cylinder: (ox + t dx)^2 + (oz + t dz)^2 = r^2 */
    {
        const f64 a = d.x*d.x + d.z*d.z, b = 2.0*(o.x*d.x + o.z*d.z), c = o.x*o.x + o.z*o.z - r*r;
        const f64 disc = b*b - 4.0*a*c;
        if (a > 0.0 && disc >= 0.0)
        {
            const f64 s = RcSqrt(disc);
            const f64 root[2] = { (-b - s) / (2.0*a), (-b + s) / (2.0*a) };
            for (u32 i = 0; i < 2; ++i)
            {
                const f64 y = o.y + root[i]*d.y;
                if (root[i] >= 0.0 && RcAbs(y) <= h && root[i] < best) { best = root[i]; }
            }
        }
    }
    /* the caps */
    for (u32 k = 0; k < 2; ++k)
    {
        const f64 cy = (k == 0) ? h : -h;
        const struct rc_d3 oc = { o.x, o.y - cy, o.z };
        const f64 a = RcDot(d, d), b = 2.0*RcDot(oc, d), c = RcDot(oc, oc) - r*r;
        const f64 disc = b*b - 4.0*a*c;
        if (disc >= 0.0)
        {
            const f64 s = RcSqrt(disc);
            const f64 root[2] = { (-b - s) / (2.0*a), (-b + s) / (2.0*a) };
            for (u32 i = 0; i < 2; ++i)
            {
                const f64 y = o.y + root[i]*d.y;
                const u32 beyond = (k == 0) ? (y >= h) : (y <= -h);
                if (root[i] >= 0.0 && beyond && root[i] < best) { best = root[i]; }
            }
        }
    }

    /* the line's distance to the segment (-h .. h on y): the closest point of the segment to the line */
    {
        /* minimize |o + t d - (0, s, 0)|^2 over t and s in [-h, h]: solve the 2x2 system, clamp s, re-solve t */
        const struct rc_d3 axis = { 0.0, 1.0, 0.0 };
        const f64 dd = RcDot(d, d), da = RcDot(d, axis), od = RcDot(o, d), oa = RcDot(o, axis);
        const f64 det = dd - da*da;
        f64 s = (det > 1e-12) ? (oa*dd - od*da) / det : 0.0;
        s = (s < -h) ? -h : (s > h) ? h : s;
        const f64 t = (s*da - od) / dd;
        const struct rc_d3 p = RcSub(RcAdd(o, RcScale(d, t)), RcScale(axis, s));
        *graze = RcLength(p) - r;
        /* a line parallel to the axis: the formula above is exact too (det ~ 0, s = 0 then clamped: the closest point on the line is at any s) */
        if (det <= 1e-12)
        {
            const struct rc_d3 q = { o.x, 0.0, o.z };
            *graze = RcLength(q) - r;
        }
    }
    return best;
}

struct test_Output RaycastCapsuleTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    (void) env;

    u32 grazing = 0, hits = 0;
    const u32 rounds = 20000;
    for (u32 round = 0; round < rounds; ++round)
    {
        const struct capsule capsule = { .radius = RngF32Range(0.05f, 2.0f), .half_height = (round % 7 == 0) ? 0.0f : RngF32Range(0.0f, 3.0f) };
        struct ray ray = RcRandomRay(4.0f);
        if (round % 5 == 0)
        {
            ray.dir = V3(0.0f, ray.dir.y, 0.0f);	/* parallel to the axis: the special branch */
            if (ray.dir.y == 0.0f) { ray.dir.y = 1.0f; }
        }
        const f32 t = CapsuleRaycastParameter(&capsule, &ray);
        f64 graze;
        const f64 t_ref = RcCapsuleReference(&graze, capsule.radius, capsule.half_height, &ray);
        const f64 scale = 4.0 + capsule.radius + capsule.half_height;
        if (RcAbs(graze) <= RAYCAST_TEST_TOLERANCE * scale)
        {
            grazing += 1;
            continue;
        }
        hits += (t != F32_INFINITY);
        if (t < 0.0f || t != t || !RcAgree(t, t_ref, RAYCAST_TEST_TOLERANCE * scale / V3Length(ray.dir)))
        {
            fprintf(stderr, "capsule r %g h %g, ray o (%.9g %.9g %.9g) d (%.9g %.9g %.9g): t %.9g, reference %.9g\n",
                capsule.radius, capsule.half_height, ray.origin.x, ray.origin.y, ray.origin.z, ray.dir.x, ray.dir.y, ray.dir.z, t, t_ref);
            TEST_FAILURE;
        }
    }
    fprintf(stdout, "\tcapsule: %u rays, %u hits, %u grazing skipped\n", rounds, hits, grazing);

    return output;
}

/********************************** triangle **********************************/

/* f64 Moeller-Trumbore, two-sided; *edge: the smallest barycentric coordinate of the line's hit (near 0: an edge) */
static f64 RcTriReference(f64 *edge, const v3 a, const v3 b, const v3 c, const struct ray *ray)
{
    const struct rc_d3 A = RcD3(a), B = RcD3(b), C = RcD3(c), o = RcD3(ray->origin), d = RcD3(ray->dir);
    const struct rc_d3 e1 = RcSub(B, A), e2 = RcSub(C, A);
    const struct rc_d3 p = RcCross(d, e2);
    const f64 det = RcDot(e1, p);
    *edge = -1.0;
    if (RcAbs(det) < 1e-300) { return RC_F64_INFINITY; }
    const struct rc_d3 s = RcSub(o, A);
    const f64 u = RcDot(s, p) / det;
    const struct rc_d3 q = RcCross(s, e1);
    const f64 v = RcDot(d, q) / det;
    const f64 w = 1.0 - u - v;
    *edge = (u < v) ? ((u < w) ? u : w) : ((v < w) ? v : w);
    const f64 t = RcDot(e2, q) / det;
    if (u < 0.0 || v < 0.0 || w < 0.0 || t < 0.0) { return RC_F64_INFINITY; }
    return t;
}

struct test_Output RaycastTriTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    (void) env;

    u32 grazing = 0, hits = 0;
    const u32 rounds = 20000;
    for (u32 round = 0; round < rounds; ++round)
    {
        const f32 size = RngF32Range(0.1f, 3.0f);
        const v3 a = V3(RngF32Range(-size, size), RngF32Range(-size, size), RngF32Range(-size, size));
        const v3 b = V3(RngF32Range(-size, size), RngF32Range(-size, size), RngF32Range(-size, size));
        const v3 c = V3(RngF32Range(-size, size), RngF32Range(-size, size), RngF32Range(-size, size));
        const struct ray ray = RcRandomRay(3.0f);
        const f32 t = TriRaycastParameter(a, b, c, &ray);
        f64 edge;
        const f64 t_ref = RcTriReference(&edge, a, b, c, &ray);
        if (RcAbs(edge) <= 1e-4)
        {
            grazing += 1;	/* the line passes within 1e-4 (barycentric) of an edge */
            continue;
        }
        hits += (t != F32_INFINITY);
        if (t < 0.0f || t != t || !RcAgree(t, t_ref, RAYCAST_TEST_TOLERANCE * (3.0 + size) / V3Length(ray.dir)))
        {
            fprintf(stderr, "triangle round %u: t %.9g, reference %.9g (edge %.3g)\n", round, t, t_ref, edge);
            TEST_FAILURE;
        }
    }
    fprintf(stdout, "\ttriangle: %u rays, %u hits, %u near an edge skipped\n", rounds, hits, grazing);

    return output;
}

/********************************** hull **********************************/

/* a mesh of the hull's faces, each triangulated as a fan from its first vertex */
static struct triMesh *RcHullMesh(struct arena *mem, const struct dcel *hull)
{
    struct triMesh *mesh = ArenaPush(mem, sizeof(struct triMesh));
    mesh->v_count = hull->v_count;
    mesh->v = hull->v;
    mesh->tri_count = 0;
    for (u32 fi = 0; fi < hull->f_count; ++fi)
    {
        mesh->tri_count += hull->f[fi].count - 2;
    }
    mesh->tri = ArenaPush(mem, mesh->tri_count*sizeof(v3u32));
    u32 t = 0;
    for (u32 fi = 0; fi < hull->f_count; ++fi)
    {
        const struct dcelFace *f = hull->f + fi;
        const u32 v0 = hull->e[f->first].origin;
        for (u32 i = 1; i + 1 < f->count; ++i)
        {
            mesh->tri[t++] = (v3u32) { .buf = { v0, hull->e[f->first + i].origin, hull->e[f->first + i + 1].origin } };
        }
    }
    return mesh;
}

/* the first hit over all triangles of a mesh, one ray setup (as the BVH traversal does) */
static f32 RcMeshBruteForce(const struct triMesh *mesh, const struct ray *ray)
{
    const struct rayWoop woop = RayWoop(ray);
    f32 best = F32_INFINITY;
    for (u32 i = 0; i < mesh->tri_count; ++i)
    {
        const v3u32 tri = mesh->tri[i];
        const f32 t = TriRaycastParameterEx(&woop, mesh->v[tri.x], mesh->v[tri.y], mesh->v[tri.z]);
        best = (t < best) ? t : best;
    }
    return best;
}

/* max over the hull's planes of the signed distance of p: 0 on the surface, < 0 inside */
static f32 RcHullSignedDistance(const struct dcel *hull, const v3 p)
{
    f32 best = -F32_INFINITY;
    for (u32 fi = 0; fi < hull->f_count; ++fi)
    {
        const f32 d = V3Dot(hull->plane[fi].normal, p) - hull->plane[fi].signed_distance;
        best = (d > best) ? d : best;
    }
    return best;
}

static struct dcel RcRandomHull(struct arena *mem, const f32 size, const u32 point_count)
{
    v3 *p = ArenaPush(mem, point_count*sizeof(v3));
    for (u32 i = 0; i < point_count; ++i)
    {
        p[i] = V3(RngF32Range(-size, size), RngF32Range(-size, size), RngF32Range(-size, size));
    }
    return DcelConvexHull(mem, p, point_count);
}

/*
Boxes against AabbRaycastParameter (the same box), random hulls against their triangulated faces (TriRaycastParameterEx
over the fan triangles: a different algorithm on the same geometry); every finite hit lies on the hull's surface.
*/
struct test_Output RaycastHullTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    u32 grazing = 0, hits = 0;
    const u32 rounds = 4000;
    for (u32 round = 0; round < rounds; ++round)
    {
        ArenaPushRecord(env->mem_1);
        const f32 size = RngF32Range(0.1f, 3.0f);
        const u32 is_box = (round % 2 == 0);
        const v3 hw = V3(RngF32Range(0.1f, size), RngF32Range(0.1f, size), RngF32Range(0.1f, size));
        const struct dcel hull = (is_box) ? DcelBox(env->mem_1, hw) : RcRandomHull(env->mem_1, size, 8 + round % 40);
        if (hull.f_count == 0)
        {
            ArenaPopRecord(env->mem_1);
            continue;	/* degenerate cloud */
        }
        const struct triMesh *mesh = RcHullMesh(env->mem_1, &hull);

        for (u32 k = 0; k < 8; ++k)
        {
            const struct ray ray = RcRandomRay(2.0f*size);
            const f32 t = HullRaycastParameter(&hull, &ray);
            const f32 t_mesh = RcMeshBruteForce(mesh, &ray);
            const f32 tol = (f32) RAYCAST_TEST_TOLERANCE * 4.0f * size / V3Length(ray.dir);
            /* a ray grazing the hull: the line passes within tol of the surface at its closest approach */
            if (t == F32_INFINITY || t_mesh == F32_INFINITY)
            {
                if (t != t_mesh)
                {
                    /* one hit, one miss: a graze if the hit is within tol of leaving the hull (an edge or a vertex) */
                    const f32 t_hit = (t == F32_INFINITY) ? t_mesh : t;
                    const v3 p_before = V3Add(ray.origin, V3Scale(ray.dir, t_hit - tol));
                    const v3 p_after = V3Add(ray.origin, V3Scale(ray.dir, t_hit + tol));
                    if (RcHullSignedDistance(&hull, p_before) > -(f32) RAYCAST_TEST_TOLERANCE * size && RcHullSignedDistance(&hull, p_after) > -(f32) RAYCAST_TEST_TOLERANCE * size)
                    {
                        grazing += 1;
                        continue;
                    }
                    fprintf(stderr, "hull round %u (%s, %u faces): hull t %.9g, mesh t %.9g\n", round, is_box ? "box" : "random", hull.f_count, t, t_mesh);
                    ArenaPopRecord(env->mem_1);
                    TEST_FAILURE;
                }
                continue;
            }
            hits += 1;
            const v3 p = V3Add(ray.origin, V3Scale(ray.dir, t));
            const f32 surface = RcHullSignedDistance(&hull, p);
            if (t < 0.0f || F32Abs(t - t_mesh) > tol || F32Abs(surface) > (f32) RAYCAST_TEST_TOLERANCE * 4.0f * size)
            {
                fprintf(stderr, "hull round %u (%s, %u faces): hull t %.9g, mesh t %.9g, hit's plane distance %.3g\n", round, is_box ? "box" : "random", hull.f_count, t, t_mesh, surface);
                ArenaPopRecord(env->mem_1);
                TEST_FAILURE;
            }
            if (is_box)
            {
                const struct aabb box = { .min = V3Negate(hw), .max = hw };
                const f32 t_box = AabbRaycastParameter(&box, &ray);
                if (F32Abs(t - t_box) > tol)
                {
                    fprintf(stderr, "box round %u: hull t %.9g, aabb t %.9g\n", round, t, t_box);
                    ArenaPopRecord(env->mem_1);
                    TEST_FAILURE;
                }
            }
        }
        ArenaPopRecord(env->mem_1);
    }
    fprintf(stdout, "\thull: %u hulls x 8 rays, %u hits, %u grazing skipped\n", rounds, hits, grazing);

    return output;
}

/********************************** mesh BVH **********************************/

/* a terrain grid with random heights, cells x cells quads */
static struct c_Shape RcTerrain(struct arena *mem, const u32 cells, const f32 cell, const f32 relief)
{
    const u32 n = cells + 1;
    struct triMesh *mesh = ArenaPush(mem, sizeof(struct triMesh));
    mesh->v_count = n*n;
    mesh->tri_count = 2*cells*cells;
    mesh->v = ArenaPush(mem, mesh->v_count*sizeof(v3));
    mesh->tri = ArenaPush(mem, mesh->tri_count*sizeof(v3u32));
    for (u32 i = 0; i < n; ++i)
    {
        for (u32 j = 0; j < n; ++j)
        {
            const f32 y = relief*RngF32Range(-1.0f, 1.0f);
            mesh->v[i*n + j] = V3(cell*(f32) i - 0.5f*cell*cells, y, cell*(f32) j - 0.5f*cell*cells);
        }
    }
    u32 t = 0;
    for (u32 i = 0; i < cells; ++i)
    {
        for (u32 j = 0; j < cells; ++j)
        {
            const u32 a = i*n + j, b = (i + 1)*n + j, c = i*n + j + 1, d = (i + 1)*n + j + 1;
            mesh->tri[t++] = (v3u32) { .buf = { a, c, b } };
            mesh->tri[t++] = (v3u32) { .buf = { b, c, d } };
        }
    }
    struct c_Shape shape = { .type = C_SHAPE_TRI_MESH };
    shape.mesh_bvh = TriMeshBVHConstruct(mem, mesh, 8);
    return shape;
}

/* the BVH traversal finds exactly the brute-force first hit (the same triangle test with the same ray setup: bit for bit) */
struct test_Output RaycastMeshBVHTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    ArenaPushRecord(env->mem_1);
    const struct c_Shape terrain = RcTerrain(env->mem_1, 32, 0.5f, 0.4f);
    u32 hits = 0;
    const u32 rounds = 4000;
    for (u32 round = 0; round < rounds; ++round)
    {
        struct ray ray = RcRandomRay(10.0f);
        if (round % 3 == 0)
        {
            ray.origin.y = 5.0f;	/* from above, down through the terrain */
            ray.dir.y = -F32Abs(ray.dir.y) - 0.1f;
        }
        const f32 t = TriMeshBVHRaycastParameter(&terrain.mesh_bvh, &ray);
        const f32 t_brute = RcMeshBruteForce(terrain.mesh_bvh.mesh, &ray);
        hits += (t != F32_INFINITY);
        if (t != t_brute)
        {
            fprintf(stderr, "mesh bvh round %u: bvh t %.9g, brute force t %.9g\n", round, t, t_brute);
            ArenaPopRecord(env->mem_1);
            TEST_FAILURE;
        }
    }
    ArenaPopRecord(env->mem_1);
    fprintf(stdout, "\tmesh bvh: %u rays over 2048 triangles, %u hits\n", rounds, hits);

    return output;
}

/*
Watertight: a ray from outside a closed mesh (a hull's faces) through a point inside it hits a triangle no later
than that point, shared edges and vertices included.
*/
struct test_Output RaycastWatertightTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    u32 rays = 0, passing = 0;
    for (u32 round = 0; round < 200; ++round)
    {
        ArenaPushRecord(env->mem_1);
        const f32 size = RngF32Range(0.1f, 3.0f);
        const struct dcel hull = RcRandomHull(env->mem_1, size, 8 + round % 40);
        if (hull.f_count == 0)
        {
            ArenaPopRecord(env->mem_1);
            continue;
        }
        const struct triMesh *mesh = RcHullMesh(env->mem_1, &hull);
        /* an interior point: the mean of the vertices (convex) */
        v3 inside = V3Zero();
        for (u32 i = 0; i < hull.v_count; ++i) { inside = V3Add(inside, hull.v[i]); }
        inside = V3Scale(inside, 1.0f / (f32) hull.v_count);

        for (u32 k = 0; k < 100; ++k)
        {
            /* an origin outside: beyond the bounding box on a random side, aimed at a random point near a vertex, an edge, or the center */
            v3 origin = V3(RngF32Range(-4.0f*size, 4.0f*size), RngF32Range(-4.0f*size, 4.0f*size), RngF32Range(-4.0f*size, 4.0f*size));
            if (RcHullSignedDistance(&hull, origin) <= 0.0f) { continue; }
            v3 target = inside;
            if (k % 3 == 1)
            {
                const v3 v = hull.v[k % hull.v_count];	/* through a vertex (the shear puts it exactly on the ray) */
                target = v;
            }
            else if (k % 3 == 2)
            {
                const struct dcelEdge *e = hull.e + (k % hull.e_count);
                target = V3Scale(V3Add(hull.v[e->origin], hull.v[hull.e[e->twin].origin]), 0.5f);	/* an edge midpoint */
            }
            const struct ray ray = Ray(origin, V3Sub(target, origin));
            const f32 t = RcMeshBruteForce(mesh, &ray);
            /*
             * The hull's own raycast (Cyrus-Beck on the planes, checked against the mesh on generic rays in
             * RaycastHullTest) is the reference: the mesh must hit where it does, shared edges and vertices
             * included. The ray may enter at the target (t = 1), enter earlier and leave at it, or pass over
             * a ridge; a ray that never goes deeper than the tolerance near the target may go either way.
             */
            const f32 t_hull = HullRaycastParameter(&hull, &ray);
            const f32 tol = 1e-4f;
            if (t == t_hull || (t != F32_INFINITY && t_hull != F32_INFINITY && F32Abs(t - t_hull) <= tol))
            {
                rays += 1;
                continue;
            }
            const f32 depth_before = RcHullSignedDistance(&hull, V3Add(ray.origin, V3Scale(ray.dir, 0.99f)));
            const f32 depth_after = RcHullSignedDistance(&hull, V3Add(ray.origin, V3Scale(ray.dir, 1.01f)));
            if (depth_before > -tol*size && depth_after > -tol*size)
            {
                passing += 1;
                continue;
            }
            fprintf(stderr, "watertight round %u ray %u: %u faces, mesh t %.9g, hull t %.9g (target kind %u, depth near it %.3g %.3g)\n",
                round, k, hull.f_count, t, t_hull, k % 3, depth_before, depth_after);
            ArenaPopRecord(env->mem_1);
            TEST_FAILURE;
        }
        ArenaPopRecord(env->mem_1);
    }
    fprintf(stdout, "\twatertight: %u rays at hull vertices, edges and centers agree with the hull raycast; %u grazing left alone\n", rays, passing);

    return output;
}

/********************************** world forms **********************************/

/* the world form of each shape on a placed shape equals the local form on the ray taken into the shape's frame */
struct test_Output RaycastWorldTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };

    u32 checked = 0;
    for (u32 round = 0; round < 2000; ++round)
    {
        ArenaPushRecord(env->mem_1);
        const ds_TransformWorld transform =
        {
            .rotation = QNormalize(Q(RngF32Range(-1.0f, 1.0f), RngF32Range(-1.0f, 1.0f), RngF32Range(-1.0f, 1.0f), RngF32Range(-1.0f, 1.0f))),
            .position = V3F64(RngF32Range(-50.0f, 50.0f), RngF32Range(-50.0f, 50.0f), RngF32Range(-50.0f, 50.0f)),
        };
        const struct ray near = RcRandomRay(4.0f);
        /* the world ray: the local ray placed by the transform (f64 origin) */
        const struct rayWorld world =
        {
            .origin = V3F64Add(transform.position, V3F64(QUnitRotate(transform.rotation, near.origin).x, QUnitRotate(transform.rotation, near.origin).y, QUnitRotate(transform.rotation, near.origin).z)),
            .dir = QUnitRotate(transform.rotation, near.dir),
        };
        /* back to local, the way the world functions do it: the reference is the local functions on this ray */
        const struct ray local =
        {
            .origin = QUnitRotate(QConj(transform.rotation), V3F64Demote(V3F64Sub(world.origin, transform.position))),
            .dir = QUnitRotate(QConj(transform.rotation), world.dir),
        };
        const f32 tol = 1e-4f;

        const struct sphere sphere = { .radius = RngF32Range(0.2f, 3.0f) };
        const struct capsule capsule = { .radius = RngF32Range(0.2f, 2.0f), .half_height = RngF32Range(0.0f, 2.0f) };
        const struct dcel hull = DcelBox(env->mem_1, V3(RngF32Range(0.2f, 2.0f), RngF32Range(0.2f, 2.0f), RngF32Range(0.2f, 2.0f)));
        const struct aabb box = { .min = V3(-1.0f, -0.5f, -2.0f), .max = V3(1.0f, 0.5f, 2.0f) };
        const v3 a = V3(RngF32Range(-2.0f, 2.0f), RngF32Range(-2.0f, 2.0f), RngF32Range(-2.0f, 2.0f));
        const v3 b = V3(RngF32Range(-2.0f, 2.0f), RngF32Range(-2.0f, 2.0f), RngF32Range(-2.0f, 2.0f));
        const v3 c = V3(RngF32Range(-2.0f, 2.0f), RngF32Range(-2.0f, 2.0f), RngF32Range(-2.0f, 2.0f));
        const f32 t[5] =
        {
            SphereRaycastWorldParameter(&sphere, &transform, &world),
            CapsuleRaycastWorldParameter(&capsule, &transform, &world),
            HullRaycastWorldParameter(&hull, &transform, &world),
            AabbRaycastWorldParameter(&box, &transform, &world),
            TriRaycastWorldParameter(a, b, c, &transform, &world),
        };
        const f32 t_local[5] =
        {
            SphereRaycastParameter(&sphere, &local),
            CapsuleRaycastParameter(&capsule, &local),
            HullRaycastParameter(&hull, &local),
            AabbRaycastParameter(&box, &local),
            TriRaycastParameter(a, b, c, &local),
        };
        for (u32 i = 0; i < 5; ++i)
        {
            checked += 1;
            const u32 same = (t[i] == t_local[i]) || F32Abs(t[i] - t_local[i]) <= tol * (1.0f + F32Abs(t_local[i]));
            if (!same)
            {
                fprintf(stderr, "world round %u shape %u: world t %.9g, local t %.9g\n", round, i, t[i], t_local[i]);
                ArenaPopRecord(env->mem_1);
                TEST_FAILURE;
            }
        }
        ArenaPopRecord(env->mem_1);
    }
    fprintf(stdout, "\tworld: %u world-local pairs agree\n", checked);

    return output;
}

static struct test_Output (*raycast_tests[])(struct test_Environment *) =
{
    RaycastSphereTest,
    RaycastCapsuleTest,
    RaycastTriTest,
    RaycastHullTest,
    RaycastMeshBVHTest,
    RaycastWatertightTest,
    RaycastWorldTest,
};

struct suite_Correctness m_raycast_suite =
{
    .id = "Raycast",
    .unit_test = raycast_tests,
    .unit_test_count = sizeof(raycast_tests) / sizeof(raycast_tests[0]),
};

struct suite_Correctness *raycast_correctness_suite = &m_raycast_suite;
