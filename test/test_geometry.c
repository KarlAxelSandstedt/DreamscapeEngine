#include "ds_test.h"
#include "ds_geometry.h"
#include "ds_voronoi.h"

static v3 V3UnitSphere(void)
{
    const f32 z = RngF32Range(-1.0f, 1.0f);
    const f32 r = F32Sqrt(1 - z*z);
    const f32 theta = 2.0f*F32_PI*RngF32Normalized();
    return V3(r*F32Cos(theta), r*F32Sin(theta), z);
}

/*
 * SegmentDistanceSquared with short and degenerate segments (length 1 .. subnormal .. 0): the parameters stay in
 * [0, 1], the points on their segments, the distance finite and within the short segment's length of the
 * distance from its first end (SegmentPointDistanceSquared).
 */
static u32 SegmentShortCase(const v3 a0, const v3 a_dir, const v3 b0, const v3 b1)
{
    const struct segment a = Segment(a0, V3Add(a0, a_dir));
    const struct segment b = Segment(b0, b1);
    f32 t1, t2;
    SegmentClosestParameter(&t1, &t2, &a, &b);
    if (!(0.0f <= t1 && t1 <= 1.0f && 0.0f <= t2 && t2 <= 1.0f))
    {
        fprintf(stderr, "SegmentShortCase: t1 %g t2 %g\n", t1, t2);
        return 0;
    }

    v3 c1, c2, c;
    const f32 dist = F32Sqrt(SegmentDistanceSquared(&c1, &c2, &a, &b));
    const f32 ref = F32Sqrt(SegmentPointDistanceSquared(&c, &b, a0));
    const f32 len = V3Length(a_dir);
    const u32 ok = (dist == dist) && dist <= ref + 1e-6f && dist >= ref - len - 1e-6f;
    if (!ok)
    {
        fprintf(stderr, "SegmentShortCase: |a| %g dist %.9g ref %.9g\n", len, dist, ref);
    }
    return ok;
}

struct test_Output SegmentShortTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    const f32 length[] = { 1.0f, 1e-2f, 3e-3f, 1e-4f, 1e-8f, 1e-15f, 1e-20f, 1e-22f, 1e-23f, 1e-30f, 1e-38f, 1e-40f, 1e-45f, 0.0f };
    const v3 b0 = V3(-1.0f, 0.5f, 0.25f);
    const v3 b1 = V3(1.0f, 0.5f, -0.25f);
    for (u32 i = 0; i < sizeof(length) / sizeof(length[0]); ++i)
    {
        for (u32 k = 0; k < 1000; ++k)
        {
            const v3 a0 = V3(RngF32Range(-1.0f, 1.0f), RngF32Range(-1.0f, 1.0f), RngF32Range(-1.0f, 1.0f));
            const v3 dir = V3Scale(V3UnitSphere(), length[i]);
            /* short a vs a unit b, b vs short a, two short segments, short a parallel to b */
            TEST_TRUE(SegmentShortCase(a0, dir, b0, b1));
            TEST_TRUE(SegmentShortCase(a0, V3Scale(V3Sub(b1, b0), length[i]), b0, b1));
            TEST_TRUE(SegmentShortCase(a0, dir, b0, V3Add(b0, V3Scale(V3UnitSphere(), length[i]))));
            {
                const struct segment a = Segment(b0, b1);
                const struct segment s = Segment(a0, V3Add(a0, dir));
                v3 c1, c2, c;
                const f32 dist = F32Sqrt(SegmentDistanceSquared(&c1, &c2, &a, &s));
                const f32 ref = F32Sqrt(SegmentPointDistanceSquared(&c, &a, a0));
                TEST_TRUE(dist == dist && dist <= ref + 1e-6f && dist >= ref - length[i] - 1e-6f);
            }
        }
    }

	return output;
}

static u32 ClipNear(const v3 a, const v3 b)
{
    return V3Length(V3Sub(a, b)) <= 1e-6f;
}

/* clip s to face fi of the box; 1 if non-empty and the result is (e0, e1) */
static u32 ClipCase(const struct dcel *box, const u32 fi, const v3 p0, const v3 p1, const v3 e0, const v3 e1)
{
    const struct segment s = Segment(p0, p1);
    struct segment clip;
    return DcelFaceSegmentClip(&clip, box, fi, &s) && ClipNear(clip.p[0], e0) && ClipNear(clip.p[1], e1);
}

/* clip s to face fi of the box; 1 if empty */
static u32 ClipEmptyCase(const struct dcel *box, const u32 fi, const v3 p0, const v3 p1)
{
    const struct segment s = Segment(p0, p1);
    struct segment clip;
    return !DcelFaceSegmentClip(&clip, box, fi, &s);
}

/*
 * DcelFaceSegmentClip on the unit box stub ([-0.5, 0.5]^3, face 0 = +y): the face's sides are
 * |x|, |z| <= 0.5. Segments at y = 0.7 (above the face; the clip does not project onto it).
 */
struct test_Output DcelFaceSegmentClipTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    const struct dcel box = DcelBoxStub();

    /* inside: unchanged */
    TEST_TRUE(ClipCase(&box, 0, V3(-0.2f, 0.7f, 0.0f), V3(0.3f, 0.7f, 0.1f), V3(-0.2f, 0.7f, 0.0f), V3(0.3f, 0.7f, 0.1f)));
    /* crosses one side: p1 moved, p0 kept */
    TEST_TRUE(ClipCase(&box, 0, V3(0.0f, 0.7f, 0.0f), V3(1.0f, 0.7f, 0.0f), V3(0.0f, 0.7f, 0.0f), V3(0.5f, 0.7f, 0.0f)));
    /* crosses one side: p0 moved, p1 kept */
    TEST_TRUE(ClipCase(&box, 0, V3(1.0f, 0.7f, 0.0f), V3(0.0f, 0.7f, 0.0f), V3(0.5f, 0.7f, 0.0f), V3(0.0f, 0.7f, 0.0f)));
    /* crosses two opposite sides; direction kept */
    TEST_TRUE(ClipCase(&box, 0, V3(-1.0f, 0.7f, 0.2f), V3(1.0f, 0.7f, 0.2f), V3(-0.5f, 0.7f, 0.2f), V3(0.5f, 0.7f, 0.2f)));
    TEST_TRUE(ClipCase(&box, 0, V3(1.0f, 0.7f, 0.2f), V3(-1.0f, 0.7f, 0.2f), V3(0.5f, 0.7f, 0.2f), V3(-0.5f, 0.7f, 0.2f)));
    /* diagonal through two adjacent sides */
    TEST_TRUE(ClipCase(&box, 0, V3(-1.0f, 0.7f, -1.0f), V3(1.0f, 0.7f, 1.0f), V3(-0.5f, 0.7f, -0.5f), V3(0.5f, 0.7f, 0.5f)));
    /* along the face normal: no projection, unchanged */
    TEST_TRUE(ClipCase(&box, 0, V3(0.1f, -2.0f, 0.1f), V3(0.1f, 2.0f, 0.1f), V3(0.1f, -2.0f, 0.1f), V3(0.1f, 2.0f, 0.1f)));

    /* wholly outside one side (spans the z range): empty */
    TEST_TRUE(ClipEmptyCase(&box, 0, V3(0.6f, 0.7f, -1.0f), V3(0.8f, 0.7f, 1.0f)));
    /* misses the corner (x + z = 1.2): every side alone keeps a part, together nothing */
    TEST_TRUE(ClipEmptyCase(&box, 0, V3(1.2f, 0.7f, 0.0f), V3(0.0f, 0.7f, 1.2f)));
    /* along the face normal, outside: empty */
    TEST_TRUE(ClipEmptyCase(&box, 0, V3(0.6f, -2.0f, 0.1f), V3(0.6f, 2.0f, 0.1f)));

    /* in a side plane (x = 0.5): kept, clipped by the z sides, finite */
    TEST_TRUE(ClipCase(&box, 0, V3(0.5f, 0.7f, -1.0f), V3(0.5f, 0.7f, 1.0f), V3(0.5f, 0.7f, -0.5f), V3(0.5f, 0.7f, 0.5f)));
    /* touches a side at an end: a single point */
    TEST_TRUE(ClipCase(&box, 0, V3(0.5f, 0.7f, 0.0f), V3(1.0f, 0.7f, 0.0f), V3(0.5f, 0.7f, 0.0f), V3(0.5f, 0.7f, 0.0f)));

    /* degenerate (p0 == p1): itself inside, empty outside */
    TEST_TRUE(ClipCase(&box, 0, V3(0.1f, 0.7f, 0.1f), V3(0.1f, 0.7f, 0.1f), V3(0.1f, 0.7f, 0.1f), V3(0.1f, 0.7f, 0.1f)));
    TEST_TRUE(ClipEmptyCase(&box, 0, V3(0.6f, 0.7f, 0.1f), V3(0.6f, 0.7f, 0.1f)));

    /* other faces: face 1 = +x (sides |y|, |z| <= 0.5), face 5 = -y */
    TEST_TRUE(ClipCase(&box, 1, V3(0.7f, -1.0f, 0.0f), V3(0.7f, 1.0f, 0.0f), V3(0.7f, -0.5f, 0.0f), V3(0.7f, 0.5f, 0.0f)));
    TEST_TRUE(ClipCase(&box, 5, V3(0.0f, -0.7f, 0.0f), V3(1.0f, -0.7f, 0.0f), V3(0.0f, -0.7f, 0.0f), V3(0.5f, -0.7f, 0.0f)));

    /*
     * A zero-length edge (v1 = v0) clips nothing: face 0 becomes the triangle v0 v2 v3, |x|, |z| <= 0.5 and x <= z.
     * The old normalized side plane was NaN there (ignored by every comparison, so the result was the same).
     */
    v3 v[8];
    for (u32 i = 0; i < 8; ++i)
    {
        v[i] = box.v[i];
    }
    v[1] = v[0];
    struct dcel collapsed = box;
    collapsed.v = v;
    TEST_TRUE(ClipCase(&collapsed, 0, V3(-0.4f, 0.7f, 0.0f), V3(0.4f, 0.7f, 0.0f),
                                      V3(-0.4f, 0.7f, 0.0f), V3(0.0f, 0.7f, 0.0f)));
    TEST_TRUE(ClipEmptyCase(&collapsed, 0, V3(0.1f, 0.7f, -0.4f), V3(0.4f, 0.7f, -0.1f)));

	return output;
}

static u32 DcelTestV3Equal(const v3 a, const v3 b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

/*
 * DcelBox for random half widths: the topology checks (DcelAssertTopology), outward planes (each face's normal is
 * an axis up to the normalization's 2 eps, its distance the half width within 4 eps), every vertex on or behind
 * every plane (on its own faces' planes exactly), the exact box, edge directions of exactly twice a half width
 * along an axis, and DcelSupportFace on the axes. The stubs pass the topology checks too. Release builds also check
 * that invalid half widths give the box stub (debug asserts on them).
 */
struct test_Output DcelBoxTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    const struct dcel stub = DcelBoxStub();
    DcelAssertTopology(&stub);
    v3 tri_v[3] = { V3(0.0f, 0.0f, 0.0f), V3(1.0f, 0.0f, 0.0f), V3(0.0f, 1.0f, 0.0f) };
    struct dcel tri = DcelTriStub();
    tri.v = tri_v;
    DcelAssertTopology(&tri);

    for (u32 i = 0; i < 1000; ++i)
    {
        struct arena record = *env->mem_2;
        const v3 hw = V3(RngF32Range(1e-3f, 1e3f), RngF32Range(1e-3f, 1e3f), RngF32Range(1e-3f, 1e3f));
        const struct dcel box = DcelBox(env->mem_2, hw);
        TEST_TRUE(box.v != stub.v && box.f_count == 6 && box.e_count == 24 && box.v_count == 8);
        DcelAssertTopology(&box);
        TEST_TRUE(DcelTestV3Equal(box.bbox.min, V3Negate(hw)) && DcelTestV3Equal(box.bbox.max, hw));

        for (u32 fi = 0; fi < box.f_count; ++fi)
        {
            const v3 n = box.plane[fi].normal;
            const u32 axis = (n.x != 0.0f) ? 0 : ((n.y != 0.0f) ? 1 : 2);
            TEST_TRUE(F32Abs(F32Abs(n.buf[axis]) - 1.0f) <= 2.0f*F32_EPSILON);
            TEST_TRUE(n.buf[(axis + 1) % 3] == 0.0f && n.buf[(axis + 2) % 3] == 0.0f);
            TEST_TRUE(F32Abs(box.plane[fi].signed_distance - hw.buf[axis]) <= 4.0f*F32_EPSILON*hw.buf[axis]);
            for (u32 vi = 0; vi < box.v_count; ++vi)
            {
                TEST_TRUE(PlanePointSignedDistance(box.plane + fi, box.v[vi]) <= 0.0f);
            }
            for (u32 k = 0; k < box.f[fi].count; ++k)
            {
                TEST_TRUE(PlanePointSignedDistance(box.plane + fi, box.v[box.e[box.f[fi].first + k].origin]) == 0.0f);
            }

            /* the support face of n is this face */
            TEST_EQUAL(fi, DcelSupportFace(&box, n));
        }

        for (u32 ei = 0; ei < box.e_count; ++ei)
        {
            const v3 dir = DcelEdgeDirection(&box, ei);
            const v3 unit = DcelEdgeUnitDirection(&box, ei);
            const u32 axis = (dir.x != 0.0f) ? 0 : ((dir.y != 0.0f) ? 1 : 2);
            TEST_TRUE(F32Abs(dir.buf[axis]) == 2.0f*hw.buf[axis]);
            TEST_TRUE(dir.buf[(axis + 1) % 3] == 0.0f && dir.buf[(axis + 2) % 3] == 0.0f);
            TEST_TRUE(F32Abs(unit.buf[axis] - ((dir.buf[axis] < 0.0f) ? -1.0f : 1.0f)) <= 2.0f*F32_EPSILON);
            TEST_TRUE(unit.buf[(axis + 1) % 3] == 0.0f && unit.buf[(axis + 2) % 3] == 0.0f);
        }
        *env->mem_2 = record;
    }

#ifndef DS_DEBUG
    const v3 invalid[] =
    {
        V3(0.0f, 1.0f, 1.0f), V3(-1.0f, 1.0f, 1.0f), V3(1.0f, -1.0f, -1.0f), V3(1.0f, 1.0f, F32Nan()),
    };
    for (u32 i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i)
    {
        const struct dcel box = DcelBox(env->mem_2, invalid[i]);
        TEST_TRUE(box.v == stub.v && box.plane == stub.plane);
    }
#endif

	return output;
}

/*
 * DcelPlanesTryCompute on the triangle stub: a triangle gives opposite unit normals (within 16 eps L^2 / |n| of the f64
 * normal, L the longest edge, |n| twice the area) and planes through its vertices; collinear points and an area vector
 * beyond f32 (coordinates 1e20) fail. DcelBboxCompute of no vertices is the zero box.
 */
struct test_Output DcelPlanesTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    struct plane pl[2];
    struct dcel tri = DcelTriStub();
    tri.plane = pl;

    for (u32 i = 0; i < 10000; ++i)
    {
        v3 v[3];
        for (u32 k = 0; k < 3; ++k)
        {
            v[k] = V3(RngF32Range(-10.0f, 10.0f), RngF32Range(-10.0f, 10.0f), RngF32Range(-10.0f, 10.0f));
        }
        tri.v = v;
        v3 n;
        if (!TriCcwNormalTryCompute(&n, v[0], v[1], v[2]))
        {
            continue;
        }
        TEST_EQUAL(1, DcelPlanesTryCompute(&tri));

        const f64 a[3] = { (f64) v[1].x - v[0].x, (f64) v[1].y - v[0].y, (f64) v[1].z - v[0].z };
        const f64 b[3] = { (f64) v[2].x - v[0].x, (f64) v[2].y - v[0].y, (f64) v[2].z - v[0].z };
        const f64 r[3] = { a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0] };
        const f64 r_len = (f64) F32Sqrt((f32) (r[0]*r[0] + r[1]*r[1] + r[2]*r[2]));
        const f64 c[3] = { (f64) v[2].x - v[1].x, (f64) v[2].y - v[1].y, (f64) v[2].z - v[1].z };
        const f64 a_sq = a[0]*a[0] + a[1]*a[1] + a[2]*a[2];
        const f64 b_sq = b[0]*b[0] + b[1]*b[1] + b[2]*b[2];
        const f64 c_sq = c[0]*c[0] + c[1]*c[1] + c[2]*c[2];
        const f64 l_sq = (a_sq > b_sq) ? ((a_sq > c_sq) ? a_sq : c_sq) : ((b_sq > c_sq) ? b_sq : c_sq);
        /* r_len is rounded to f32 (no f64 sqrt in the engine): its eps is covered by the bound */
        const f64 tol = 16.0*F32_EPSILON*l_sq / r_len;
        for (u32 f = 0; f < 2; ++f)
        {
            const f64 sign = (f == 0) ? 1.0 : -1.0;
            const v3 m = pl[f].normal;
            const f64 err[3] = { m.x - sign*r[0]/r_len, m.y - sign*r[1]/r_len, m.z - sign*r[2]/r_len };
            TEST_TRUE(err[0]*err[0] + err[1]*err[1] + err[2]*err[2] <= tol*tol);
            for (u32 k = 0; k < 3; ++k)
            {
                TEST_TRUE(F32Abs(PlanePointSignedDistance(pl + f, v[k])) <= 64.0f*F32_EPSILON*10.0f);
            }
        }
    }

    v3 collinear[3] = { V3(0.0f, 0.0f, 0.0f), V3(1.0f, 1.0f, 1.0f), V3(2.0f, 2.0f, 2.0f) };
    tri.v = collinear;
    TEST_EQUAL(0, DcelPlanesTryCompute(&tri));
    v3 huge[3] = { V3(0.0f, 0.0f, 0.0f), V3(1e20f, 0.0f, 0.0f), V3(0.0f, 1e20f, 0.0f) };
    tri.v = huge;
    TEST_EQUAL(0, DcelPlanesTryCompute(&tri));

    tri.v_count = 0;
    DcelBboxCompute(&tri);
    TEST_TRUE(DcelTestV3Equal(tri.bbox.min, V3Zero()) && DcelTestV3Equal(tri.bbox.max, V3Zero()));

	return output;
}

/********************************** Voronoi API **********************************/

/*
SegmentVoronoi, TriVoronoi, TetrahedronVoronoi and their origin variants (ds_voronoi.h) against a double reference:
the closest point over every vertex, edge, face and the interior, computed on the same f32 inputs. Origin variants
get the vertices relative to the query (rounded to f32), as GJK passes them.

Checks for every result r != VORONOI_INVALID, c = sum w[i] t[i] (double), d = |c - p|, d_ref = |c_ref - p|:
    contract    w finite, in [0, 1], |sum w - 1| <= 8 u; bit i of r clear => w[i] == 0; r only names vertices
                (so c lies on the reported feature, within the simplex)
    distance    d <= d_ref + tol
    point       |c - c_ref| <= tol + sqrt(2 d_ref tol + tol^2): for c on a convex set, (c - c_ref).(p - c_ref) <= 0
                gives |c - c_ref|^2 <= d^2 - d_ref^2. The position is ill-conditioned away from the simplex: two
                features nearly as close (a coplanar tetrahedron's overlapping faces) differ by sqrt(d tol).
    tol = VD_TOLERANCE u (S kappa + A),  S = max |t[i] - p|,  A = max(|t[i]|, |p|) (c's rounding in absolute
    coordinates), kappa = max(1, S / |e|, S^2 / |n|) over the non-degenerate edges and faces, and S^3 / |6 V|
    for results inside a tetrahedron: the weights' conditioning (a sliver's plane is barely defined, so its
    weights are too; a flat tetrahedron's volume does not condition its face points).

Families (each for all six functions):
    well-shaped     random, scales 1e-3 .. 1e3, translations up to 1e4 scale                VORONOI_INVALID fails
    slivers         an edge, face or volume 1e-2 .. 1e-6 of the scale                      VORONOI_INVALID counted
    degenerate      exactly coincident, collinear, coplanar (lattice)                      VORONOI_INVALID counted
    near-flat       tetrahedra flat to 1e-3 .. 1e-9 (and 0) of the scale
Tetrahedron errors in the sliver, degenerate and near-flat families are counted, not failed: nearly flat tetrahedra can
give the wrong face (WARNING in ds_voronoi.h; agent_notes/gjk_design.txt).
Queries: every vertex, edge (midpoint, random), face and interior point, moved by 0 .. 1e-1 scale along a random
direction, a face normal or an edge; far points. Over a vertex along the normal is the case that fell through
every TriVoronoi test before the edge fallback (numerators of rounding size disagree in sign).
Exact cases: integer simplices, queries on and beside the features: exact regions and weights.
*/

#define VD_TOLERANCE 64.0

struct vd3 { f64 x, y, z; };

static struct vd3 Vd3(const f64 x, const f64 y, const f64 z) { struct vd3 r = { x, y, z }; return r; }
static struct vd3 Vd3V3(const v3 v) { return Vd3(v.x, v.y, v.z); }
static struct vd3 Vd3Add(const struct vd3 a, const struct vd3 b) { return Vd3(a.x + b.x, a.y + b.y, a.z + b.z); }
static struct vd3 Vd3Sub(const struct vd3 a, const struct vd3 b) { return Vd3(a.x - b.x, a.y - b.y, a.z - b.z); }
static struct vd3 Vd3Scale(const struct vd3 a, const f64 s) { return Vd3(s*a.x, s*a.y, s*a.z); }
static f64 Vd3Dot(const struct vd3 a, const struct vd3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
static struct vd3 Vd3Cross(const struct vd3 a, const struct vd3 b) { return Vd3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x); }
static f64 VdMax(const f64 a, const f64 b) { return (a > b) ? a : b; }
static f64 VdAbs(const f64 a) { return (a < 0.0) ? -a : a; }
static v3 VdRound(const struct vd3 a) { return V3((f32) a.x, (f32) a.y, (f32) a.z); }

static f64 VdSqrt(const f64 x)
{
    if (!(x > 0.0))
    {
        return 0.0;
    }
    f64 s = (f64) F32Sqrt((f32) x);
    if (s == 0.0)
    {
        return 0.0;
    }
    s = 0.5*(s + x/s);
    s = 0.5*(s + x/s);
    return s;
}

static f64 Vd3Length(const struct vd3 a) { return VdSqrt(Vd3Dot(a, a)); }

static struct vd3 VdUnit(void)
{
    const v3 u = V3UnitSphere();
    struct vd3 d = Vd3V3(u);
    return Vd3Scale(d, 1.0 / Vd3Length(d));
}

static struct vd3 VdBall(void)
{
    return Vd3Scale(VdUnit(), (f64) RngF32Normalized());
}

static void VdCandidate(struct vd3 *best, f64 *best_dist_sq, const struct vd3 c, const struct vd3 p)
{
    const struct vd3 d = Vd3Sub(c, p);
    const f64 dist_sq = Vd3Dot(d, d);
    if (dist_sq < *best_dist_sq)
    {
        *best_dist_sq = dist_sq;
        *best = c;
    }
}

/* closest point of the simplex t[i], bit i of mask set, to p: brute force over all its features */
static struct vd3 VdClosest(const struct vd3 *t, const u32 count, const u32 mask, const struct vd3 p)
{
    struct vd3 best = p;
    f64 best_dist_sq = 1e300;
    u32 idx[4];
    u32 k = 0;
    for (u32 i = 0; i < count; ++i)
    {
        if (mask & (1u << i))
        {
            idx[k++] = i;
        }
    }

    for (u32 i = 0; i < k; ++i)
    {
        VdCandidate(&best, &best_dist_sq, t[idx[i]], p);
    }

    for (u32 i = 0; i < k; ++i)
    for (u32 j = i + 1; j < k; ++j)
    {
        const struct vd3 a = t[idx[i]];
        const struct vd3 e = Vd3Sub(t[idx[j]], a);
        const f64 ee = Vd3Dot(e, e);
        if (ee > 0.0)
        {
            f64 s = Vd3Dot(Vd3Sub(p, a), e) / ee;
            s = (s < 0.0) ? 0.0 : (s > 1.0) ? 1.0 : s;
            VdCandidate(&best, &best_dist_sq, Vd3Add(a, Vd3Scale(e, s)), p);
        }
    }

    for (u32 i = 0; i < k; ++i)
    for (u32 j = i + 1; j < k; ++j)
    for (u32 l = j + 1; l < k; ++l)
    {
        const struct vd3 a = t[idx[i]];
        const struct vd3 e1 = Vd3Sub(t[idx[j]], a);
        const struct vd3 e2 = Vd3Sub(t[idx[l]], a);
        const struct vd3 d = Vd3Sub(p, a);
        const f64 a11 = Vd3Dot(e1, e1), a12 = Vd3Dot(e1, e2), a22 = Vd3Dot(e2, e2);
        const f64 b1 = Vd3Dot(e1, d), b2 = Vd3Dot(e2, d);
        const f64 det = a11*a22 - a12*a12;
        /* double rounding noise of a degenerate face is no plane */
        if (det > 1e-24*a11*a22)
        {
            const f64 u = (b1*a22 - b2*a12) / det;
            const f64 v = (a11*b2 - a12*b1) / det;
            if (u >= 0.0 && v >= 0.0 && u + v <= 1.0)
            {
                VdCandidate(&best, &best_dist_sq, Vd3Add(a, Vd3Add(Vd3Scale(e1, u), Vd3Scale(e2, v))), p);
            }
        }
    }

    if (k == 4)
    {
        const struct vd3 *T[4] = { t + idx[0], t + idx[1], t + idx[2], t + idx[3] };
        const struct vd3 e1 = Vd3Sub(*T[1], *T[0]), e2 = Vd3Sub(*T[2], *T[0]), e3 = Vd3Sub(*T[3], *T[0]);
        const f64 v = Vd3Dot(e1, Vd3Cross(e2, e3));
        /* double rounding noise of a flat tetrahedron is no volume */
        if (VdAbs(v) > 1e-13*Vd3Length(e1)*Vd3Length(e2)*Vd3Length(e3))
        {
            u32 inside = 1;
            for (u32 i = 0; i < 4; ++i)
            {
                struct vd3 q[4] = { *T[0], *T[1], *T[2], *T[3] };
                q[i] = p;
                const f64 vi = Vd3Dot(Vd3Sub(q[1], q[0]), Vd3Cross(Vd3Sub(q[2], q[0]), Vd3Sub(q[3], q[0])));
                inside &= (vi*v >= 0.0);
            }
            if (inside)
            {
                VdCandidate(&best, &best_dist_sq, p, p);
            }
        }
    }

    return best;
}

/*
 * the weights' conditioning: max(1, S / |e|, S^2 / |n|) over non-degenerate edges and faces, and S^3 / |6 V| for a
 * result inside the tetrahedron (its weights are volume ratios; any other result is a face, edge or vertex point)
 */
static f64 VdKappa(const struct vd3 *t, const u32 count, const f64 S, const u32 inside)
{
    f64 kappa = 1.0;
    for (u32 i = 0; i < count; ++i)
    for (u32 j = i + 1; j < count; ++j)
    {
        const f64 len = Vd3Length(Vd3Sub(t[j], t[i]));
        if (len > 0.0)
        {
            kappa = VdMax(kappa, S / len);
        }
        for (u32 l = j + 1; l < count; ++l)
        {
            const f64 n = Vd3Length(Vd3Cross(Vd3Sub(t[j], t[i]), Vd3Sub(t[l], t[i])));
            if (n > 0.0)
            {
                kappa = VdMax(kappa, S*S / n);
            }
        }
    }
    if (inside)
    {
        const f64 v = VdAbs(Vd3Dot(Vd3Sub(t[1], t[0]), Vd3Cross(Vd3Sub(t[2], t[0]), Vd3Sub(t[3], t[0]))));
        if (v > 0.0)
        {
            kappa = VdMax(kappa, S*S*S / v);
        }
    }
    return kappa;
}

struct vd_Stats
{
    u64 n;
    u64 invalid;
    u64 zero_weight_bits;   /* a named vertex with weight 0 (allowed: underflow) */
    f64 worst_distance;     /* (d - d_ref) / (u (S kappa + A)) */
    f64 worst_point;        /* |c - c_ref| / its bound at VD_TOLERANCE = 1 */
    u64 counted;            /* beyond tolerance, counted only (known limitation: nearly flat tetrahedra) */
};

static enum voronoi VdCall(f32 w[4], const v3 *t, const u32 count, const u32 origin, const v3 p)
{
    switch (count)
    {
        case 2:  return (origin) ? SegmentOriginVoronoi(w, t[0], t[1]) : SegmentVoronoi(w, t[0], t[1], p);
        case 3:  return (origin) ? TriOriginVoronoi(w, t[0], t[1], t[2]) : TriVoronoi(w, t[0], t[1], t[2], p);
        default: return (origin) ? TetrahedronOriginVoronoi(w, t[0], t[1], t[2], t[3]) : TetrahedronVoronoi(w, t[0], t[1], t[2], t[3], p);
    }
}

/*
 * One query: t_d, p_d in double are rounded to the function's f32 inputs (origin: t - p). Returns 0 on a failed
 * check (printed); an INVALID result returns invalid_ok; distance/point errors beyond tolerance are only counted
 * when count_only.
 */
static u32 VdQuery(struct vd_Stats *stats, const struct vd3 *t_d, const u32 count, const u32 origin, const struct vd3 p_d, const u32 invalid_ok, const u32 count_only)
{
    v3 t[4];
    struct vd3 tr[4];
    v3 p = V3(0.0f, 0.0f, 0.0f);
    for (u32 i = 0; i < count; ++i)
    {
        t[i] = (origin) ? VdRound(Vd3Sub(t_d[i], p_d)) : VdRound(t_d[i]);
        tr[i] = Vd3V3(t[i]);
    }
    if (!origin)
    {
        p = VdRound(p_d);
    }
    const struct vd3 pr = Vd3V3(p);

    f32 w[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    const enum voronoi r = VdCall(w, t, count, origin, p);
    stats->n += 1;
    if (r == VORONOI_INVALID)
    {
        stats->invalid += 1;
        if (!invalid_ok)
        {
            fprintf(stderr, "Voronoi (%u vertices, origin %u): VORONOI_INVALID\n", count, origin);
            goto fail;
        }
        return 1;
    }

    f64 S = 0.0;
    f64 A = Vd3Length(pr);
    for (u32 i = 0; i < count; ++i)
    {
        S = VdMax(S, Vd3Length(Vd3Sub(tr[i], pr)));
        A = VdMax(A, Vd3Length(tr[i]));
    }
    const f64 scale = F32_EPSILON * (VdMax(S, 1e-30) * VdKappa(tr, count, S, r == VORONOI_T0123) + A);
    const f64 tol = VD_TOLERANCE * scale;

    /* contract */
    f64 sum = 0.0;
    struct vd3 c = Vd3(0.0, 0.0, 0.0);
    for (u32 i = 0; i < count; ++i)
    {
        const u32 named = (r >> i) & 0x1;
        if (!(w[i] >= 0.0f && w[i] <= 1.0f) || (!named && w[i] != 0.0f))
        {
            fprintf(stderr, "Voronoi (%u vertices, origin %u): weight %u = %g, region 0x%x\n", count, origin, i, (f64) w[i], (u32) r);
            goto fail;
        }
        stats->zero_weight_bits += (named && w[i] == 0.0f);
        sum += (f64) w[i];
        c = Vd3Add(c, Vd3Scale(tr[i], (f64) w[i]));
    }
    if ((u32) r >> count || VdAbs(sum - 1.0) > 8.0 * F32_EPSILON)
    {
        fprintf(stderr, "Voronoi (%u vertices, origin %u): region 0x%x, weight sum - 1 = %g\n", count, origin, (u32) r, sum - 1.0);
        goto fail;
    }

    /* distance and point */
    const struct vd3 c_ref = VdClosest(tr, count, (1u << count) - 1, pr);
    const f64 d = Vd3Length(Vd3Sub(c, pr));
    const f64 d_ref = Vd3Length(Vd3Sub(c_ref, pr));
    const f64 point_error = Vd3Length(Vd3Sub(c, c_ref));
    stats->worst_distance = VdMax(stats->worst_distance, (d - d_ref) / scale);
    stats->worst_point = VdMax(stats->worst_point, point_error / (scale + VdSqrt(2.0*d_ref*scale + scale*scale)));
    if (d <= d_ref + tol && point_error <= tol + VdSqrt(2.0*d_ref*tol + tol*tol))
    {
        return 1;
    }
    if (count_only)
    {
        stats->counted += 1;
        return 1;
    }
    fprintf(stderr, "Voronoi (%u vertices, origin %u): region 0x%x, d - d_ref = %g, |c - c_ref| = %g, d_ref = %g, tol %g\n",
        count, origin, (u32) r, d - d_ref, point_error, d_ref, tol);

fail:
    for (u32 i = 0; i < count; ++i)
    {
        fprintf(stderr, "\tt[%u] = (%a, %a, %a)\n", i, (f64) t[i].x, (f64) t[i].y, (f64) t[i].z);
    }
    fprintf(stderr, "\tp = (%a, %a, %a), w = (%g, %g, %g, %g)\n", (f64) p.x, (f64) p.y, (f64) p.z, (f64) w[0], (f64) w[1], (f64) w[2], (f64) w[3]);
    return 0;
}

/* every vertex, edge (midpoint, random), face and interior point, offset along random/normal/edge directions; far points */
static u32 VdQueries(struct vd_Stats stats[2], const struct vd3 *t, const u32 count, const f64 scale, const u32 invalid_ok, const u32 count_only)
{
    static const f64 offset[] = { 0.0, 1e-9, 1e-7, 1e-5, 1e-3, 1e-1 };
    struct vd3 point[16];
    u32 point_count = 0;
    for (u32 i = 0; i < count; ++i)
    {
        point[point_count++] = t[i];
        for (u32 j = i + 1; j < count; ++j)
        {
            const struct vd3 e = Vd3Sub(t[j], t[i]);
            point[point_count++] = Vd3Add(t[i], Vd3Scale(e, (j == i + 1) ? 0.5 : (f64) RngF32Normalized()));
        }
    }
    if (count >= 3)
    {
        f64 a = (f64) RngF32Normalized(), b = (f64) RngF32Normalized();
        if (a + b > 1.0)
        {
            a = 1.0 - a;
            b = 1.0 - b;
        }
        point[point_count++] = Vd3Add(t[0], Vd3Add(Vd3Scale(Vd3Sub(t[1], t[0]), a), Vd3Scale(Vd3Sub(t[2], t[0]), b)));
    }
    if (count == 4)
    {
        f64 l[4], l_sum = 0.0;
        for (u32 i = 0; i < 4; ++i)
        {
            l[i] = (f64) RngF32Normalized() + 1e-3;
            l_sum += l[i];
        }
        struct vd3 q = Vd3(0.0, 0.0, 0.0);
        for (u32 i = 0; i < 4; ++i)
        {
            q = Vd3Add(q, Vd3Scale(t[i], l[i] / l_sum));
        }
        point[point_count++] = q;
    }

    for (u32 i = 0; i < point_count; ++i)
    for (u32 o = 0; o < sizeof(offset) / sizeof(offset[0]); ++o)
    {
        /* direction: random, a face normal (or a random unit for segments), or an edge */
        struct vd3 dir = VdUnit();
        const u64 kind = RngU64Range(0, 2);
        if (kind == 1 && count >= 3)
        {
            const u32 f = (count == 3) ? 0 : (u32) RngU64Range(0, 3);
            const u32 a = f, b = (f + 1) % count, c = (f + 2) % count;
            const struct vd3 n = Vd3Cross(Vd3Sub(t[b], t[a]), Vd3Sub(t[c], t[a]));
            const f64 n_len = Vd3Length(n);
            if (n_len > 0.0)
            {
                dir = Vd3Scale(n, 1.0 / n_len);
            }
        }
        else if (kind == 2)
        {
            const u32 a = (u32) RngU64Range(0, count - 1), b = (a + 1) % count;
            const struct vd3 e = Vd3Sub(t[b], t[a]);
            const f64 e_len = Vd3Length(e);
            if (e_len > 0.0)
            {
                dir = Vd3Scale(e, 1.0 / e_len);
            }
        }
        const f64 sign = (RngU64Range(0, 1)) ? 1.0 : -1.0;
        const struct vd3 p = Vd3Add(point[i], Vd3Scale(dir, sign*offset[o]*scale));
        for (u32 origin = 0; origin < 2; ++origin)
        {
            if (!VdQuery(stats + origin, t, count, origin, p, invalid_ok, count_only))
            {
                return 0;
            }
        }
    }

    for (u32 i = 0; i < 2; ++i)
    {
        const struct vd3 p = Vd3Add(t[0], Vd3Scale(VdUnit(), ((i == 0) ? 3.0 : 1e2)*scale));
        for (u32 origin = 0; origin < 2; ++origin)
        {
            if (!VdQuery(stats + origin, t, count, origin, p, invalid_ok, count_only))
            {
                return 0;
            }
        }
    }

    return 1;
}

static void VdStatsPrint(const char *family, const u32 count, const struct vd_Stats stats[2])
{
    const char *id[] = { "", "segment", "triangle", "tetrahedron" };
    for (u32 origin = 0; origin < 2; ++origin)
    {
        fprintf(stdout, "\t%s, %s%s: %llu queries, INVALID %llu, named zero weights %llu, beyond tolerance (counted) %llu; worst ratio to the bound at tolerance 1: distance %.3g, point %.3g\n",
            family, id[count - 1], (origin) ? " (origin)" : "", (unsigned long long) stats[origin].n, (unsigned long long) stats[origin].invalid,
            (unsigned long long) stats[origin].zero_weight_bits, (unsigned long long) stats[origin].counted, stats[origin].worst_distance, stats[origin].worst_point);
    }
}

static f64 VdScale(void)
{
    static const f64 decade[] = { 1e-3, 1e-2, 1e-1, 1.0, 1e1, 1e2, 1e3 };
    return decade[RngU64Range(0, 6)] * (1.0 + 2.0*(f64) RngF32Normalized());
}

/* random well-shaped simplices (min edge, area, volume bounded by the max edge) */
struct test_Output VoronoiWellShapedTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };
    for (u32 count = 2; count <= 4; ++count)
    {
        struct vd_Stats stats[2] = { 0 };
        for (u32 i = 0; i < 4000; ++i)
        {
            const f64 scale = VdScale();
            static const f64 far_scale[] = { 0.0, 1.0, 1e2, 1e4 };
            const struct vd3 T = Vd3Scale(VdUnit(), far_scale[RngU64Range(0, 3)]*scale);
            struct vd3 t[4];
            for (;;)
            {
                f64 e_max = 0.0;
                for (u32 k = 0; k < count; ++k)
                {
                    t[k] = Vd3Add(T, Vd3Scale(VdBall(), scale));
                }
                for (u32 k = 0; k < count; ++k)
                for (u32 l = k + 1; l < count; ++l)
                {
                    e_max = VdMax(e_max, Vd3Length(Vd3Sub(t[l], t[k])));
                }
                const struct vd3 e1 = Vd3Sub(t[1], t[0]);
                const f64 measure = (count == 2) ? Vd3Length(e1)
                                  : (count == 3) ? Vd3Length(Vd3Cross(e1, Vd3Sub(t[2], t[0])))
                                  : VdAbs(Vd3Dot(e1, Vd3Cross(Vd3Sub(t[2], t[0]), Vd3Sub(t[3], t[0]))));
                const f64 bound = (count == 2) ? 0.3*scale : (count == 3) ? 0.2*e_max*e_max : 0.05*e_max*e_max*e_max;
                if (measure >= bound)
                {
                    break;
                }
            }
            if (!VdQueries(stats, t, count, scale, 0, 0))
            {
                TEST_FAILURE;
            }
        }
        VdStatsPrint("well-shaped", count, stats);
    }
	return output;
}

/* slivers: a short edge, a flat face or a flat volume, 1e-2 .. 1e-6 of the scale */
struct test_Output VoronoiSliverTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };
    static const f64 eps[] = { 1e-2, 1e-4, 1e-6 };
    for (u32 count = 2; count <= 4; ++count)
    {
        struct vd_Stats stats[2] = { 0 };
        for (u32 i = 0; i < 4000; ++i)
        {
            const f64 scale = VdScale();
            const f64 e = eps[RngU64Range(0, 2)]*scale;
            struct vd3 t[4];
            for (u32 k = 0; k < count; ++k)
            {
                t[k] = Vd3Scale(VdBall(), scale);
            }
            const u64 kind = RngU64Range(0, 1);
            const u32 last = count - 1;
            if (count == 2 || kind == 0)
            {
                /* needle: the last vertex next to the first */
                t[last] = Vd3Add(t[0], Vd3Scale(VdUnit(), e));
            }
            else if (count == 3)
            {
                /* flat: t2 next to the line t0 t1 */
                const f64 a = (f64) RngF32Range(-0.5f, 1.5f);
                t[2] = Vd3Add(Vd3Add(t[0], Vd3Scale(Vd3Sub(t[1], t[0]), a)), Vd3Scale(VdUnit(), e));
            }
            else
            {
                /* flat: t3 next to the plane of t0 t1 t2 */
                const f64 a = (f64) RngF32Range(-0.5f, 1.0f), b = (f64) RngF32Range(-0.5f, 1.0f);
                t[3] = Vd3Add(Vd3Add(t[0], Vd3Add(Vd3Scale(Vd3Sub(t[1], t[0]), a), Vd3Scale(Vd3Sub(t[2], t[0]), b))), Vd3Scale(VdUnit(), e));
            }
            if (!VdQueries(stats, t, count, scale, 1, count == 4))
            {
                TEST_FAILURE;
            }
        }
        VdStatsPrint("slivers", count, stats);
    }
	return output;
}

/* exactly coincident, collinear and coplanar simplices on a lattice (exact in f32) */
struct test_Output VoronoiDegenerateTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };
    for (u32 count = 2; count <= 4; ++count)
    {
        struct vd_Stats stats[2] = { 0 };
        for (u32 i = 0; i < 4000; ++i)
        {
            /* power of two scale: lattice points stay exact */
            const f64 scale = (f64) (1u << RngU64Range(0, 12)) / 64.0;
            struct vd3 base = Vd3((f64) RngU64Range(0, 64) - 32.0, (f64) RngU64Range(0, 64) - 32.0, (f64) RngU64Range(0, 64) - 32.0);
            struct vd3 d1 = Vd3((f64) RngU64Range(0, 8) - 4.0, (f64) RngU64Range(0, 8) - 4.0, (f64) RngU64Range(0, 8) - 4.0);
            struct vd3 d2 = Vd3((f64) RngU64Range(0, 8) - 4.0, (f64) RngU64Range(0, 8) - 4.0, (f64) RngU64Range(0, 8) - 4.0);
            struct vd3 t[4];
            const u64 kind = RngU64Range(0, 2);
            for (u32 k = 0; k < count; ++k)
            {
                const f64 m1 = (f64) RngU64Range(0, 6) - 3.0, m2 = (f64) RngU64Range(0, 6) - 3.0;
                /* 0: all coincident with t0 or each other (two distinct points at most); 1: collinear; 2: coplanar */
                t[k] = (kind == 0) ? Vd3Add(base, Vd3Scale(d1, (f64) RngU64Range(0, 1)))
                     : (kind == 1) ? Vd3Add(base, Vd3Scale(d1, m1))
                     : Vd3Add(base, Vd3Add(Vd3Scale(d1, m1), Vd3Scale(d2, m2)));
                t[k] = Vd3Scale(t[k], scale);
            }
            if (count == 2 && kind == 2)
            {
                t[1] = t[0];
            }
            if (!VdQueries(stats, t, count, 4.0*scale, 1, count == 4))
            {
                TEST_FAILURE;
            }
        }
        VdStatsPrint("degenerate", count, stats);
    }
	return output;
}

/*
 * Nearly flat tetrahedra: t3 = a point in the plane of t0 t1 t2 (inside or outside the triangle) moved off it by
 * 1e-3 .. 1e-9 of the scale, or not at all (flat until rounded). Besides the standard queries: points on the
 * tetrahedron and past it, 1e-6 off (where the face filter on noisy signs dropped the right faces), and points
 * beside the plane, laterally outside (where noisy volume numerators can all come out > 0, "inside").
 */
struct test_Output VoronoiNearFlatTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };
    static const f64 flatness[] = { 1e-3, 1e-5, 1e-6, 1e-7, 1e-8, 1e-9, 0.0 };
    static const f64 side[] = { 0.0, 1e-7, -1e-7, 1e-3, -1e-3 };
    struct vd_Stats stats[2] = { 0 };
    for (u32 i = 0; i < 6000; ++i)
    {
        const f64 scale = VdScale();
        static const f64 far_scale[] = { 0.0, 1.0, 1e2 };
        const struct vd3 T = Vd3Scale(VdUnit(), far_scale[RngU64Range(0, 2)]*scale);
        struct vd3 t[4];
        struct vd3 n;
        f64 n_len;
        do
        {
            for (u32 k = 0; k < 3; ++k)
            {
                t[k] = Vd3Add(T, Vd3Scale(VdBall(), scale));
            }
            n = Vd3Cross(Vd3Sub(t[1], t[0]), Vd3Sub(t[2], t[0]));
            n_len = Vd3Length(n);
        } while (n_len < 0.2*scale*scale);
        n = Vd3Scale(n, 1.0 / n_len);
        const f64 a = (f64) RngF32Range(-0.5f, 1.5f), b = (f64) RngF32Range(-0.5f, 1.5f);
        const struct vd3 in_plane = Vd3Add(t[0], Vd3Add(Vd3Scale(Vd3Sub(t[1], t[0]), a), Vd3Scale(Vd3Sub(t[2], t[0]), b)));
        t[3] = Vd3Add(in_plane, Vd3Scale(n, flatness[RngU64Range(0, 6)]*scale*((RngU64Range(0, 1)) ? 1.0 : -1.0)));

        if (!VdQueries(stats, t, 4, scale, 0, 1))
        {
            TEST_FAILURE;
        }

        /* on the tetrahedron (random weights), some extended past it, moved by 1e-6 of the scale */
        for (u32 k = 0; k < 16; ++k)
        {
            f64 l[4], l_sum = 0.0;
            for (u32 v = 0; v < 4; ++v)
            {
                l[v] = (f64) RngF32Normalized();
                l_sum += l[v];
            }
            struct vd3 q = Vd3(0.0, 0.0, 0.0);
            for (u32 v = 0; v < 4; ++v)
            {
                q = Vd3Add(q, Vd3Scale(t[v], l[v] / l_sum));
            }
            if (k % 3 == 0)
            {
                q = Vd3Add(q, Vd3Scale(Vd3Sub(t[RngU64Range(0, 3)], t[RngU64Range(0, 3)]), 2.0*(f64) RngF32Normalized()));
            }
            q = Vd3Add(q, Vd3Scale(VdUnit(), 1e-6*scale));
            for (u32 origin = 0; origin < 2; ++origin)
            {
                if (!VdQuery(stats + origin, t, 4, origin, q, 0, 1))
                {
                    TEST_FAILURE;
                }
            }
        }

        /* beside the plane, laterally outside */
        const struct vd3 centroid = Vd3Scale(Vd3Add(Vd3Add(t[0], t[1]), Vd3Add(t[2], t[3])), 0.25);
        for (u32 k = 0; k < 8; ++k)
        {
            struct vd3 d = VdUnit();
            d = Vd3Sub(d, Vd3Scale(n, Vd3Dot(d, n)));
            const f64 d_len = Vd3Length(d);
            if (d_len == 0.0)
            {
                continue;
            }
            const struct vd3 q = Vd3Add(Vd3Add(centroid, Vd3Scale(d, ((k & 1) ? 1.5 : 3.0)*scale / d_len)), Vd3Scale(n, side[k % 5]*scale));
            for (u32 origin = 0; origin < 2; ++origin)
            {
                if (!VdQuery(stats + origin, t, 4, origin, q, 0, 1))
                {
                    TEST_FAILURE;
                }
            }
        }
    }
    VdStatsPrint("near-flat", 4, stats);
	return output;
}

/* closest points of segments (p1, q1), (p2, q2) in double (Ericson 5.1.9, degenerate segments included) */
static f64 VdSegmentSegment(const struct vd3 p1, const struct vd3 q1, const struct vd3 p2, const struct vd3 q2)
{
    const struct vd3 d1 = Vd3Sub(q1, p1), d2 = Vd3Sub(q2, p2), r = Vd3Sub(p1, p2);
    const f64 a = Vd3Dot(d1, d1), e = Vd3Dot(d2, d2), f = Vd3Dot(d2, r);
    f64 s = 0.0, t = 0.0;
    if (a <= 0.0 && e <= 0.0)
    {
    }
    else if (a <= 0.0)
    {
        t = f / e;
        t = (t < 0.0) ? 0.0 : (t > 1.0) ? 1.0 : t;
    }
    else
    {
        const f64 c = Vd3Dot(d1, r);
        if (e <= 0.0)
        {
            s = -c / a;
            s = (s < 0.0) ? 0.0 : (s > 1.0) ? 1.0 : s;
        }
        else
        {
            const f64 b = Vd3Dot(d1, d2), denom = a*e - b*b;
            s = (denom > 0.0) ? (b*f - c*e) / denom : 0.0;
            s = (s < 0.0) ? 0.0 : (s > 1.0) ? 1.0 : s;
            t = (b*s + f) / e;
            if (t < 0.0)
            {
                t = 0.0;
                s = -c / a;
                s = (s < 0.0) ? 0.0 : (s > 1.0) ? 1.0 : s;
            }
            else if (t > 1.0)
            {
                t = 1.0;
                s = (b - c) / a;
                s = (s < 0.0) ? 0.0 : (s > 1.0) ? 1.0 : s;
            }
        }
    }
    return Vd3Length(Vd3Sub(Vd3Add(p1, Vd3Scale(d1, s)), Vd3Add(p2, Vd3Scale(d2, t))));
}

/* distance of segment (s0, s1) and triangle t in double: piercing, end points vs the triangle, edges vs the segment */
static f64 VdSegmentTriangle(const struct vd3 *t, const struct vd3 s0, const struct vd3 s1)
{
    const struct vd3 n = Vd3Cross(Vd3Sub(t[1], t[0]), Vd3Sub(t[2], t[0]));
    const f64 h0 = Vd3Dot(n, Vd3Sub(s0, t[0])), h1 = Vd3Dot(n, Vd3Sub(s1, t[0]));
    if ((h0 < 0.0 && h1 > 0.0) || (h0 > 0.0 && h1 < 0.0))
    {
        const struct vd3 x = Vd3Add(s0, Vd3Scale(Vd3Sub(s1, s0), h0 / (h0 - h1)));
        if (Vd3Length(Vd3Sub(VdClosest(t, 3, 0x7, x), x)) == 0.0)
        {
            return 0.0;
        }
    }
    f64 d = VdMax(0.0, Vd3Length(Vd3Sub(VdClosest(t, 3, 0x7, s0), s0)));
    d = (Vd3Length(Vd3Sub(VdClosest(t, 3, 0x7, s1), s1)) < d) ? Vd3Length(Vd3Sub(VdClosest(t, 3, 0x7, s1), s1)) : d;
    for (u32 a = 0; a < 3; ++a)
    {
        const f64 e = VdSegmentSegment(t[a], t[(a + 1) % 3], s0, s1);
        d = (e < d) ? e : d;
    }
    return d;
}

/*
 * TriSegmentVoronoi against VdSegmentTriangle: the contract (w as TriVoronoi, s in [0, 1]) and the distance of the
 * returned points, d <= d_ref + tol (tol as VdQuery, S = max |t_i - s_j|). Triangles: well-shaped, slivers, exact
 * lattice degenerate; segments: piercing, parallel just above and below, in the plane, through edges and vertices,
 * zero-length and short, far; both windings.
 */
struct test_Output VoronoiTriSegmentTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };
    const char *family_id[] = { "well-shaped", "slivers", "degenerate" };
    static const f64 offset[] = { 0.0, 1e-9, 1e-7, 1e-5, 1e-3, 1e-1, 1.0 };
    for (u32 family = 0; family < 3; ++family)
    {
        u64 n = 0, invalid = 0;
        f64 worst = 0.0;
        for (u32 i = 0; i < 20000; ++i)
        {
            const f64 scale = (family == 2) ? (f64) (1u << RngU64Range(0, 12)) / 64.0 : VdScale();
            struct vd3 t[3];
            if (family == 0)
            {
                for (;;)
                {
                    for (u32 k = 0; k < 3; ++k)
                    {
                        t[k] = Vd3Scale(VdBall(), scale);
                    }
                    const f64 e_max = VdMax(VdMax(Vd3Length(Vd3Sub(t[1], t[0])), Vd3Length(Vd3Sub(t[2], t[1]))), Vd3Length(Vd3Sub(t[0], t[2])));
                    if (Vd3Length(Vd3Cross(Vd3Sub(t[1], t[0]), Vd3Sub(t[2], t[0]))) >= 0.2*e_max*e_max)
                    {
                        break;
                    }
                }
            }
            else if (family == 1)
            {
                static const f64 eps[] = { 1e-2, 1e-4, 1e-6 };
                t[0] = Vd3Scale(VdBall(), scale);
                t[1] = Vd3Scale(VdBall(), scale);
                const f64 a = (f64) RngF32Range(-0.5f, 1.5f);
                t[2] = Vd3Add(Vd3Add(t[0], Vd3Scale(Vd3Sub(t[1], t[0]), a)), Vd3Scale(VdUnit(), eps[RngU64Range(0, 2)]*scale));
            }
            else
            {
                const struct vd3 base = Vd3((f64) RngU64Range(0, 64) - 32.0, (f64) RngU64Range(0, 64) - 32.0, (f64) RngU64Range(0, 64) - 32.0);
                const struct vd3 d1 = Vd3((f64) RngU64Range(0, 8) - 4.0, (f64) RngU64Range(0, 8) - 4.0, (f64) RngU64Range(0, 8) - 4.0);
                for (u32 k = 0; k < 3; ++k)
                {
                    t[k] = Vd3Scale(Vd3Add(base, Vd3Scale(d1, (f64) RngU64Range(0, 6) - 3.0)), scale);
                }
            }

            /* segment: through a point of the triangle (vertex, edge, face) or near it, in a chosen direction */
            const struct vd3 normal = Vd3Cross(Vd3Sub(t[1], t[0]), Vd3Sub(t[2], t[0]));
            const f64 normal_len = Vd3Length(normal);
            const struct vd3 unit_n = (normal_len > 0.0) ? Vd3Scale(normal, 1.0 / normal_len) : VdUnit();
            f64 a = (f64) RngF32Normalized(), b = (f64) RngF32Normalized();
            if (a + b > 1.0)
            {
                a = 1.0 - a;
                b = 1.0 - b;
            }
            const u64 at = RngU64Range(0, 2);
            struct vd3 q = (at == 0) ? t[RngU64Range(0, 2)]
                         : (at == 1) ? Vd3Add(t[0], Vd3Scale(Vd3Sub(t[1], t[0]), a))
                         : Vd3Add(t[0], Vd3Add(Vd3Scale(Vd3Sub(t[1], t[0]), a), Vd3Scale(Vd3Sub(t[2], t[0]), b)));
            q = Vd3Add(q, Vd3Scale(unit_n, ((RngU64Range(0, 1)) ? 1.0 : -1.0)*offset[RngU64Range(0, 6)]*scale));
            /* direction: random, along the normal (piercing), in the plane (parallel) */
            const u64 kind = RngU64Range(0, 2);
            struct vd3 dir = VdUnit();
            if (kind == 1)
            {
                dir = unit_n;
            }
            else if (kind == 2)
            {
                dir = Vd3Sub(dir, Vd3Scale(unit_n, Vd3Dot(dir, unit_n)));
                const f64 dir_len = Vd3Length(dir);
                dir = (dir_len > 0.0) ? Vd3Scale(dir, 1.0 / dir_len) : unit_n;
            }
            static const f64 length[] = { 0.0, 1e-6, 1e-2, 0.5, 2.0, 10.0 };
            const f64 len = length[RngU64Range(0, 5)]*scale;
            const f64 along = (f64) RngF32Range(-1.0f, 1.0f);
            const struct vd3 s0_d = Vd3Add(q, Vd3Scale(dir, (along - 1.0)*0.5*len));
            const struct vd3 s1_d = Vd3Add(q, Vd3Scale(dir, (along + 1.0)*0.5*len));

            v3 tf[3];
            struct vd3 tr[3];
            const u32 flip = (u32) RngU64Range(0, 1);
            for (u32 k = 0; k < 3; ++k)
            {
                tf[k] = VdRound(t[(flip) ? 2 - k : k]);
                tr[k] = Vd3V3(tf[k]);
            }
            const v3 s0 = VdRound(s0_d), s1 = VdRound(s1_d);
            const struct vd3 sr0 = Vd3V3(s0), sr1 = Vd3V3(s1);

            f32 w[3] = { -1.0f, -1.0f, -1.0f };
            f32 sp = -1.0f;
            const enum voronoi r = TriSegmentVoronoi(w, &sp, tf[0], tf[1], tf[2], s0, s1);
            n += 1;
            if (r == VORONOI_INVALID)
            {
                invalid += 1;
                fprintf(stderr, "TriSegmentVoronoi: VORONOI_INVALID\n");
                TEST_FAILURE;
            }

            f64 S = 0.0, A = VdMax(Vd3Length(sr0), Vd3Length(sr1));
            for (u32 k = 0; k < 3; ++k)
            {
                S = VdMax(S, VdMax(Vd3Length(Vd3Sub(tr[k], sr0)), Vd3Length(Vd3Sub(tr[k], sr1))));
                A = VdMax(A, Vd3Length(tr[k]));
            }
            const f64 unit = F32_EPSILON * (VdMax(S, 1e-30) * VdKappa(tr, 3, VdMax(S, 1e-30), 0) + A);
            f64 sum = 0.0;
            struct vd3 c_t = Vd3(0.0, 0.0, 0.0);
            u32 ok = (0.0f <= sp && sp <= 1.0f) && ((u32) r >> 3) == 0;
            for (u32 k = 0; k < 3; ++k)
            {
                ok &= (w[k] >= 0.0f && w[k] <= 1.0f) && (((r >> k) & 0x1) || w[k] == 0.0f);
                sum += (f64) w[k];
                c_t = Vd3Add(c_t, Vd3Scale(tr[k], (f64) w[k]));
            }
            ok &= VdAbs(sum - 1.0) <= 8.0 * F32_EPSILON;
            const struct vd3 c_s = Vd3Add(sr0, Vd3Scale(Vd3Sub(sr1, sr0), (f64) sp));
            const f64 d = Vd3Length(Vd3Sub(c_t, c_s));
            const f64 d_ref = VdSegmentTriangle(tr, sr0, sr1);
            worst = VdMax(worst, (d - d_ref) / unit);
            if (!ok || d > d_ref + VD_TOLERANCE*unit)
            {
                fprintf(stderr, "TriSegmentVoronoi (%s): region 0x%x, w = (%g, %g, %g), s = %g, d = %g, d_ref = %g, tol %g\n",
                    family_id[family], (u32) r, (f64) w[0], (f64) w[1], (f64) w[2], (f64) sp, d, d_ref, VD_TOLERANCE*unit);
                for (u32 k = 0; k < 3; ++k)
                {
                    fprintf(stderr, "\tt[%u] = (%a, %a, %a)\n", k, (f64) tf[k].x, (f64) tf[k].y, (f64) tf[k].z);
                }
                fprintf(stderr, "\ts0 = (%a, %a, %a), s1 = (%a, %a, %a)\n", (f64) s0.x, (f64) s0.y, (f64) s0.z, (f64) s1.x, (f64) s1.y, (f64) s1.z);
                TEST_FAILURE;
            }
        }
        fprintf(stdout, "\ttriangle-segment, %s: %llu queries, INVALID %llu; worst (d - d_ref) / (u (S kappa + A)): %.3g\n",
            family_id[family], (unsigned long long) n, (unsigned long long) invalid, worst);
    }

    /* exact: triangle (0,0,0) (4,0,0) (0,4,0) */
    {
        const v3 t0 = V3(0.0f, 0.0f, 0.0f), t1 = V3(4.0f, 0.0f, 0.0f), t2 = V3(0.0f, 4.0f, 0.0f);
        f32 w[3], sp;
        /* piercing the face */
        TEST_TRUE(TriSegmentVoronoi(w, &sp, t0, t1, t2, V3(1.0f, 1.0f, -1.0f), V3(1.0f, 1.0f, 1.0f)) == VORONOI_F012 && sp == 0.5f && w[0] == 0.5f && w[1] == 0.25f && w[2] == 0.25f);
        /* in the plane, across edge 01: on the edge */
        TEST_TRUE(TriSegmentVoronoi(w, &sp, t0, t1, t2, V3(2.0f, -1.0f, 0.0f), V3(2.0f, -3.0f, 0.0f)) == VORONOI_E01 && sp == 0.0f && w[0] == 0.5f && w[1] == 0.5f);
        /* above vertex 0 along the normal: V0 at distance 1 */
        TEST_TRUE(TriSegmentVoronoi(w, &sp, t0, t1, t2, V3(0.0f, 0.0f, 1.0f), V3(0.0f, 0.0f, 3.0f)) == VORONOI_V0 && sp == 0.0f && w[0] == 1.0f);
        /* ending on vertex 1 from outside */
        TEST_TRUE(TriSegmentVoronoi(w, &sp, t0, t1, t2, V3(6.0f, -1.0f, 0.0f), V3(4.0f, 0.0f, 0.0f)) == VORONOI_V1 && sp == 1.0f && w[1] == 1.0f);
        /* beside edge 12, parallel to it in the plane: E12, distance sqrt 2 */
        TEST_TRUE(TriSegmentVoronoi(w, &sp, t0, t1, t2, V3(4.0f, 2.0f, 0.0f), V3(2.0f, 4.0f, 0.0f)) == VORONOI_E12);
        /* zero-length segment over the face: the point query */
        TEST_TRUE(TriSegmentVoronoi(w, &sp, t0, t1, t2, V3(1.0f, 1.0f, 2.0f), V3(1.0f, 1.0f, 2.0f)) == VORONOI_F012 && w[0] == 0.5f && w[1] == 0.25f);
        /* collinear triangle: the segment vs its edges */
        TEST_TRUE(TriSegmentVoronoi(w, &sp, t0, V3(2.0f, 0.0f, 0.0f), t1, V3(1.0f, -1.0f, 0.0f), V3(1.0f, 1.0f, 0.0f)) != VORONOI_INVALID);
    }

	return output;
}

static u32 VdExactCase(const v3 *t, const u32 count, const v3 p, const enum voronoi region, const f32 *w_expected)
{
    for (u32 origin = 0; origin < 2; ++origin)
    {
        v3 t_in[4];
        for (u32 i = 0; i < count; ++i)
        {
            t_in[i] = (origin) ? V3Sub(t[i], p) : t[i];
        }
        f32 w[4] = { -1.0f, -1.0f, -1.0f, -1.0f };
        const enum voronoi r = VdCall(w, t_in, count, origin, p);
        u32 ok = (r == region);
        for (u32 i = 0; i < count; ++i)
        {
            ok &= (w[i] == w_expected[i]);
        }
        if (!ok)
        {
            fprintf(stderr, "Voronoi exact (%u vertices, origin %u): p = (%g, %g, %g): region 0x%x (expected 0x%x), w = (%g, %g, %g, %g)\n",
                count, origin, (f64) p.x, (f64) p.y, (f64) p.z, (u32) r, (u32) region, (f64) w[0], (f64) w[1], (f64) w[2], (f64) w[3]);
            return 0;
        }
    }
    return 1;
}

/*
 * Integer simplices and queries: every numerator is exact, so regions are decided exactly, including the ties
 * on region boundaries (on a vertex, on an edge's line, on the face's plane).
 */
struct test_Output VoronoiExactTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };
    static const f32 h_set[] = { 0.0f, 1.0f, -1.0f, 3.0f, -3.0f };

    /* segment (0,0,0) (4,0,0); h moves perpendicular */
    {
        const v3 t[2] = { V3(0.0f, 0.0f, 0.0f), V3(4.0f, 0.0f, 0.0f) };
        for (u32 hi = 0; hi < 5; ++hi)
        {
            const f32 h = h_set[hi];
            TEST_TRUE(VdExactCase(t, 2, V3(2.0f, h, 0.0f), VORONOI_E01, (f32[]) { 0.5f, 0.5f }));
            TEST_TRUE(VdExactCase(t, 2, V3(1.0f, 0.0f, h), VORONOI_E01, (f32[]) { 0.75f, 0.25f }));
            TEST_TRUE(VdExactCase(t, 2, V3(0.0f, h, 0.0f), VORONOI_V0, (f32[]) { 1.0f, 0.0f }));
            TEST_TRUE(VdExactCase(t, 2, V3(-1.0f, h, 0.0f), VORONOI_V0, (f32[]) { 1.0f, 0.0f }));
            TEST_TRUE(VdExactCase(t, 2, V3(4.0f, 0.0f, h), VORONOI_V1, (f32[]) { 0.0f, 1.0f }));
            TEST_TRUE(VdExactCase(t, 2, V3(5.0f, h, h), VORONOI_V1, (f32[]) { 0.0f, 1.0f }));
        }
        /* degenerate segment: V1 (header) */
        const v3 s[2] = { V3(1.0f, 2.0f, 3.0f), V3(1.0f, 2.0f, 3.0f) };
        TEST_TRUE(VdExactCase(s, 2, V3(0.0f, 0.0f, 0.0f), VORONOI_V1, (f32[]) { 0.0f, 1.0f }));
        TEST_TRUE(VdExactCase(s, 2, V3(1.0f, 2.0f, 3.0f), VORONOI_V1, (f32[]) { 0.0f, 1.0f }));
    }

    /* triangle (0,0,0) (4,0,0) (0,4,0), ccw about +z; h moves along the normal */
    {
        const v3 t[3] = { V3(0.0f, 0.0f, 0.0f), V3(4.0f, 0.0f, 0.0f), V3(0.0f, 4.0f, 0.0f) };
        for (u32 hi = 0; hi < 5; ++hi)
        {
            const f32 h = h_set[hi];
            TEST_TRUE(VdExactCase(t, 3, V3(1.0f, 1.0f, h), VORONOI_F012, (f32[]) { 0.5f, 0.25f, 0.25f }));
            TEST_TRUE(VdExactCase(t, 3, V3(2.0f, 0.0f, h), VORONOI_E01, (f32[]) { 0.5f, 0.5f, 0.0f }));
            TEST_TRUE(VdExactCase(t, 3, V3(2.0f, -3.0f, h), VORONOI_E01, (f32[]) { 0.5f, 0.5f, 0.0f }));
            TEST_TRUE(VdExactCase(t, 3, V3(0.0f, 2.0f, h), VORONOI_E02, (f32[]) { 0.5f, 0.0f, 0.5f }));
            TEST_TRUE(VdExactCase(t, 3, V3(-1.0f, 1.0f, h), VORONOI_E02, (f32[]) { 0.75f, 0.0f, 0.25f }));
            TEST_TRUE(VdExactCase(t, 3, V3(2.0f, 2.0f, h), VORONOI_E12, (f32[]) { 0.0f, 0.5f, 0.5f }));
            TEST_TRUE(VdExactCase(t, 3, V3(3.0f, 3.0f, h), VORONOI_E12, (f32[]) { 0.0f, 0.5f, 0.5f }));
            /* on a vertex (all its numerators 0) and beyond it */
            TEST_TRUE(VdExactCase(t, 3, V3(0.0f, 0.0f, h), VORONOI_V0, (f32[]) { 1.0f, 0.0f, 0.0f }));
            TEST_TRUE(VdExactCase(t, 3, V3(-1.0f, -1.0f, h), VORONOI_V0, (f32[]) { 1.0f, 0.0f, 0.0f }));
            TEST_TRUE(VdExactCase(t, 3, V3(4.0f, 0.0f, h), VORONOI_V1, (f32[]) { 0.0f, 1.0f, 0.0f }));
            TEST_TRUE(VdExactCase(t, 3, V3(5.0f, -1.0f, h), VORONOI_V1, (f32[]) { 0.0f, 1.0f, 0.0f }));
            TEST_TRUE(VdExactCase(t, 3, V3(0.0f, 4.0f, h), VORONOI_V2, (f32[]) { 0.0f, 0.0f, 1.0f }));
            TEST_TRUE(VdExactCase(t, 3, V3(-1.0f, 5.0f, h), VORONOI_V2, (f32[]) { 0.0f, 0.0f, 1.0f }));
        }
        /* exactly degenerate: collinear (the middle vertex, an edge), coincident */
        const v3 line[3] = { V3(0.0f, 0.0f, 0.0f), V3(2.0f, 0.0f, 0.0f), V3(4.0f, 0.0f, 0.0f) };
        TEST_TRUE(VdExactCase(line, 3, V3(2.0f, 1.0f, 0.0f), VORONOI_V1, (f32[]) { 0.0f, 1.0f, 0.0f }));
        TEST_TRUE(VdExactCase(line, 3, V3(-1.0f, 1.0f, 0.0f), VORONOI_V0, (f32[]) { 1.0f, 0.0f, 0.0f }));
        TEST_TRUE(VdExactCase(line, 3, V3(5.0f, 0.0f, 1.0f), VORONOI_V2, (f32[]) { 0.0f, 0.0f, 1.0f }));
        const v3 point[3] = { V3(1.0f, 1.0f, 1.0f), V3(1.0f, 1.0f, 1.0f), V3(1.0f, 1.0f, 1.0f) };
        f32 w[3];
        TEST_TRUE(TriVoronoi(w, point[0], point[1], point[2], V3(3.0f, 0.0f, 0.0f)) != VORONOI_INVALID);
        TEST_TRUE(TriOriginVoronoi(w, point[0], point[1], point[2]) != VORONOI_INVALID);
    }

    /* tetrahedron (0,0,0) (4,0,0) (0,4,0) (0,0,4) */
    {
        const v3 t[4] = { V3(0.0f, 0.0f, 0.0f), V3(4.0f, 0.0f, 0.0f), V3(0.0f, 4.0f, 0.0f), V3(0.0f, 0.0f, 4.0f) };
        TEST_TRUE(VdExactCase(t, 4, V3(1.0f, 1.0f, 1.0f), VORONOI_T0123, (f32[]) { 0.25f, 0.25f, 0.25f, 0.25f }));
        TEST_TRUE(VdExactCase(t, 4, V3(1.0f, 1.0f, -2.0f), VORONOI_F012, (f32[]) { 0.5f, 0.25f, 0.25f, 0.0f }));
        TEST_TRUE(VdExactCase(t, 4, V3(1.0f, -2.0f, 1.0f), VORONOI_F013, (f32[]) { 0.5f, 0.25f, 0.0f, 0.25f }));
        TEST_TRUE(VdExactCase(t, 4, V3(2.0f, -1.0f, -1.0f), VORONOI_E01, (f32[]) { 0.5f, 0.5f, 0.0f, 0.0f }));
        TEST_TRUE(VdExactCase(t, 4, V3(-1.0f, -1.0f, -1.0f), VORONOI_V0, (f32[]) { 1.0f, 0.0f, 0.0f, 0.0f }));
        TEST_TRUE(VdExactCase(t, 4, V3(0.0f, 0.0f, 0.0f), VORONOI_V0, (f32[]) { 1.0f, 0.0f, 0.0f, 0.0f }));
        TEST_TRUE(VdExactCase(t, 4, V3(6.0f, -1.0f, -1.0f), VORONOI_V1, (f32[]) { 0.0f, 1.0f, 0.0f, 0.0f }));
        /* on the face x + y + z = 4 (inside counts as inside: the face's numerator is 0) */
        f32 w[4];
        TEST_TRUE(TetrahedronVoronoi(w, t[0], t[1], t[2], t[3], V3(2.0f, 1.0f, 1.0f)) != VORONOI_INVALID);
        /* coplanar (divisor 0): the closest face's feature */
        const v3 flat[4] = { V3(0.0f, 0.0f, 0.0f), V3(4.0f, 0.0f, 0.0f), V3(0.0f, 4.0f, 0.0f), V3(4.0f, 4.0f, 0.0f) };
        TEST_TRUE(TetrahedronVoronoi(w, flat[0], flat[1], flat[2], flat[3], V3(1.0f, 1.0f, 2.0f)) != VORONOI_INVALID);
        TEST_TRUE(TetrahedronOriginVoronoi(w, V3Sub(flat[0], V3(3.0f, 3.0f, 2.0f)), V3Sub(flat[1], V3(3.0f, 3.0f, 2.0f)), V3Sub(flat[2], V3(3.0f, 3.0f, 2.0f)), V3Sub(flat[3], V3(3.0f, 3.0f, 2.0f))) != VORONOI_INVALID);
    }

    /* regression: the origin over a vertex of a 1e-2 triangle (exactly the face; numerators of rounding size) */
    {
        const v3 t0 = V3(-0x1.2c8654p-27f, 0x1.197d4p-28f, -0x1.64507p-29f);
        const v3 t1 = V3(-0x1.c44e34p-10f, 0x1.36494cp-9f, 0x1.394e3p-7f);
        const v3 t2 = V3(-0x1.28e50ap-8f, -0x1.20b00ap-7f, 0x1.658126p-10f);
        f32 w[3];
        const enum voronoi r = TriOriginVoronoi(w, t0, t1, t2);
        TEST_TRUE(r != VORONOI_INVALID && (r & VORONOI_V0) && w[0] > 0.999999f);
    }

    /*
     * regression: a sliver, p over t1 on the E12 / V1 boundary: bc12[1] (exact +0.014) rounds to 0 and fails E12,
     * while bc012[0] / div012 = -0.33 is no rounding: the closest point is t1, not a clamped face point.
     */
    {
        const v3 t0 = V3(0x1.8656fap+8f, -0x1.7abe32p+10f, -0x1.ad17fep+9f);
        const v3 t1 = V3(0x1.412ee6p+8f, -0x1.a4128ep+6f, 0x1.628d54p+7f);
        const v3 t2 = V3(0x1.9c416p+8f, -0x1.e99deap+10f, -0x1.27fd0ep+10f);
        const v3 p = V3(0x1.00ab8ap+8f, 0x1.372f3p+3f, 0x1.0b9f3cp+4f);
        f32 w[3];
        const enum voronoi r = TriVoronoi(w, t0, t1, t2, p);
        TEST_TRUE(r != VORONOI_INVALID && (r & VORONOI_V1) && w[1] > 0.999999f);
    }

	return output;
}

/*
 * Planes: exact cases, including the degenerate ones of the segment functions (parallel, in the plane, touching),
 * and random planes and segments, where the clip parameter and the segment test must agree with the end points'
 * signs and clip points, projections and transformed points must lie on the plane (tolerance scaled by |n| and
 * the coordinates' size).
 */
struct test_Output PlaneExactTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    /* y = 1 with a unit normal and with |n| = 3 */
    const struct plane pl = Plane(V3(0.0f, 1.0f, 0.0f), V3(0.0f, 1.0f, 0.0f));
    const struct plane pl3 = Plane(V3(0.0f, 3.0f, 0.0f), V3(0.0f, 1.0f, 0.0f));
    const struct segment crossing = Segment(V3(0.0f, 0.0f, 0.0f), V3(0.0f, 2.0f, 0.0f));
    const struct segment one_side = Segment(V3(0.0f, 2.0f, 0.0f), V3(1.0f, 3.0f, 0.0f));
    const struct segment parallel = Segment(V3(0.0f, 2.0f, 0.0f), V3(1.0f, 2.0f, 0.0f));
    const struct segment in_plane = Segment(V3(0.0f, 1.0f, 0.0f), V3(1.0f, 1.0f, 0.0f));
    const struct segment touching = Segment(V3(0.0f, 1.0f, 0.0f), V3(0.0f, 3.0f, 0.0f));

    TEST_EQUAL(0.5f, PlaneSegmentClipParameter(&pl, &crossing));
    TEST_EQUAL(0.5f, PlaneSegmentClipParameter(&pl3, &crossing));
    TEST_EQUAL(-1.0f, PlaneSegmentClipParameter(&pl, &one_side));
    TEST_EQUAL(F32_INFINITY, PlaneSegmentClipParameter(&pl, &parallel));
    TEST_EQUAL(0.0f, PlaneSegmentClipParameter(&pl, &in_plane));
    TEST_EQUAL(0.0f, PlaneSegmentClipParameter(&pl, &touching));

    TEST_EQUAL(1, PlaneSegmentTest(&pl, &crossing));
    TEST_EQUAL(0, PlaneSegmentTest(&pl, &one_side));
    TEST_EQUAL(0, PlaneSegmentTest(&pl, &parallel));
    TEST_EQUAL(1, PlaneSegmentTest(&pl, &in_plane));
    TEST_EQUAL(1, PlaneSegmentTest(&pl, &touching));

    TEST_EQUAL(1, PlanePointInfrontCheck(&pl, V3(0.0f, 2.0f, 0.0f)));
    TEST_EQUAL(0, PlanePointInfrontCheck(&pl, V3(0.0f, 1.0f, 0.0f)));
    TEST_EQUAL(1, PlanePointBehindCheck(&pl, V3(0.0f, 0.0f, 0.0f)));
    TEST_EQUAL(0, PlanePointBehindCheck(&pl, V3(0.0f, 1.0f, 0.0f)));
    TEST_EQUAL(3.0f, PlanePointSignedDistance(&pl3, V3(7.0f, 2.0f, -5.0f)));
    TEST_EQUAL(3.0f, PlanePointDistance(&pl3, V3(7.0f, 0.0f, -5.0f)));

    struct plane normalized = pl3;
    PlaneNormalize(&normalized);
    TEST_EQUAL(1.0f, normalized.normal.y);
    TEST_EQUAL(1.0f, normalized.signed_distance);
    const struct plane constructed = PlaneNormalized(V3(0.0f, 3.0f, 0.0f), V3(2.0f, 1.0f, 4.0f));
    TEST_EQUAL(1.0f, constructed.normal.y);
    TEST_EQUAL(1.0f, constructed.signed_distance);

    const struct plane tri = PlaneCcwTriangle(V3(0.0f, 0.0f, 2.0f), V3(1.0f, 0.0f, 2.0f), V3(0.0f, 1.0f, 2.0f));
    TEST_EQUAL(1.0f, tri.normal_direction.z);
    TEST_EQUAL(2.0f, tri.signed_distance);

    v3 proj;
    TEST_EQUAL(1.0f, PlanePointProjection(&proj, &pl3, V3(5.0f, 4.0f, 0.0f)));
    TEST_TRUE(proj.x == 5.0f && proj.y == 1.0f && proj.z == 0.0f);

	return output;
}

struct test_Output PlaneRandomTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    const f32 scale = 10.0f;
    for (u32 k = 0; k < 20000; ++k)
    {
        const f32 n_len = RngF32Range(0.5f, 2.0f);
        const v3 n = V3Scale(V3UnitSphere(), n_len);
        const v3 q = V3(RngF32Range(-scale, scale), RngF32Range(-scale, scale), RngF32Range(-scale, scale));
        const struct plane pl = Plane(n, q);
        /* distances are rounded to ~eps |n| (|x| + |q|): with |x|, |q| <= ~2 sqrt(3) scale */
        const f32 tol = 64.0f * F32_EPSILON * n_len * scale;

        const v3 p0 = V3(RngF32Range(-scale, scale), RngF32Range(-scale, scale), RngF32Range(-scale, scale));
        const v3 p1 = V3(RngF32Range(-scale, scale), RngF32Range(-scale, scale), RngF32Range(-scale, scale));
        const struct segment s = Segment(p0, p1);
        const f32 d0 = PlanePointSignedDistance(&pl, p0);
        const f32 d1 = PlanePointSignedDistance(&pl, p1);
        const u32 one_side = (0.0f < d0 && 0.0f < d1) || (d0 < 0.0f && d1 < 0.0f);
        TEST_EQUAL(!one_side, PlaneSegmentTest(&pl, &s));
        if (!one_side && d0 != d1)
        {
            const f32 t = PlaneSegmentClipParameter(&pl, &s);
            TEST_TRUE(0.0f <= t && t <= 1.0f);
            TEST_TRUE(F32Abs(PlanePointSignedDistance(&pl, SegmentBc(&s, t))) <= tol);
        }

        v3 proj;
        PlanePointProjection(&proj, &pl, p0);
        TEST_TRUE(F32Abs(PlanePointSignedDistance(&pl, proj)) <= tol);

        struct plane unit = pl;
        PlaneNormalize(&unit);
        TEST_TRUE(F32Abs(V3Length(unit.normal) - 1.0f) <= 4.0f * F32_EPSILON);
        TEST_TRUE(F32Abs(PlanePointSignedDistance(&unit, q)) <= tol / n_len);
        const struct plane constructed = PlaneNormalized(n, q);
        TEST_TRUE(F32Abs(PlanePointSignedDistance(&constructed, q)) <= tol / n_len);

        const m3 rot = M3QUnit(QUnitAxisAngle(V3UnitSphere(), RngF32Range(-F32_PI, F32_PI)));
        const v3 pos = V3(RngF32Range(-scale, scale), RngF32Range(-scale, scale), RngF32Range(-scale, scale));
        const struct plane moved = PlaneTransform(&pl, rot, pos);
        TEST_TRUE(F32Abs(PlanePointSignedDistance(&moved, V3Add(M3V3Mul(rot, p0), pos)) - d0) <= 2.0f * tol);
    }

	return output;
}

/*
 * BboxRotate: random boxes (half widths 1e-3 to 1e3, some flat; centers up to 1e4 from the origin) and rotations.
 * Every corner rotated exactly (f64 sums of the exact products of the f32 entries) must lie in the result, and the
 * result may exceed the exact rotated box by at most 16 eps of its error scale (|R| (|c| + h))_k.
 */
struct test_Output BboxRotateTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    const f32 half_width[] = { 1e-3f, 1.0f, 1e3f };
    const f32 center[] = { 0.0f, 1.0f, 1e4f };
    f64 worst_excess = 0.0;
    for (u32 k = 0; k < 100000; ++k)
    {
        const f32 hs = half_width[k % 3];
        const f32 cs = center[(k / 3) % 3];
        const v3 c = V3(RngF32Range(-cs, cs), RngF32Range(-cs, cs), RngF32Range(-cs, cs));
        v3 h = V3(RngF32Range(0.0f, hs), RngF32Range(0.0f, hs), RngF32Range(0.0f, hs));
        if (k % 7 == 0) { h.buf[k % 3] = 0.0f; }
        const struct aabb box = { .min = V3Sub(c, h), .max = V3Add(c, h) };
        const m3 r = M3QUnit(QUnitAxisAngle(V3UnitSphere(), RngF32Range(-F32_PI, F32_PI)));
        const struct aabb rotated = BboxRotate(box, r);

        const f64 row[3][3] =
        {
            { r.a11, r.a12, r.a13 },
            { r.a21, r.a22, r.a23 },
            { r.a31, r.a32, r.a33 },
        };
        for (u32 axis = 0; axis < 3; ++axis)
        {
            f64 lo = (f64) F32_INFINITY;
            f64 hi = -(f64) F32_INFINITY;
            for (u32 i = 0; i < 8; ++i)
            {
                const f64 p[3] =
                {
                    (i & 1) ? box.max.x : box.min.x,
                    (i & 2) ? box.max.y : box.min.y,
                    (i & 4) ? box.max.z : box.min.z,
                };
                const f64 q = row[axis][0]*p[0] + row[axis][1]*p[1] + row[axis][2]*p[2];
                lo = (q < lo) ? q : lo;
                hi = (q > hi) ? q : hi;
            }
            TEST_TRUE((f64) rotated.min.buf[axis] <= lo && hi <= (f64) rotated.max.buf[axis]);

            f64 scale = 0.0;
            for (u32 j = 0; j < 3; ++j)
            {
                const f64 cj = 0.5*((f64) box.min.buf[j] + (f64) box.max.buf[j]);
                const f64 hj = 0.5*((f64) box.max.buf[j] - (f64) box.min.buf[j]);
                scale += ((row[axis][j] < 0.0) ? -row[axis][j] : row[axis][j]) * (((cj < 0.0) ? -cj : cj) + hj);
            }
            const f64 excess_lo = (lo - (f64) rotated.min.buf[axis]) / (F32_EPSILON * scale);
            const f64 excess_hi = ((f64) rotated.max.buf[axis] - hi) / (F32_EPSILON * scale);
            TEST_TRUE(excess_lo <= 16.0 && excess_hi <= 16.0);
            worst_excess = (excess_lo > worst_excess) ? excess_lo : worst_excess;
            worst_excess = (excess_hi > worst_excess) ? excess_hi : worst_excess;
        }
    }
    fprintf(stdout, "\tBboxRotateTest: 100000 boxes contain their exactly rotated corners; largest excess %.2f eps of (|R| (|c| + h))_k\n", worst_excess);

	return output;
}

/* f64 helpers for the segment references: squared distance of point p to segment ab */
static f64 SegTestPointSegmentSq(const f64 p[3], const f64 a[3], const f64 b[3])
{
    const f64 d[3] = { b[0] - a[0], b[1] - a[1], b[2] - a[2] };
    const f64 w[3] = { p[0] - a[0], p[1] - a[1], p[2] - a[2] };
    const f64 dd = d[0]*d[0] + d[1]*d[1] + d[2]*d[2];
    f64 t = (dd > 0.0) ? (w[0]*d[0] + w[1]*d[1] + w[2]*d[2]) / dd : 0.0;
    t = (t < 0.0) ? 0.0 : ((t > 1.0) ? 1.0 : t);
    const f64 e[3] = { a[0] + t*d[0] - p[0], a[1] + t*d[1] - p[1], a[2] + t*d[2] - p[2] };
    return e[0]*e[0] + e[1]*e[1] + e[2]*e[2];
}

/* exact squared distance of segments s1, s2 (f64, from their f32 end points): ternary search over s1 (convex) */
static f64 SegTestSegmentSq(const struct segment *s1, const struct segment *s2)
{
    const f64 a0[3] = { s1->p[0].x, s1->p[0].y, s1->p[0].z }, a1[3] = { s1->p[1].x, s1->p[1].y, s1->p[1].z };
    const f64 b0[3] = { s2->p[0].x, s2->p[0].y, s2->p[0].z }, b1[3] = { s2->p[1].x, s2->p[1].y, s2->p[1].z };
    f64 lo = 0.0, hi = 1.0;
    for (u32 i = 0; i < 200; ++i)
    {
        const f64 m1 = lo + (hi - lo) / 3.0, m2 = hi - (hi - lo) / 3.0;
        f64 p1[3], p2[3];
        for (u32 k = 0; k < 3; ++k) { p1[k] = a0[k] + m1*(a1[k] - a0[k]); p2[k] = a0[k] + m2*(a1[k] - a0[k]); }
        if (SegTestPointSegmentSq(p1, b0, b1) < SegTestPointSegmentSq(p2, b0, b1)) { hi = m2; } else { lo = m1; }
    }
    f64 p[3];
    for (u32 k = 0; k < 3; ++k) { p[k] = a0[k] + 0.5*(lo + hi)*(a1[k] - a0[k]); }
    return SegTestPointSegmentSq(p, b0, b1);
}

/* a unit vector orthogonal to the unit vector d */
static v3 SegTestOrthogonal(const v3 d)
{
    v3 u;
    do
    {
        u = V3UnitSphere();
        u = V3Sub(u, V3Scale(d, V3Dot(u, d)));
    } while (V3Dot(u, u) < 0.01f);
    return V3Normalize(u);
}

/*
 * SegmentDistanceSquared on nearly parallel unit segments (angles 1e-1 .. 1e-7, overlapping, 0 .. 1 apart) in a
 * local frame and up to 10 from the origin, against the exact distance: error <= 16 eps (1 + coordinate size).
 * The dot-product form of SegmentClosestParameter erred by up to angle * length (1e-4 at 1e-4 rad).
 */
struct test_Output SegmentNearParallelTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    f32 worst = 0.0f;
    for (u32 pos = 0; pos <= 10; pos += 10)
    {
        for (u32 k = 1; k <= 7; ++k)
        {
            f32 angle = 1.0f;
            for (u32 j = 0; j < k; ++j) { angle *= 0.1f; }
            for (u32 i = 0; i < 3000; ++i)
            {
                const v3 d1 = V3UnitSphere();
                const v3 u = SegTestOrthogonal(d1);
                const v3 w = SegTestOrthogonal(d1);
                const v3 d2 = V3Add(V3Scale(d1, F32Cos(angle)), V3Scale(w, F32Sin(angle)));
                const f32 p = (f32) pos;
                const v3 o = V3(RngF32Range(-p, p), RngF32Range(-p, p), RngF32Range(-p, p));
                const v3 q = V3Add(V3Add(o, V3Scale(u, RngF32Range(0.0f, 1.0f))), V3Scale(d1, RngF32Range(-0.9f, 0.9f)));
                const struct segment s1 = Segment(o, V3Add(o, d1));
                const struct segment s2 = Segment(q, V3Add(q, d2));

                v3 c1, c2;
                const f32 dist = F32Sqrt(SegmentDistanceSquared(&c1, &c2, &s1, &s2));
                const f32 ref = F32Sqrt((f32) SegTestSegmentSq(&s1, &s2));
                const f32 err = F32Abs(dist - ref);
                worst = F32Max(worst, err / (F32_EPSILON * (1.0f + p)));
                TEST_TRUE(err <= 16.0f * F32_EPSILON * (1.0f + 2.0f * p));
            }
        }
    }
    fprintf(stdout, "\tSegmentNearParallelTest: 42000 near-parallel pairs; largest error %.2f eps (1 + coordinate size)\n", worst);

	return output;
}

/*
 * SegmentLineClosestParameterCheck near its parallel limit (callers: >= 0.5 degrees): t1's error along a unit edge,
 * against an f64 evaluation of the same f32 input, <= eps / sin^2(angle). The dot-product numerators exceeded it at
 * every angle tested (2.7e-3 at 0.5 degrees).
 */
struct test_Output SegmentLineClosestTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    const f32 degrees[] = { 0.5f, 1.0f, 5.0f, 30.0f };
    for (u32 a = 0; a < 4; ++a)
    {
        const f32 angle = degrees[a] * F32_PI / 180.0f;
        const f32 sin_angle = F32Sin(angle);
        f64 worst = 0.0;
        for (u32 i = 0; i < 20000; ++i)
        {
            const v3 d1 = V3UnitSphere();
            const v3 u = SegTestOrthogonal(d1);
            const v3 v = V3Cross(d1, u);
            const v3 d2 = V3Add(V3Scale(d1, F32Cos(angle)), V3Scale(u, sin_angle));
            const v3 p1 = V3(RngF32Range(-1.0f, 1.0f), RngF32Range(-1.0f, 1.0f), RngF32Range(-1.0f, 1.0f));
            const v3 p2 = V3Add(V3Add(V3Add(p1, V3Scale(v, 0.1f)), V3Scale(d1, RngF32Range(-0.3f, 0.3f))), V3Scale(d2, -0.5f));
            const struct segment s1 = Segment(p1, V3Add(p1, d1));
            const struct segment s2 = Segment(p2, V3Add(p2, d2));

            f32 t1, t2;
            SegmentLineClosestParameterCheck(&t1, &t2, &s1, &s2);

            const f64 b1[3] = { s1.dir.x, s1.dir.y, s1.dir.z }, b2[3] = { s2.dir.x, s2.dir.y, s2.dir.z };
            const f64 r[3] = { (f64) s2.p[0].x - s1.p[0].x, (f64) s2.p[0].y - s1.p[0].y, (f64) s2.p[0].z - s1.p[0].z };
            const f64 n[3] = { b1[1]*b2[2] - b1[2]*b2[1], b1[2]*b2[0] - b1[0]*b2[2], b1[0]*b2[1] - b1[1]*b2[0] };
            const f64 rd2[3] = { r[1]*b2[2] - r[2]*b2[1], r[2]*b2[0] - r[0]*b2[2], r[0]*b2[1] - r[1]*b2[0] };
            const f64 t1_ref = (rd2[0]*n[0] + rd2[1]*n[1] + rd2[2]*n[2]) / (n[0]*n[0] + n[1]*n[1] + n[2]*n[2]);
            const f64 err = (t1 > t1_ref) ? t1 - t1_ref : t1_ref - t1;
            worst = (err > worst) ? err : worst;
        }
        fprintf(stdout, "\tSegmentLineClosestTest: %4.1f degrees, largest t1 error %.2e (bound eps / sin^2 = %.2e)\n", degrees[a], worst, F32_EPSILON / (sin_angle*sin_angle));
        TEST_TRUE(worst <= F32_EPSILON / (sin_angle*sin_angle));
    }

	return output;
}

/* point-segment functions: point segments, and random segments against f64 */
struct test_Output SegmentPointTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    const v3 a = V3(1.0f, 2.0f, 3.0f);
    const struct segment point = Segment(a, a);
    const v3 p = V3(4.0f, 6.0f, 3.0f);
    const f32 t = SegmentPointClosestBcParameter(&point, p);
    TEST_TRUE(0.0f <= t && t <= 1.0f);
    v3 c;
    TEST_EQUAL(25.0f, SegmentPointDistanceSquared(&c, &point, p));
    TEST_TRUE(c.x == a.x && c.y == a.y && c.z == a.z);

    for (u32 i = 0; i < 20000; ++i)
    {
        const v3 s0 = V3(RngF32Range(-10.0f, 10.0f), RngF32Range(-10.0f, 10.0f), RngF32Range(-10.0f, 10.0f));
        const v3 s1 = V3Add(s0, V3Scale(V3UnitSphere(), RngF32Range(0.01f, 5.0f)));
        const v3 q = V3(RngF32Range(-10.0f, 10.0f), RngF32Range(-10.0f, 10.0f), RngF32Range(-10.0f, 10.0f));
        const struct segment s = Segment(s0, s1);

        const f64 d[3] = { (f64) s1.x - s0.x, (f64) s1.y - s0.y, (f64) s1.z - s0.z };
        const f64 w[3] = { (f64) q.x - s0.x, (f64) q.y - s0.y, (f64) q.z - s0.z };
        const f64 t_line = (w[0]*d[0] + w[1]*d[1] + w[2]*d[2]) / (d[0]*d[0] + d[1]*d[1] + d[2]*d[2]);
        const f64 t_ref = (t_line < 0.0) ? 0.0 : ((t_line > 1.0) ? 1.0 : t_line);
        const f32 len = V3Length(s.dir);
        /* t is measured in units of |dir|: rounding of the coordinates (~20) gives ~eps 20 / |dir| */
        const f64 tol = 64.0 * F32_EPSILON * 20.0 / len;

        const f32 t_closest = SegmentPointClosestBcParameter(&s, q);
        const f32 t_projected = SegmentPointProjectedBcParameter(&s, q);
        TEST_TRUE(0.0f <= t_closest && t_closest <= 1.0f);
        TEST_TRUE((t_closest > t_ref ? t_closest - t_ref : t_ref - t_closest) <= tol);
        TEST_TRUE((t_projected > t_line ? t_projected - t_line : t_line - t_projected) <= tol * (1.0 + (t_line < 0.0 ? -t_line : t_line)));
        const f32 dist_sq = SegmentPointDistanceSquared(&c, &s, q);
        TEST_TRUE(F32Abs(V3DistanceSquared(c, q) - dist_sq) == 0.0f);
    }

	return output;
}

/*
 * TriCcwNormal(Direction)TryCompute: degenerate triangles fail with a zero vector; simple triangles are exact; needle
 * triangles (the tiny angle at p0, where crossing the edges at p0 loses ~eps / angle) are within 16 eps of the f64
 * normal of the same f32 input.
 */
struct test_Output TriNormalTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    v3 n;
    TEST_EQUAL(0, TriCcwNormalTryCompute(&n, V3(0.0f, 0.0f, 0.0f), V3(1.0f, 0.0f, 0.0f), V3(2.0f, 0.0f, 0.0f)));
    TEST_TRUE(n.x == 0.0f && n.y == 0.0f && n.z == 0.0f);
    TEST_EQUAL(0, TriCcwNormalDirectionTryCompute(&n, V3(1.0f, 2.0f, 3.0f), V3(1.0f, 2.0f, 3.0f), V3(1.0f, 2.0f, 3.0f)));
    TEST_TRUE(n.x == 0.0f && n.y == 0.0f && n.z == 0.0f);
    /* |dir|^2 underflows: degenerate at f32 scale */
    TEST_EQUAL(0, TriCcwNormalTryCompute(&n, V3(0.0f, 0.0f, 0.0f), V3(1e-25f, 0.0f, 0.0f), V3(0.0f, 1e-25f, 0.0f)));
    TEST_EQUAL(1, TriCcwNormalTryCompute(&n, V3(0.0f, 0.0f, 0.0f), V3(1.0f, 0.0f, 0.0f), V3(0.0f, 1.0f, 0.0f)));
    TEST_TRUE(n.x == 0.0f && n.y == 0.0f && n.z == 1.0f);
    TEST_EQUAL(1, TriCcwNormalDirectionTryCompute(&n, V3(0.0f, 0.0f, 0.0f), V3(2.0f, 0.0f, 0.0f), V3(0.0f, 3.0f, 0.0f)));
    TEST_TRUE(n.x == 0.0f && n.y == 0.0f && n.z == 6.0f);

    f64 worst = 0.0;
    for (u32 i = 0; i < 20000; ++i)
    {
        const v3 d = V3UnitSphere();
        const v3 u = SegTestOrthogonal(d);
        const v3 p0 = V3(RngF32Range(-1.0f, 1.0f), RngF32Range(-1.0f, 1.0f), RngF32Range(-1.0f, 1.0f));
        const f32 len = RngF32Range(1.0f, 10.0f);
        const f32 width = len * RngF32Range(1e-5f, 1e-2f);
        const v3 p1 = V3Add(p0, V3Scale(d, len));
        const v3 p2 = V3Add(p1, V3Scale(u, width));
        TEST_EQUAL(1, TriCcwNormalTryCompute(&n, p0, p1, p2));

        const f64 a[3] = { (f64) p1.x - p0.x, (f64) p1.y - p0.y, (f64) p1.z - p0.z };
        const f64 b[3] = { (f64) p2.x - p0.x, (f64) p2.y - p0.y, (f64) p2.z - p0.z };
        const f64 r[3] = { a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0] };
        const f64 r_len_sq = r[0]*r[0] + r[1]*r[1] + r[2]*r[2];
        /* |n x r| / |r| = sin of the angle between n and the reference */
        const f64 c[3] = { n.y*r[2] - n.z*r[1], n.z*r[0] - n.x*r[2], n.x*r[1] - n.y*r[0] };
        const f64 sin_sq = (c[0]*c[0] + c[1]*c[1] + c[2]*c[2]) / r_len_sq;
        const f64 dot = n.x*r[0] + n.y*r[1] + n.z*r[2];
        TEST_TRUE(dot > 0.0);
        worst = (sin_sq > worst) ? sin_sq : worst;
        TEST_TRUE(sin_sq <= (16.0*F32_EPSILON) * (16.0*F32_EPSILON));
    }
    fprintf(stdout, "\tTriNormalTest: 20000 needle triangles (angle 1e-5 .. 1e-2 at p0); largest normal angle error %.2e rad\n", F32Sqrt((f32) worst));

	return output;
}

static struct test_Output (*geometry_tests[])(struct test_Environment *) =
{
	TriNormalTest,
	SegmentNearParallelTest,
	SegmentLineClosestTest,
	SegmentPointTest,
	BboxRotateTest,
	PlaneExactTest,
	PlaneRandomTest,
	SegmentShortTest,
	DcelFaceSegmentClipTest,
	DcelBoxTest,
	DcelPlanesTest,
};

struct suite_Correctness m_geometry_suite =
{
	.id = "geometry",
	.unit_test = geometry_tests,
	.unit_test_count = sizeof(geometry_tests) / sizeof(geometry_tests[0]),
};

struct suite_Correctness *geometry_correctness_suite = &m_geometry_suite;

static struct test_Output (*voronoi_tests[])(struct test_Environment *) =
{
	VoronoiExactTest,
	VoronoiWellShapedTest,
	VoronoiSliverTest,
	VoronoiDegenerateTest,
	VoronoiNearFlatTest,
	VoronoiTriSegmentTest,
};

struct suite_Correctness m_voronoi_suite =
{
	.id = "voronoi",
	.unit_test = voronoi_tests,
	.unit_test_count = sizeof(voronoi_tests) / sizeof(voronoi_tests[0]),
};

struct suite_Correctness *voronoi_correctness_suite = &m_voronoi_suite;
