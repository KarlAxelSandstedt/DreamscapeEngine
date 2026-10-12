#include "ds_test.h"
#include "geometry.h"

static v3 V3UnitSphere(void)
{
    const f32 z = RngF32Range(-1.0f, 1.0f);
    const f32 r = F32Sqrt(1 - z*z);
    const f32 theta = 2.0f*F32_PI*RngF32Normalized();
    return V3(r*F32Cos(theta), r*F32Sin(theta), z);
}

static f32 TriCcwSegmentDistanceSquaredSlow(enum TriVoronoiRegion *reg, const struct segment *s, const struct TriVoronoiTMP *tv)
{
    f32 dist_sq, dist_sq_min = F32_INFINITY;
    v3 c1;
    enum TriVoronoiRegion region;

    if (TriCcwSegmentClip(&c1, s, tv))
    {
        dist_sq_min = 0.0f;
        *reg = TRI_VORONOI_FACE;
    }
    else 
    {
        dist_sq = TriCcwPointDistanceSquared(&c1, &region, s->p[0], tv);
        if (dist_sq < dist_sq_min)
        {
            dist_sq_min = dist_sq;
            *reg = region;
        }

        dist_sq = TriCcwPointDistanceSquared(&c1, &region, s->p[1], tv);
        if (dist_sq < dist_sq_min)
        {
            dist_sq_min = dist_sq;
            *reg = region;
        }

        for (u32 i = 0; i < 3; ++i)
        {
            f32 t1, t2;
            SegmentClosestParameter(&t1, &t2, s, tv->s + i);
	        dist_sq = V3DistanceSquared(SegmentBc(s, t1), SegmentBc(tv->s + i, t2));
            if (dist_sq < dist_sq_min)
            {
                dist_sq_min = dist_sq;
                if (t2 == 0.0f)
                {
                    *reg = TRI_VORONOI_VERTEX0 + i;
                }
                else if (t2 == 1.0f)
                {
                    *reg = TRI_VORONOI_VERTEX0 + ((i+1) % 3);
                }
                else
                {
                    *reg = TRI_VORONOI_EDGE01 + i;
                }
            }
        }
    }

    return dist_sq_min;
}

struct test_Output TriCcwPointDistanceSquaredTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    f32 max_diff = 0.0f;
    const u32 n = 10000;
    for (u32 i = 0; i < n; ++i)
    {
        v3 tri[3];
        tri[0] = V3UnitSphere();
        tri[1] = V3UnitSphere();
        tri[2] = V3UnitSphere();

        enum TriVoronoiRegion region;
        struct TriVoronoiTMP tv;
        if (!TriVoronoiInitCcw(&tv, tri))
            continue;

        u32 g = 0;
        for (u32 j = 0; j < n; ++j)
        {
            v3 p[2];
            p[0] = V3UnitSphere();
            p[1] = V3UnitSphere();
            struct segment segment = SegmentConstruct(p[0], p[1]);
            //if (V3Dot(segment.dir, segment.dir) < 0.0001)
            //    continue;

            enum TriVoronoiRegion region_slow = TRI_VORONOI_FACE;
            v3 c_t, c_s;
            const f32 dist = F32Sqrt(TriCcwSegmentDistanceSquared(&c_t, &c_s, &region, &segment, &tv));
            const f32 dist_slow = F32Sqrt(TriCcwSegmentDistanceSquaredSlow(&region_slow, &segment, &tv));
            const f32 abs_diff = F32Abs(dist - dist_slow);
            if (abs_diff > 1e-6)
                ++g;
            if (max_diff < abs_diff)
            {
                max_diff = abs_diff;
                fprintf(stderr, "New maximum difference: %f (fast=%f, slow=%f)\n", F32Abs(dist-dist_slow), dist, dist_slow);
                fprintf(stderr, "Triangle Config:\n");
                V3Print("\t", tv.t[0]);
                V3Print("\t", tv.t[1]);
                V3Print("\t", tv.t[2]);
                fprintf(stderr, "Segment Config:\n");
                V3Print("\t", p[0]);
                V3Print("\t", p[1]);
                fprintf(stderr, "fast region: %s\n", g_table_tri_voronoi_region_string[region]);
                fprintf(stderr, "slow region: %s\n", g_table_tri_voronoi_region_string[region_slow]);
                if (max_diff > 0.003f)
                {
                    TEST_FAILURE;
                }
            }
        }
        if (g)
            fprintf(stderr, "Triangle %u: percentage of tri-seg pair yielding a maximum difference >1e-6: %.2f%%\n", i, 100.0f * (f32) g / n);
    }

	return output;
}

/*
 * SegmentDistanceSquared with short and degenerate segments (length 1 .. subnormal .. 0): the parameters stay in
 * [0, 1], the points on their segments, the distance finite and within the short segment's length of the
 * distance from its first end (SegmentPointDistanceSquared).
 */
static u32 SegmentShortCase(const v3 a0, const v3 a_dir, const v3 b0, const v3 b1)
{
    const struct segment a = SegmentConstruct(a0, V3Add(a0, a_dir));
    const struct segment b = SegmentConstruct(b0, b1);
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
                const struct segment a = SegmentConstruct(b0, b1);
                const struct segment s = SegmentConstruct(a0, V3Add(a0, dir));
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

/*
 * clip s to face fi of the moved box; 1 if non-empty and the result is (e0, e1); with_local (identity transform):
 * DcelFaceSegmentClip too.
 */
static u32 ClipCase(const struct dcel *box, const m3 rot, const v3 pos, const u32 local, const u32 fi, const v3 p0, const v3 p1, const v3 e0, const v3 e1)
{
    const struct segment s = SegmentConstruct(p0, p1);
    struct segment clip, clip_local;
    if (!DcelFaceSegmentClipWorld(&clip, box, rot, pos, fi, &s) || !ClipNear(clip.p[0], e0) || !ClipNear(clip.p[1], e1))
    {
        return 0;
    }
    return !local || (DcelFaceSegmentClip(&clip_local, box, fi, &s) && ClipNear(clip_local.p[0], e0) && ClipNear(clip_local.p[1], e1));
}

/* clip s to face fi of the moved box; 1 if empty (with_local: DcelFaceSegmentClip too) */
static u32 ClipEmptyCase(const struct dcel *box, const m3 rot, const v3 pos, const u32 local, const u32 fi, const v3 p0, const v3 p1)
{
    const struct segment s = SegmentConstruct(p0, p1);
    struct segment clip;
    return !DcelFaceSegmentClipWorld(&clip, box, rot, pos, fi, &s) && (!local || !DcelFaceSegmentClip(&clip, box, fi, &s));
}

/*
 * DcelFaceSegmentClip/World on the unit box stub ([-0.5, 0.5]^3, face 0 = +y): the face's sides are
 * |x|, |z| <= 0.5. Segments at y = 0.7 (above the face; the clip does not project onto it).
 */
struct test_Output DcelFaceSegmentClipTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    const struct dcel box = DcelBoxStub();
    const m3 I = M3Identity();
    const v3 o = V3(0.0f, 0.0f, 0.0f);
    const u32 with_local = 1;

    /* inside: unchanged */
    TEST_TRUE(ClipCase(&box, I, o, with_local, 0, V3(-0.2f, 0.7f, 0.0f), V3(0.3f, 0.7f, 0.1f), V3(-0.2f, 0.7f, 0.0f), V3(0.3f, 0.7f, 0.1f)));
    /* crosses one side: p1 moved, p0 kept */
    TEST_TRUE(ClipCase(&box, I, o, with_local, 0, V3(0.0f, 0.7f, 0.0f), V3(1.0f, 0.7f, 0.0f), V3(0.0f, 0.7f, 0.0f), V3(0.5f, 0.7f, 0.0f)));
    /* crosses one side: p0 moved, p1 kept */
    TEST_TRUE(ClipCase(&box, I, o, with_local, 0, V3(1.0f, 0.7f, 0.0f), V3(0.0f, 0.7f, 0.0f), V3(0.5f, 0.7f, 0.0f), V3(0.0f, 0.7f, 0.0f)));
    /* crosses two opposite sides; direction kept */
    TEST_TRUE(ClipCase(&box, I, o, with_local, 0, V3(-1.0f, 0.7f, 0.2f), V3(1.0f, 0.7f, 0.2f), V3(-0.5f, 0.7f, 0.2f), V3(0.5f, 0.7f, 0.2f)));
    TEST_TRUE(ClipCase(&box, I, o, with_local, 0, V3(1.0f, 0.7f, 0.2f), V3(-1.0f, 0.7f, 0.2f), V3(0.5f, 0.7f, 0.2f), V3(-0.5f, 0.7f, 0.2f)));
    /* diagonal through two adjacent sides */
    TEST_TRUE(ClipCase(&box, I, o, with_local, 0, V3(-1.0f, 0.7f, -1.0f), V3(1.0f, 0.7f, 1.0f), V3(-0.5f, 0.7f, -0.5f), V3(0.5f, 0.7f, 0.5f)));
    /* along the face normal: no projection, unchanged */
    TEST_TRUE(ClipCase(&box, I, o, with_local, 0, V3(0.1f, -2.0f, 0.1f), V3(0.1f, 2.0f, 0.1f), V3(0.1f, -2.0f, 0.1f), V3(0.1f, 2.0f, 0.1f)));

    /* wholly outside one side (spans the z range): empty */
    TEST_TRUE(ClipEmptyCase(&box, I, o, with_local, 0, V3(0.6f, 0.7f, -1.0f), V3(0.8f, 0.7f, 1.0f)));
    /* misses the corner (x + z = 1.2): every side alone keeps a part, together nothing */
    TEST_TRUE(ClipEmptyCase(&box, I, o, with_local, 0, V3(1.2f, 0.7f, 0.0f), V3(0.0f, 0.7f, 1.2f)));
    /* along the face normal, outside: empty */
    TEST_TRUE(ClipEmptyCase(&box, I, o, with_local, 0, V3(0.6f, -2.0f, 0.1f), V3(0.6f, 2.0f, 0.1f)));

    /* in a side plane (x = 0.5): kept, clipped by the z sides, finite */
    TEST_TRUE(ClipCase(&box, I, o, with_local, 0, V3(0.5f, 0.7f, -1.0f), V3(0.5f, 0.7f, 1.0f), V3(0.5f, 0.7f, -0.5f), V3(0.5f, 0.7f, 0.5f)));
    /* touches a side at an end: a single point */
    TEST_TRUE(ClipCase(&box, I, o, with_local, 0, V3(0.5f, 0.7f, 0.0f), V3(1.0f, 0.7f, 0.0f), V3(0.5f, 0.7f, 0.0f), V3(0.5f, 0.7f, 0.0f)));

    /* degenerate (p0 == p1): itself inside, empty outside */
    TEST_TRUE(ClipCase(&box, I, o, with_local, 0, V3(0.1f, 0.7f, 0.1f), V3(0.1f, 0.7f, 0.1f), V3(0.1f, 0.7f, 0.1f), V3(0.1f, 0.7f, 0.1f)));
    TEST_TRUE(ClipEmptyCase(&box, I, o, with_local, 0, V3(0.6f, 0.7f, 0.1f), V3(0.6f, 0.7f, 0.1f)));

    /* other faces: face 1 = +x (sides |y|, |z| <= 0.5), face 5 = -y */
    TEST_TRUE(ClipCase(&box, I, o, with_local, 1, V3(0.7f, -1.0f, 0.0f), V3(0.7f, 1.0f, 0.0f), V3(0.7f, -0.5f, 0.0f), V3(0.7f, 0.5f, 0.0f)));
    TEST_TRUE(ClipCase(&box, I, o, with_local, 5, V3(0.0f, -0.7f, 0.0f), V3(1.0f, -0.7f, 0.0f), V3(0.0f, -0.7f, 0.0f), V3(0.5f, -0.7f, 0.0f)));

    /* moved box: rotated 90 deg about y (x -> -z), at (10, 0, 0); face 0 stays +y, sides |x - 10|, |z| <= 0.5 */
    {
        const m3 rot = M3QUnit(QUnitAxisAngle(V3(0.0f, 1.0f, 0.0f), F32_PI/2.0f));
        const v3 pos = V3(10.0f, 0.0f, 0.0f);
        TEST_TRUE(ClipCase(&box, rot, pos, 0, 0, V3(10.0f, 0.7f, 0.0f), V3(10.0f, 0.7f, 2.0f), V3(10.0f, 0.7f, 0.0f), V3(10.0f, 0.7f, 0.5f)));
        TEST_TRUE(ClipEmptyCase(&box, rot, pos, 0, 0, V3(0.0f, 0.7f, 0.0f), V3(1.0f, 0.7f, 0.0f)));
    }

	return output;
}

static struct test_Output (*geometry_tests[])(struct test_Environment *) =
{
	TriCcwPointDistanceSquaredTest,
	SegmentShortTest,
	DcelFaceSegmentClipTest,
};

struct suite_Correctness m_geometry_suite =
{
	.id = "geometry",
	.unit_test = geometry_tests,
	.unit_test_count = sizeof(geometry_tests) / sizeof(geometry_tests[0]),
};

struct suite_Correctness *geometry_correctness_suite = &m_geometry_suite;
