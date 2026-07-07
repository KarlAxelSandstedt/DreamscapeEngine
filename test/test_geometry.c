#include "ds_test.h"
#include "geometry.h"

static void Vec3UnitSphere(vec3 v)
{
    const f32 z = RngF32Range(-1.0f, 1.0f);
    const f32 r = f32_sqrt(1 - z*z);
    const f32 theta = 2.0f*F32_PI*RngF32Normalized();
    Vec3Set(v, r*f32_cos(theta), r*f32_sin(theta), z);
}

static f32 TriCcwSegmentDistanceSquaredSlow(enum TriVoronoiRegion *reg, const struct segment *s, const struct TriVoronoi *tv)
{
    f32 dist_sq, dist_sq_min = F32_INFINITY;
    vec3 c1, c2;
    enum TriVoronoiRegion region;

    if (TriCcwSegmentClip(c1, s, tv))
    {
        dist_sq_min = 0.0f;
        *reg = TRI_VORONOI_FACE;
    }
    else 
    {
        dist_sq = TriCcwPointDistanceSquared(c1, &region, s->p[0], tv);
        if (dist_sq < dist_sq_min)
        {
            dist_sq_min = dist_sq;
            *reg = region;
        }

        dist_sq = TriCcwPointDistanceSquared(c1, &region, s->p[1], tv);
        if (dist_sq < dist_sq_min)
        {
            dist_sq_min = dist_sq;
            *reg = region;
        }

        for (u32 i = 0; i < 3; ++i)
        {
            f32 t1, t2;
            SegmentClosestParameter(&t1, &t2, s, tv->s + i);
	        SegmentBc(c1, s, t1);
	        SegmentBc(c2, tv->s + i, t2);
	        dist_sq = Vec3DistanceSquared(c1, c2);
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
        vec3 tri[3];
        Vec3UnitSphere(tri[0]);
        Vec3UnitSphere(tri[1]);
        Vec3UnitSphere(tri[2]);

        enum TriVoronoiRegion region;
        struct TriVoronoi tv;
        if (!TriVoronoiInitCcw(&tv, tri))
            continue;

        u32 g = 0;
        for (u32 j = 0; j < n; ++j)
        {
            vec3 p[2];
            Vec3UnitSphere(p[0]);
            Vec3UnitSphere(p[1]);
            struct segment segment = SegmentConstruct(p[0], p[1]);
            //if (Vec3Dot(segment.dir, segment.dir) < 0.0001)
            //    continue;

            enum TriVoronoiRegion region_slow;
            vec3 c_t, c_s;
            const f32 dist = f32_sqrt(TriCcwSegmentDistanceSquared(c_t, c_s, &region, &segment, &tv));
            const f32 dist_slow = f32_sqrt(TriCcwSegmentDistanceSquaredSlow(&region_slow, &segment, &tv));
            const f32 abs_diff = f32_abs(dist - dist_slow);
            if (abs_diff > 1e-6)
                ++g;
            if (max_diff < abs_diff)
            {
                max_diff = abs_diff;
                fprintf(stderr, "New maximum difference: %f (fast=%f, slow=%f)\n", f32_abs(dist-dist_slow), dist, dist_slow);
                fprintf(stderr, "Triangle Config:\n");
                Vec3Print("\t", tv.t[0]);
                Vec3Print("\t", tv.t[1]);
                Vec3Print("\t", tv.t[2]);
                fprintf(stderr, "Segment Config:\n");
                Vec3Print("\t", p[0]);
                Vec3Print("\t", p[1]);
                fprintf(stderr, "fast region: %s\n", g_table_tri_voronoi_region_string[region]);
                fprintf(stderr, "slow region: %s\n", g_table_tri_voronoi_region_string[region_slow]);
                if (max_diff > 0.003f)
                {
                    Breakpoint(max_diff > 0.003f);
                    TriCcwSegmentDistanceSquared(c_t, c_s, &region, &segment, &tv);
                    TriCcwSegmentDistanceSquaredSlow(&region_slow, &segment, &tv);
                }
            }
        }
        if (g)
            fprintf(stderr, "Triangle %u: percentage of tri-seg pair yielding a maximum difference >1e-6: %.2f%%\n", i, 100.0f * (f32) g / n);
    }

	return output;
}

static struct test_Output (*geometry_tests[])(struct test_Environment *) =
{
	TriCcwPointDistanceSquaredTest,
};

struct suite_Correctness m_geometry_suite =
{
	.id = "geometry",
	.unit_test = geometry_tests,
	.unit_test_count = sizeof(geometry_tests) / sizeof(geometry_tests[0]),
};

struct suite_Correctness *geometry_correctness_suite = &m_geometry_suite;
