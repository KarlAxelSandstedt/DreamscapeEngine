
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

#include "ds_dynamics.h"
#include "collision.h"
#include "ds_voronoi.h"

/*
GJK Implementation: 

Read: TODO 
     (O) Erin-Catto gjk 2010
     (O) Ericcson 3.4
     (O) Ericcson 5.1.5 (Solve3)
     (O) Ericcson 5.1.6 (Solve4)
     () Theory and termination: Gino, collision detection in interactive 3d env. 4.3.1-4.3.8
     (O) Erin-Catto triangles (FP-precision)
     (O) Erin Catto continous collision 2013 (where the cache leads)

Notes:
*/

/*
Erin-Catto GJK 2010
===================

/*
TODO: GJK: do you cache the final simplex in previous calls to use as arbitrary simplex at
    the start of next GJK call?
*/

/*
GJK distance
============
Box3D's variant (agent_notes/gjk_design.txt): runs in shape A's frame on the Minkowski difference
B - A, solves the simplex with the Voronoi API (ds_voronoi.h), takes the search direction from the
simplex geometry, stops on a repeated support pair, on no progress (restoring the previous simplex)
or on overlap, and warm-starts from a cache of support indices.
*/

/* A shape as a point cloud in its local frame: hull vertices, a sphere's center, a capsule's segment. */
struct GJKProxy
{
    const v3 *  v;
    u32         v_count;
    v3          buf[2];     /* storage for sphere/capsule points; v points here for those shapes */
};

/* Simplex vertex: both support points in frame A and their difference. */
struct GJKSimplexVertex
{
    v3  a;                  /* support point of A */
    v3  b;                  /* support point of B, moved into frame A */
    v3  w;                  /* b - a: point of the Minkowski difference B - A */
    u32 index_a;            /* proxy vertex indices of a and b */
    u32 index_b;
};

struct GJKSimplex
{
    struct GJKSimplexVertex v[4];
    f32                     weight[4];  /* barycentric weights of the closest point, from the Voronoi API */
    u32                     count;      /* 1..4 */
};

/* Set up the proxy of shape (hull, sphere or capsule) in its local frame. */
static void GJKProxyInit(struct GJKProxy *proxy, const struct c_Shape *shape)
{
    //TODO hull: v = hull.v; sphere: buf[0] = origin; capsule: buf[0..1] = (0, +-half_height, 0)
}

/*
 * Return the distance between shapes s1 (A) and s2 (B), and set c1, c2 to the closest points on A and B
 * (world space) and n to the unit normal from A to B. Radii are not included: a sphere is its center,
 * a capsule its segment. cache_in (NULL: cold start) warm-starts the query from the previous frame;
 * cache_out receives this frame's cache (the caller allocates it, e.g. on the frame arena). cache_in is
 * read before cache_out is written.
 *
 * Overlap returns 0 with c1 == c2 and n = 0. If the shapes are proven farther apart than max_distance,
 * returns F32_INFINITY early; c1, c2 and n are then garbage (pass F32_INFINITY to always converge).
 */
static f32 GJK(v3 *c1, v3 *c2, v3 *n, struct GJKCache *cache_out, const struct GJKCache *cache_in, const struct c_Shape *s1, const ds_Transform *t1, const struct c_Shape *s2, const ds_Transform *t2, const f32 max_distance)
{
    //TODO 1. proxies; relative transform of B in A's frame (rotation for B's support direction)
    //TODO 2. simplex from cache_in (flush on metric change), or a single vertex (index 0, 0)
    struct GJKSimplex simplex = { 0 };
    struct GJKSimplex backup = { 0 };
    f32 distance_sq = F32_INFINITY;

    for (u32 i = 0; i < g_numerics_config->gjk_max_iterations; ++i)
    {
        /* closest feature of the simplex to the origin */
        f32 w[4];
        enum voronoi region = VORONOI_INVALID;
        switch (simplex.count)
        {
            case 1:
            {
                region = VORONOI_V0;
                w[0] = 1.0f;
            } break;

            case 2:
            {
                region = SegmentOriginVoronoi(w, simplex.v[0].w, simplex.v[1].w);
            } break;

            case 3:
            {
                region = TriOriginVoronoi(w, simplex.v[0].w, simplex.v[1].w, simplex.v[2].w);
            } break;

            case 4:
            {
                region = TetrahedronOriginVoronoi(w, simplex.v[0].w, simplex.v[1].w, simplex.v[2].w, simplex.v[3].w);
            } break;
        }

        /* numerically degenerate simplex: the previous one is (nearly) the closest */
        if (region == VORONOI_INVALID)
        {
            simplex = backup;
            break;
        }

        //TODO keep the vertices in region (bit i -> v[i]) with their weights, compacted

        if (region == VORONOI_T0123)
        {
            //TODO overlap: witness points, c1 == c2, n = 0, write cache_out, return 0
        }

        //TODO closest point = sum weight[i] * v[i].w; dist_sq = |closest|^2
        //TODO no strict progress (!(dist_sq < distance_sq)): simplex = backup; break
        //TODO distance_sq = dist_sq

        //TODO search direction from the geometry (vertex: -a, edge: (ab x -a) x ab, triangle: +-(ab x ac));
        //     |d|^2 < 1000 * F32_MIN_POSITIVE_NORMAL -> overlap

        //TODO support points of A in -d and B in d (B's direction rotated into B's frame); w = b - a
        //TODO max_distance early exit: dot(d, w) proves the distance exceeds max_distance -> return F32_INFINITY

        //TODO repeated support pair (index_a, index_b) already in the simplex: break

        backup = simplex;
        //TODO add the vertex: simplex.v[simplex.count++] = ...
    }

    //TODO 4. witness points from the weights, distance, normal; back to world space
    //TODO 5. write cache_out
    return F32_INFINITY;
}
