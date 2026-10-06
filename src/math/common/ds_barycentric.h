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

/*
ds_barycentric.h
================
Barycentric coordinates in unnormalized form: the signed sub-measures (numerators) and the full
measure (divisor). Used by GJK (src/math/collision/ds_distance.c).

There are deliberately no normalized versions. Barycentric coordinates of an unclamped projection are
ill-conditioned for nearly degenerate primitives (a sliver's plane or a short segment's direction is
barely defined), so callers must:
    1. decide the Voronoi region from the signs of the numerators (no division),
    2. normalize only the chosen feature, with that feature's own numerators and divisor,
    3. handle divisor <= 0 there (exactly degenerate, or rounding) as a failure with a workable
       result, e.g. GJK restores its previous simplex.
In a face or edge region all chosen numerators are > 0, so the weights lie in (0, 1).
*/

#ifndef __DS_BARYCENTRIC_H__
#define __DS_BARYCENTRIC_H__

//TODO remove, should only include necessary headers for calculations
#include "geometry.h"

/*
 * Unnormalized barycentric coordinates of point projected onto the line through (s0, s1). bc[i] is the
 * weight of s_i, and bc[0] + bc[1] = divisor = |s1 - s0|^2; bc[i] / divisor are the barycentric
 * coordinates. bc[0] < 0: the projection lies beyond s1; bc[1] < 0: beyond s0.
 *
 * WARNING: divisor == 0 for a degenerate segment (s0 == s1); the caller must handle it before dividing.
 */
static inline void SegmentBCUnnormalized(f32 bc[2], f32 *divisor, const v3 s0, const v3 s1, const v3 point);
/* SegmentBCUnnormalized with origin as the query point. */
static inline void SegmentOriginBCUnnormalized(f32 bc[2], f32 *divisor, const v3 s0, const v3 s1);

/*
 * Unnormalized barycentric coordinates of p projected onto the plane of triangle (t0, t1, t2).
 * bc[i] is the weight of t_i, and bc[0] + bc[1] + bc[2] = divisor = |n|^2; bc[i] / divisor are the 
 * barycentric coordinates.
 *
 * WARNING: divisor == 0 for a degenerate (collinear) triangle; the caller must handle it before dividing.
 */
static inline void TriBCUnnormalized(f32 bc[3], f32 *divisor, const v3 t0, const v3 t1, const v3 t2, const v3 p);
/* TriBCUnnormalized with origin as the query point. */
static inline void TriOriginBCUnnormalized(f32 bc[3], f32 *divisor, const v3 t0, const v3 t1, const v3 t2);

/*
 * Feature of a simplex (vertex, edge, face, tetrahedron) as the bitmask of its vertices: bit i set
 * means vertex i belongs to the feature. Edge and face names list their vertices in ascending order.
 */
enum voronoi
{
    VORONOI_INVALID = 0x0,
    VORONOI_V0      = 0x1,
    VORONOI_V1      = 0x2,
    VORONOI_E01     = 0x3,
    VORONOI_V2      = 0x4,
    VORONOI_E02     = 0x5,
    VORONOI_E12     = 0x6,
    VORONOI_F012    = 0x7,
    VORONOI_V3      = 0x8,
    VORONOI_E03     = 0x9,
    VORONOI_E13     = 0xa,
    VORONOI_F013    = 0xb,
    VORONOI_E23     = 0xc,
    VORONOI_F023    = 0xd,
    VORONOI_F123    = 0xe,
    VORONOI_T0123   = 0xf,
    VORONOI_COUNT
};

/*
 * Return the feature of segment (s0, s1) closest to p, and its barycentric weights: w[i] is the
 * weight of s_i, 0 for vertices outside the feature; all w[i] in [0, 1], summing to 1.
 *
 * A degenerate segment (s0 == s1) gives VORONOI_V1. Returns VORONOI_INVALID only if the geometry is
 * numerically degenerate (rounding); w is then garbage.
 */
static inline enum voronoi SegmentVoronoi(f32 w[2], const v3 s0, const v3 s1, const v3 p);
/* SegmentVoronoi with origin as the query point. */
static inline enum voronoi SegmentOriginVoronoi(f32 w[2], const v3 s0, const v3 s1);

/*
 * Return the feature of triangle (t0, t1, t2) closest to p, and its barycentric weights: w[i] is the
 * weight of t_i, 0 for vertices outside the feature; all w[i] in [0, 1], summing to 1.
 *
 * A collinear triangle or coincident vertices give an edge or a vertex. Returns VORONOI_INVALID only if the
 * geometry is numerically degenerate (rounding); w is then garbage.
 */
static inline enum voronoi TriVoronoi(f32 w[3], const v3 t0, const v3 t1, const v3 t2, const v3 p);
/* TriVoronoi with origin as the query point. */
static inline enum voronoi TriOriginVoronoi(f32 w[3], const v3 t0, const v3 t1, const v3 t2);

/*
 * Return the feature of tetrahedron (t0, t1, t2, t3) closest to p, and its barycentric weights: w[i] is
 * the weight of t_i, 0 for vertices outside the feature; all w[i] in [0, 1], summing to 1. p inside
 * gives VORONOI_T0123. 
 * (Not implemented yet.)
 *
 * Returns VORONOI_INVALID if the geometry is degenerate; w is then garbage.
 */
static inline enum voronoi TetrahedronVoronoi(f32 w[4], const v3 t0, const v3 t1, const v3 t2, const v3 t3, const v3 p);
/* TetrahedronVoronoi with origin as the query point. */
static inline enum voronoi TetrahedronOriginVoronoi(f32 w[4], const v3 t0, const v3 t1, const v3 t2, const v3 t3);


/* ================================================================================================== */


static inline void SegmentBCUnnormalized(f32 bc[2], f32 *divisor, const v3 s0, const v3 s1, const v3 point)
{
    const v3 s01 = V3Sub(s1, s0);
    const v3 s0p = V3Sub(point, s0);
    const v3 ps1 = V3Sub(s1, point);

    bc[0] = V3Dot(s01, ps1);
    bc[1] = V3Dot(s01, s0p);
    *divisor = V3Dot(s01, s01);
}

static inline void SegmentOriginBCUnnormalized(f32 bc[2], f32 *divisor, const v3 s0, const v3 s1)
{
    const v3 s01 = V3Sub(s1, s0);
    bc[0] = V3Dot(s01, s1);
    bc[1] = -V3Dot(s01, s0);
    *divisor = V3Dot(s01, s01);
}

static inline void TriBCUnnormalized(f32 bc[3], f32 *divisor, const v3 t0, const v3 t1, const v3 t2, const v3 p)
{
    /*
     * Barycentric coordinates (Ericson 3.4): q = u*t0 + v*t1 + w*t2 with u + v + w = 1, and
     *
     *     u = Area(q, t1, t2) / Area(t0, t1, t2),  v = Area(q, t2, t0) / Area(t0, t1, t2),  w = 1 - u - v
     *
     * using signed areas. Why area ratios: keep the edge t12 fixed. Area(q, t1, t2) is linear in the
     * distance from q to that edge: 0 on the edge, Area(t0, t1, t2) at t0. u behaves exactly the same way.
     *
     * Signed areas from crosses: a x b is an area vector, normal to a and b with length 2 * area of the
     * triangle they span. Let n = (t1 - t0) x (t2 - t0), m = n / |n|, and write p = q + h*m, with q the
     * projection of p onto the plane. Splitting the sub-cross of p:
     *
     *     (t1 - p) x (t2 - p) = (t1 - q) x (t2 - q)                  (1)
     *                         - h (t1 - q) x m - h m x (t2 - q)      (2)
     *                         + h^2 m x m                            (3) = 0
     *
     * (1) is the area vector of the projected triangle (q, t1, t2): t1 - q and t2 - q lie in the plane,
     * so (1) = 2 Area(q, t1, t2) m. (2) crosses in-plane vectors with m, so it lies in the plane: the
     * tilt added by lifting p off the plane. Dotting with n removes (2) and keeps the length of (1),
     * scaled by |n|; the full triangle gives the same scale:
     *
     *     dot((t1 - p) x (t2 - p), n) = 2 Area(q, t1, t2) |n|,     dot(n, n) = 2 Area(t0, t1, t2) |n|
     */
    const v3 t01 = V3Sub(t1, t0);
    const v3 t12 = V3Sub(t2, t1);
    const v3 t20 = V3Sub(t0, t2);
    /* flip arguments for CCW cross */
    const v3 n = V3Cross(t20, t01);

	const v3 pt0 = V3Sub(t0, p);
	const v3 pt1 = V3Sub(t1, p);
	const v3 pt2 = V3Sub(t2, p);
    /* flip arguments for CCW cross */
    const v3 c_t01p = V3Cross(pt0, t01);
    const v3 c_t12p = V3Cross(pt1, t12);
    const v3 c_t20p = V3Cross(pt2, t20);

    bc[0] = V3Dot(c_t12p, n);
    bc[1] = V3Dot(c_t20p, n);
    bc[2] = V3Dot(c_t01p, n);
    *divisor = V3Dot(n, n);
}

static inline void TriOriginBCUnnormalized(f32 bc[3], f32 *divisor, const v3 t0, const v3 t1, const v3 t2)
{
    const v3 t01 = V3Sub(t1, t0);
    const v3 t02 = V3Sub(t2, t0);
    const v3 n = V3Cross(t01, t02);

    const v3 c_01 = V3Cross(t0, t1);
    const v3 c_12 = V3Cross(t1, t2);
    const v3 c_20 = V3Cross(t2, t0);

    bc[0] = V3Dot(c_12, n);
    bc[1] = V3Dot(c_20, n);
    bc[2] = V3Dot(c_01, n);
    *divisor = V3Dot(n, n);
}

static inline enum voronoi SegmentVoronoi(f32 w[2], const v3 s0, const v3 s1, const v3 p)
{
    f32 bc[2], divisor;
    enum voronoi r = VORONOI_INVALID;
    SegmentBCUnnormalized(bc, &divisor, s0, s1, p);
    /* Handles degenerate case, divisor == bc[0] == bc[1] == 0.0f */
    if (bc[0] <= 0.0f)
    {
        r = VORONOI_V1;
        w[0] = 0.0f;
        w[1] = 1.0f;
    }
    else if (bc[1] <= 0.0f)
    {
        r = VORONOI_V0;
        w[0] = 1.0f;
        w[1] = 0.0f;
    }
    /* safety net: divisor <= 0 with both bc > 0 happens only through rounding */
    else if (divisor > 0.0f)
    {
        r = VORONOI_E01;
        w[0] = bc[0] / divisor;
        w[1] = bc[1] / divisor;
    }

    return r;
}

static inline enum voronoi SegmentOriginVoronoi(f32 w[2], const v3 s0, const v3 s1)
{
    f32 bc[2], divisor;
    enum voronoi r = VORONOI_INVALID;
    SegmentOriginBCUnnormalized(bc, &divisor, s0, s1);
    /* see SegmentVoronoi */
    if (bc[0] <= 0.0f)
    {
        r = VORONOI_V1;
        w[0] = 0.0f;
        w[1] = 1.0f;
    }
    else if (bc[1] <= 0.0f)
    {
        r = VORONOI_V0;
        w[0] = 1.0f;
        w[1] = 0.0f;
    }
    else if (divisor > 0.0f)
    {
        r = VORONOI_E01;
        w[0] = bc[0] / divisor;
        w[1] = bc[1] / divisor;
    }

    return r;
}

static inline enum voronoi TriVoronoi(f32 w[3], const v3 t0, const v3 t1, const v3 t2, const v3 p)
{
    f32 bc01[2], bc12[2], bc20[2], bc012[3], div01, div12, div20, div012;

    w[0] = 0.0f;
    w[1] = 0.0f;
    w[2] = 0.0f;

    SegmentBCUnnormalized(bc01, &div01, t0, t1, p);
    SegmentBCUnnormalized(bc20, &div20, t2, t0, p);
    if (bc20[0] <= 0.0f && bc01[1] <= 0.0f)
    {
        w[0] = 1.0f;
        return VORONOI_V0;
    }

    SegmentBCUnnormalized(bc12, &div12, t1, t2, p);
    if (bc01[0] <= 0.0f && bc12[1] <= 0.0f)
    {
        w[1] = 1.0f;
        return VORONOI_V1;
    }

    if (bc12[0] <= 0.0f && bc20[1] <= 0.0f)
    {
        w[2] = 1.0f;
        return VORONOI_V2;
    }

    TriBCUnnormalized(bc012, &div012, t0, t1, t2, p);
    if (bc12[0] > 0.0f && bc12[1] > 0.0f && bc012[0] <= 0.0f)
    {
        w[1] = bc12[0] / div12;
        w[2] = bc12[1] / div12;
        return (div12 > 0.0f)
            ? VORONOI_E12
            : VORONOI_INVALID;
    }
    
    if (bc20[0] > 0.0f && bc20[1] > 0.0f && bc012[1] <= 0.0f)
    {
        w[2] = bc20[0] / div20;
        w[0] = bc20[1] / div20;
        return (div20 > 0.0f)
            ? VORONOI_E02
            : VORONOI_INVALID;
    }

    if (bc01[0] > 0.0f && bc01[1] > 0.0f && bc012[2] <= 0.0f)
    {
        w[0] = bc01[0] / div01;
        w[1] = bc01[1] / div01;
        return (div01 > 0.0f)
            ? VORONOI_E01
            : VORONOI_INVALID;
    }

    if (bc012[0] > 0.0f && bc012[1] > 0.0f && bc012[2] > 0.0f)
    {
        w[0] = bc012[0] / div012;
        w[1] = bc012[1] / div012;
        w[2] = bc012[2] / div012;
        return (div012 > 0.0f)
            ? VORONOI_F012
            : VORONOI_INVALID;
    }

    return VORONOI_INVALID;
}

static inline enum voronoi TriOriginVoronoi(f32 w[3], const v3 t0, const v3 t1, const v3 t2)
{
    f32 bc01[2], bc12[2], bc20[2], bc012[3], div01, div12, div20, div012;

    w[0] = 0.0f;
    w[1] = 0.0f;
    w[2] = 0.0f;

    SegmentOriginBCUnnormalized(bc01, &div01, t0, t1);
    SegmentOriginBCUnnormalized(bc20, &div20, t2, t0);
    if (bc20[0] <= 0.0f && bc01[1] <= 0.0f)
    {
        w[0] = 1.0f;
        return VORONOI_V0;
    }

    SegmentOriginBCUnnormalized(bc12, &div12, t1, t2);
    if (bc01[0] <= 0.0f && bc12[1] <= 0.0f)
    {
        w[1] = 1.0f;
        return VORONOI_V1;
    }

    if (bc12[0] <= 0.0f && bc20[1] <= 0.0f)
    {
        w[2] = 1.0f;
        return VORONOI_V2;
    }

    TriOriginBCUnnormalized(bc012, &div012, t0, t1, t2);
    if (bc12[0] > 0.0f && bc12[1] > 0.0f && bc012[0] <= 0.0f)
    {
        w[1] = bc12[0] / div12;
        w[2] = bc12[1] / div12;
        return (div12 > 0.0f)
            ? VORONOI_E12
            : VORONOI_INVALID;
    }
    
    if (bc20[0] > 0.0f && bc20[1] > 0.0f && bc012[1] <= 0.0f)
    {
        w[2] = bc20[0] / div20;
        w[0] = bc20[1] / div20;
        return (div20 > 0.0f)
            ? VORONOI_E02
            : VORONOI_INVALID;
    }

    if (bc01[0] > 0.0f && bc01[1] > 0.0f && bc012[2] <= 0.0f)
    {
        w[0] = bc01[0] / div01;
        w[1] = bc01[1] / div01;
        return (div01 > 0.0f)
            ? VORONOI_E01
            : VORONOI_INVALID;
    }

    if (bc012[0] > 0.0f && bc012[1] > 0.0f && bc012[2] > 0.0f)
    {
        w[0] = bc012[0] / div012;
        w[1] = bc012[1] / div012;
        w[2] = bc012[2] / div012;
        return (div012 > 0.0f)
            ? VORONOI_F012
            : VORONOI_INVALID;
    }

    return VORONOI_INVALID;
}

//TODO
static inline enum voronoi TetrahedronVoronoi(f32 w[4], const v3 t0, const v3 t1, const v3 t2, const v3 t3, const v3 p)
{
    return VORONOI_INVALID;
}

//TODO
static inline enum voronoi TetrahedronOriginVoronoi(f32 w[4], const v3 t0, const v3 t1, const v3 t2, const v3 t3)
{
    return VORONOI_INVALID;
}

#endif
