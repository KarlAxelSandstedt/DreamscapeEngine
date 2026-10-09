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
ds_voronoi.h
============
Closest features of segments, triangles and tetrahedra to a point, with barycentric weights.

Voronoi API
-----------
Description:
    XVoronoi returns the feature (vertex, edge, face or the tetrahedron itself) closest to a point, as an
    enum voronoi, and the weight w[i] of every vertex: 0 outside the feature, in [0, 1] and summing to 1
    up to rounding (see Internals). Every function has an origin variant (query point = origin).

Usage:
    f32 w[3];
    const enum voronoi region = TriVoronoi(w, t0, t1, t2, p);
    if (region == VORONOI_INVALID)          (numerically degenerate input, w is garbage: must be handled)
    closest point  = w[0] t0 + w[1] t1 + w[2] t2
    vertex i is part of the feature  <=>  region & (1 << i)

Internals:
    enum voronoi is the bitmask of the feature's vertices. Degenerate input (coincident, collinear or
    coplanar vertices) resolves to a lower feature; VORONOI_INVALID only comes from a divisor <= 0 at
    normalization through rounding. Regions are decided from the signs of the unnormalized barycentric
    coordinates below, then only the chosen feature is normalized.

    w[i] = bc[i] / divisor, with the divisor from the feature alone (|s01|^2, |n|^2) and the numerators
    from the vertices relative to the point. The numerators' rounding grows with r = |vertex - point|, so
    the weights sum to 1 only up to

        edge:   ~eps * r / f            f: the feature's size (edge length, sqrt of the face's |n|)
        face:   ~eps * (r / f)^2

    Near the feature (r ~ f, e.g. contacts) that is a few eps; far from it the closest point
    sum_i w[i] t_i shrinks by the same factor (measured ~1e-4 at r ~ 100 f). Callers that need an exact
    affine blend at range divide by sum_i w[i].

Barycentric API
---------------
Description:
    XBCUnnormalized returns the signed numerators bc[i] and the divisor of the barycentric coordinates of
    the point's unclamped projection: bc[0] + ... = divisor, and bc[i] / divisor is the coordinate of
    vertex i. bc[i] < 0 means the point lies beyond the feature opposite vertex i.

Usage (only for own region logic; otherwise use the Voronoi API):
    1. decide the region from the signs of the numerators (no division),
    2. normalize only the chosen feature, with that feature's own numerators and divisor,
    3. handle divisor <= 0 there (exactly degenerate, or rounding) as a failure with a workable result.

Internals:
    There are deliberately no normalized versions: barycentric coordinates of an unclamped projection are
    ill-conditioned for nearly degenerate primitives (a sliver's plane or a short segment's direction is
    barely defined). In a face or edge region all chosen numerators are > 0, so the weights lie in (0, 1).
    The general versions use edge forms (precise for a far point), the origin versions Box3D's forms; see
    the error derivation in TetrahedronBCUnnormalized.
*/

#ifndef __DS_VORONOI_H__
#define __DS_VORONOI_H__

#include "ds_vector.h"

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
 * weight of s_i, 0 for vertices outside the feature; all w[i] in [0, 1] and summing to 1 up to
 * rounding (see the Voronoi API).
 *
 * A degenerate segment (s0 == s1) gives VORONOI_V1. Returns VORONOI_INVALID only if the geometry is
 * numerically degenerate (rounding); w is then garbage.
 */
static inline enum voronoi SegmentVoronoi(f32 w[2], const v3 s0, const v3 s1, const v3 p);
/* SegmentVoronoi with origin as the query point. */
static inline enum voronoi SegmentOriginVoronoi(f32 w[2], const v3 s0, const v3 s1);

/*
 * Return the feature of triangle (t0, t1, t2) closest to p, and its barycentric weights: w[i] is the
 * weight of t_i, 0 for vertices outside the feature; all w[i] in [0, 1] and summing to 1 up to
 * rounding (see the Voronoi API).
 *
 * A collinear triangle or coincident vertices give an edge or a vertex. Returns VORONOI_INVALID only if the
 * geometry is numerically degenerate (rounding); w is then garbage.
 */
static inline enum voronoi TriVoronoi(f32 w[3], const v3 t0, const v3 t1, const v3 t2, const v3 p);
/* TriVoronoi with origin as the query point. */
static inline enum voronoi TriOriginVoronoi(f32 w[3], const v3 t0, const v3 t1, const v3 t2);

/*
 * Return the feature of tetrahedron (t0, t1, t2, t3) closest to p, and its barycentric weights: w[i] is
 * the weight of t_i, 0 for vertices outside the feature; all w[i] in [0, 1] and summing to 1 up to
 * rounding (see the Voronoi API). p inside gives VORONOI_T0123.
 *
 * A flat (coplanar) tetrahedron gives its closest face, edge or vertex. Returns VORONOI_INVALID only if
 * every candidate face is numerically degenerate (rounding); w is then garbage.
 */
static inline enum voronoi TetrahedronVoronoi(f32 w[4], const v3 t0, const v3 t1, const v3 t2, const v3 t3, const v3 p);
/* TetrahedronVoronoi with origin as the query point. */
static inline enum voronoi TetrahedronOriginVoronoi(f32 w[4], const v3 t0, const v3 t1, const v3 t2, const v3 t3);

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
 * Unnormalized barycentric coordinates of p with respect to tetrahedron (t0, t1, t2, t3). bc[i] is the
 * weight of t_i: 6 * the signed volume of the tetrahedron with p in place of t_i. bc[0] + bc[1] + bc[2] +
 * bc[3] = divisor = 6 * signed volume of (t0, t1, t2, t3); all five are multiplied by the sign of the
 * volume, so divisor >= 0 for either vertex order. bc[i] < 0: p lies beyond the face opposite t_i; all
 * bc[i] > 0: p lies inside.
 *
 * WARNING: divisor == 0 for a degenerate (coplanar) tetrahedron; the caller must handle it before dividing.
 */
static inline void TetrahedronBCUnnormalized(f32 bc[4], f32 *divisor, const v3 t0, const v3 t1, const v3 t2, const v3 t3, const v3 p);
/* TetrahedronBCUnnormalized with origin as the query point. */
static inline void TetrahedronOriginBCUnnormalized(f32 bc[4], f32 *divisor, const v3 t0, const v3 t1, const v3 t2, const v3 t3);


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
    /* n = (t1 - t0) x (t2 - t0) */
    const v3 n = V3Cross(t20, t01);

	const v3 pt0 = V3Sub(t0, p);
	const v3 pt1 = V3Sub(t1, p);
	const v3 pt2 = V3Sub(t2, p);
    /* (t_i - p) x (t_j - t_i) = area vector of the sub-triangle (t_i, t_j, p), wound like (t0, t1, t2) */
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

static inline void TetrahedronBCUnnormalized(f32 bc[4], f32 *divisor, const v3 t0, const v3 t1, const v3 t2, const v3 t3, const v3 p)
{
    /*
     * V(a, b, c, d) = (b - a) . ((c - a) x (d - a))       (6 * signed tetrahedron volume)
     *
     * divisor = V(t0, t1, t2, t3)
     * bc[i]   = V with t_i replaced by p
     *
     * bc[i] is the height of p above the face opposite t_i, times its area (the volume): 0 on that
     * plane, sign flips across it. V is linear in each vertex and 0 for a repeated one, so it
     * follows that for p = sum_j l_j t_j:
     *
     *      V(p, t1, t2, t3) = sum_j l_j V(t_j, t1, t2, t3) = l_0 * divisor
     *
     * 
     * Edge form. Any vertex can be the base of V (swapping two vertices flips the sign) and cyclic shifts
     * of the three vectors keep it, so p never has to share a cross product with another p-relative vector:
     *
     *      bc[0]    = V(p, t1, t2, t3) = -V(t1, p, t2, t3) = (t1 - p) . ((t2 - t1) x (t3 - t1))
     *      bc[1..3] = (p - t0) in the slot of t_i, edges from t0 elsewhere
     *
     * Error derivation. Each float operation rounds once, by a relative u = 2^-24. Let D = distance of p,
     * L = edge length, h = height of p above a face plane.
     *
     *  1. Subtractions are harmless: a - b is off by at most u |a - b|, long or short.
     *
     *  2. Cross products are not: a component a_y b_z - a_z b_y rounds two products of size |a| |b|,
     *     so its error is ~ u |a| |b|, however small the result.
     *
     *  3. Box3D's form crosses two long vectors a = t2 - p and b = t3 - p (|a|, |b| ~ D). They differ
     *     only by the edge e = t3 - t2, so the long parts cancel:
     *
     *      a x b = a x (a + e) = a x e,     |a x b| ~ D L      (thin triangle p, t2, t3)
     *      error ~ u D^2                ->  relative error ~ u D / L
     *
     *  4. The edge form crosses two edges: result ~ L^2, error ~ u L^2, relative error ~ u. Only the
     *     final dot with (t1 - p) can cancel, when p lies close to the face plane:
     *
     *      products ~ D L^2,   result ~ h L^2     ->  relative error ~ u D / h
     *
     *     This part is inherent: p itself is only known to u D.
     *
     * Both forms cost the same; the rule is: never build a short result from long intermediates.
     *
     * Measured (unit-size tetrahedra, worst relative weight error, double reference):
     *
     *      |p|     1         100       10^4      10^6
     *      edge    2.8e-5    2.0e-4    2.1e-3    3.3e+1      (~ u D, the inherent part)
     *      Box3D   3.2e-5    9.7e-1    1.9e+5    inf         (~ u D^2: the extra D / L)
     */
    const v3 t01 = V3Sub(t1, t0);
    const v3 t02 = V3Sub(t2, t0);
    const v3 t03 = V3Sub(t3, t0);
    const v3 t0p = V3Sub(p, t0);
    const v3 t12 = V3Sub(t2, t1);
    const v3 t13 = V3Sub(t3, t1);
    const v3 pt1 = V3Sub(t1, p);
    
    *divisor = V3Dot(t01, V3Cross(t02, t03));
    const f32 sign = (*divisor >= 0.0f) ? 1.0f : -1.0f;
    *divisor *= sign;
    bc[0] = sign*V3Dot(pt1, V3Cross(t12, t13));
    bc[1] = sign*V3Dot(t0p, V3Cross(t02, t03));
    bc[2] = sign*V3Dot(t01, V3Cross(t0p, t03));
    bc[3] = sign*V3Dot(t01, V3Cross(t02, t0p));
}

static inline void TetrahedronOriginBCUnnormalized(f32 bc[4], f32 *divisor, const v3 t0, const v3 t1, const v3 t2, const v3 t3)
{
    const v3 t01 = V3Sub(t1, t0);
    const v3 t02 = V3Sub(t2, t0);
    const v3 t03 = V3Sub(t3, t0);

    *divisor = V3Dot(t01, V3Cross(t02, t03));
    const f32 sign = (*divisor >= 0.0f) ? 1.0f : -1.0f;
    *divisor *= sign;
    bc[0] = sign*V3Dot(t1, V3Cross(t2, t3));
    bc[1] = sign*V3Dot(t0, V3Cross(t3, t2));
    bc[2] = sign*V3Dot(t0, V3Cross(t1, t3));
    bc[3] = sign*V3Dot(t0, V3Cross(t2, t1));
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

/* vertices of the face opposite t_f, in increasing order */
static const u32 table_tetrahedron_face_barycentric[4][3] =
{
    { 1, 2, 3 },
    { 0, 2, 3 },
    { 0, 1, 3 },
    { 0, 1, 2 },
};

static inline enum voronoi TetrahedronVoronoi(f32 w[4], const v3 t0, const v3 t1, const v3 t2, const v3 t3, const v3 p)
{
    /*
     * Ericson 5.1.6: p inside all four face half-spaces is its own closest point. Otherwise the closest
     * point lies on a face whose plane p is on or beyond (bc[i] <= 0 for the face opposite t_i); take the
     * closest TriVoronoi result over those faces. A flat tetrahedron (divisor == 0) is the union of its
     * faces, so all four are candidates.
     */
    f32 bc[4], divisor;
    TetrahedronBCUnnormalized(bc, &divisor, t0, t1, t2, t3, p);
    if (divisor > 0.0f && bc[0] > 0.0f && bc[1] > 0.0f && bc[2] > 0.0f && bc[3] > 0.0f)
    {
        w[0] = bc[0] / divisor;
        w[1] = bc[1] / divisor;
        w[2] = bc[2] / divisor;
        w[3] = bc[3] / divisor;
        return VORONOI_T0123;
    }

    const v3 t[4] = { t0, t1, t2, t3 };
    f32 best_w[3] = { 0.0f, 0.0f, 0.0f };
    f32 best_dist_sq = F32_INFINITY;
    u32 best_face = 0;
    enum voronoi best_region = VORONOI_INVALID;
    for (u32 f = 0; f < 4; ++f)
    {
        if (divisor > 0.0f && bc[f] > 0.0f)
        {
            continue;
        }

        const u32 *fi = table_tetrahedron_face_barycentric[f];
        f32 wf[3];
        const enum voronoi r = TriVoronoi(wf, t[fi[0]], t[fi[1]], t[fi[2]], p);
        if (r == VORONOI_INVALID)
        {
            continue;
        }

        const v3 c = V3Add(V3Add(V3Scale(t[fi[0]], wf[0]), V3Scale(t[fi[1]], wf[1])), V3Scale(t[fi[2]], wf[2]));
        const f32 dist_sq = V3DistanceSquared(c, p);
        /* strict <: on ties (e.g. an edge shared by two faces) the first face wins */
        if (dist_sq < best_dist_sq)
        {
            best_dist_sq = dist_sq;
            best_region = r;
            best_face = f;
            best_w[0] = wf[0];
            best_w[1] = wf[1];
            best_w[2] = wf[2];
        }
    }

    if (best_region == VORONOI_INVALID)
    {
        return VORONOI_INVALID;
    }

    /* map the face's local vertices 0, 1, 2 back to tetrahedron vertices */
    const u32 *fi = table_tetrahedron_face_barycentric[best_face];
    u32 region = 0;
    w[0] = 0.0f;
    w[1] = 0.0f;
    w[2] = 0.0f;
    w[3] = 0.0f;
    for (u32 j = 0; j < 3; ++j)
    {
        w[fi[j]] = best_w[j];
        region |= ((best_region >> j) & 0x1) << fi[j];
    }

    return (enum voronoi) region;
}

static inline enum voronoi TetrahedronOriginVoronoi(f32 w[4], const v3 t0, const v3 t1, const v3 t2, const v3 t3)
{
    /* see TetrahedronVoronoi, with p = origin */
    f32 bc[4], divisor;
    TetrahedronOriginBCUnnormalized(bc, &divisor, t0, t1, t2, t3);
    if (divisor > 0.0f && bc[0] > 0.0f && bc[1] > 0.0f && bc[2] > 0.0f && bc[3] > 0.0f)
    {
        w[0] = bc[0] / divisor;
        w[1] = bc[1] / divisor;
        w[2] = bc[2] / divisor;
        w[3] = bc[3] / divisor;
        return VORONOI_T0123;
    }

    const v3 t[4] = { t0, t1, t2, t3 };
    f32 best_w[3] = { 0.0f, 0.0f, 0.0f };
    f32 best_dist_sq = F32_INFINITY;
    u32 best_face = 0;
    enum voronoi best_region = VORONOI_INVALID;
    for (u32 f = 0; f < 4; ++f)
    {
        if (divisor > 0.0f && bc[f] > 0.0f)
        {
            continue;
        }

        const u32 *fi = table_tetrahedron_face_barycentric[f];
        f32 wf[3];
        const enum voronoi r = TriOriginVoronoi(wf, t[fi[0]], t[fi[1]], t[fi[2]]);
        if (r == VORONOI_INVALID)
        {
            continue;
        }

        const v3 c = V3Add(V3Add(V3Scale(t[fi[0]], wf[0]), V3Scale(t[fi[1]], wf[1])), V3Scale(t[fi[2]], wf[2]));
        const f32 dist_sq = V3LengthSquared(c);
        /* strict <: on ties (e.g. an edge shared by two faces) the first face wins */
        if (dist_sq < best_dist_sq)
        {
            best_dist_sq = dist_sq;
            best_region = r;
            best_face = f;
            best_w[0] = wf[0];
            best_w[1] = wf[1];
            best_w[2] = wf[2];
        }
    }

    if (best_region == VORONOI_INVALID)
    {
        return VORONOI_INVALID;
    }

    /* map the face's local vertices 0, 1, 2 back to tetrahedron vertices */
    const u32 *fi = table_tetrahedron_face_barycentric[best_face];
    u32 region = 0;
    w[0] = 0.0f;
    w[1] = 0.0f;
    w[2] = 0.0f;
    w[3] = 0.0f;
    for (u32 j = 0; j < 3; ++j)
    {
        w[fi[j]] = best_w[j];
        region |= ((best_region >> j) & 0x1) << fi[j];
    }

    return (enum voronoi) region;
}

#endif
