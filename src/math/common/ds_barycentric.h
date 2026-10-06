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

#ifndef __DS_BARYCENTRIC_H__
#define __DS_BARYCENTRIC_H__

#include "geometry.h"

//TODO Inline?
static void SegmentBCUnnormalized(f32 *dist0, f32 *dist1, const struct segment s, const v3 point)
{
}

//TODO Inline?
/*
 * Unnormalized barycentric coordinates of p projected onto the plane of CCW triangle (t0, t1, t2).
 * bc_i is the weight of t_i, and bc0 + bc1 + bc2 = divisor = |n|^2; bc_i / divisor are the 
 * barycentric coordinates.
 *
 * WARNING: divisor <= 0 means a degenerate (collinear) triangle.
 */
static void TriBCUnnormalized(f32 *bc0, f32 *bc1, f32 *bc2, f32 *divisor, const v3 t0, const v3 t1, const v3 t2, const v3 p)
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
    const v3 t20 = V3Sub(t2, t0);
    /* flip arguments for CCW cross */
    const v3 n = V3Cross(t20, t01);

	const v3 pt0 = V3Sub(t0, p);
	const v3 pt1 = V3Sub(t1, p);
	const v3 pt2 = V3Sub(t2, p);
    /* flip arguments for CCW cross */
    const v3 c_t01p = V3Cross(pt0, t01);
    const v3 c_t12p = V3Cross(pt1, t12);
    const v3 c_t20p = V3Cross(pt2, t20);

    *bc0 = V3Dot(c_t01p, n);
    *bc1 = V3Dot(c_t12p, n);
    *bc2 = V3Dot(c_t20p, n);
    *divisor = V3Dot(n, n);
}

//TODO Inline?
/* TriBCUnnormalized with origin as the query point. */
static void TriOriginBCUnnormalized(f32 *bc0, f32 *bc1, f32 *bc2, f32 *divisor, const v3 t0, const v3 t1, const v3 t2)
{
    const v3 t01 = V3Sub(t1, t0);
    const v3 t02 = V3Sub(t2, t0);
    const v3 n = V3Cross(t01, t02);

    const v3 c_01 = V3Cross(t0, t1);
    const v3 c_12 = V3Cross(t1, t2);
    const v3 c_20 = V3Cross(t2, t0);

    *bc0 = V3Dot(c_01, n);
    *bc1 = V3Dot(c_12, n);
    *bc2 = V3Dot(c_20, n);
    *divisor = V3Dot(n, n);
}

#endif
