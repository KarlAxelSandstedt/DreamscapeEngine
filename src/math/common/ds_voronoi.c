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

#include "ds_geometry.h"

/* Voronoi queries that need ds_geometry.h (segments); declared in ds_voronoi.h */

enum voronoi TriSegmentVoronoi(f32 w[3], f32 *s, const v3 t0, const v3 t1, const v3 t2, const v3 s0, const v3 s1)
{
    /*
     * Ericson 5.1.10: the segment pierces the triangle, or the closest pair involves a segment end point (point vs
     * triangle) or a triangle edge (segment vs segment). Each candidate is exact up to rounding; the closest wins.
     *
     * Piercing is decided from the signs of the end points' plane distances h_i (unnormalized normal): strictly
     * opposite signs bound lambda = h0 / (h0 - h1) to (0, 1). A crossing point just outside the face by rounding
     * falls to the edge candidates, which then give distance ~0.
     */
    const v3 t[3] = { t0, t1, t2 };
    const v3 n = V3Cross(V3Sub(t1, t0), V3Sub(t2, t0));
    const f32 h0 = V3Dot(n, V3Sub(s0, t0));
    const f32 h1 = V3Dot(n, V3Sub(s1, t0));
    if ((h0 < 0.0f && 0.0f < h1) || (h1 < 0.0f && 0.0f < h0))
    {
        const f32 lambda = h0 / (h0 - h1);
        const v3 x = V3Add(s0, V3Scale(V3Sub(s1, s0), lambda));
        if (TriVoronoi(w, t0, t1, t2, x) == VORONOI_F012)
        {
            *s = lambda;
            return VORONOI_F012;
        }
    }

    enum voronoi best = VORONOI_INVALID;
    f32 best_dist_sq = F32_INFINITY;
    f32 best_w[3] = { 0.0f, 0.0f, 0.0f };
    f32 best_s = 0.0f;

    /* end points vs the triangle */
    const v3 end[2] = { s0, s1 };
    for (u32 i = 0; i < 2; ++i)
    {
        f32 w_i[3];
        const enum voronoi r = TriVoronoi(w_i, t0, t1, t2, end[i]);
        if (r == VORONOI_INVALID)
        {
            continue;
        }
        const v3 c = V3Add(V3Add(V3Scale(V3Sub(t0, end[i]), w_i[0]), V3Scale(V3Sub(t1, end[i]), w_i[1])), V3Scale(V3Sub(t2, end[i]), w_i[2]));
        const f32 dist_sq = V3Dot(c, c);
        if (dist_sq < best_dist_sq)
        {
            best_dist_sq = dist_sq;
            best = r;
            best_w[0] = w_i[0];
            best_w[1] = w_i[1];
            best_w[2] = w_i[2];
            best_s = (f32) i;
        }
    }

    /* the edges vs the segment */
    const struct segment seg = Segment(s0, s1);
    for (u32 a = 0; a < 3; ++a)
    {
        const u32 b = (a + 1) % 3;
        const struct segment edge = Segment(t[a], t[b]);
        f32 t_edge, t_seg;
        SegmentClosestParameter(&t_edge, &t_seg, &edge, &seg);
        const v3 c = V3Sub(SegmentBc(&edge, t_edge), SegmentBc(&seg, t_seg));
        const f32 dist_sq = V3Dot(c, c);
        if (dist_sq < best_dist_sq)
        {
            best_dist_sq = dist_sq;
            best_w[0] = 0.0f;
            best_w[1] = 0.0f;
            best_w[2] = 0.0f;
            best_w[a] = 1.0f - t_edge;
            best_w[b] = t_edge;
            best = (enum voronoi) (((t_edge < 1.0f) << a) | ((0.0f < t_edge) << b));
            best_s = t_seg;
        }
    }

    w[0] = best_w[0];
    w[1] = best_w[1];
    w[2] = best_w[2];
    *s = best_s;
    return best;
}
