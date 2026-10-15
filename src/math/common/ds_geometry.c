/*
==========================================================================
    Copyright (C) 2025, 2026 Axel Sandstedt 

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

#include <string.h>
#include "ds_base.h"
#include "ds_math.h"
#include "ds_geometry.h"
#include "ds_float.h"
#include "ds_vector.h"
#include "ds_matrix.h"


void SegmentClosestParameter(f32 *t1, f32 *t2, const struct segment *s1, const struct segment *s2)
{
	const v3 diff = V3Sub(s2->p[0], s1->p[0]);
	const f32 d1d1 = V3LengthSquared(s1->dir);
	const f32 d2d2 = V3LengthSquared(s2->dir);

	*t1 = 0.0f;
	*t2 = 0.0f;

	/*
	 * A segment is a point only if |dir|^2 == 0: every quotient below is clamped (Inf included) or already in
	 * [0, divisor], so short segments need no length threshold.
	 */
	if (d1d1 > 0.0f && d2d2 > 0.0f)
	{
		const f32 d1d2 = V3Dot(s1->dir, s2->dir);
		const f32 diffd1 = V3Dot(diff, s1->dir);
		const f32 diffd2 = V3Dot(diff, s2->dir);
		/*
		 * Lagrange's identity: d1d1 d2d2 - d1d2^2 = |d1 x d2|^2, and t1's numerator diffd1 d2d2 - diffd2 d1d2 is
		 * (diff x d2) . (d1 x d2). The cross products keep their relative error near eps / sin(angle) for nearly
		 * parallel segments, where the dot forms cancel to eps / sin^2(angle) and leave t1 arbitrary below
		 * ~1e-3 rad (distance error up to angle * length). denom == 0: parallel, t1 = 0 is as good as any.
		 */
		const v3 n = V3Cross(s1->dir, s2->dir);
		const f32 denom = V3Dot(n, n);
		if (denom > 0.0f)
		{
			*t1 = F32Clamp(V3Dot(V3Cross(diff, s2->dir), n) / denom, 0.0f, 1.0f);
		}

		/*
		 *  t2 = (L1_P1*(1-t1) + L1_P2*t1 - L2_P1) * DIR2 / DIR2*DIR2
		 *     = (-DIFF + DIR1*t1) * DIR2 / DIR2*DIR2
		 *     = (-DIFF*DIR2 + DIR1*DIR2*t1) / DIR2*DIR2
		 */
		*t2 = F32Clamp(*t1*d1d2 - diffd2, 0.0f, d2d2);

		if (*t2 == 0.0f)
		{
			/*
			 *  t1 = (L2_P1*(1-t2) + L2_P2*t2 - L1_P1) * DIR1 / DIR1*DIR1
			 *     = (DIFF + DIR2*t2) * DIR1 / DIR1*DIR1
			 *     = DIFF*DIR1 / DIR1*DIR1
			 */
            *t1 = F32Clamp(diffd1 / d1d1, 0.0f, 1.0f);
		}
		else if (*t2 == d2d2)
		{
			*t2 = 1.0f;
			*t1 = F32Clamp((diffd1 + d1d2) / d1d1, 0.0f, 1.0f);
		}	
		else
		{
			*t2 /= d2d2;
		}
	}
	/* S2 is point */
	else if (d1d1 > 0.0f)
	{
		/* 
		 * SIGNED PROJECTED LENGTH 
		 * 	= (L2_P1 - L1_P1) * DIR1 / |DIR1| 
		 * 	= t1*|DIR1|
		 * => t = DIFF*DIR1 / (DIR1*DIR1) 
		 */
        *t1 = F32Clamp(V3Dot(diff, s1->dir) / d1d1, 0.0f, 1.0f);
	}
	else if (d2d2 > 0.0f)
	{
		*t2 = F32Clamp(-V3Dot(diff, s2->dir) / d2d2, 0.0f, 1.0f);
	}

	ds_Assert(0.0f <= *t1 && *t1 <= 1.0f);
	ds_Assert(0.0f <= *t2 && *t2 <= 1.0f);
}




                                                                        





struct aabb BboxVertexSet(const v3 *v, const u32 count)
{
    if (count == 0)
    {
        return (struct aabb) { 0 };
    }

	v3 min = V3(F32_INFINITY, F32_INFINITY, F32_INFINITY);
	v3 max = V3(-F32_INFINITY, -F32_INFINITY, -F32_INFINITY);
	for (u32 i = 0; i < count; ++i)
	{
        min = V3Min(min, v[i]);
        max = V3Max(max, v[i]);
	}

    const struct aabb bbox = { .min = min, .max = max };
    return bbox;
}






	









/*
 * dcel: doubly-connected edge list (de Berg et al., Computational Geometry: Algorithms and Applications, section 2.2).
 * It can also represent 2D planar graphs; here it holds convex polyhedra, each face one polygon without holes,
 * implicitly defined by its first half edge.
 */

static v3 box_stub_vertex[8] =
{
	{ .x = 0.5f, .y = 0.5f, .z = 0.5f },
	{ .x = 0.5f, .y = 0.5f, .z = -0.5f },
	{ .x = -0.5f, .y = 0.5f, .z = -0.5f },
	{ .x = -0.5f, .y = 0.5f, .z = 0.5f },
	{ .x = 0.5f, .y = -0.5f, .z = 0.5f },
	{ .x = 0.5f, .y = -0.5f, .z = -0.5f },
	{ .x = -0.5f, .y = -0.5f, .z = -0.5f },
	{ .x = -0.5f, .y = -0.5f, .z = 0.5f },
};

static const struct dcelFace box_face[] =
{
	{ .first  =  0, .count = 4 },
	{ .first  =  4, .count = 4 },
	{ .first  =  8, .count = 4 },
	{ .first  = 12, .count = 4 },
	{ .first  = 16, .count = 4 },
	{ .first  = 20, .count = 4 },
};

static const struct dcelEdge box_edge[] =
{
	{ .origin = 0, .twin =  7,  .face_ccw = 0, },
	{ .origin = 1, .twin = 11,  .face_ccw = 0, },
	{ .origin = 2, .twin = 15,  .face_ccw = 0, },
	{ .origin = 3, .twin = 19,  .face_ccw = 0, },

	{ .origin = 0, .twin = 18,  .face_ccw = 1, },
	{ .origin = 4, .twin = 21,  .face_ccw = 1, },
	{ .origin = 5, .twin =  8,  .face_ccw = 1, },
	{ .origin = 1, .twin =  0,  .face_ccw = 1, },

	{ .origin = 1, .twin =  6,  .face_ccw = 2, },
	{ .origin = 5, .twin = 20,  .face_ccw = 2, },
	{ .origin = 6, .twin = 12,  .face_ccw = 2, },
	{ .origin = 2, .twin =  1,  .face_ccw = 2, },

	{ .origin = 2, .twin = 10,  .face_ccw = 3, },
	{ .origin = 6, .twin = 23,  .face_ccw = 3, },
	{ .origin = 7, .twin = 16,  .face_ccw = 3, },
	{ .origin = 3, .twin =  2,  .face_ccw = 3, },

	{ .origin = 3, .twin = 14,  .face_ccw = 4, },
	{ .origin = 7, .twin = 22,  .face_ccw = 4, },
	{ .origin = 4, .twin =  4,  .face_ccw = 4, },
	{ .origin = 0, .twin =  3,  .face_ccw = 4, },

	{ .origin = 6, .twin =  9,  .face_ccw = 5, },
	{ .origin = 5, .twin =  5,  .face_ccw = 5, },
	{ .origin = 4, .twin = 17,  .face_ccw = 5, },
	{ .origin = 7, .twin = 13,  .face_ccw = 5, },
};

/* box_face's planes for box_stub_vertex (what DcelPlanesTryCompute returns for them) */
static struct plane box_stub_plane[6] =
{
	{ .normal = {{  0.0f,  1.0f,  0.0f }}, .signed_distance = 0.5f },
	{ .normal = {{  1.0f,  0.0f,  0.0f }}, .signed_distance = 0.5f },
	{ .normal = {{  0.0f,  0.0f, -1.0f }}, .signed_distance = 0.5f },
	{ .normal = {{ -1.0f,  0.0f,  0.0f }}, .signed_distance = 0.5f },
	{ .normal = {{  0.0f,  0.0f,  1.0f }}, .signed_distance = 0.5f },
	{ .normal = {{  0.0f, -1.0f,  0.0f }}, .signed_distance = 0.5f },
};

struct dcel DcelBoxStub(void)
{
	struct dcel box = 
	{
		.v = box_stub_vertex,
		.bbox = { .min = {{ -0.5f, -0.5f, -0.5f }}, .max = {{ 0.5f, 0.5f, 0.5f }} },
		.e = box_edge,
		.f = box_face,
		.plane = box_stub_plane,
		.e_count = 24,
		.v_count = 8,
		.f_count = 6,
	};

	return box; 
}

static const struct dcelFace tri_face[2] =
{
	{ .first  =  0, .count = 3 },
	{ .first  =  3, .count = 3 },
};
                                                        
static const struct dcelEdge tri_edge[6] =                    
{
	{ .origin = 0, .twin = 5,  .face_ccw = 0, },     
	{ .origin = 1, .twin = 4,  .face_ccw = 0, },    
	{ .origin = 2, .twin = 3,  .face_ccw = 0, },

	{ .origin = 0, .twin = 2,  .face_ccw = 1, },
	{ .origin = 2, .twin = 1,  .face_ccw = 1, },
	{ .origin = 1, .twin = 0,  .face_ccw = 1, },
};

struct dcel DcelTriStub(void)
{
	struct dcel tri = 
	{
		.v = NULL,
		.plane = NULL,
		.e = tri_edge,
		.f = tri_face,
		.e_count = 6,
		.v_count = 3,
		.f_count = 2,
	};

	return tri;
}

struct dcel DcelBox(struct arena *mem, const v3 hw)
{
	/* also rejects NaN; a negative half width would mirror the box and turn its planes inward */
	if (!(0.0f < hw.x && 0.0f < hw.y && 0.0f < hw.z))
	{
		Log(T_PHYSICS, S_WARNING, "DcelBox: half widths (%f, %f, %f) not all > 0, using the box stub", hw.x, hw.y, hw.z);
		ds_AssertString(0, "DcelBox: half widths must be > 0");
		return DcelBoxStub();
	}

	v3 *box_vertex = ArenaPush(mem, 8*sizeof(v3));

	box_vertex[0] = V3(hw.x,  hw.y,  hw.z); 
	box_vertex[1] = V3(hw.x,  hw.y, -hw.z);	
	box_vertex[2] = V3(-hw.x,  hw.y, -hw.z);	
	box_vertex[3] = V3(-hw.x,  hw.y,  hw.z);	
	box_vertex[4] = V3(hw.x, -hw.y,  hw.z);
	box_vertex[5] = V3(hw.x, -hw.y, -hw.z);	
	box_vertex[6] = V3(-hw.x, -hw.y, -hw.z);	
	box_vertex[7] = V3(-hw.x, -hw.y,  hw.z);	

	struct dcel box = 
	{
		.v = box_vertex,
		.plane = ArenaPush(mem, 6*sizeof(struct plane)),
		.e = box_edge,
		.f = box_face,
		.e_count = 24,
		.v_count = 8,
		.f_count = 6,
	};

	if (!DcelPlanesTryCompute(&box))
	{
		Log(T_PHYSICS, S_WARNING, "DcelBox: no planes for half widths (%f, %f, %f), box stub used", hw.x, hw.y, hw.z);
		ds_AssertString(0, "DcelBox: no planes (half widths beyond the f32 range)");
		box = DcelBoxStub();
	}
	else
	{
		DcelBboxCompute(&box);
	}

	return box; 
}

u32 DcelPlanesTryCompute(struct dcel *h)
{
	u32 valid = 1;
	for (u32 fi = 0; fi < h->f_count; ++fi)
	{
		/*
		 * Newell's method. Every edge (v1, v2) of the face adds the doubled signed area of its shadow on the
		 * yz, zx and xy planes (trapezoid rule); summed over the closed polygon these are the components of
		 * twice the area vector, area * unit normal, pointing to the side from which the face is CCW. For a
		 * triangle it equals the cross product (v1 - v0) x (v2 - v0). Coordinates are taken relative to the
		 * face's first vertex o to keep the products small (less rounding).
		 */
		const struct dcelFace *f = h->f + fi;
		const v3 o = h->v[h->e[f->first].origin];
		v3 n = V3Zero();
		v3 sum = V3Zero();
		for (u32 i = 0; i < f->count; ++i)
		{
			const u32 i1 = h->e[f->first + i].origin;
			const u32 i2 = h->e[f->first + ((i + 1) % f->count)].origin;
			const v3 v1 = V3Sub(h->v[i1], o);
			const v3 v2 = V3Sub(h->v[i2], o);
			n.x += (v1.y - v2.y) * (v1.z + v2.z);
			n.y += (v1.z - v2.z) * (v1.x + v2.x);
			n.z += (v1.x - v2.x) * (v1.y + v2.y);
			sum = V3Add(sum, v1);
		}

		/* |n| = 2 * area; 0 (no area) or Inf (|n|^2 beyond f32): no normal (normalizing would give 0 or NaN) */
		const f32 length = V3Length(n);
		if (0.0f < length && length < F32_INFINITY)
		{
			/* the plane passes through the average of the face's vertices, spreading their deviation */
			const v3 centroid = V3Add(o, V3Scale(sum, 1.0f / (f32) f->count));
			h->plane[fi].normal = V3Scale(n, 1.0f / length);
			h->plane[fi].signed_distance = V3Dot(h->plane[fi].normal, centroid);
		}
		else
		{
			valid = 0;
		}
	}

	return valid;
}

u32 DcelSupportFace(const struct dcel *h, const v3 dir)
{
	u32 best = 0;
	f32 best_dot = V3Dot(h->plane[0].normal, dir);
	for (u32 fi = 1; fi < h->f_count; ++fi)
	{
		const f32 dot = V3Dot(h->plane[fi].normal, dir);
		if (best_dot < dot)
		{
			best = fi;
			best_dot = dot;
		}
	}

	return best;
}

u32 DcelFaceSegmentClip(struct segment *clip, const struct dcel *h, const u32 fi, const struct segment *s)
{
	const v3 n = h->plane[fi].normal;
	const struct dcelFace *f = h->f + fi;
	f32 t_min = 0.0f;
	f32 t_max = 1.0f;
	/* edges (last, first), (first, first + 1), ...: the previous vertex is p0, no index wrap per edge */
	v3 p0 = h->v[h->e[f->first + f->count - 1].origin];
	for (u32 i = 0; i < f->count; ++i)
	{
		const v3 p1 = h->v[h->e[f->first + i].origin];
		if (!SegmentEdgeSideClip(&t_min, &t_max, s, p0, p1, n))
		{
			return 0;
		}
		p0 = p1;
	}

	if (t_max < t_min)
	{
		return 0;
	}

	*clip = Segment(SegmentBc(s, t_min), SegmentBc(s, t_max));
	return 1;
}

void DcelPrint(const struct dcel *dcel)
{	
	fprintf(stderr, "dcel[%p]\n{\n", (const void *) dcel);	

	fprintf(stderr, "\tv[%u]\n\t{\n", dcel->v_count);
	for (u32 i = 0; i < dcel->v_count; ++i)
	{
		fprintf(stderr, "\t\t{ %f, %f, %f }\n"
				, dcel->v[i].buf[0]
				, dcel->v[i].buf[1]
				, dcel->v[i].buf[2]);
	}
	fprintf(stderr, "\t}\n");

	fprintf(stderr, "\tf[%u]\n\t{\n", dcel->f_count);
	for (u32 i = 0; i < dcel->f_count; ++i)
	{

		fprintf(stderr, "\t\tf(%u)\n\t\t{\n", i);
		fprintf(stderr, "\t\t\te[%u]\n\t\t\t{\n", dcel->f[i].count);
		for (u32 ei = 0; ei < dcel->f[i].count; ++ei)
		{
			fprintf(stderr, "\t\t\t\te(%u) = { origin : %u, twin : %u, ccw : %u }\n" 
					, dcel->f[i].first + ei
					, dcel->e[dcel->f[i].first + ei].origin
					, dcel->e[dcel->f[i].first + ei].twin
					, dcel->e[dcel->f[i].first + ei].face_ccw);
		}
		fprintf(stderr, "\t\t\t}\n");
		fprintf(stderr, "\t\t}\n");
	}
	fprintf(stderr, "\t}\n");

	fprintf(stderr, "}\n");	
}

void DcelAssertTopology(const struct dcel *h)
{
	/* the ranges are disjoint (an edge's face_ccw names one face) and their counts sum to e_count: a partition */
	u32 edge_sum = 0;
	for (u32 fi = 0; fi < h->f_count; ++fi)
	{
		const struct dcelFace *f = h->f + fi;
		ds_Assert(3 <= f->count && f->first + f->count <= h->e_count);
		for (u32 i = 0; i < f->count; ++i)
		{
			const u32 ei = f->first + i;
			const u32 next = f->first + (i + 1) % f->count;
			const struct dcelEdge *e = h->e + ei;
			ds_Assert(e->face_ccw == fi);
			ds_Assert(e->origin < h->v_count);
			ds_Assert(e->twin < h->e_count && e->twin != ei);
			ds_Assert(h->e[e->twin].twin == ei);
			/* the twin runs the opposite way: it starts where e ends */
			ds_Assert(h->e[e->twin].origin == h->e[next].origin);
		}
		edge_sum += f->count;
	}
	ds_Assert(edge_sum == h->e_count);
}
