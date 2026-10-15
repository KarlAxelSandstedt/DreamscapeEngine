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
#include "ds_hash_map.h"
#include "ds_list.h"
#include "queue.h"


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

struct ddcelFace
{
	POOL_NODE;
	struct DLL	ce_list;
	v3		        normal;
	u32 		    first;	/* first half edge */
	u32 		    count;	/* edge count */
};

struct ddcelEdge
{
    POOL_NODE;
	u32 		origin;		/* vertex index origin */
	u32 		twin; 		/* twin half edge */
	u32 		next;		/* next ccw edge */
	u32 		prev;		/* prev ccw edge */
	u32 		face_ccw; 	/* face to the left of half edge */
	u32		horizon; /* used in horizon derivation step */
};

struct conflictEdge
{
    struct DLLNode   face_edge;
    struct DLLNode   vertex_edge;
	u32                 vertex;
	u32                 face;
    POOL_NODE;
};

POOL_DECLARE(ddcelFace);
POOL_DECLARE(ddcelEdge);
POOL_DECLARE(conflictEdge);

POOL_DEFINE(ddcelFace);
POOL_DEFINE(ddcelEdge);
POOL_DEFINE(conflictEdge);

struct conflictVertex
{
	struct DLL	ce_list;
	u32		index;
	/* Needed in last step of iteration */
	u32		last_iter;	/* last iteration it was added to a face's conflict list*/
	u32		last_face;	/* last iteration it was added to a face's conflict list*/
};

struct horizionVertex
{
	u32	edge1;			/* new edge; going INTO conflict vertex */
	u32	edge2;			/* new edge; going OUT OF  conflict vertex */
	u32 	edge_in;		/* edge going in to vertex */
	u32 	edge_out;		/* edge going out from vertex */
	u32 	edge_out_twin_face;	/* face of edge_out's twin */
	u32	next;			/* next horizon vertex */
	u32	colinear;		/* is twin face colinear */
};

/*
 * (Computational Geometry Algorithms and Applications, Section 2.2) 
 * ddcel - **dynamic** doubly-connected edge list. similar to a dcel but contains extra information in order to
 * construct itself iteratively. 
 */
struct ddcel
{
	struct ddcelFacePool 		face_pool;
	struct ddcelEdgePool		edge_pool;
	/* pools are not growable, so safe to use these */
	struct ddcelFace *	f;		
	struct ddcelEdge *	e;
	const v3 *		v;
	u32 			v_count;

	/* internal */
	struct arena		tmp1;
	struct conflictEdgePool 	ce_pool;
	struct conflictEdge *	ce;
	struct conflictVertex *cv;
	struct horizionVertex * hv;
};

static void DdcelFaceSet(struct ddcelFace *face, const u32 first, const u32 count)
{
	face->first = first;
	face->count = count;
	DLLFlush(face->ce_list);
}

static void DdcelEdgeSet(struct ddcelEdge *edge, const u32 origin, const u32 twin, const u32 prev, const u32 next, const u32 face_ccw)
{
	edge->origin = origin;
	edge->twin = twin;
	edge->next = next;
	edge->prev = prev;
	edge->face_ccw = face_ccw;
	edge->horizon = 0;
}

static void DdcelAssertTopology(const struct ddcel *ddcel)
{
	struct arena *tmp = ArenaPushScratch();

	u32 face_count = 0;
	u32 vertex_count = 0;

	u32 *vertex_check = ArenaPushZero(tmp, ddcel->v_count * sizeof(u32));
	u32 *edge_check = ArenaPushZero(tmp, ddcel->edge_pool.count_max * sizeof(u32));
	u32 *face_check = ArenaPushZero(tmp, 3*ddcel->v_count * sizeof(u32));

	for (u32 i = 0; i < ddcel->edge_pool.count_max; ++i)
	{
		if (ds_PoolSlotAllocated(ddcel->e + i) && !edge_check[i])
		{
			face_count += 1;
			u32 next;
		        u32 prev; 
			u32 current = i;
			u32 edge_count = 0;
			do
			{
				edge_count += 1;
				const struct ddcelEdge *c = ddcel->e + current;
				const struct ddcelEdge *p = ddcel->e + c->prev;
				const struct ddcelEdge *n = ddcel->e + c->next;
				const struct ddcelEdge *t = ddcel->e + c->twin;

				ds_Assert(c->horizon == 0);
				ds_Assert(c->origin < ddcel->v_count);
				ds_Assert(p->next == current);
				ds_Assert(n->prev == current);
				ds_Assert(t->twin == current);
				ds_Assert(t->origin == n->origin);

				edge_check[current] = 1;
				vertex_count += (1 - vertex_check[c->origin]);
				vertex_check[c->origin] = 1;

				current = c->next;
			} while (current != i);

			ds_Assert(edge_count >= 3);
		}
	}

	v3 center = V3Zero();
	for (u32 i = 0; i < ddcel->v_count; ++i)
	{
		if (vertex_check[i])
		{
			center = V3Add(center, ddcel->v[i]);
		}
	}
	center = V3Scale(center, 1.0f/vertex_count);

	for (u32 i = 0; i < ddcel->face_pool.count_max; ++i)
	{
		if (ds_PoolSlotAllocated(ddcel->f + i))
		{
			v3 diff;
			diff = V3Sub(center, ddcel->v[ddcel->e[ddcel->f[i].first].origin]);
			ds_Assert(V3Dot(diff, ddcel->f[i].normal) < 0.0f);
		}
	}

    ArenaPopScratch();
	ds_Assert(face_count >= 4);
}

/*
 * Move the initial tetrahedron to cv[0 .. 4): the extreme points along the axis of the largest extent, the point
 * farthest from their line, and the point farthest from their plane (ties: the lowest index). As large as the input
 * allows, so its face normals are well conditioned. Return 0 if the points lie within tol of a point, a line or a
 * plane: no tetrahedron.
 */
static u32 InternalConvexHullTetrahedronIndices(struct ddcel *ddcel, const f32 tol)
{
	const v3 *v = ddcel->v;
	const u32 n = ddcel->v_count;

	u32 i0 = 0;
	u32 i1 = 0;
	f32 extent = 0.0f;
	for (u32 axis = 0; axis < 3; ++axis)
	{
		u32 lo = 0;
		u32 hi = 0;
		for (u32 i = 1; i < n; ++i)
		{
			lo = (v[i].buf[axis] < v[lo].buf[axis]) ? i : lo;
			hi = (v[hi].buf[axis] < v[i].buf[axis]) ? i : hi;
		}
		const f32 axis_extent = v[hi].buf[axis] - v[lo].buf[axis];
		if (axis == 0 || extent < axis_extent)
		{
			extent = axis_extent;
			i0 = lo;
			i1 = hi;
		}
	}
	const v3 d = V3Sub(v[i1], v[i0]);
	const f32 dd = V3Dot(d, d);
	if (!(tol*tol < dd))
	{
		return 0;
	}

	/* distance from the line i0 i1: |(v - v0) x d| / |d| */
	u32 i2 = 0;
	f32 best = 0.0f;
	for (u32 i = 0; i < n; ++i)
	{
		const v3 c = V3Cross(V3Sub(v[i], v[i0]), d);
		const f32 c_sq = V3Dot(c, c);
		if (best < c_sq)
		{
			best = c_sq;
			i2 = i;
		}
	}
	if (!(tol*tol*dd < best))
	{
		return 0;
	}

	/* distance from the plane i0 i1 i2: |(v - v0).m| / |m| */
	const v3 m = V3Cross(d, V3Sub(v[i2], v[i0]));
	u32 i3 = 0;
	best = 0.0f;
	for (u32 i = 0; i < n; ++i)
	{
		const f32 height = F32Abs(V3Dot(V3Sub(v[i], v[i0]), m));
		if (best < height)
		{
			best = height;
			i3 = i;
		}
	}
	if (!(tol*V3Length(m) < best))
	{
		return 0;
	}

	/* cv holds a permutation of the point indices: swap the four to the front (distinct, so each is at k >= j) */
	const u32 index[4] = { i0, i1, i2, i3 };
	for (u32 j = 0; j < 4; ++j)
	{
		u32 k = j;
		while (ddcel->cv[k].index != index[j])
		{
			k += 1;
		}
		const struct conflictVertex tmp = ddcel->cv[j];
		ddcel->cv[j] = ddcel->cv[k];
		ddcel->cv[k] = tmp;
	}

	return 1;
}

/* Return 0 if a face of the initial tetrahedron is degenerate (no normal): no hull */
static u32 InternalConvexHullTetrahedronDdcel(struct ddcel *ddcel, const f32 tol)
{
	const v3 *v = ddcel->v;
	const u32 v_count = ddcel->v_count;
	v3 a, b, c, cr;
	a = V3Sub(v[ddcel->cv[1].index], v[ddcel->cv[0].index]);
	b = V3Sub(v[ddcel->cv[2].index], v[ddcel->cv[0].index]);
	c = V3Sub(v[ddcel->cv[3].index], v[ddcel->cv[0].index]);
	cr = V3Cross(a, b);

	/* CCW == inside gives negative dot product for any polygon on a convex polyhedron */
	if (V3Dot(cr, c) > 0.0f)
	{
		/* Make 0->1->2->0 CCW */
		const u32 tmp = ddcel->cv[1].index;
		ddcel->cv[1].index = ddcel->cv[2].index;
		ddcel->cv[2].index = tmp;
	}

	struct ddcelFace *f0 =  ddcelFacePoolAdd(&ddcel->face_pool).address;
	struct ddcelFace *f1 =  ddcelFacePoolAdd(&ddcel->face_pool).address;
	struct ddcelFace *f2 =  ddcelFacePoolAdd(&ddcel->face_pool).address;
	struct ddcelFace *f3 =  ddcelFacePoolAdd(&ddcel->face_pool).address;

	struct ddcelEdge *e0 =  ddcelEdgePoolAdd(&ddcel->edge_pool).address;
	struct ddcelEdge *e1 =  ddcelEdgePoolAdd(&ddcel->edge_pool).address;
	struct ddcelEdge *e2 =  ddcelEdgePoolAdd(&ddcel->edge_pool).address;
	struct ddcelEdge *e3 =  ddcelEdgePoolAdd(&ddcel->edge_pool).address;
	struct ddcelEdge *e4 =  ddcelEdgePoolAdd(&ddcel->edge_pool).address;
	struct ddcelEdge *e5 =  ddcelEdgePoolAdd(&ddcel->edge_pool).address;
	struct ddcelEdge *e6 =  ddcelEdgePoolAdd(&ddcel->edge_pool).address;
	struct ddcelEdge *e7 =  ddcelEdgePoolAdd(&ddcel->edge_pool).address;
	struct ddcelEdge *e8 =  ddcelEdgePoolAdd(&ddcel->edge_pool).address;
	struct ddcelEdge *e9 =  ddcelEdgePoolAdd(&ddcel->edge_pool).address;
	struct ddcelEdge *e10 = ddcelEdgePoolAdd(&ddcel->edge_pool).address;
	struct ddcelEdge *e11 = ddcelEdgePoolAdd(&ddcel->edge_pool).address;

	DdcelFaceSet(f0, 0, 3);
	DdcelFaceSet(f1, 3, 3);
	DdcelFaceSet(f2, 6, 3);
	DdcelFaceSet(f3, 9, 3);

	DdcelEdgeSet(e0,  ddcel->cv[0].index,  4,  2,  1, 0);
	DdcelEdgeSet(e1,  ddcel->cv[1].index, 10,  0,  2, 0);
	DdcelEdgeSet(e2,  ddcel->cv[2].index,  7,  1,  0, 0);

	DdcelEdgeSet(e3,  ddcel->cv[3].index, 11,  5,  4, 1);
	DdcelEdgeSet(e4,  ddcel->cv[1].index,  0,  3,  5, 1);
	DdcelEdgeSet(e5,  ddcel->cv[0].index,  6,  4,  3, 1);

	DdcelEdgeSet(e6,  ddcel->cv[3].index,  5,  8,  7, 2);
	DdcelEdgeSet(e7,  ddcel->cv[0].index,  2,  6,  8, 2);
	DdcelEdgeSet(e8,  ddcel->cv[2].index,  9,  7,  6, 2);

	DdcelEdgeSet(e9,  ddcel->cv[3].index,  8, 11, 10, 3);
	DdcelEdgeSet(e10, ddcel->cv[2].index,  1,  9, 11, 3);
	DdcelEdgeSet(e11, ddcel->cv[1].index,  3, 10,  9, 3);

    const u32 valid = TriCcwNormalTryCompute(&ddcel->f[0].normal, ddcel->v[e0->origin], ddcel->v[e1->origin], ddcel->v[e2->origin])
                    & TriCcwNormalTryCompute(&ddcel->f[1].normal, ddcel->v[e3->origin], ddcel->v[e4->origin], ddcel->v[e5->origin])
                    & TriCcwNormalTryCompute(&ddcel->f[2].normal, ddcel->v[e6->origin], ddcel->v[e7->origin], ddcel->v[e8->origin])
                    & TriCcwNormalTryCompute(&ddcel->f[3].normal, ddcel->v[e9->origin], ddcel->v[e10->origin], ddcel->v[e11->origin]);
	if (valid)
	{
		DdcelAssertTopology(ddcel);
	}

	return valid;
}

static void InternalConvexHullTetrahedronConflicts(struct ddcel *ddcel, const f32 tol)
{
	const v3 *v = ddcel->v;
	const u32 v_count = ddcel->v_count;
	v3 b;
	for (u32 cv_i = 4; cv_i < v_count; ++cv_i)
	{
		struct conflictVertex *cv = ddcel->cv + cv_i;
		for (u32 f_i = 0; f_i < 4; ++f_i)
		{
			const u32 v0_i = ddcel->e[ddcel->f[f_i].first].origin;
			b = V3Sub(v[cv->index], v[v0_i]);
			/* If point is "in front" of face, we have a conflict */
			if (V3Dot(ddcel->f[f_i].normal, b) > tol)
			{
				struct slot slot = conflictEdgePoolAdd(&ddcel->ce_pool);
				DLLAppend(cv->ce_list, ddcel->ce_pool.buf, slot.index, vertex_edge);
				DLLAppend(ddcel->f[f_i].ce_list, ddcel->ce_pool.buf, slot.index, face_edge);

				struct conflictEdge *edge = slot.address;
				edge->vertex = cv_i;
				edge->face = f_i;
			}
		}
	}
}

/* Return 0 if a new face is degenerate (no normal): the ddcel is then left broken and the hull fails */
u32 ConvexHullIteration(struct ddcel *ddcel, const u32 cvi, const f32 tol)
{
	if (ddcel->cv[cvi].ce_list.count == 0) { return 1; }

	struct conflictVertex *cv = ddcel->cv + cvi;
	struct conflictEdge *ce = NULL;
	for (i32 i = cv->ce_list.first; i != DLL_SENTINEL; i = ce->vertex_edge.next)
	{
		ce = ddcel->ce + i;
		const u32 fi = ce->face;
		struct ddcelEdge *e = ddcel->e + ddcel->f[fi].first;
		for (u32 j = 0; j < ddcel->f[fi].count; ++j)
		{
			struct ddcelEdge *twin = ddcel->e + e->twin;
			ds_Assert(e->horizon == 0);
			ds_Assert(twin->horizon == 0);
			e = ddcel->e + e->next;
		}
	}
	
	/* (5) Get horizon edges:
	 * At this point, all edges has horizon set to 0. Whenever we visit an edge, 
	 * we flip the edge and its twin's horizon value. After the loop the horizon
	 * will consist of the edges with value 1.  */
	for (i32 i = cv->ce_list.first; i != DLL_SENTINEL; i = ce->vertex_edge.next)
	{
		ce = ddcel->ce + i;
		const u32 fi = ce->face;
		struct ddcelEdge *e = ddcel->e + ddcel->f[fi].first;
		for (u32 j = 0; j < ddcel->f[fi].count; ++j)
		{
			struct ddcelEdge *twin = ddcel->e + e->twin;
			//fprintf(stderr, "horizon edge par (%u,%u)\n", ds_PoolIndex(&ddcel->edge_pool, e), e->twin);
			e->horizon += 1;
			twin->horizon += 1;
			ds_Assert(e->horizon <= 2 && twin->horizon <= 2);
			ds_Assert(twin->twin == ddcelEdgePoolIndex(&ddcel->edge_pool, e));
			e = ddcel->e + e->next;
		}
	}

	/* (6) loop through and remove all faces and non-horizon edges. Sort horizon edges in a 
	 * ad-hoc dll structure horizon vertex and cache additional data.
	 */
	u32 horizon = 0;
	u32 horizon_count = 0;
	for (i32 i = cv->ce_list.first; i != DLL_SENTINEL; i = ce->vertex_edge.next)
	{
		ce = ddcel->ce + i;
		const u32 fi = ce->face;
		struct ddcelFace *f = ddcel->f + fi;

		u32 ej = f->first;
		for (u32 j = 0; j < f->count; ++j)
		{
			struct ddcelEdge *e = ddcel->e + ej;
			const u32 next = e->next;
			if (e->horizon == 2)
			{
				ddcelEdgePoolRemove(&ddcel->edge_pool, ej);	
			}
			else
			{
				//fprintf(stderr, "horizon edge par (%u,%u)\n", ej, e->twin);
				struct ddcelEdge *e_twin = ddcel->e + e->twin;
				struct ddcelFace *f_twin = ddcel->f + e_twin->face_ccw;

				ds_Assert(e->horizon == 1);
				ds_Assert(e_twin->horizon == 1);
				e->horizon = 0;
				e_twin->horizon = 0;

				horizon_count += 1;
				horizon = e->origin;

				ddcel->hv[e->origin].edge_out_twin_face = e_twin->face_ccw;
				ddcel->hv[e->origin].edge_out = ej;
				ddcel->hv[e->origin].next = e_twin->origin;
				ddcel->hv[e_twin->origin].edge_in = ej;

				v3 diff;
				diff = V3Sub(ddcel->v[cv->index], ddcel->v[e->origin]);
				ddcel->hv[e->origin].colinear = (F32Abs(V3Dot(f_twin->normal, diff)) < tol)
					? 1
					: 0;
			}
			ej = next;
		}
	}

	/* (7) We don't want to start in the middle of a colinear_face, so we traverse the horizon until we find a 
	 * new triangle.
	 */
	const u32 face = ddcel->hv[horizon].edge_out_twin_face;
	u32 prev_horizon = U32_MAX;
	for (u32 i = 0; i < horizon_count; ++i)
	{
		prev_horizon = horizon;
		horizon = ddcel->hv[horizon].next;
		if (ddcel->hv[horizon].edge_out_twin_face != ddcel->hv[prev_horizon].edge_out_twin_face)
		{
			break;
		}
	}

	/* (8) Traverse the horizon creaing new faces and removing any colinear horizon
	 * twin edges. 
	 */
	for (u32 i = 0; i < horizon_count; ++i)
	{	
		const u32 twin_face = ddcel->hv[horizon].edge_out_twin_face;
		const u32 ccw_face = ddcel->e[ddcel->hv[horizon].edge_out].face_ccw;
		struct ddcelFace *f_ccw = ddcel->f + ccw_face;
		struct ddcelFace *f_twin = ddcel->f + twin_face;

		struct slot se1 = ddcelEdgePoolAdd(&ddcel->edge_pool);
		struct slot se2 = ddcelEdgePoolAdd(&ddcel->edge_pool);

		struct ddcelEdge *e1 = se1.address;
		struct ddcelEdge *e2 = se2.address;

		ddcel->hv[horizon].edge1 = se1.index;
		ddcel->hv[horizon].edge2 = se2.index;

		u32 prev_horizon_edge1 = U32_MAX;
		if (i != 0)
		{ 
			prev_horizon_edge1 = ddcel->hv[prev_horizon].edge1;
			ddcel->e[ddcel->hv[prev_horizon].edge1].twin = se2.index;
		}

		if (ddcel->hv[horizon].colinear)
		{
			u32 edge = ddcel->hv[horizon].edge_out;
			const u32 twin_start_next = ddcel->e[
					            ddcel->e[edge].twin]
							          .next;
			//fprintf(stderr, "extending face %u\n", twin_face);
			u32 twin_end_prev;
			f_twin->first = se1.index;
			f_twin->count += 2;
			i -= 1;
			do
			{
				f_twin->count -= 1;
				i += 1;
				twin_end_prev = ddcel->e[
					        ddcel->e[edge].twin]
					                      .prev;
				prev_horizon = horizon;
				horizon = ddcel->hv[horizon].next;

				ddcel->hv[prev_horizon].edge1 = se1.index;
				ddcel->hv[prev_horizon].edge2 = se2.index;

				ddcelEdgePoolRemove(&ddcel->edge_pool, ddcel->e[edge].twin);
				ddcelEdgePoolRemove(&ddcel->edge_pool, edge);
				edge = ddcel->hv[horizon].edge_out;
			} while (ddcel->hv[horizon].edge_out_twin_face == twin_face);

			DdcelEdgeSet(e1, horizon, U32_MAX, twin_end_prev, se2.index, twin_face);
			DdcelEdgeSet(e2, cv->index, prev_horizon_edge1, se1.index, twin_start_next, twin_face);
			ddcel->e[twin_end_prev].next = se1.index;
			ddcel->e[twin_start_next].prev = se2.index;

			/* Note: Since we reuse the twin face, we also reuse its list of conflicts, so we are done. */
		}
		else
		{
			struct slot sf = ddcelFacePoolAdd(&ddcel->face_pool);
			//fprintf(stderr, "added face %u\n", sf.index);

			const u32 e0i = ddcel->hv[horizon].edge_out;

			struct ddcelFace *f = sf.address;
			struct ddcelEdge *e0 = ddcel->e + e0i;

			DdcelEdgeSet(e2, cv->index, ddcel->hv[prev_horizon].edge1, se1.index, e0i, sf.index);
			DdcelEdgeSet(e1, ddcel->e[e0->twin].origin, U32_MAX, e0i, se2.index, sf.index);
			DdcelEdgeSet(e0, e0->origin, e0->twin, se2.index, se1.index, sf.index);

			DdcelFaceSet(f, e0i, 3);
            if (!TriCcwNormalTryCompute(&f->normal, ddcel->v[e0->origin], ddcel->v[e1->origin], ddcel->v[e2->origin]))
            {
                /* the new point is collinear with this horizon edge, which the coplanar merge above missed */
                return 0;
            }
		
			/*TODO: We may add same point twich here, need to add a "has_been_mapped" thingy to not add again*/
			ce = NULL;
			for (i32 j = f_ccw->ce_list.first; j != DLL_SENTINEL; j = ce->face_edge.next)
			{
				ce = ddcel->ce + j;
				if (ce->vertex != cvi && (ddcel->cv[ce->vertex].last_face != sf.index || ddcel->cv[ce->vertex].last_iter != cvi))
				{
					ddcel->cv[ce->vertex].last_iter = cvi;
					ddcel->cv[ce->vertex].last_face = sf.index;
					v3 diff;
					diff = V3Sub(ddcel->v[ddcel->cv[ce->vertex].index], ddcel->v[e0->origin]);
					if (V3Dot(f->normal, diff) > tol)
					{
						struct slot slot = conflictEdgePoolAdd(&ddcel->ce_pool);
						DLLAppend(f->ce_list, ddcel->ce_pool.buf, slot.index, face_edge);
						DLLAppend(ddcel->cv[ce->vertex].ce_list, ddcel->ce_pool.buf, slot.index, vertex_edge);

						struct conflictEdge *new = slot.address;
						new->vertex = ce->vertex;
						new->face = sf.index;
					}
				}
			}

			ce = NULL;
			for (i32 j = f_twin->ce_list.first; j != DLL_SENTINEL; j = ce->face_edge.next)
			{
				ce = ddcel->ce + j;
				v3 diff;
				ds_Assert(ce->vertex != cvi)
				if (ddcel->cv[ce->vertex].last_face != sf.index || ddcel->cv[ce->vertex].last_iter != cvi)
				{
					ddcel->cv[ce->vertex].last_iter = cvi;
					ddcel->cv[ce->vertex].last_face = sf.index;
					diff = V3Sub(ddcel->v[ddcel->cv[ce->vertex].index], ddcel->v[e0->origin]);
					if (V3Dot(f->normal, diff) > tol)
					{
						struct slot slot = conflictEdgePoolAdd(&ddcel->ce_pool);
						DLLAppend(f->ce_list, ddcel->ce_pool.buf, slot.index, face_edge);
						DLLAppend(ddcel->cv[ce->vertex].ce_list, ddcel->ce_pool.buf, slot.index, vertex_edge);

						struct conflictEdge *new = slot.address;
						new->vertex = ce->vertex;
						new->face = sf.index;
					}
				}
			}

			prev_horizon = horizon;
			horizon = ddcel->hv[horizon].next;
		}
	}
	ddcel->e[ddcel->hv[prev_horizon].edge1].twin = ddcel->hv[horizon].edge2;
	ddcel->e[ddcel->hv[horizon].edge2].twin = ddcel->hv[prev_horizon].edge1;

	/* (9) Update all conflict_edge lists of vertices conflicting with face,
	 * and remove all conflicting edges to face.  */
	while (cv->ce_list.first != DLL_SENTINEL)
	{
		ce = ddcel->ce + cv->ce_list.first;
		const u32 fi = ce->face;
		struct ddcelFace *f = ddcel->f + fi;

        i32 next;
        for (i32 cei = f->ce_list.first; cei != DLL_SENTINEL; cei = next)
        {
			ce = ddcel->ce + cei;
            next = ce->face_edge.next;
			struct conflictVertex *cvj = ddcel->cv + ce->vertex;
			DLLRemove(cvj->ce_list, ddcel->ce_pool.buf, cei, vertex_edge);
			conflictEdgePoolRemove(&ddcel->ce_pool, cei);
        }
		ddcelFacePoolRemove(&ddcel->face_pool, fi);
		//fprintf(stderr, "removed face %u\n", fi);
	}

	return 1;
}

/*
 * The finished hull as a dcel pushed on mem: faces in pool order, each face's edges contiguous in ring order, and only
 * the vertices the edges use, in input order (interior points are dropped). The index maps are temporaries on tmp.
 * tmp or mem full: the empty dcel, nothing pushed on mem.
 */
static struct dcel DcelDdcel(struct arena *mem, struct arena *tmp, const struct ddcel *ddcel)
{
	/* emap: ddcel edge -> dcel edge; vmap: input point -> dcel vertex, U32_MAX if no edge starts there */
	u32 *emap = ArenaPush(tmp, ddcel->edge_pool.count_max*sizeof(u32));
	u32 *vmap = ArenaPush(tmp, ddcel->v_count*sizeof(u32));
	if (!emap || !vmap)
	{
		return DcelEmpty();
	}
	for (u32 i = 0; i < ddcel->v_count; ++i)
	{
		vmap[i] = U32_MAX;
	}

	u32 e_count = 0;
	for (u32 fj = 0; fj < ddcel->face_pool.count_max; ++fj)
	{
		if (ds_PoolSlotAllocated(ddcel->f + fj))
		{
			u32 next = ddcel->f[fj].first;
			for (u32 ei = 0; ei < ddcel->f[fj].count; ++ei)
			{
				emap[next] = e_count++;
				vmap[ddcel->e[next].origin] = 0;
				next = ddcel->e[next].next;
			}
		}
	}

	u32 v_count = 0;
	for (u32 i = 0; i < ddcel->v_count; ++i)
	{
		if (vmap[i] != U32_MAX)
		{
			vmap[i] = v_count++;
		}
	}

	ArenaPushRecord(mem);
	v3 *v = ArenaPush(mem, v_count*sizeof(v3));
	struct dcelEdge *edge = ArenaPush(mem, e_count*sizeof(struct dcelEdge));
	struct dcelFace *face = ArenaPush(mem, ddcel->face_pool.count*sizeof(struct dcelFace));
	struct plane *plane = ArenaPush(mem, ddcel->face_pool.count*sizeof(struct plane));
	if (!v || !edge || !face || !plane)
	{
		ArenaPopRecord(mem);
		return DcelEmpty();
	}
	ArenaRemoveRecord(mem);

	for (u32 i = 0; i < ddcel->v_count; ++i)
	{
		if (vmap[i] != U32_MAX)
		{
			v[vmap[i]] = ddcel->v[i];
		}
	}

	u32 off = 0;
	u32 fi = 0;
	for (u32 fj = 0; fj < ddcel->face_pool.count_max; ++fj)
	{
		if (ds_PoolSlotAllocated(ddcel->f + fj))
		{
			face[fi].count = ddcel->f[fj].count;
			face[fi].first = off;
			u32 next = ddcel->f[fj].first;
			for (u32 ei = 0; ei < ddcel->f[fj].count; ++ei)
			{
				edge[off + ei].origin = vmap[ddcel->e[next].origin];
				edge[off + ei].face_ccw = fi;
				edge[off + ei].twin = emap[ddcel->e[next].twin];
				next = ddcel->e[next].next;
			}
			off += ddcel->f[fj].count;
			fi += 1;
		}
	}
	ds_Assert(fi == ddcel->face_pool.count && off == e_count);

	const struct dcel h =
	{
		.v = v,
		.e = edge,
		.f = face,
		.plane = plane,
		.v_count = v_count,
		.e_count = e_count,
		.f_count = fi,
	};
	//DcelPrint(&h);
	//DcelAssertTopology(&h);
	return h;
}

struct dcel DcelConvexHull(struct arena *mem, const v3 *v, const u32 v_count, const f32 tol)
{
	struct dcel dcel = DcelEmpty();
	if (v_count < 4) { goto end; }	

	struct arena *tmp1 = ArenaPushScratch();
	struct arena *tmp2 = ArenaPushScratch();

	const u32 edge_count_upper_bound = 6*v_count - 12;
	const u32 face_count_upper_bound = 2*v_count - 4;
	struct ddcel ddcel =
	{
		.face_pool = ddcelFacePoolAlloc(tmp1, 2*face_count_upper_bound, NOT_GROWABLE),	/* add additional space for easier memory management */
		.edge_pool = ddcelEdgePoolAlloc(tmp1, 2+edge_count_upper_bound, NOT_GROWABLE),	/* add additional space for easier memory management */
		.v = v,
		.v_count = v_count,
		.ce_pool = conflictEdgePoolAlloc(tmp2, (tmp2->mem_size / sizeof(struct conflictEdge))-1, NOT_GROWABLE),
		.cv = ArenaPush(tmp1, v_count * sizeof(struct conflictVertex)),
		.hv = ArenaPush(tmp1, v_count * sizeof(struct horizionVertex)),
	};

	ddcel.tmp1 = *ArenaPushScratch();
	ddcel.e = (struct ddcelEdge *) ddcel.edge_pool.buf;
	ddcel.f = (struct ddcelFace *) ddcel.face_pool.buf;
	ddcel.ce = (struct conflictEdge *) ddcel.ce_pool.buf;

	/* (1) permutation - Random permutation of remaining points */
	for (u32 i = 0; i < v_count; ++i)
	{
		DLLFlush(ddcel.cv[i].ce_list);
		ddcel.cv[i].index = i;
		ddcel.cv[i].last_iter = U32_MAX;
		ddcel.cv[i].last_face = U32_MAX;
	}
	for (u32 i = 0; i < v_count; ++i)
	{
		const u32 rng = (u32) RngU64Range(i, v_count-1);
		const u32 tmp = ddcel.cv[i].index;
		ddcel.cv[i].index = ddcel.cv[rng].index;
		ddcel.cv[rng].index = tmp;
	}

	/* (2) Get inital points for tetrahedron */
	if (InternalConvexHullTetrahedronIndices(&ddcel, tol) == 0) { goto end; }

	/* (3) initiate DCEL from points */
	if (!InternalConvexHullTetrahedronDdcel(&ddcel, tol)) { goto end; }

	/* (4) setup conflict graph */
	InternalConvexHullTetrahedronConflicts(&ddcel, tol);

	/* iteratetively solve and add conflicts until no vertices left */
	for (u32 i = 4; i < v_count; ++i)
	{
		if (!ConvexHullIteration(&ddcel, i, tol))
		{
			Log(T_PHYSICS, S_WARNING, "DcelConvexHull: degenerate face while adding point %u of %u, no hull", i, v_count);
			goto end;
		}
		DdcelAssertTopology(&ddcel);
	}

	ArenaPushRecord(mem);
	dcel = DcelDdcel(mem, &ddcel.tmp1, &ddcel);	
	if (dcel.f_count && !DcelPlanesTryCompute(&dcel))
	{
		Log(T_PHYSICS, S_WARNING, "DcelConvexHull: hull of %u points has a degenerate face, using the box stub", v_count);
		ds_AssertString(0, "DcelConvexHull: degenerate face");
		ArenaPopRecord(mem);
		dcel = DcelBoxStub();
	}
	else
	{
		DcelBboxCompute(&dcel);
		ArenaRemoveRecord(mem);
	}
end:
	ArenaPopScratch();
	ArenaPopScratch();
	ArenaPopScratch();

	return dcel;
}
