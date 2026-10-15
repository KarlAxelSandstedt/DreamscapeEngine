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

/*
c_hull.c: DcelConvexHull, the convex hull of a point cloud as a dcel (collision.h).

Algorithm
---------
The randomized incremental hull with a conflict graph (de Berg et al., Computational Geometry: Algorithms and
Applications, chapter 11): the points are added in a random order (a local generator with a fixed seed, so the hull
depends only on its input), starting from a tetrahedron of extreme points. Every face keeps the unprocessed points in
front of it, every point the faces it is in front of. Adding point p removes the faces p sees, joins p to the horizon
with a cone of triangles, and takes the new faces' conflicts from the points of the faces around them.

The book assumes no four points are coplanar. Real input (boxes, cylinders, scanned or modelled meshes) is full of
coplanar and nearly coplanar points, so after every cone the new faces are merged as in Gregorius' quickhull
("Implementing Quickhull", GDC 2014; Box3D's hull.c is the reference implementation followed here): a face across an
edge that is concave, or not convex by tol, is absorbed and the merged face gets a Newell plane, until every edge of
the cone is convex by more than tol. Merging only in the plane of a single face (the builder's earlier approach) is not
enough: tolerance decisions about nearly coplanar faces disagree with each other (springclean #65).

Tolerance
---------
tol is not the caller's: it is DcelConvexHullTolerance, a quarter of GJK's touch band at the hull's scale,

	tol = GJK_TOUCH_TOLERANCE / 4 * eps * max |v|.

It decides when points are coplanar (merging, visibility: tol; a point is outside only beyond 2 tol), so it is also how
flat the merged faces are: their vertices lie within ~1.5 tol of the face plane (measured over the stress clouds of
DcelConvexHullTest), and every input point within ~2.5 tol of the hull. The collision code treats faces as exactly flat
(planes for the SAT and clipping, vertices for GJK), so those must agree within its own tolerances:

	hull flatness (~37 eps L)  <  contact accuracy (tests: 64 eps L)  <  touch band (GJK_TOUCH_TOLERANCE = 100 eps L)

with L >= max |v| (GJK measures L over both shapes' support points). Changing GJK_TOUCH_TOLERANCE moves the hull's
tolerance with it; a larger hull tolerance than the touch band would let planes and vertices disagree about touching.

The bounds above need input that is coplanar within tol where it is meant to be. Points scattered about a plane by more
(DcelConvexHullTest's "box noisy", 4 tol) leave nearly coplanar faces that differ by more than tol, so not all are
merged. Measured over 4800 such clouds: most hulls are flat and contain the input to ~8 tol, but a face outline can
get a reflex corner where its neighbour fills a dent, and such a face has little area for its extent, so its Newell
plane tilts (worst: a vertex 21 tol off its face, a point 103 tol in front of one); 0.2% fail ("face left with fewer
than 3 edges"). Blocking merges that would leave a reflex corner was tried and failed more hulls (horizons that are not
one loop). This builder is temporary (a quickhull implementation is planned), so noisy input is left at that.

Memory and failure
------------------
Working memory is 3 scratch arenas (the first ~370 B per point, sized from the point count; the second all for the
conflict pool; the third for DcelDdcel's maps). Every failure (too many points, a full conflict pool, a horizon that is
not one loop, a merge that does not give a face) logs a warning and returns the empty dcel; a finished hull with a
zero-area face asserts in debug and gives the box stub in release.
*/

#include "ds_base.h"
#include "ds_math.h"
#include "ds_list.h"
#include "collision.h"

struct ddcelFace
{
	POOL_NODE;
	struct DLL	ce_list;	/* conflicts: the unprocessed points more than 2 tol in front */
	v3		        normal;		/* unit, 0 for a face without area; the plane: dot(normal, x) = offset */
	f32		        offset;
	v3		        centroid;	/* average of the face's vertices (the plane passes through it) */
	f32		        area;
	u32 		    first;	/* first half edge */
	u32 		    count;	/* edge count */
	u32		        visible;	/* the iteration whose point sees the face, U32_MAX if none */
	u32		        cone;		/* the iteration that created the face, U32_MAX for the tetrahedron */
	u32		        flipped;	/* a new face with the interior point in front of it (merged first) */
};

struct ddcelEdge
{
    POOL_NODE;
	u32 		origin;		/* vertex index origin */
	u32 		twin; 		/* twin half edge */
	u32 		next;		/* next ccw edge */
	u32 		prev;		/* prev ccw edge */
	u32 		face_ccw; 	/* face to the left of half edge */
	u32		horizon;	/* during an iteration: 1 a horizon edge, 2 placed in the horizon's order */
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
	u32		last_iter;	/* the last iteration that made it a candidate (no duplicates) */
};

struct horizionVertex
{
	u32 	edge_out;		/* horizon edge going out from vertex */
	u32	next;			/* next horizon vertex */
	u32	iter;			/* the iteration that set it */
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
	v3			interior;	/* the initial tetrahedron's centroid, inside every hull */
	u32 *			visible;	/* the faces the current point sees */
	u32 *			cone;		/* the horizon's edges in order, then the new faces */
	u32 *			cand;		/* the candidates: points that may be in front of new or changed faces */
	u32			cand_count;
	u32			broken;		/* set (logged) when an iteration fails */
};

static void DdcelFaceSet(struct ddcelFace *face, const u32 first, const u32 count, const u32 cone)
{
	face->first = first;
	face->count = count;
	face->visible = U32_MAX;
	face->cone = cone;
	face->flipped = 0;
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
 * Face merging as in Gregorius' quickhull ("Implementing Quickhull", GDC 2014), following the reference implementation
 * in Box3D (hull.c: b3NewellPlane, b3IsEdgeConvex/Concave, b3HullBuilder_ConnectFaces/ConnectEdges/MergeFaces), on
 * this builder's index-based dcel and de Berg's conflict graph. The caller's tol plays Box3D's minRadius (convexity
 * and visibility), 2 tol its minOutside (conflicts).
 */

/* Walk fi's ring from first: set and return its edge count */
static u32 DdcelFaceRecount(struct ddcel *d, const u32 fi)
{
	u32 count = 0;
	u32 ei = d->f[fi].first;
	do
	{
		count += 1;
		ei = d->e[ei].next;
	} while (ei != d->f[fi].first && count <= d->edge_pool.count_max);
	d->f[fi].count = count;
	return count;
}

/* fi's plane through the average of its vertices, normal from all of them (Newell); no area: normal 0 */
static void DdcelFaceNewell(struct ddcel *d, const u32 fi)
{
	struct ddcelFace *f = d->f + fi;
	const v3 o = d->v[d->e[f->first].origin];
	v3 n = V3Zero();
	v3 sum = V3Zero();
	u32 ei = f->first;
	for (u32 i = 0; i < f->count; ++i)
	{
		const v3 v1 = V3Sub(d->v[d->e[ei].origin], o);
		const v3 v2 = V3Sub(d->v[d->e[d->e[ei].next].origin], o);
		n.x += (v1.y - v2.y) * (v1.z + v2.z);
		n.y += (v1.z - v2.z) * (v1.x + v2.x);
		n.z += (v1.x - v2.x) * (v1.y + v2.y);
		sum = V3Add(sum, v1);
		ei = d->e[ei].next;
	}
	const f32 length = V3Length(n);
	f->centroid = V3Add(o, V3Scale(sum, 1.0f / (f32) f->count));
	f->area = 0.5f * length;
	f->normal = (0.0f < length && length < F32_INFINITY) ? V3Scale(n, 1.0f / length) : V3Zero();
	f->offset = V3Dot(f->normal, f->centroid);
}

static f32 DdcelFaceDistance(const struct ddcel *d, const u32 fi, const v3 x)
{
	return V3Dot(d->f[fi].normal, x) - d->f[fi].offset;
}

static void DdcelConflictRemove(struct ddcel *d, const u32 cei)
{
	const u32 q = d->ce[cei].vertex;
	const u32 fi = d->ce[cei].face;
	DLLRemove(d->cv[q].ce_list, d->ce_pool.buf, cei, vertex_edge);
	DLLRemove(d->f[fi].ce_list, d->ce_pool.buf, cei, face_edge);
	conflictEdgePoolRemove(&d->ce_pool, cei);
}

static void DdcelConflictAdd(struct ddcel *d, const u32 q, const u32 fi)
{
	const struct slot slot = conflictEdgePoolAdd(&d->ce_pool);
	if (!slot.address)
	{
		if (!d->broken)
		{
			Log(T_PHYSICS, S_WARNING, "DcelConvexHull: conflict graph full (%u point-face pairs), no hull",
				d->ce_pool.length);
		}
		d->broken = 1;
		return;
	}
	DLLAppend(d->cv[q].ce_list, d->ce_pool.buf, slot.index, vertex_edge);
	DLLAppend(d->f[fi].ce_list, d->ce_pool.buf, slot.index, face_edge);
	struct conflictEdge *ce = slot.address;
	ce->vertex = q;
	ce->face = fi;
}

/* Make point q (not the iteration's point cvi) a candidate, once per iteration */
static void DdcelCandidateAdd(struct ddcel *d, const u32 q, const u32 cvi)
{
	if (q != cvi && d->cv[q].last_iter != cvi)
	{
		d->cv[q].last_iter = cvi;
		d->cand[d->cand_count++] = q;
	}
}

/* fi's conflict points become candidates; keep_conflicts 0: fi's conflict edges go too (the face is leaving) */
static void DdcelFaceCandidates(struct ddcel *d, const u32 fi, const u32 cvi, const u32 keep_conflicts)
{
	i32 next;
	for (i32 cei = d->f[fi].ce_list.first; cei != DLL_SENTINEL; cei = next)
	{
		next = d->ce[cei].face_edge.next;
		DdcelCandidateAdd(d, d->ce[cei].vertex, cvi);
		if (!keep_conflicts)
		{
			DdcelConflictRemove(d, (u32) cei);
		}
	}
}

/* Remove the edges begin, begin.next, ... up to (not including) end */
static void DdcelEdgesRemove(struct ddcel *d, const u32 begin, const u32 end)
{
	u32 ei = begin;
	while (ei != end)
	{
		const u32 next = d->e[ei].next;
		ddcelEdgePoolRemove(&d->edge_pool, ei);
		ei = next;
	}
}

/*
 * prev -> next, consecutive in face fi, both border face h: their shared vertex is redundant and goes, prev taking
 * next's end (Box3D b3HullBuilder_ConnectEdges). A triangle h goes with it, prev then bordering its third edge's face;
 * if that face is fi itself, h is an island in fi and fi's three edges around it go as well (Box3D would make prev the
 * twin of its own face's edge). A larger h loses the vertex too and gets a new plane; an older h also drops the
 * conflicts it lost.
 */
static void DdcelVertexRemove(struct ddcel *d, const u32 prev, const u32 next, const u32 cvi, const f32 tol)
{
	const u32 fi = d->e[prev].face_ccw;
	const u32 hi = d->e[d->e[prev].twin].face_ccw;
	if (DdcelFaceRecount(d, hi) == 3)
	{
		const u32 dead0 = d->e[prev].twin;
		const u32 dead1 = d->e[next].twin;
		const u32 dead2 = d->e[dead1].prev;
		const u32 twin = d->e[dead2].twin;
		DdcelFaceCandidates(d, hi, cvi, 0);

		if (d->e[twin].face_ccw == fi)
		{
			/* fi's ring runs prev -> next -> twin or twin -> prev -> next around h */
			const u32 twin_last = (d->e[next].next == twin);
			const u32 before = (twin_last) ? d->e[prev].prev : d->e[twin].prev;
			const u32 after = (twin_last) ? d->e[twin].next : d->e[next].next;
			if ((!twin_last && d->e[twin].next != prev) || before == twin || before == next || after == prev
				|| after == twin)
			{
				LogString(T_PHYSICS, S_WARNING, "DcelConvexHull: island triangle not wrapped by one face, no hull");
				d->broken = 1;
				return;
			}
			d->e[before].next = after;
			d->e[after].prev = before;
			if (d->f[fi].first == prev || d->f[fi].first == next || d->f[fi].first == twin)
			{
				d->f[fi].first = before;
			}
			ddcelEdgePoolRemove(&d->edge_pool, prev);
			ddcelEdgePoolRemove(&d->edge_pool, twin);
		}
		else
		{
			d->e[prev].next = d->e[next].next;
			d->e[d->e[prev].next].prev = prev;
			d->e[prev].twin = twin;
			d->e[twin].twin = prev;
			if (d->f[fi].first == next)
			{
				d->f[fi].first = prev;
			}
		}
		ddcelEdgePoolRemove(&d->edge_pool, next);
		ddcelEdgePoolRemove(&d->edge_pool, dead0);
		ddcelEdgePoolRemove(&d->edge_pool, dead1);
		ddcelEdgePoolRemove(&d->edge_pool, dead2);
		ddcelFacePoolRemove(&d->face_pool, hi);
	}
	else
	{
		const u32 twin = d->e[next].twin;
		const u32 prev_twin = d->e[prev].twin;
		if (d->f[hi].first == prev_twin)
		{
			d->f[hi].first = twin;
		}
		d->e[twin].next = d->e[prev_twin].next;
		d->e[d->e[twin].next].prev = twin;
		ddcelEdgePoolRemove(&d->edge_pool, prev_twin);

		d->e[prev].next = d->e[next].next;
		d->e[d->e[prev].next].prev = prev;
		d->e[prev].twin = twin;
		d->e[twin].twin = prev;
		if (d->f[fi].first == next)
		{
			d->f[fi].first = prev;
		}
		ddcelEdgePoolRemove(&d->edge_pool, next);

		if (DdcelFaceRecount(d, hi) < 3)
		{
			LogString(T_PHYSICS, S_WARNING, "DcelConvexHull: face left with fewer than 3 edges, no hull");
			d->broken = 1;
			return;
		}
		DdcelFaceNewell(d, hi);
		if (d->f[hi].cone != cvi)
		{
			i32 next_ce;
			for (i32 cei = d->f[hi].ce_list.first; cei != DLL_SENTINEL; cei = next_ce)
			{
				next_ce = d->ce[cei].face_edge.next;
				const v3 q = d->v[d->cv[d->ce[cei].vertex].index];
				if (DdcelFaceDistance(d, hi, q) <= 2.0f*tol)
				{
					DdcelConflictRemove(d, (u32) cei);
				}
			}
		}
	}
}

/*
 * Remove fi's redundant vertices (two consecutive edges bordering one face) until none is left. Box3D checks only the
 * two corners a merge creates, one call each; that misses chains, and the second call can get an edge the first
 * removed (when the absorbed face kept a single edge).
 */
static void DdcelFaceClean(struct ddcel *d, const u32 fi, const u32 cvi, const f32 tol)
{
	u32 changed = 1;
	while (changed && !d->broken)
	{
		changed = 0;
		const u32 count = DdcelFaceRecount(d, fi);
		if (count < 3)
		{
			LogString(T_PHYSICS, S_WARNING, "DcelConvexHull: face left with fewer than 3 edges, no hull");
			d->broken = 1;
			return;
		}
		u32 ei = d->f[fi].first;
		for (u32 i = 0; i < count; ++i)
		{
			const u32 next = d->e[ei].next;
			if (d->e[d->e[ei].twin].face_ccw == d->e[d->e[next].twin].face_ccw)
			{
				DdcelVertexRemove(d, ei, next, cvi, tol);
				changed = 1;
				break;
			}
			ei = next;
		}
	}
}

/*
 * Merge the face across edge (twin side) into edge's face f: the run of edges they share goes, f takes the other
 * face's remaining edges, its redundant vertices go (DdcelFaceClean), and it gets a new plane (Box3D
 * b3HullBuilder_ConnectFaces). The other face's conflict points become candidates. Sets d->broken (logged) if the
 * result is not a face.
 */
static void DdcelConnectFaces(struct ddcel *d, const u32 edge, const u32 cvi, const f32 tol)
{
	const u32 fi = d->e[edge].face_ccw;
	const u32 twin = d->e[edge].twin;
	const u32 gi = d->e[twin].face_ccw;

	u32 edge_prev = d->e[edge].prev;
	u32 edge_next = d->e[edge].next;
	u32 twin_prev = d->e[twin].prev;
	u32 twin_next = d->e[twin].next;
	const u32 count = d->f[fi].count;
	u32 steps = 0;
	while (d->e[d->e[edge_prev].twin].face_ccw == gi && steps++ < count)
	{
		edge_prev = d->e[edge_prev].prev;
		twin_next = d->e[twin_next].next;
	}
	while (d->e[d->e[edge_next].twin].face_ccw == gi && steps++ < count)
	{
		edge_next = d->e[edge_next].next;
		twin_prev = d->e[twin_prev].prev;
	}
	if (count <= steps)
	{
		LogString(T_PHYSICS, S_WARNING, "DcelConvexHull: a face to merge borders its neighbour all around, no hull");
		d->broken = 1;
		return;
	}

	d->f[fi].first = edge_prev;
	DdcelFaceCandidates(d, gi, cvi, 0);
	for (u32 a = twin_next; a != d->e[twin_prev].next; a = d->e[a].next)
	{
		d->e[a].face_ccw = fi;
	}

	DdcelEdgesRemove(d, d->e[edge_prev].next, edge_next);
	DdcelEdgesRemove(d, d->e[twin_prev].next, twin_next);
	ddcelFacePoolRemove(&d->face_pool, gi);

	d->e[edge_prev].next = twin_next;
	d->e[twin_next].prev = edge_prev;
	d->e[twin_prev].next = edge_next;
	d->e[edge_next].prev = twin_prev;
	DdcelFaceClean(d, fi, cvi, tol);
	if (d->broken)
	{
		return;
	}
	DdcelFaceNewell(d, fi);

	/* a safety net: the result must not border itself */
	u32 ei = d->f[fi].first;
	for (u32 i = 0; i < d->f[fi].count; ++i)
	{
		if (d->e[d->e[ei].twin].face_ccw == fi)
		{
			LogString(T_PHYSICS, S_WARNING, "DcelConvexHull: merged face borders itself, no hull");
			d->broken = 1;
			return;
		}
		ei = d->e[ei].next;
	}
}

/* edge's face has the other face's centroid more than tol in front: the edge folds inward */
static u32 DdcelEdgeConcave(const struct ddcel *d, const u32 ei, const f32 tol)
{
	const u32 gi = d->e[d->e[ei].twin].face_ccw;
	return tol < DdcelFaceDistance(d, d->e[ei].face_ccw, d->f[gi].centroid);
}

/* edge's face has the other face's centroid more than tol behind it */
static u32 DdcelEdgeConvex(const struct ddcel *d, const u32 ei, const f32 tol)
{
	const u32 gi = d->e[d->e[ei].twin].face_ccw;
	return DdcelFaceDistance(d, d->e[ei].face_ccw, d->f[gi].centroid) < -tol;
}

/* Merge across fi's first concave edge (or not convex by tol, coplanar != 0); return 1 if merged */
static u32 DdcelFaceMerge(struct ddcel *d, const u32 fi, const u32 cvi, const f32 tol, const u32 coplanar)
{
	u32 ei = d->f[fi].first;
	for (u32 i = 0; i < d->f[fi].count; ++i)
	{
		const u32 twin = d->e[ei].twin;
		const u32 merge = (coplanar)
			? (!DdcelEdgeConvex(d, ei, tol) || !DdcelEdgeConvex(d, twin, tol))
			: (DdcelEdgeConcave(d, ei, tol) || DdcelEdgeConcave(d, twin, tol));
		if (merge)
		{
			DdcelConnectFaces(d, ei, cvi, tol);
			return 1;
		}
		ei = d->e[ei].next;
	}
	return 0;
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

	DdcelFaceSet(f0, 0, 3, U32_MAX);
	DdcelFaceSet(f1, 3, 3, U32_MAX);
	DdcelFaceSet(f2, 6, 3, U32_MAX);
	DdcelFaceSet(f3, 9, 3, U32_MAX);

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

	u32 valid = 1;
	ddcel->interior = V3Zero();
	for (u32 i = 0; i < 4; ++i)
	{
		DdcelFaceNewell(ddcel, i);
		valid &= (0.0f < ddcel->f[i].area);
		ddcel->interior = V3Add(ddcel->interior, V3Scale(ddcel->v[ddcel->cv[i].index], 0.25f));
	}
	if (valid)
	{
		DdcelAssertTopology(ddcel);
	}

	return valid;
}

/* Return 0 if the conflict pool is full (logged): no hull */
static u32 InternalConvexHullTetrahedronConflicts(struct ddcel *ddcel, const f32 tol)
{
	const v3 *v = ddcel->v;
	const u32 v_count = ddcel->v_count;
	for (u32 cv_i = 4; cv_i < v_count; ++cv_i)
	{
		struct conflictVertex *cv = ddcel->cv + cv_i;
		for (u32 f_i = 0; f_i < 4; ++f_i)
		{
			/* a conflict: more than 2 tol in front of the face */
			if (2.0f*tol < DdcelFaceDistance(ddcel, f_i, v[cv->index]))
			{
				struct slot slot = conflictEdgePoolAdd(&ddcel->ce_pool);
				if (!slot.address)
				{
					Log(T_PHYSICS, S_WARNING, "DcelConvexHull: conflict graph full (%u point-face pairs), no hull", ddcel->ce_pool.length);
					return 0;
				}
				DLLAppend(cv->ce_list, ddcel->ce_pool.buf, slot.index, vertex_edge);
				DLLAppend(ddcel->f[f_i].ce_list, ddcel->ce_pool.buf, slot.index, face_edge);

				struct conflictEdge *edge = slot.address;
				edge->vertex = cv_i;
				edge->face = f_i;
			}
		}
	}

	return 1;
}

/*
 * Add point cvi (de Berg et al., section 11.2): the faces it sees go, a cone of triangles joins it to the horizon, and
 * the cone's faces are merged until every edge is convex by more than tol (above). The points of the faces that went,
 * of the faces beyond the horizon and of merged-away faces are the candidates for the new faces' conflicts. Return 0
 * (logged) if the horizon is not one loop, a merge cannot be done, or the conflict pool is full: no hull.
 */
static u32 ConvexHullIteration(struct ddcel *d, const u32 cvi, const f32 tol)
{
	struct conflictVertex *cv = d->cv + cvi;
	if (cv->ce_list.count == 0)
	{
		return 1;
	}
	const v3 p = d->v[cv->index];

	/* (5) the faces p sees: from its farthest conflict face, every neighbour p is more than tol in front of */
	u32 seed = d->ce[cv->ce_list.first].face;
	for (i32 cei = cv->ce_list.first; cei != DLL_SENTINEL; cei = d->ce[cei].vertex_edge.next)
	{
		const u32 fi = d->ce[cei].face;
		seed = (DdcelFaceDistance(d, seed, p) < DdcelFaceDistance(d, fi, p)) ? fi : seed;
	}
	u32 visible_count = 0;
	d->visible[visible_count++] = seed;
	d->f[seed].visible = cvi;
	for (u32 k = 0; k < visible_count; ++k)
	{
		const struct ddcelFace *f = d->f + d->visible[k];
		u32 ei = f->first;
		for (u32 i = 0; i < f->count; ++i)
		{
			const u32 gi = d->e[d->e[ei].twin].face_ccw;
			if (d->f[gi].visible != cvi && tol < DdcelFaceDistance(d, gi, p))
			{
				d->f[gi].visible = cvi;
				d->visible[visible_count++] = gi;
			}
			ei = d->e[ei].next;
		}
	}

	/* (6) the seen faces' points are candidates; p's conflicts end here */
	d->cand_count = 0;
	for (u32 k = 0; k < visible_count; ++k)
	{
		DdcelFaceCandidates(d, d->visible[k], cvi, 0);
	}
	while (cv->ce_list.first != DLL_SENTINEL)
	{
		DdcelConflictRemove(d, (u32) cv->ce_list.first);
	}

	/* (7) the horizon: the seen faces' edges whose twin's face is not seen; the faces beyond give candidates too */
	u32 horizon_count = 0;
	u32 horizon_start = U32_MAX;
	for (u32 k = 0; k < visible_count; ++k)
	{
		const struct ddcelFace *f = d->f + d->visible[k];
		u32 ei = f->first;
		for (u32 i = 0; i < f->count; ++i)
		{
			const u32 twin = d->e[ei].twin;
			const u32 gi = d->e[twin].face_ccw;
			if (d->f[gi].visible != cvi)
			{
				const u32 origin = d->e[ei].origin;
				if (d->hv[origin].iter == cvi)
				{
					Log(T_PHYSICS, S_WARNING, "DcelConvexHull: horizon of point %u is not one loop, no hull", cvi);
					return 0;
				}
				d->hv[origin].iter = cvi;
				d->hv[origin].edge_out = ei;
				d->hv[origin].next = d->e[twin].origin;
				d->e[ei].horizon = 1;
				horizon_start = origin;
				horizon_count += 1;
				DdcelFaceCandidates(d, gi, cvi, 1);
			}
			ei = d->e[ei].next;
		}
	}

	/* (8) the seen faces and the edges between them go (horizon edges stay for the cone) */
	for (u32 k = 0; k < visible_count; ++k)
	{
		const struct ddcelFace *f = d->f + d->visible[k];
		u32 ei = f->first;
		const u32 count = f->count;
		for (u32 i = 0; i < count; ++i)
		{
			const u32 next = d->e[ei].next;
			if (!d->e[ei].horizon)
			{
				ddcelEdgePoolRemove(&d->edge_pool, ei);
			}
			ei = next;
		}
		ddcelFacePoolRemove(&d->face_pool, d->visible[k]);
	}

	/* (9) the horizon in order: one loop through every horizon edge */
	u32 vi = horizon_start;
	for (u32 k = 0; k < horizon_count; ++k)
	{
		const u32 ei = d->hv[vi].edge_out;
		if (d->hv[vi].iter != cvi || d->e[ei].horizon != 1)
		{
			Log(T_PHYSICS, S_WARNING, "DcelConvexHull: horizon of point %u is not one loop, no hull", cvi);
			return 0;
		}
		d->e[ei].horizon = 2;
		d->cone[k] = ei;
		vi = d->hv[vi].next;
	}
	if (vi != horizon_start || horizon_count < 3)
	{
		Log(T_PHYSICS, S_WARNING, "DcelConvexHull: horizon of point %u is not one loop, no hull", cvi);
		return 0;
	}

	/* (10) the cone: a triangle (u, w, p) on every horizon edge u -> w, which keeps its direction */
	for (u32 k = 0; k < horizon_count; ++k)
	{
		const u32 e0 = d->cone[k];
		const struct slot sf = ddcelFacePoolAdd(&d->face_pool);
		const struct slot se1 = ddcelEdgePoolAdd(&d->edge_pool);
		const struct slot se2 = ddcelEdgePoolAdd(&d->edge_pool);
		ds_Assert(sf.address && se1.address && se2.address);

		const u32 w = d->e[d->e[e0].twin].origin;
		DdcelEdgeSet(d->e + e0, d->e[e0].origin, d->e[e0].twin, se2.index, se1.index, sf.index);
		DdcelEdgeSet(se1.address, w, U32_MAX, e0, se2.index, sf.index);
		DdcelEdgeSet(se2.address, cv->index, U32_MAX, se1.index, e0, sf.index);
		DdcelFaceSet(sf.address, e0, 3, cvi);
		DdcelFaceNewell(d, sf.index);
		d->f[sf.index].flipped = (0.0f < DdcelFaceDistance(d, sf.index, d->interior));
		d->cone[k] = sf.index;
	}
	for (u32 k = 0; k < horizon_count; ++k)
	{
		/* (w, p) of face k is (p, w) of face k + 1 */
		const u32 e1 = d->e[d->f[d->cone[k]].first].next;
		const u32 e2 = d->e[d->f[d->cone[(k + 1) % horizon_count]].first].prev;
		d->e[e1].twin = e2;
		d->e[e2].twin = e1;
	}

	/* (11) merge: flipped faces into their largest neighbour, then across concave edges, then edges not convex */
	for (u32 k = 0; k < horizon_count && !d->broken; ++k)
	{
		const u32 fi = d->cone[k];
		if (ds_PoolSlotAllocated(d->f + fi) && d->f[fi].flipped)
		{
			d->f[fi].flipped = 0;
			u32 best = d->f[fi].first;
			u32 ei = d->f[fi].first;
			for (u32 i = 0; i < d->f[fi].count; ++i)
			{
				const f32 area = d->f[d->e[d->e[ei].twin].face_ccw].area;
				best = (d->f[d->e[d->e[best].twin].face_ccw].area < area) ? ei : best;
				ei = d->e[ei].next;
			}
			DdcelConnectFaces(d, best, cvi, tol);
		}
	}
	for (u32 coplanar = 0; coplanar <= 1; ++coplanar)
	{
		for (u32 k = 0; k < horizon_count && !d->broken; ++k)
		{
			const u32 fi = d->cone[k];
			while (!d->broken && ds_PoolSlotAllocated(d->f + fi) && DdcelFaceMerge(d, fi, cvi, tol, coplanar))
			{
			}
		}
	}
	if (d->broken)
	{
		return 0;
	}

	/* (12) the new faces' conflicts: every candidate more than 2 tol in front */
	for (u32 c = 0; c < d->cand_count; ++c)
	{
		const u32 q = d->cand[c];
		const v3 x = d->v[d->cv[q].index];
		for (u32 k = 0; k < horizon_count; ++k)
		{
			const u32 fi = d->cone[k];
			if (ds_PoolSlotAllocated(d->f + fi) && 2.0f*tol < DdcelFaceDistance(d, fi, x))
			{
				DdcelConflictAdd(d, q, fi);
			}
		}
	}

	return !d->broken;
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

f32 DcelConvexHullTolerance(const v3 *v, const u32 v_count)
{
	f32 extent_sq = 0.0f;
	for (u32 i = 0; i < v_count; ++i)
	{
		extent_sq = F32Max(extent_sq, V3Dot(v[i], v[i]));
	}
	return 0.25f * GJK_TOUCH_TOLERANCE * F32_EPSILON * F32Sqrt(extent_sq);
}

struct dcel DcelConvexHull(struct arena *mem, const v3 *v, const u32 v_count)
{
	struct dcel dcel = DcelEmpty();
	if (v_count < 4)
	{
		return dcel;
	}
	const f32 tol = DcelConvexHullTolerance(v, v_count);

	struct arena *tmp1 = ArenaPushScratch();
	struct arena *tmp2 = ArenaPushScratch();

	/*
	 * Live faces and half edges never exceed a triangulated hull's (2V - 4, 6V - 12): the seen faces go before the
	 * cone is added, and merging only removes.
	 */
	const u32 edge_count_upper_bound = 6*v_count - 12;
	const u32 face_count_upper_bound = 2*v_count - 4;
	struct ddcel ddcel =
	{
		.face_pool = ddcelFacePoolAlloc(tmp1, face_count_upper_bound, NOT_GROWABLE),
		.edge_pool = ddcelEdgePoolAlloc(tmp1, edge_count_upper_bound, NOT_GROWABLE),
		.v = v,
		.v_count = v_count,
		.ce_pool = conflictEdgePoolAlloc(tmp2, (tmp2->mem_size / sizeof(struct conflictEdge))-1, NOT_GROWABLE),
		.cv = ArenaPush(tmp1, v_count * sizeof(struct conflictVertex)),
		.hv = ArenaPush(tmp1, v_count * sizeof(struct horizionVertex)),
		.visible = ArenaPush(tmp1, face_count_upper_bound * sizeof(u32)),
		.cone = ArenaPush(tmp1, edge_count_upper_bound * sizeof(u32)),
		.cand = ArenaPush(tmp1, v_count * sizeof(u32)),
	};

	ddcel.tmp1 = *ArenaPushScratch();
	if (!ddcel.face_pool.length || !ddcel.edge_pool.length || !ddcel.ce_pool.length || !ddcel.cv || !ddcel.hv
		|| !ddcel.visible || !ddcel.cone || !ddcel.cand)
	{
		const u64 needed = (u64) face_count_upper_bound*(sizeof(struct ddcelFace) + sizeof(u32))
			+ (u64) edge_count_upper_bound*(sizeof(struct ddcelEdge) + sizeof(u32))
			+ (u64) v_count*(sizeof(struct conflictVertex) + sizeof(struct horizionVertex) + sizeof(u32));
		Log(T_PHYSICS, S_WARNING, "DcelConvexHull: %u points need %luB of scratch, the arena has %luB, no hull",
			v_count, needed, tmp1->mem_size);
		goto end;
	}
	ddcel.e = (struct ddcelEdge *) ddcel.edge_pool.buf;
	ddcel.f = (struct ddcelFace *) ddcel.face_pool.buf;
	ddcel.ce = (struct conflictEdge *) ddcel.ce_pool.buf;

	/*
	 * (1) permutation - Random permutation of remaining points. A local generator with a fixed seed: the hull depends
	 * only on its input, not on the thread's random numbers (which it leaves untouched).
	 */
	for (u32 i = 0; i < v_count; ++i)
	{
		DLLFlush(ddcel.cv[i].ce_list);
		ddcel.cv[i].index = i;
		ddcel.cv[i].last_iter = U32_MAX;
		ddcel.hv[i].iter = U32_MAX;
	}
	struct rng rng;
	RngLocalInit(&rng, 0);
	for (u32 i = 0; i < v_count; ++i)
	{
		const u32 j = (u32) RngLocalU64Range(&rng, i, v_count-1);
		const u32 tmp = ddcel.cv[i].index;
		ddcel.cv[i].index = ddcel.cv[j].index;
		ddcel.cv[j].index = tmp;
	}

	/* (2) Get inital points for tetrahedron */
	if (InternalConvexHullTetrahedronIndices(&ddcel, tol) == 0) { goto end; }

	/* (3) initiate DCEL from points */
	if (!InternalConvexHullTetrahedronDdcel(&ddcel, tol)) { goto end; }

	/* (4) setup conflict graph */
	if (!InternalConvexHullTetrahedronConflicts(&ddcel, tol)) { goto end; }

	/* iteratetively solve and add conflicts until no vertices left */
	for (u32 i = 4; i < v_count; ++i)
	{
		if (!ConvexHullIteration(&ddcel, i, tol))
		{
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
