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

#include "ds_dynamics.h"
#include "ds_collision.h"
#include "ds_float.h"
#include "ds_vector.h"
#include "ds_matrix.h"
#include "ds_voronoi.h"

/********************************** Contact Manifold helpers **********************************/

void c_ManifoldDebugPrint(const struct c_Manifold *cm)
{
	fprintf(stderr, "Contact Manifold:\n{\n");
	fprintf(stderr, "\t.v_count = %u\n", cm->v_count);
	for (u32 i = 0; i < cm->v_count; ++i)
	{
		fprintf(stderr, "\t.v[%u] = { %f, %f, %f }, .depth[%u] = %f\n", i, cm->v[i].buf[0], cm->v[i].buf[1], cm->v[i].buf[2], i, cm->depth[i]);
	}
	fprintf(stderr, "\t.n = { %f, %f, %f }\n", cm->n.buf[0], cm->n.buf[1], cm->n.buf[2]);
	fprintf(stderr, "}\n");
}

u32 c_ManifoldCheck(const struct c_Manifold *cm)
{
	u32 valid = cm->v_count <= 4
	         && V3FiniteCheck(cm->n)
	         && V3Dot(cm->n, cm->n) > 0.0f;
	for (u32 i = 0; valid && i < cm->v_count; ++i)
	{
		valid = V3FiniteCheck(cm->v[i])
		     && F32FiniteCheck(cm->depth[i])
		     && cm->depth[i] >= 0.0f;
	}

	return valid;
}

void c_ManifoldTransform(struct c_Manifold *dst, const struct c_Manifold *src, const m3 rot, const v3 translation)
{
    dst->v_count = src->v_count;
    dst->n = M3V3Mul(rot, src->n);
    for (u32 vi = 0; vi < dst->v_count; ++vi)
    {
        dst->depth[vi] = src->depth[vi];
        dst->v[vi] = V3Add(M3V3Mul(rot, src->v[vi]), translation);
    }
}

static f32 c_ManifoldDeepest(const struct c_Manifold *m)
{
    f32 deepest = 0.0f;
    for (u32 i = 0; i < m->v_count; ++i)
    {
        deepest = F32Max(deepest, m->depth[i]);
    }
    return deepest;
}

/********************************** Contact Methods **********************************/

typedef struct PolygonClipPoint PolygonClipPoint;
struct PolygonClipPoint
{
    u32             index;   /* index == 0 => vertex, index == 1 => edge */
    u32             feature[2];
    v3              v;
};

/*
 * Sutherland-Hodgman 3D polygon clipping + negative face voronoi region projection: 
 *
 *  Clip polygon against orthogonal side-planes of the input face. Any clip points 
 *  infront of the face is discarded, after which the remaining points are projected
 *  onto the face and stored in cp, with their corresponding positive depth in cp_depth.
 *  
 *            fv: Expected to be filled with face vertices in counter-clockwise order.
 */
static void PolygonCcwClipNegativeFaceAndProject(struct arena *mem, sat_FeatureId **features, v3 **cp, f32 **cp_depth, u32 *cp_deepest, u32 *cp_count, f32 *max_depth, u32 *deepest_feature, const struct dcel *const h[2], const v3 *const v[2], const u32 f[2], const u32 b_f, const v3 face_normal)
{
    const u32 b_v = 1 - b_f;
	const struct dcelFace *ref_face = h[b_f]->f + f[b_f];
	const struct dcelFace *inc_face = h[b_v]->f + f[b_v];

    const u32 count = 2*inc_face->count + ref_face->count;
    struct PolygonClipPoint *poly[2] =
    {
        ArenaPush(mem, count*sizeof(struct PolygonClipPoint)),
        ArenaPush(mem, count*sizeof(struct PolygonClipPoint)),
    };
    u32 poly_count[2] = { 0, 0 };
	v3 *ref_v = ArenaPush(mem, ref_face->count * sizeof(v3));
	*cp = ArenaPush(mem, count*sizeof(v3));
	*cp_depth = ArenaPush(mem, count*sizeof(f32));
	*features = ArenaPush(mem, count*sizeof(sat_FeatureId));

	v3 ref_center = V3(0.0f, 0.0f, 0.0f);
	for (u32 i = 0; i < ref_face->count; ++i)
	{
		const u32 vi = h[b_f]->e[ref_face->first + i].origin;
		ref_v[i] = v[b_f][vi];
		ref_center = V3Add(ref_center, ref_v[i]);
	}
    /* through the face's vertex average, as the stored planes (DcelPlanesTryCompute): depths agree with the face query */
    struct plane ref_plane = Plane(face_normal, V3Scale(ref_center, 1.0f / (f32) ref_face->count));

	u32 cur = 0;
	for (u32 i = 0; i < inc_face->count; ++i)
	{
		const u32 vi = h[b_v]->e[inc_face->first + i].origin;
        struct PolygonClipPoint pcp =
        {
            .index = 0,
            .feature[0] = sat_FeatureIdConstruct(vi, SAT_FEATURE_TYPE_VERTEX),
            .feature[1] = sat_FeatureIdConstruct(inc_face->first + i, SAT_FEATURE_TYPE_EDGE),
        };
        pcp.v = v[b_v][vi];
		poly[cur][poly_count[cur]++] = pcp;
	}

    /* Clip polygon against voronoi face region */
	for (u32 j = 0; j < ref_face->count; ++j)
	{
		const v3 side_plane_direction = V3Cross(V3Sub(ref_v[(j+1) % ref_face->count], ref_v[j]), ref_plane.normal);
		const struct plane side_plane = Plane(side_plane_direction, ref_v[j]);

        const struct PolygonClipPoint *in = poly[cur];
        const u32 in_count = poly_count[cur];
        struct PolygonClipPoint *out = poly[1 - cur];
        u32 out_count = 0;
        if (in_count)
        {
            /* edges (in_count - 1 -> 0), (0 -> 1), ...; keep the part with d <= 0 */
            struct PolygonClipPoint v1 = in[in_count - 1];
            f32 d1 = PlanePointSignedDistance(&side_plane, v1.v);
            for (u32 i = 0; i < in_count; ++i)
            {
                const struct PolygonClipPoint v2 = in[i];
                const f32 d2 = PlanePointSignedDistance(&side_plane, v2.v);
                if ((d1 < 0.0f && 0.0f < d2) || (0.0f < d1 && d2 < 0.0f))
                {
                    /* signs strictly opposite: d1 - d2 != 0 and the fraction lies in (0, 1); v1's edge feature */
                    struct PolygonClipPoint crossing = v1;
                    crossing.index = 1;
                    crossing.v = V3Add(v1.v, V3Scale(V3Sub(v2.v, v1.v), d1 / (d1 - d2)));
                    out[out_count++] = crossing;
                }
                if (d2 <= 0.0f)
                {
                    out[out_count++] = v2;
                }
                v1 = v2;
                d1 = d2;
            }
        }

        cur = 1 - cur;
        poly_count[cur] = out_count;
	}

	*max_depth = -F32_INFINITY;
    *deepest_feature = 0;
    *cp_deepest = 0;
    *cp_count = 0;
	for (u32 i = 0; i < poly_count[cur]; ++i)
	{
        const struct PolygonClipPoint *pcp = poly[cur] + i;
		(*cp)[*cp_count] = pcp->v;
        (*cp_depth)[ *cp_count ] = -PlanePointSignedDistance(&ref_plane, (*cp)[ *cp_count ]);

		if ((*cp_depth)[*cp_count] >= 0.0f)
		{
            (*features)[*cp_count] = pcp->feature[ pcp->index ];
			(*cp)[*cp_count] = V3AddScaled((*cp)[*cp_count], ref_plane.normal, (*cp_depth)[*cp_count]);
			if (*max_depth < (*cp_depth)[ *cp_count ])
			{
                *max_depth = (*cp_depth)[ *cp_count ];
				*cp_deepest = *cp_count;
                *deepest_feature = pcp->feature[ pcp->index ];
			}
			*cp_count += 1;
		}
	}

    ds_Assert(*cp_count <= count);
}

static u32 PolygonCcwContact(struct c_Manifold *cm, sat_FeatureId cm_features[4], const v3 cm_n, const sat_FeatureId *features, const v3 *cp, const f32 *depth, const u32 cp_deepest, const u32 cp_count)
{
	u32 collision = 1;
	cm->n = cm_n;
	switch (cp_count)
	{
		case 0:
		{
            cm->v_count = 0;
			collision = 0;
		} break;

		case 1:
		{
			cm->v_count = 1;
			cm->v[0] = cp[0];
			cm->depth[0] = depth[0];
            cm_features[0] = features[0];
		} break;

		case 2:
		{
			cm->v_count = 2;
			cm->v[0] = cp[0];
			cm->v[1] = cp[1];
			cm->depth[0] = depth[0];
			cm->depth[1] = depth[1];
            cm_features[0] = features[0];
            cm_features[1] = features[1];
		} break;

		case 3:
		{
			cm->v_count = 3;
			const v3 n = V3Cross(V3Sub(cp[1], cp[0]), V3Sub(cp[2], cp[0]));
			if (V3Dot(n, cm->n) >= 0.0f)
			{
				cm->v[0] = cp[0];
				cm->v[1] = cp[1];
				cm->v[2] = cp[2];
				cm->depth[0] = depth[0];
				cm->depth[1] = depth[1];
				cm->depth[2] = depth[2];
                cm_features[0] = features[0];
                cm_features[1] = features[1];
                cm_features[2] = features[2];
			}
			else
			{
				cm->v[0] = cp[0];
				cm->v[2] = cp[1];
				cm->v[1] = cp[2];
				cm->depth[0] = depth[0];
				cm->depth[2] = depth[1];
				cm->depth[1] = depth[2];
                cm_features[0] = features[0];
                cm_features[2] = features[1];
                cm_features[1] = features[2];
			}
		} break;

		default:
		{
			/* (1) First point is deepest point */
			cm->v_count = 4;
			cm->v[0] = cp[cp_deepest];
			cm->depth[0] = depth[cp_deepest];
            cm_features[0] = features[cp_deepest];
			/* (2) Third point is point furthest away from deepest point */
			f32 max_dist = 0.0f;
			u32 max_i = (cp_deepest + 2) % cp_count;
			for (u32 i = 0; i < cp_count; ++i)
			{
				if (i == (cp_deepest + 1) % cp_count || (i+1) % cp_count == cp_deepest)
				{
					continue;
				}

				const f32 dist = V3DistanceSquared(cp[cp_deepest], cp[i]);
				if (max_dist < dist)
				{
					max_dist = dist;
					max_i = i;
				}
			}
			cm->v[2] = cp[max_i];
			cm->depth[2] = depth[max_i];
            cm_features[2] = features[max_i];

			/* (3, 4) Second point and forth is point that gives largest (in magnitude) 
			 * areas with the previous points on each side of the previous segment 
			 */
			u32 max_pos_i = (cp_deepest + 1) % cp_count;
			u32 max_neg_i = (max_i + 1) % cp_count;
			f32 max_neg = 0.0f;
			f32 max_pos = 0.0f;

			for (u32 i = (cp_deepest + 1) % cp_count; i != max_i; i = (i+1) % cp_count)
			{
				const v3 n = V3Cross(V3Sub(cm->v[0], cp[i]), V3Sub(cm->v[2], cp[i]));
				const f32 d = V3LengthSquared(n);
				if (max_pos < d)
				{
					max_pos = d;
					max_pos_i = i;
				}
			}

			for (u32 i = (max_i + 1) % cp_count; i != cp_deepest; i = (i+1) % cp_count)
			{
				const v3 n = V3Cross(V3Sub(cm->v[0], cp[i]), V3Sub(cm->v[2], cp[i]));
				const f32 d = V3LengthSquared(n);
				if (max_neg < d)
				{
					max_neg = d;
					max_neg_i = i;
				}
			}

			ds_Assert(cp_deepest != max_i);
			ds_Assert(cp_deepest != max_pos_i);
			ds_Assert(cp_deepest != max_neg_i);
			ds_Assert(max_i != max_pos_i);
			ds_Assert(max_i != max_neg_i);
			ds_Assert(max_pos_i != max_neg_i);
	
			/* collinear points have no orientation: then the order below */
			v3 dir;
			const u32 oriented = TriCcwNormalDirectionTryCompute(&dir, cm->v[0], cp[max_pos_i], cm->v[2]);
			if (oriented && V3Dot(dir, cm->n) < 0.0f)
			{
				cm->v[3] = cp[max_pos_i];
				cm->v[1] = cp[max_neg_i];
				cm->depth[3] = depth[max_pos_i];
				cm->depth[1] = depth[max_neg_i];
                cm_features[3] = features[max_pos_i];
                cm_features[1] = features[max_neg_i];
			}
			else
			{
				cm->v[3] = cp[max_neg_i];
				cm->v[1] = cp[max_pos_i];
				cm->depth[3] = depth[max_neg_i];
				cm->depth[1] = depth[max_pos_i];
                cm_features[3] = features[max_neg_i];
                cm_features[1] = features[max_pos_i];
			}
		} break;
	}

    return collision;
}

struct sat_FaceQuery
{
	v3 normal;
	u32 fi;
	u32 vi;     /* the other shape's deepest vertex (sat_PairContact's fallback) */
	f32 depth;
};

struct sat_EdgeQuery
{
	struct segment s1;
	struct segment s2;
	u32	e1;
	u32	e2;
	v3 normal;
	f32 depth;
};

/*
 * pl: both hulls' face planes in v's frame; the incident face is the face of b_v most anti-parallel to the reference
 * normal (stored planes, as DcelSupportFace).
 */
static u32 HullFaceContact(struct arena *mem_tmp, struct c_Manifold *cm, sat_FeatureId cm_features[4], f32 *max_depth, sat_FeatureId *max_feature, const struct sat_FaceQuery *query, const struct dcel *const h[2], const v3 *const v[2], const struct plane *const pl[2], const u32 b_f, const u32 ref)
{
	/* (1) determine incident_face */
    const u32 b_v = 1 - b_f;
    u32 f[2];
    f[b_f] = query->fi;
    f[b_v] = 0;
    f32 min_dot = V3Dot(pl[b_v][0].normal, query->normal);
    for (u32 fi = 1; fi < h[b_v]->f_count; ++fi)
    {
        const f32 dot = V3Dot(pl[b_v][fi].normal, query->normal);
        if (dot < min_dot)
        {
            min_dot = dot;
            f[b_v] = fi;
        }
    }
	
	u32 deepest_point = 0;
	u32 cp_count = 0;
    v3 *cp = NULL;
    f32 *depth = NULL;
    sat_FeatureId *features;
    PolygonCcwClipNegativeFaceAndProject(mem_tmp, &features, &cp, &depth, &deepest_point, &cp_count, max_depth, max_feature, h, v, f, b_f, query->normal);

    u32 collision;
    if (b_f == ref)
    {
        collision = PolygonCcwContact(cm, cm_features, query->normal, features, cp, depth, deepest_point, cp_count);
    }
    else
    {
        const v3 cm_n = V3Scale(query->normal, -1.0f);
        collision = PolygonCcwContact(cm, cm_features, cm_n, features, cp, depth, deepest_point, cp_count);
        for (u32 i = 0; i < cm->v_count; ++i)
        {
            cm->v[i] = V3AddScaled(cm->v[i], cm->n, cm->depth[i]);
        }
    }

    return collision;
}

static u32 EEIsMinkowskiFace(const v3 n1_1, const v3 n1_2, const v3 n2_1, const v3 n2_2, const v3 arc_n1, const v3 arc_n2)
{
	const f32 n1_1d = V3Dot(n1_1, arc_n2);
	const f32 n1_2d = V3Dot(n1_2, arc_n2);
	const f32 n2_1d = V3Dot(n2_1, arc_n1);
	const f32 n2_2d = V3Dot(n2_2, arc_n1);

	/*
	 * last check is the hemisphere test: arc plane normals points "to the left" of the arc 1->2. 
	 * Thus, given the fact that the two first tests passes, which tells us that the two arcs 
	 * cross each others planes, the hemisphere test finally tells us if the arcs cross each other.
	 *
	 * If n2_1 lies in the positive half-space defined by arc_n1, and we know that n2_2 lies in the
	 * negative half-space, then the two arcs cross each other iff n2_1->n2_2 CCW relative to n1_2.
	 * This holds since from the first two check and n2_1->n2_2 CCW relative to n1_2, it must hold
	 * that arc_n2*n1_1 < 0.0f. If the arc is CW to n1_2, arc_n2*n1_1 > 0.0f.
	 *
	 * Similarly, if n2_1 lies in the negative half-space, then the two arcs cross each other iff 
	 * n2_1->n2_2 CW relative to n1_2 <=> arc_n2*n1_1 > 0.0f.
	 *
	 * It follows that intersection <=> (arc_n1*n2_1 > 0 && arc_n2*n1_2 > 0) || 
	 * 				    (arc_n1*n2_1 < 0 && arc_n2*n1_2 < 0)
	 *				<=>  arc_n1*n2_1 * arc_n2*n1_2) > 0
	 *				<=>  n2_1d * n1_2d > 0
	 */
	return (n1_1d*n1_2d < 0.0f && n2_1d*n2_2d < 0.0f && n1_2d*n2_1d > 0.0f) ? 1 : 0;
}

void c_ContactResultSortTriangles(struct arena *frame, struct c_ContactResult *result)
{
    if (!result->manifold_count)
    {
        result->tri = NULL;
        result->tri_manifold = NULL;
        return;
    }

    //ProfZoneNamed("Bad Triangle Sorting");

    result->tri_manifold = ArenaPushPacked(frame, result->manifold_count*sizeof(u32));
    if (!result->tri_manifold)
    {
	    Log(T_SYSTEM, S_FATAL, "Out of memory in %s\n", __func__);
	    FatalCleanupAndExit();
    }

    u32 *t = result->tri;
    u32 *m = result->tri_manifold;
    for (u32 h = 0; h < result->manifold_count; ++h)
    {
        m[h] = h;
        for (u32 i = h; i; --i)
        {
            if (t[i-1] < t[i])
            {
                break;
            }

            const u32 t_tmp = t[i-1];
            t[i-1] = t[i];
            t[i] = t_tmp;

            const u32 m_tmp = m[i-1];
            m[i-1] = m[i];
            m[i] = m_tmp;
        }
    }

    //ProfZoneEnd;
}

/********************************** SHAPE CONTACTS **********************************/

/*
 * Preconditions:
 *      radii >= 0                              (otherwise r_sum - dist can be negative)
 *      r_sum == 0 or ~1.1e-19 <= r_sum <= ~1.8e19 m
 *                                              (r_sum^2 neither underflows nor overflows; below, a subnormal
 *                                              r_sum^2 can round up, so dist > r_sum; above, r_sum^2 = Inf accepts
 *                                              any distance: dist = Inf, depth = -Inf, n = 0)
 *      finite positions, unit rotation
 *      IEEE subnormals (no flush-to-zero), correctly rounded sqrtf
 *
 * Guarantees (given the preconditions):
 *      depth >= 0 exactly: the branch requires dist_sq <= fl(r_sum^2); a correctly rounded sqrt is monotone and
 *      sqrt(fl(x^2)) = |x| in binary floating point (no under/overflow), so dist <= r_sum.
 *
 *      n finite: dist > 0 implies dist >= sqrt(1.4e-45) ~ 3.7e-23 (smallest positive subnormal dist_sq), so
 *      1/dist <= 2.7e22, and |n_i| = |p_i| / dist <= sqrt(1.5) ~ 1.22 (a subnormal p_i^2 rounds down by at
 *      most 1.5x; p_i^2 underflowing to 0 means |p_i| <= 2.6e-23). |n| = 1 up to O(eps) whenever dist_sq is
 *      a normal float (|p| >~ 1.1e-19); below that, |n| lies in ~[0.7, 1.22].
 *
 * Degenerate input:
 *      dist == 0 (coincident centers): n = (1, 0, 0) in s0's frame (arbitrary; rotates with s0 in world),
 *      depth = r_sum, v = r0 n.
 *      |p| below the positions' resolution (~eps max(|a|, |b|)): n is valid but its direction is noise.
 */
struct c_ContactResult c_SphereContact(struct arena *frame, const struct c_ContactResult *cached_result, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1)
{
    ds_Assert(s0->type == C_SHAPE_SPHERE);
	ds_Assert(s1->type == C_SHAPE_SPHERE);

    struct c_ContactResult result = { 0 };

    const f32 r_sum = s0->sphere.radius + s1->sphere.radius;
    const v3 p = QUnitRotate(QConj(t0->rotation), V3F64Demote(V3F64Sub(t1->position, t0->position)));
    const f32 dist_sq = V3LengthSquared(p);
    if (dist_sq <= r_sum*r_sum)
    {
        /* Note: When |p| is 0 or close to 0, the normal becomes arbitrary or noise, respectively. */
        const f32 dist = F32Sqrt(dist_sq);

        result.manifold_count = 1;
        result.contact_manifold = ArenaPushPacked(frame, sizeof(struct c_Manifold));
        struct c_Manifold *manifold = result.contact_manifold;

        manifold->n = (dist > 0.0f)
                    ? V3Scale(p, 1.0f / dist)
                    : V3(1.0f, 0.0f, 0.0f);
		manifold->v_count = 1;
        manifold->depth[0] = r_sum - dist;
        manifold->v[0] = V3Scale(manifold->n, s0->sphere.radius);
    }

    return result;
}

/*
 * Capsule vs sphere: the sphere vs sphere contact against the closest axis point c = (0, clamp(p.y, -h, h), 0),
 * p = sphere center in the capsule's frame; d = p - c.
 *
 * Preconditions: as c_SphereContact, with r_sum = capsule radius + sphere radius.
 *
 * Guarantees: as c_SphereContact (the proofs only use that the test and the sqrt share dist_sq).
 *      n, v and depth are continuous in p (clamp is continuous; no region switch at the caps).
 *
 * Degenerate input:
 *      d == 0 (center on the segment): n = (1, 0, 0) in s0's frame (perpendicular to the axis: a shortest
 *      way out), depth = r_sum, v = c + r_capsule n.
 *      |d| below the positions' resolution: n is valid but its direction is noise.
 */
struct c_ContactResult c_CapsuleSphereContact(struct arena *frame, const struct c_ContactResult *cached_result, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1)
{
    ds_Assert(s0->type == C_SHAPE_CAPSULE);
    ds_Assert(s1->type == C_SHAPE_SPHERE);

    struct c_ContactResult result = { 0 };

    const v3 p = QUnitRotate(QConj(t0->rotation), V3F64Demote(V3F64Sub(t1->position, t0->position)));
    const f32 r_sum = s0->capsule.radius + s1->sphere.radius;

    const f32 clamp_y = F32Clamp(p.y, -s0->capsule.half_height, s0->capsule.half_height);
    const f32 diff_y = p.y - clamp_y;
    const f32 dist_sq = p.x*p.x + p.z*p.z + diff_y*diff_y;
    if (dist_sq <= r_sum*r_sum)
    {
        result.manifold_count = 1;
        result.contact_manifold = ArenaPushPacked(frame, sizeof(struct c_Manifold));
        struct c_Manifold *m = result.contact_manifold;

        const f32 dist = F32Sqrt(dist_sq);
        m->v_count = 1;
        /* degenerate case: sphere center is on capsule segment.*/
        m->n = (dist > 0.0f)
            ? V3Scale(V3(p.x, diff_y, p.z), 1.0f/dist)
            : V3(1.0f, 0.0f, 0.0f);
        m->v[0] = V3Add(V3(0.0f, clamp_y, 0.0f), V3Scale(m->n, s0->capsule.radius));
        m->depth[0] = r_sum - dist;
        ds_Assert(m->depth[0] >= 0.0f);
    }

    return result;
}


/* Preconditions: e_len_sq_xz < capsule_parallel_tan_sq e.y^2 (so e.y != 0); c, e, n as in c_CapsuleContact.  */
static u32 c_CapsuleParallelManifold(struct c_Manifold *manifold, const v3 c, const v3 e, const f32 h0, const f32 r0, const f32 r_sum)
{
    const f32 u0 = (-h0 - c.y) / e.y;
    const f32 u1 = ( h0 - c.y) / e.y;
    const f32 u_min = F32Max(-1.0f, F32Min(u0, u1));
    const f32 u_max = F32Min( 1.0f, F32Max(u0, u1));
    if (u_min >= u_max)
    {
        /* no part of core 1 is beside core 0 (end to end) */
        return 0;
    }

    const f32 u[2] = { u_min, u_max };
    v3 v[2];
    f32 depth[2];
    for (u32 i = 0; i < 2; ++i)
    {
        const v3 q = V3Add(c, V3Scale(e, u[i]));
        /* q.y is within [-h0, h0] up to round-off */
        const v3 p = V3(0.0f, F32Clamp(q.y, -h0, h0), 0.0f);
        depth[i] = r_sum - V3Dot(V3Sub(q, p), manifold->n);
        if (depth[i] < 0.0f)
        {
            /* tilted: this end doesn't touch */
            return 0;
        }
        v[i] = V3Add(p, V3Scale(manifold->n, r0));
    }

    manifold->v_count = 2;
    manifold->v[0] = v[0];
    manifold->v[1] = v[1];
    manifold->depth[0] = depth[0];
    manifold->depth[1] = depth[1];
    return 1;
}

/*
 * Capsule vs capsule: local manifold in s0's frame (n from s0 to s1); one point, or two when the
 * capsules are nearly parallel and lie side by side.
 */
struct c_ContactResult c_CapsuleContact(struct arena *frame, const struct c_ContactResult *cached_result, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1)
{
    ds_Assert(s0->type == C_SHAPE_CAPSULE);
    ds_Assert(s1->type == C_SHAPE_CAPSULE);

    struct c_ContactResult result = { 0 };

    const f32 r_sum = s0->capsule.radius + s1->capsule.radius;
    const q inv = QConj(t0->rotation);
    const v3 c = QUnitRotate(inv, V3F64Demote(V3F64Sub(t1->position, t0->position)));
    const v3 e = QUnitRotate(inv, QYUnitRotate(t1->rotation, s1->capsule.half_height));

    const f32 e_len_sq_xz = e.x*e.x + e.z*e.z;
 
    /*
     * in xz: 
     *  Q(u) = c + u*e;
     *  QDist(u) = (c + u*e).(c + u*e) = c.c + (u^2)*e.e + 2*u*c.e;
     *  QDist'(u) = 2*u*e.e + 2c.e.
     *
     *  Min at QDist'(u) = 0 => u = -c.e / e.e
     *
     *  u yields closest point (in xz) on Q(u) = c + u*e. if e_len_sq_xz == 0, core 1 is parallel
     *  to y and every u is equally close, so u = 0 is valid.
     */
    const f32 u0 = (e_len_sq_xz > 0.0f) ? F32Clamp(-(c.x*e.x + c.z*e.z) / e_len_sq_xz, -1.0f, 1.0f) : 0.0f;

    const v3 Q1 = V3Add(c, V3Scale(e, u0));
    const f32 Q0_y = F32Clamp(Q1.y, -s0->capsule.half_height, s0->capsule.half_height);
    v3 Q1_shift;
    if (Q1.y == Q0_y)
    {
        /* u0 minimizes the xz-distance and the y-distance is 0, so (u0, Q0_y) is the closest pair in xyz. */
        Q1_shift = V3(Q1.x, 0.0f, Q1.z);
    }
    else
    {
        const f32 e_len_sq = e_len_sq_xz + e.y*e.y;
        const v3 c_shift = V3(c.x, c.y - Q0_y, c.z);
        const f32 u1 = (e_len_sq > 0.0f) ? F32Clamp(-V3Dot(c_shift, e) / e_len_sq, -1.0f, 1.0f) : 0.0f;
        Q1_shift = V3Add(c_shift, V3Scale(e, u1));
    }

    const f32 dist_sq = V3LengthSquared(Q1_shift);
    if (dist_sq > r_sum*r_sum)
    {
        return result;
    }

    const f32 dist = F32Sqrt(dist_sq);
    result.manifold_count = 1;
    result.contact_manifold = ArenaPushPacked(frame, sizeof(struct c_Manifold));
    struct c_Manifold *manifold = result.contact_manifold;

    /*
     * Q1_shift carries rounding error ~ eps (|c| + |e| + h0): below the noise band (as GJK's touch band, 100 eps L)
     * its direction is noise, so the cores count as touching or crossing.
     */
    const f32 dist_noise = 100.0f*F32_EPSILON*(F32Abs(c.x) + F32Abs(c.y) + F32Abs(c.z) + F32Abs(e.x) + F32Abs(e.y) + F32Abs(e.z) + s0->capsule.half_height);
    if (dist > dist_noise)
    {
        manifold->n = V3Scale(Q1_shift, 1.0f / dist);
    }
    else if (e_len_sq_xz > 0.0f)
    {
        /* cores cross: y x e is perpendicular to both cores; orient it towards capsule 1's center */
        manifold->n = V3Scale(V3(e.z, 0.0f, -e.x), 1.0f / F32Sqrt(e_len_sq_xz));
        if (manifold->n.x*c.x + manifold->n.z*c.z < 0.0f)
        {
            manifold->n = V3Negate(manifold->n);
        }
    }
    else
    {
        /* cores coaxial: any direction perpendicular to y */
        manifold->n = V3(1.0f, 0.0f, 0.0f);
    }

    /*
     * Y-axis parallel check of e:
     * 
     *      tan(theta) = sqrt(e_len_sq_xz) / |e.y|, so e_len_sq_xz < tan_sq e.y^2 <=> theta < max angle
     * (default 2.86 deg: tan_sq ~ 0.0025). Relative (scale-free in e); antiparallel counts; e == 0 fails.
     */
    if (e_len_sq_xz < g_numerics_config->capsule_parallel_tan_sq*e.y*e.y
        && c_CapsuleParallelManifold(manifold, c, e, s0->capsule.half_height, s0->capsule.radius, r_sum))
    {
        return result;
    }

    manifold->v_count = 1;
    manifold->depth[0] = r_sum - dist;
    manifold->v[0] = V3Add(V3(0.0f, Q0_y, 0.0f), V3Scale(manifold->n, s0->capsule.radius));

    return result;
}

struct c_ContactResult c_HullSphereContact(struct arena *frame, const struct c_ContactResult *cached_result, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1)
{
	ds_Assert(s0->type == C_SHAPE_CONVEX_HULL);
	ds_Assert(s1->type == C_SHAPE_SPHERE);

    struct c_ContactResult result = { 0 };
    result.gjk_cache = ArenaPushPacked(frame, sizeof(struct GJKCache));

    const f32 radius = s1->sphere.radius;
    const q inv = QConj(t0->rotation);
    const v3 p = QUnitRotate(inv, V3F64Demote(V3F64Sub(t1->position, t0->position)));
    const ds_Transform t_1_to_0 = { .rotation = QIdentity(), .position = p };

    v3 c[2], n;
    const f32 distance = GJK(c+0, c+1, &n, result.gjk_cache, cached_result->gjk_cache, s0, s1, &t_1_to_0, radius);
    if (radius < distance)
    {
        return result;
    }

    result.manifold_count = 1;
    result.contact_manifold = ArenaPushPacked(frame, sizeof(struct c_Manifold));
    struct c_Manifold *m = result.contact_manifold;
	m->v_count = 1;

    /* shallow penetration */
    if (0.0f < distance)
    {
        m->n = n;
        m->v[0] = c[0];
        m->depth[0] = radius - distance;
    }
	/* deep penetration */
    else
    {
		const struct dcel *h = &s0->hull;

		f32 min_depth = F32_INFINITY;
		for (u32 fi = 0; fi < h->f_count; ++fi)
		{
            const f32 depth = -PlanePointSignedDistance(h->plane + fi, p);
			if (depth < min_depth)
			{
				m->n = h->plane[fi].normal;
				min_depth = depth;
			}
		}
        /*
         * min_depth < 0: the center lies outside by -min_depth (within GJK's touch band); unclamped, v is the
         * projection onto the face's plane from either side and the depth exact.
         */
		m->depth[0] = F32Max(0.0f, radius + min_depth);
		m->v[0] = V3Add(p, V3Scale(m->n, min_depth));
	}

    return result;
}

/*
 * Hull edge e1 (s1, between faces with normals n1_1, n1_2) vs the capsule core: if the pair is a candidate (Gauss
 * map) and not parallel, compute its axis and separation and keep it in query if it separates more.
 */
static void HullCapsuleEECheck(struct sat_EdgeQuery *query, const u32 e1, const struct segment *s1, const v3 n1_1, const v3 n1_2, const struct segment *core)
{
	const f32 n1_1d = V3Dot(n1_1, core->dir);
	const f32 n1_2d = V3Dot(n1_2, core->dir);

	/*
	 * Gauss map test: the hull edge is the arc n1_1 -> n1_2. The core has no faces, so every direction
	 * perpendicular to it sees the whole core as extreme: its Gauss map is the great circle n.dir = 0.
	 * The pair can realize a separating axis iff the arc crosses the circle, i.e. its end points lie on
	 * opposite sides of the circle's plane:
	 *
	 *	n1_1d * n1_2d < 0
	 *
	 * (EEIsMinkowskiFace with the second arc replaced by a full circle: only the first plane test remains.)
	 * A zero length core gives 0 and is never a candidate.
	 */
	if (n1_1d*n1_2d < 0.0f)
	{
		/* Skip nearly parallel pairs: their cross product is noise, and a face realizes the axis then */
		const v3 cross = V3Cross(s1->dir, core->dir);
		const f32 s1s1_d = V3Dot(s1->dir, s1->dir);
		const f32 s2s2_d = V3Dot(core->dir, core->dir);
		if (V3Dot(cross, cross) > g_numerics_config->edge_parallel_sin_sq*s1s1_d*s2s2_d)
		{
			/*
			 * The axis is the arc's point on the circle: n(t) = (1 - t) n1_1 + t n1_2 with n(t).dir = 0,
			 *
			 *	(1 - t) n1_1d + t n1_2d = 0  <=>  t = n1_1d / (n1_1d - n1_2d)
			 *
			 * The signs differ, so |n1_1d - n1_2d| >= |n1_1d| and t is in [0, 1]. Lying between two
			 * outward face normals, the axis points out of the hull: no hull center needed to orient it.
			 */
			const f32 t = n1_1d / (n1_1d - n1_2d);
			v3 axis = V3Interpolate(n1_2, n1_1, t);
			const f32 axis_len_sq = V3Dot(axis, axis);
			/* 0 only for antiparallel n1_1, n1_2 (a knife edge): excluded by the c_Shape preconditions */
			ds_Assert(axis_len_sq > 1000.0f*F32_MIN_POSITIVE_NORMAL);
			axis = V3Scale(axis, 1.0f / F32Sqrt(axis_len_sq));

			/* The axis is perpendicular to both lines, so any pair of points measures the separation */
			const f32 separation = V3Dot(axis, V3Sub(core->p[0], s1->p[0]));
			if (query->depth < separation)
			{
				query->depth = separation;
				query->normal = axis;
				query->s1 = *s1;
				query->s2 = *core;
				query->e1 = e1;
				query->e2 = 0;
			}
		}
	}
}

/*
 * Edge query (Gregorius; as HullEdgeQuery): every hull edge once against the core. query keeps the
 * largest separation; no early out, as GJK already found the shapes touching.
 */
static void HullCapsuleEESeparation(struct sat_EdgeQuery *query, const struct dcel *h, const struct segment *core)
{
	for (u32 f1 = 0; f1 < h->f_count; ++f1)
	{
		const v3 n1_1 = h->plane[f1].normal;
		const u32 end_1 = h->f[f1].first + h->f[f1].count;
		for (u32 e1_1 = h->f[f1].first; e1_1 < end_1; ++e1_1)
		{
			const u32 e1_2 = h->e[e1_1].twin;
			if (e1_2 < e1_1) { continue; }

			const struct segment s1 = Segment(h->v[h->e[e1_1].origin], h->v[h->e[e1_2].origin]);
			const v3 n1_2 = h->plane[h->e[e1_2].face_ccw].normal;
			HullCapsuleEECheck(query, e1_1, &s1, n1_1, n1_2, core);
		}
	}
}

/*
 * Hull vs capsule: local manifold in the hull's frame (n from hull to capsule). GJK against the capsule's core;
 * shallow: the core clipped to the reference face (two points) if it lies along it, else one point from the
 * closest points; deep (core touches the hull): SAT face and edge queries; the core clipped to the reference
 * face, or the closest points of the best edge and the core if that edge separates more by sat_face_bias.
 */
struct c_ContactResult c_HullCapsuleContact(struct arena *frame, const struct c_ContactResult *cached_result, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1)
{
	ds_Assert(s0->type == C_SHAPE_CONVEX_HULL);
	ds_Assert(s1->type == C_SHAPE_CAPSULE);

    struct c_ContactResult result = { 0 };
    result.gjk_cache = ArenaPushPacked(frame, sizeof(struct GJKCache));

    const f32 radius = s1->capsule.radius;
    const ds_Transform t_1_to_0 = ds_TransformWorldRelative(t0, t1);

    v3 c[2], n;
    const f32 distance = GJK(c+0, c+1, &n, result.gjk_cache, cached_result->gjk_cache, s0, s1, &t_1_to_0, radius);
    if (radius < distance)
    {
        return result;
    }

    result.manifold_count = 1;
    result.contact_manifold = ArenaPushPacked(frame, sizeof(struct c_Manifold));
    struct c_Manifold *m = result.contact_manifold;

    const struct dcel *h = &s0->hull;
    const struct segment core = SegmentCapsuleTransform(&s1->capsule, &t_1_to_0);

    /* shallow penetration: distance > 0 is beyond GJK's touch band (100 eps L), so the GJK normal is reliable */
    if (0.0f < distance)
    {
        /*
         * Two points if the GJK normal is within face_parallel_max_degrees of the reference face's normal and
         * both ends of the clipped core touch (d_i <= r); otherwise one point. Each end has its own depth.
         */
        const u32 ref = DcelSupportFace(h, n);
        const struct plane *ref_pl = h->plane + ref;
        struct segment ref_clip;
        if (g_numerics_config->face_parallel_cos < V3Dot(ref_pl->normal, n)
            && DcelFaceSegmentClip(&ref_clip, h, ref, &core))
        {
            const f32 d0 = PlanePointSignedDistance(ref_pl, ref_clip.p[0]);
            const f32 d1 = PlanePointSignedDistance(ref_pl, ref_clip.p[1]);
            if (d0 <= radius && d1 <= radius)
            {
                m->v_count = 2;
                m->n = ref_pl->normal;
                m->v[0] = V3Sub(ref_clip.p[0], V3Scale(ref_pl->normal, d0));
                m->v[1] = V3Sub(ref_clip.p[1], V3Scale(ref_pl->normal, d1));
                m->depth[0] = radius - d0;
                m->depth[1] = radius - d1;
                return result;
            }
        }

        m->v_count = 1;
        m->n = n;
        m->v[0] = c[0];
        m->depth[0] = radius - distance;
        return result;
    }

    /* deep penetration: the core's separation along face fi is the signed distance of its deeper end */
    u32 face = 0;
    f32 max_separation = -F32_INFINITY;
    for (u32 fi = 0; fi < h->f_count; ++fi)
    {
        const f32 d0 = PlanePointSignedDistance(h->plane + fi, core.p[0]);
        const f32 d1 = PlanePointSignedDistance(h->plane + fi, core.p[1]);
        const f32 separation = F32Min(d0, d1);
        if (max_separation < separation)
        {
            face = fi;
            max_separation = separation;
        }
    }

    /* face contact: the core clipped to the face; empty if nothing is left or no clipped end touches (d > r) */
    const struct plane *pl = h->plane + face;
    struct segment clip = core;
    const u32 clipped = DcelFaceSegmentClip(&clip, h, face, &core);
    const f32 depth[2] =
    {
        PlanePointSignedDistance(pl, clip.p[0]),
        PlanePointSignedDistance(pl, clip.p[1]),
    };
    const u32 deeper = (depth[0] <= depth[1]) ? 0 : 1;
    const u32 face_contact = clipped && depth[deeper] <= radius;

    struct sat_EdgeQuery e_query = { .depth = -F32_INFINITY, .e1 = U32_MAX };
    HullCapsuleEESeparation(&e_query, h, &core);

    /*
     * Edge contact if the face contact is empty or the edge separates more by sat_face_bias (face: the clipped
     * separation): the closest points of the edge's and the core's lines, v on the hull edge. Only if both lie
     * within the segments: otherwise other features realize the axis, and the edge and core don't meet
     * along it (e.g. an edge accepted by the Gauss test on rounding noise, its axis ~ a face normal).
     * The clipped separation favours two-point face contacts over the best axis: a stick across an edge
     * keeps the face normal until it sinks more than sat_face_bias / (1 - 1/sqrt 2).
     */
    if (e_query.e1 != U32_MAX && (!face_contact || depth[deeper] + g_numerics_config->sat_face_bias < e_query.depth))
    {
        /* not parallel: the edge passed the parallel test */
        f32 t_edge, t_core;
        if (SegmentLineClosestParameterCheck(&t_edge, &t_core, &e_query.s1, &e_query.s2))
        {
            m->v_count = 1;
            m->n = e_query.normal;
            m->v[0] = SegmentBc(&e_query.s1, t_edge);
            m->depth[0] = F32Max(0.0f, radius - e_query.depth);
            return result;
        }
    }

    m->n = pl->normal;

    /* as in the shallow path: two points if both clipped ends touch (d_i <= r), else the deeper end */
    if (face_contact)
    {
        if (depth[1 - deeper] <= radius)
        {
            m->v_count = 2;
            m->v[0] = V3Sub(clip.p[0], V3Scale(pl->normal, depth[0]));
            m->v[1] = V3Sub(clip.p[1], V3Scale(pl->normal, depth[1]));
            m->depth[0] = radius - depth[0];
            m->depth[1] = radius - depth[1];
            return result;
        }

        m->v_count = 1;
        m->v[0] = V3Sub(clip.p[deeper], V3Scale(pl->normal, depth[deeper]));
        m->depth[0] = radius - depth[deeper];
        return result;
    }

    /*
     * No face or edge contact: in exact arithmetic only when the shapes touch without intersecting (GJK's touch
     * band: apart by < 100 eps L), where the face clip can be empty and the realizing edge can be skipped as
     * parallel. GJK's point (c[0] == c[1]) is where they touch; touching: depth r, the face normal as the best
     * available direction.
     */
    m->v_count = 1;
    m->v[0] = c[0];
    m->depth[0] = radius;

    return result;
}

/*
 * Face query (Gregorius): every face of one hull (planes pl, in the common frame) against the other hull's vertices
 * v; a face's separation is the smallest signed distance of v. query keeps the largest; returns 1 at a separating
 * face (separation > 0).
 */
static u32 HullFaceQuery(struct sat_FaceQuery *query, const struct plane *pl, const u32 f_count, const v3 *v, const u32 v_count)
{
	for (u32 fi = 0; fi < f_count; ++fi)
	{
		f32 min_dist = F32_INFINITY;
		u32 min_vi = 0;
		for (u32 i = 0; i < v_count; ++i)
		{
			const f32 dist = PlanePointSignedDistance(pl + fi, v[i]);
			if (dist < min_dist)
			{
				min_dist = dist;
				min_vi = i;
			}
		}

		if (query->depth < min_dist)
		{
			query->fi = fi;
			query->vi = min_vi;
			query->depth = min_dist;
			query->normal = pl[fi].normal;
		}

		if (min_dist > 0.0f)
		{
			return 1;
		}
	}

	return 0;
}

/*
 * Edge pair (e1 of hull 1 = h[0], e2 of hull 2 = h[1]; planes and vertices in hull 1's frame): if a face of the
 * Minkowski difference of hulls 1 and 2 (Gauss map test, hull 2's normals negated) and not parallel, its axis
 * cross(e1, e2) oriented away from hull 1's center of mass and its separation; kept in query if larger. Orientation:
 * e1 is hull 1's support along a valid axis L, so L.(e1 - C) > 0 for every C inside hull 1 (a margin of at least
 * hull 1's inner radius).
 */
static void HullEdgeCheck(struct sat_EdgeQuery *query, const struct dcel *const h[2], const v3 *const v[2], const struct plane *const pl[2], const v3 center_1, const u32 e1, const u32 e2)
{
	const u32 t1 = h[0]->e[e1].twin;
	const u32 t2 = h[1]->e[e2].twin;
	const struct segment s1 = Segment(v[0][h[0]->e[e1].origin], v[0][h[0]->e[t1].origin]);
	const struct segment s2 = Segment(v[1][h[1]->e[e2].origin], v[1][h[1]->e[t2].origin]);
	const v3 n1_1 = pl[0][h[0]->e[e1].face_ccw].normal;
	const v3 n1_2 = pl[0][h[0]->e[t1].face_ccw].normal;
	const v3 n2_1 = V3Negate(pl[1][h[1]->e[e2].face_ccw].normal);
	const v3 n2_2 = V3Negate(pl[1][h[1]->e[t2].face_ccw].normal);
	if (!EEIsMinkowskiFace(n1_1, n1_2, n2_1, n2_2, s1.dir, s2.dir))
	{
		return;
	}

	/* parallel pairs: their cross product is noise (and no face of the Minkowski difference beyond the faces') */
	v3 axis = V3Cross(s1.dir, s2.dir);
	const f32 axis_sq = V3Dot(axis, axis);
	if (axis_sq <= g_numerics_config->edge_parallel_sin_sq*V3Dot(s1.dir, s1.dir)*V3Dot(s2.dir, s2.dir))
	{
		return;
	}

	axis = V3Scale(axis, 1.0f / F32Sqrt(axis_sq));
	if (V3Dot(axis, V3Sub(s1.p[0], center_1)) < 0.0f)
	{
		axis = V3Negate(axis);
	}

	const f32 separation = V3Dot(axis, V3Sub(s2.p[0], s1.p[0]));
	if (query->depth < separation)
	{
		query->depth = separation;
		query->normal = axis;
		query->s1 = s1;
		query->s2 = s2;
		query->e1 = e1;
		query->e2 = e2;
	}
}

/* Edge query (Gregorius): every edge pair once. Returns 1 at a separating axis (separation > 0). */
static u32 HullEdgeQuery(struct sat_EdgeQuery *query, const struct dcel *const h[2], const v3 *const v[2], const struct plane *const pl[2], const v3 center_1)
{
	for (u32 e1 = 0; e1 < h[0]->e_count; ++e1)
	{
		if (h[0]->e[e1].twin < e1) { continue; }
		for (u32 e2 = 0; e2 < h[1]->e_count; ++e2)
		{
			if (h[1]->e[e2].twin < e2) { continue; }
			HullEdgeCheck(query, h, v, pl, center_1, e1, e2);
			if (query->depth > 0.0f)
			{
				return 1;
			}
		}
	}

	return 0;
}

/*
 * Edge contact from query: the closest points of the edges' lines, v on s1 (hull 1's edge); only if both lie within the
 * segments (otherwise other features realize the axis). Writes the EE cache; returns 1 if built.
 */
static u32 HullEdgeContact(struct c_Manifold *m, struct c_SatCache *cache, const struct sat_EdgeQuery *query)
{
	/* not parallel: the pair passed the parallel test */
	f32 t1, t2;
	if (!SegmentLineClosestParameterCheck(&t1, &t2, &query->s1, &query->s2))
	{
		return 0;
	}

	m->v_count = 1;
	m->n = query->normal;
	m->v[0] = SegmentBc(&query->s1, t1);
	m->depth[0] = F32Max(0.0f, -query->depth);

	cache->type = SAT_CACHE_CONTACT_EE;
	cache->depth = m->depth[0];
	cache->normal = m->n;
	cache->feature[0] = sat_FeatureIdConstruct(query->e1, SAT_FEATURE_TYPE_EDGE);
	cache->feature[1] = sat_FeatureIdConstruct(query->e2, SAT_FEATURE_TYPE_EDGE);
	return 1;
}

/*
 * Hull edge e1 (s0, as stored) vs triangle edge a (s1: t_a -> t_b, ccw about n_t). The triangle's edges are knife
 * edges: the Gauss arc of edge a is the half circle n_t -> m_e -> -n_t, m_e = (t_b - t_a) x n_t (outward), so
 * EEIsMinkowskiFace (arcs shorter than pi) cannot tell which half is meant. The pair realizes a face of the Minkowski
 * difference iff the hull's arc n1 -> n2 meets the negated half circle:
 *
 *      a = n1.e,  b = n2.e,  a b < 0                       (the hull's arc crosses the plane u.e = 0)
 *      u = (1 - t) n1 + t n2,  t = a / (a - b) in (0, 1)   (bounded: the signs differ)
 *      u.m_e < 0                                           (-u is a triangle normal at e; u.m_e = 0: +-n_t, a face
 *                                                           axis)
 *
 * The axis u / |u| lies on the hull's arc, so it points out of the hull: oriented by the hull's geometry, also near
 * +-n_t, where the triangle's centroid lies across the axis and cannot orient it. |u| >= cos(half the hull edge's
 * turn) > 0 (no knife edges on hulls). Nearly parallel pairs are skipped as in HullEdgeCheck. Kept if it separates
 * more; e2 = a.
 */
static void HullTriangleEdgeCheck(struct sat_EdgeQuery *query, const struct dcel *h, const v3 *v, const struct plane *pl, const v3 tri[3], const v3 n_t, const u32 e1, const u32 a)
{
	const u32 t1 = h->e[e1].twin;
	const v3 n1 = pl[h->e[e1].face_ccw].normal;
	const v3 n2 = pl[h->e[t1].face_ccw].normal;
	const u32 b = (a + 1) % 3;
	const v3 e = V3Sub(tri[b], tri[a]);
	const f32 n1e = V3Dot(n1, e);
	const f32 n2e = V3Dot(n2, e);
	if (!(n1e*n2e < 0.0f))
	{
		return;
	}

	const v3 u = V3Interpolate(n2, n1, n1e / (n1e - n2e));
	if (!(V3Dot(u, V3Cross(e, n_t)) < 0.0f))
	{
		return;
	}

	const struct segment s1 = Segment(v[h->e[e1].origin], v[h->e[t1].origin]);
	const v3 cross = V3Cross(s1.dir, e);
	if (V3Dot(cross, cross) <= g_numerics_config->edge_parallel_sin_sq*V3Dot(s1.dir, s1.dir)*V3Dot(e, e))
	{
		return;
	}

	const f32 u_sq = V3Dot(u, u);
	ds_Assert(u_sq > 1000.0f*F32_MIN_POSITIVE_NORMAL);
	const v3 axis = V3Scale(u, 1.0f / F32Sqrt(u_sq));
	/* the axis is perpendicular to both lines, so any pair of points measures the separation */
	const f32 separation = V3Dot(axis, V3Sub(tri[a], s1.p[0]));
	if (query->depth < separation)
	{
		query->depth = separation;
		query->normal = axis;
		query->s1 = s1;
		query->s2 = Segment(tri[a], tri[b]);
		query->e1 = e1;
		query->e2 = a;
	}
}

/* Edge query vs a triangle: every hull edge once against the triangle's three edges. Returns 1 at a separating axis. */
static u32 HullTriangleEdgeQuery(struct sat_EdgeQuery *query, const struct dcel *h, const v3 *v, const struct plane *pl, const v3 tri[3], const v3 n_t)
{
	for (u32 e1 = 0; e1 < h->e_count; ++e1)
	{
		if (h->e[e1].twin < e1) { continue; }
		for (u32 a = 0; a < 3; ++a)
		{
			HullTriangleEdgeCheck(query, h, v, pl, tri, n_t, e1, a);
			if (query->depth > 0.0f)
			{
				return 1;
			}
		}
	}

	return 0;
}

/* a separating axis for the SAT cache: normal from s0 to s1, separation > 0 */
static void sat_CacheSeparation(struct c_SatCache *cache, const v3 normal, const f32 separation)
{
	cache->type = SAT_CACHE_SEPARATION;
	cache->normal = normal;
	cache->depth = separation;
}

/* what s1 of a sat_Pair is */
enum sat_PairType
{
	SAT_PAIR_HULL,		/* a hull: Gauss map pruned edge pairs, axes oriented by center_0 */
	SAT_PAIR_TRIANGLE,	/* a mesh triangle (DcelTriStub, pl[1] = +-n_t): knife edges (HullTriangleEdgeCheck) */
};

/* a pair for the SAT: dcels, vertices and face planes in a common frame (s0's) */
struct sat_Pair
{
	enum sat_PairType	type;
	const struct dcel *	h[2];
	const v3 *			v[2];
	const struct plane *pl[2];
	v3					center_0;	/* s0's center of mass: edge axis orientation (SAT_PAIR_HULL) */
};

/*
 * SAT cache: re-check old_cache for the pair. A separating axis is reused while it separates; a contact is
 * rebuilt from its features (face: clipped on the cached face; edge pair: re-checked and its closest points) and reused
 * while its depth moved by less than sat_cache_tolerance. Returns 1 if reused: new_cache written, *colliding set, for a
 * contact also m and (face contacts) its features.
 */
static u32 sat_CacheTryReuse(struct c_Manifold *m, u32 *colliding, sat_FeatureId features[4], struct c_SatCache *new_cache, struct arena *mem_tmp, const struct sat_Pair *pair, const struct c_SatCache *old_cache)
{
	struct ds_DynamicsMetrics *metrics = &g_dynamics_worker[ds_ThreadSelfIndex()].metrics;
	const u32 triangle = (pair->type == SAT_PAIR_TRIANGLE);
	u32 *probe = (triangle) ? &metrics->mesh_cache_probe_count : &metrics->hull_cache_probe_count;
	u32 *eviction = (triangle) ? &metrics->mesh_cache_eviction_count : &metrics->hull_cache_eviction_count;
	const f32 tolerance = g_numerics_config->sat_cache_tolerance;
	const struct dcel *const *h = pair->h;
	const v3 *const *v = pair->v;
	const struct plane *const *pl = pair->pl;
	*probe += 1;
	*colliding = 0;

	switch (old_cache->type)
	{
		case SAT_CACHE_SEPARATION:
		{
			const v3 support_0 = v[0][V3Support(v[0], h[0]->v_count, old_cache->normal)];
			const v3 support_1 = v[1][V3Support(v[1], h[1]->v_count, V3Negate(old_cache->normal))];
			const f32 separation = V3Dot(support_1, old_cache->normal) - V3Dot(support_0, old_cache->normal);
			if (separation > 0.0f)
			{
				sat_CacheSeparation(new_cache, old_cache->normal, separation);
				return 1;
			}
		} break;

		case SAT_CACHE_CONTACT_EE:
		{
			struct sat_EdgeQuery e_query = { .depth = -F32_INFINITY, .e1 = U32_MAX };
			if (triangle)
			{
				HullTriangleEdgeCheck(&e_query, h[0], v[0], pl[0], v[1], pl[1][0].normal, sat_FeatureIdIndex(old_cache->feature[0]), sat_FeatureIdIndex(old_cache->feature[1]));
			}
			else
			{
				HullEdgeCheck(&e_query, h, v, pl, pair->center_0, sat_FeatureIdIndex(old_cache->feature[0]), sat_FeatureIdIndex(old_cache->feature[1]));
			}
			if (e_query.e1 != U32_MAX && e_query.depth < 0.0f && HullEdgeContact(m, new_cache, &e_query)
					&& F32Abs(new_cache->depth - old_cache->depth) < tolerance)
			{
				*colliding = 1;
				return 1;
			}
		} break;

		case SAT_CACHE_CONTACT_FV:
		{
			/* b_f = body with reference/contact face, b_v = incident body with penetrating vertices */
			const u32 b_f = sat_FeatureIdFaceCheck(old_cache->feature[1]);
			const u32 b_v = 1 - b_f;
			const u32 face = sat_FeatureIdIndex(old_cache->feature[b_f]);
			struct sat_FaceQuery face_query = { .fi = face, .normal = pl[b_f][face].normal };
			f32 depth;
			if (HullFaceContact(mem_tmp, m, features, &depth, new_cache->feature + b_v, &face_query, h, v, pl, b_f, 0)
					&& F32Abs(depth - old_cache->depth) < tolerance)
			{
				new_cache->type = SAT_CACHE_CONTACT_FV;
				new_cache->feature[b_f] = old_cache->feature[b_f];
				new_cache->depth = depth;
				new_cache->normal = face_query.normal;
				*colliding = 1;
				return 1;
			}
		} break;

		default:
		{
			ds_AssertString(0, "Shouldn't be reachable");
		} break;
	}

	*eviction += 1;
	*colliding = 0;
	return 0;
}

/*
 * SAT of a pair (Gregorius), the SAT cache tried first (old_cache NULL: none). Face queries of both shapes and the edge
 * query (SAT_PAIR_HULL: Gauss map pruned, axes oriented by center_0; SAT_PAIR_TRIANGLE: knife edges); separated if any
 * axis separates. Otherwise the face contact (incident face clipped to the reference face, the face with the larger
 * separation), replaced by the edge contact if the clip is empty or the edge separates more than the clipped face by
 * sat_face_bias. Returns 1 if colliding: m written, and for a face contact its points' features. cache: the
 * separating axis or the contact's features; SAT_CACHE_NONE for the fallback contact.
 */
static u32 sat_PairContact(struct c_Manifold *m, sat_FeatureId features[4], struct c_SatCache *cache, struct arena *mem_tmp, const struct sat_Pair *pair, const struct c_SatCache *old_cache)
{
	u32 colliding;
	if (old_cache && sat_CacheTryReuse(m, &colliding, features, cache, mem_tmp, pair, old_cache))
	{
		return colliding;
	}

	const struct dcel *const *h = pair->h;
	const v3 *const *v = pair->v;
	const struct plane *const *pl = pair->pl;
	struct sat_FaceQuery f_query[2] = { { .depth = -F32_INFINITY }, { .depth = -F32_INFINITY } };
	struct sat_EdgeQuery e_query = { .depth = -F32_INFINITY, .e1 = U32_MAX };

	if (HullFaceQuery(&f_query[0], pl[0], h[0]->f_count, v[1], h[1]->v_count))
	{
		sat_CacheSeparation(cache, f_query[0].normal, f_query[0].depth);
		return 0;
	}

	if (HullFaceQuery(&f_query[1], pl[1], h[1]->f_count, v[0], h[0]->v_count))
	{
		sat_CacheSeparation(cache, V3Negate(f_query[1].normal), f_query[1].depth);
		return 0;
	}

	const u32 edge_separates = (pair->type == SAT_PAIR_TRIANGLE)
		? HullTriangleEdgeQuery(&e_query, h[0], v[0], pl[0], v[1], pl[1][0].normal)
		: HullEdgeQuery(&e_query, h, v, pl, pair->center_0);
	if (edge_separates)
	{
		sat_CacheSeparation(cache, e_query.normal, e_query.depth);
		return 0;
	}

	/* face contact on the face with the larger separation; its clipped separation is -(deepest clipped depth) */
	const u32 b_f = (f_query[0].depth < f_query[1].depth);
	const u32 b_v = 1 - b_f;
	f32 max_depth;
	const u32 face_contact = HullFaceContact(mem_tmp, m, features, &max_depth, cache->feature + b_v, f_query + b_f, h, v, pl, b_f, 0);

	/* Edge contact if the face contact is empty or the edge separates more than the clipped face by sat_face_bias */
	if (e_query.e1 != U32_MAX && (!face_contact || -max_depth + g_numerics_config->sat_face_bias < e_query.depth)
			&& HullEdgeContact(m, cache, &e_query))
	{
		return 1;
	}

	if (face_contact)
	{
		cache->type = SAT_CACHE_CONTACT_FV;
		cache->feature[b_f] = sat_FeatureIdConstruct(f_query[b_f].fi, SAT_FEATURE_TYPE_FACE);
		cache->depth = max_depth;
		cache->normal = f_query[b_f].normal;
		return 1;
	}

	/*
	 * No face or edge contact (the clip came out empty, the edge's closest points outside the segments): the face
	 * query's deepest vertex of the incident shape and its projection onto the reference plane. Nothing to cache; the
	 * features are written as a face contact's (a mesh triangle's vertex waits: c_MeshHullTriDelayed).
	 */
	const struct sat_FaceQuery *fq = f_query + b_f;
	const v3 p = v[b_v][fq->vi];
	m->v_count = 1;
	m->depth[0] = F32Max(0.0f, -fq->depth);
	if (b_f == 0)
	{
		m->n = fq->normal;
		m->v[0] = V3Sub(p, V3Scale(fq->normal, fq->depth));
	}
	else
	{
		m->n = V3Negate(fq->normal);
		m->v[0] = p;
	}
	cache->type = SAT_CACHE_NONE;
	cache->feature[b_f] = sat_FeatureIdConstruct(fq->fi, SAT_FEATURE_TYPE_FACE);
	cache->feature[b_v] = sat_FeatureIdConstruct(fq->vi, SAT_FEATURE_TYPE_VERTEX);
	features[0] = cache->feature[b_v];
	return 1;
}

/*
 * Hull vs hull: local manifold in s0's frame (n from s0 to s1). s1's vertices and face planes are moved into
 * s0's frame; s0 uses its stored planes. SAT and cache: sat_PairContact.
 */
struct c_ContactResult c_HullContact(struct arena *frame, const struct c_ContactResult *cached_result, const struct c_Shape *s0, const ds_TransformWorld *t0, const struct c_Shape *s1, const ds_TransformWorld *t1)
{
	ds_Assert(s0->type == C_SHAPE_CONVEX_HULL);
	ds_Assert(s1->type == C_SHAPE_CONVEX_HULL);

    struct ds_DynamicsMetrics *metrics = &g_dynamics_worker[ds_ThreadSelfIndex()].metrics;
    metrics->hull_call_count += 1;

    struct c_ContactResult result = { 0 };
    result.cache_count = 1;
    result.cache = ArenaPushPacked(frame, sizeof(struct c_SatCache));
    result.manifold_count = 1;
    result.contact_manifold = ArenaPushPacked(frame, sizeof(struct c_Manifold));

    struct arena *mem_tmp = ArenaPushScratch();

    /* s1 in s0's frame: vertices and face planes */
    const ds_Transform t_1_to_0 = ds_TransformWorldRelative(t0, t1);
    const m3 rot = M3QUnit(t_1_to_0.rotation);
    const struct dcel *h[2] = { &s0->hull, &s1->hull };
    v3 *v_1 = ArenaPush(mem_tmp, h[1]->v_count * sizeof(v3));
    struct plane *pl_1 = ArenaPush(mem_tmp, h[1]->f_count * sizeof(struct plane));
	for (u32 i = 0; i < h[1]->v_count; ++i)
	{
		v_1[i] = V3Add(M3V3Mul(rot, h[1]->v[i]), t_1_to_0.position);
	}
	for (u32 fi = 0; fi < h[1]->f_count; ++fi)
	{
		pl_1[fi] = PlaneTransform(h[1]->plane + fi, rot, t_1_to_0.position);
	}
    const v3 *v[2] = { h[0]->v, v_1 };
    const struct plane *pl[2] = { h[0]->plane, pl_1 };
    const v3 center_0 = s0->center_of_mass;

    const struct sat_Pair pair = { .type = SAT_PAIR_HULL, .h = { h[0], h[1] }, .v = { v[0], v[1] }, .pl = { pl[0], pl[1] }, .center_0 = center_0 };
    const struct c_SatCache *old_cache = (cached_result->cache_count) ? cached_result->cache : NULL;
    sat_FeatureId features[4];
    const u32 colliding = sat_PairContact(result.contact_manifold, features, result.cache, mem_tmp, &pair, old_cache);
    if (result.cache->type == SAT_CACHE_NONE)
    {
        result.cache_count = 0;
    }

    if (!colliding)
    {
        ArenaPopPacked(frame, sizeof(struct c_Manifold));
        result.manifold_count = 0;
        result.contact_manifold = NULL;
    }
    ArenaPopScratch();
    return result;
}

/* a tested triangle (c_MeshContact) */
struct c_MeshTri
{
    u32                 tri;
    u32                 contact;        /* else (hulls) a cached separating axis */
    u32                 delayed_count;  /* 0: face contact; 1, 2: the triangle's vertices (0..2) of an edge/vertex contact */
    u32                 delayed[2];
    f32                 depth;          /* deepest point: the delayed phase goes deepest first */
    struct c_Manifold   manifold;       /* local to the convex shape */
    struct c_SatCache   cache;          /* hulls */
};

/*
 * Mesh vs convex shape, shared parts: c_MeshContactInit (pose, box, traversal), c_MeshContactNext (the next
 * candidate triangle, in the convex shape's frame), c_MeshContactPush (a record), c_MeshContactFinish (internal
 * edges, output). The shape loops in between test triangles against the convex shape as stored (sphere at the
 * origin, capsule along y, hull with its stored planes).
 */
struct c_MeshContactHelper
{
    const struct triMesh *      mesh;
    const struct triMeshBvh *   mesh_bvh;
    m3                          rot_inv;    /* convex_to_mesh's inverse: x_convex = rot_inv (x_mesh - pos) */
    v3                          pos;
    struct aabb                 bbox;       /* the convex shape's box in mesh space */
    struct aabb                 local;      /* the convex shape's box in its own frame */

    const struct bvhNode *      node;
    const struct bvhNode **     stack;
    u32                         stack_count;
    u32                         leaf_next;  /* the current leaf's remaining triangles: leaf_next .. leaf_last */
    u32                         leaf_last;

    struct arena *              tmp_records;
    struct arena *              tmp;
    struct c_MeshTri *          record;
    u32                         record_count;
    u32                         record_len;
};

static void c_MeshContactInit(struct c_MeshContactHelper *helper, const struct c_Shape *mesh, const ds_TransformWorld *t_mesh, const struct c_Shape *convex, const ds_TransformWorld *t_convex)
{
	ds_Assert(mesh->type == C_SHAPE_TRI_MESH);
	ds_Assert(convex->type < C_SHAPE_TRI_MESH);

    helper->mesh_bvh = &mesh->mesh_bvh;
    helper->mesh = mesh->mesh_bvh.mesh;

    /*
     * convex_to_mesh: x_mesh = rot x_convex + pos. Triangles use its inverse, rot^T (x_mesh - pos): subtracting
     * first, the rotation only rounds a small vector (rot^T x_mesh - rot^T pos would round the large x_mesh).
     */
    const ds_Transform convex_to_mesh = ds_TransformWorldRelative(t_mesh, t_convex);
    const m3 rot = M3QUnit(convex_to_mesh.rotation);
    helper->rot_inv = M3QUnit(QConj(convex_to_mesh.rotation));
    helper->pos = convex_to_mesh.position;

    /* the convex shape's local box, moved into mesh space: center rot c + pos, half widths |rot| e */
    struct aabb local;
    switch (convex->type)
    {
        case C_SHAPE_SPHERE:
        {
            const f32 r = convex->sphere.radius;
            local = (struct aabb) { .min = V3(-r, -r, -r), .max = V3(r, r, r) };
        } break;

        case C_SHAPE_CAPSULE:
        {
            const f32 r = convex->capsule.radius;
            const f32 h = convex->capsule.half_height + r;
            local = (struct aabb) { .min = V3(-r, -h, -r), .max = V3(r, h, r) };
        } break;

        default:
        {
            local = convex->hull.bbox;
        } break;
    }
    helper->local = local;
    const v3 center = V3Add(M3V3Mul(rot, V3Scale(V3Add(local.min, local.max), 0.5f)), helper->pos);
    const m3 rot_abs = M3Columns(V3Abs(rot.col[0]), V3Abs(rot.col[1]), V3Abs(rot.col[2]));
    const v3 half = M3V3Mul(rot_abs, V3Scale(V3Sub(local.max, local.min), 0.5f));
    helper->bbox = (struct aabb) { .min = V3Sub(center, half), .max = V3Add(center, half) };

    /* TODO(perf, audit E2): 2 scratch arenas here (records, the rest), c_MeshHullTriangles a third */
    helper->tmp_records = ArenaPushScratch();
    helper->tmp = ArenaPushScratch();
    helper->node = helper->mesh_bvh->bvh.pool.buf;
    helper->stack = ArenaPush(helper->tmp, (helper->mesh_bvh->depth + 1)*sizeof(struct bvhNode *));
    helper->stack_count = 0;
    helper->leaf_next = 1;
    helper->leaf_last = 0;
    const struct bvhNode *root = helper->node + helper->mesh_bvh->bvh.bt.root;
    if (AabbTest(&helper->bbox, &root->bbox))
    {
        helper->stack[helper->stack_count++] = root;
    }

    struct memArray records = ArenaPushAlignedAll(helper->tmp_records, sizeof(struct c_MeshTri), 8);
    helper->record = records.addr;
    helper->record_count = 0;
    helper->record_len = records.len;
}

/*
 * The next triangle that can touch the convex shape: in a leaf whose box overlaps the shape's (mesh space), with its own
 * box, in the shape's frame, overlapping the shape's local box (exact: the shape lies inside its box; touching kept).
 * Returns its index and vertices in the convex shape's frame.
 */
static u32 c_MeshContactNext(struct c_MeshContactHelper *helper, u32 *tri_index, v3 tri[3])
{
    for (;;)
    {
        while (helper->leaf_next > helper->leaf_last)
        {
            if (helper->stack_count == 0)
            {
                return 0;
            }

            const struct bvhNode *node = helper->stack[--helper->stack_count];
            if (BTLeafCheck(node))
            {
                helper->leaf_next = node->bt_child[0];
                helper->leaf_last = node->bt_child[0] + node->bt_child[1] - 1;
                continue;
            }

            const struct bvhNode *left = helper->node + node->bt_child[0];
            const struct bvhNode *right = helper->node + node->bt_child[1];
            if (AabbTest(&helper->bbox, &right->bbox))
            {
                ds_Assert(helper->stack_count < helper->mesh_bvh->depth + 1);
                helper->stack[helper->stack_count++] = right;
            }
            if (AabbTest(&helper->bbox, &left->bbox))
            {
                ds_Assert(helper->stack_count < helper->mesh_bvh->depth + 1);
                helper->stack[helper->stack_count++] = left;
            }
        }

        *tri_index = helper->mesh_bvh->tri[helper->leaf_next++];
        const u32 *v = helper->mesh->tri[*tri_index].buf;
        for (u32 i = 0; i < 3; ++i)
        {
            tri[i] = M3V3Mul(helper->rot_inv, V3Sub(helper->mesh->v[v[i]], helper->pos));
        }

        /* in the shape's frame its box is exact (a leaf's box in mesh space bounds the rotated box: looser) */
        const struct aabb tri_bbox = { .min = V3Min(V3Min(tri[0], tri[1]), tri[2]), .max = V3Max(V3Max(tri[0], tri[1]), tri[2]) };
        if (AabbTest(&tri_bbox, &helper->local))
        {
            return 1;
        }
    }
}

static struct c_MeshTri *c_MeshContactPush(struct c_MeshContactHelper *helper, const u32 tri_index)
{
    if (helper->record_count == helper->record_len)
    {
        Log(T_SYSTEM, S_FATAL, "Out of memory in %s\n", __func__);
        FatalCleanupAndExit();
    }

    struct c_MeshTri *r = helper->record + helper->record_count++;
    r->tri = tri_index;
    r->contact = 0;
    r->delayed_count = 0;
    r->depth = 0.0f;
    return r;
}

/* the delayed vertices of a feature given as a mask of the triangle's vertices (enum voronoi; face: none) */
static void c_MeshTriRegionMaskDelayed(struct c_MeshTri *r, const u32 mask)
{
    if (mask != VORONOI_F012)
    {
        for (u32 i = 0; i < 3; ++i)
        {
            if (mask & (1u << i))
            {
                r->delayed[r->delayed_count++] = i;
            }
        }
    }
}

static void c_MeshSphereTriangles(struct c_MeshContactHelper *helper, const struct sphere *sphere)
{
    const f32 radius = sphere->radius;
    const f32 touch_tolerance = GJK_TOUCH_TOLERANCE * F32_EPSILON;
    u32 tri_index;
    v3 tri[3];
    while (c_MeshContactNext(helper, &tri_index, tri))
    {
        /*
         * as c_HullSphereContact, two-sided: the triangle is a flat hull with faces +-n_t; center = origin.
         * Triangles have area (c_Shape precondition), so VORONOI_INVALID (zero |e|^2 or |n|^2, NaN) and n_t = 0
         * are only asserted.
         */
        f32 w[3];
        const enum voronoi region = TriOriginVoronoi(w, tri[0], tri[1], tri[2]);
        ds_AssertString(region != VORONOI_INVALID, "mesh triangle without area or NaN input (c_Shape precondition)");

        const v3 c = V3Add(V3Add(V3Scale(tri[0], w[0]), V3Scale(tri[1], w[1])), V3Scale(tri[2], w[2]));
        const f32 distance_sq = V3LengthSquared(c);
        if (radius*radius < distance_sq)
        {
            continue;
        }

        v3 n;
        f32 depth;
        /* GJK's touch band, L = max |t_i| (the sphere's center is the origin) */
        const f32 l_sq = F32Max(F32Max(V3LengthSquared(tri[0]), V3LengthSquared(tri[1])), V3LengthSquared(tri[2]));
        /* shallow penetration */
        if (touch_tolerance*touch_tolerance*l_sq < distance_sq)
        {
            const f32 distance = F32Sqrt(distance_sq);
            n = V3Scale(c, 1.0f / distance);
            depth = radius - distance;
        }
        /* deep penetration: the face the center is in front of (on the plane: +n_t) */
        else
        {
            v3 n_t;
            const u32 has_area = TriCcwNormalTryCompute(&n_t, tri[0], tri[1], tri[2]);
            ds_AssertString(has_area, "mesh triangle without area (c_Shape precondition)");
            const f32 distance = -V3Dot(n_t, tri[0]);
            n = (0.0f <= distance) ? V3Scale(n_t, -1.0f) : n_t;
            depth = F32Max(0.0f, radius - F32Abs(distance));
        }

        struct c_MeshTri *r = c_MeshContactPush(helper, tri_index);
        struct c_Manifold *m = &r->manifold;
        m->v_count = 1;
        m->n = n;
        m->v[0] = V3Scale(n, radius);
        m->depth[0] = depth;
        r->contact = 1;
        r->depth = depth;
        /* edge/vertex regions wait for the internal-edge filter */
        c_MeshTriRegionMaskDelayed(r, region);
    }
}

/*
 * DcelFaceSegmentClip for triangle t, CCW about n (any length): s clipped to the prism over t, inclusive. Returns 0 if
 * nothing is left. Kept inline here: the general clip through a triangle stub measured 0.3% slower for capsule-mesh.
 */
static u32 c_TriSegmentSideClip(struct segment *clip, const v3 t[3], const v3 n, const struct segment *s)
{
    f32 t_min = 0.0f;
    f32 t_max = 1.0f;
    if (!SegmentEdgeSideClip(&t_min, &t_max, s, t[0], t[1], n)
        || !SegmentEdgeSideClip(&t_min, &t_max, s, t[1], t[2], n)
        || !SegmentEdgeSideClip(&t_min, &t_max, s, t[2], t[0], n)
        || t_max < t_min)
    {
        return 0;
    }

    *clip = Segment(SegmentBc(s, t_min), SegmentBc(s, t_max));
    return 1;
}

/*
 * Capsule vs triangle (two-sided), tailored from c_HullCapsuleContact: the triangle is a flat hull with faces
 * +-n_t and three knife edges, and TriSegmentVoronoi replaces GJK. In the capsule's frame; the manifold follows the
 * mesh convention (n from the capsule to the mesh, v on the capsule's surface).
 */
static void c_MeshCapsuleTriangles(struct c_MeshContactHelper *helper, const struct capsule *capsule)
{
    const f32 radius = capsule->radius;
    /* the core in the capsule's frame (SegmentCapsuleTransform's order) */
    const struct segment core = Segment(V3(0.0f, capsule->half_height, 0.0f), V3(0.0f, -capsule->half_height, 0.0f));
    const f32 touch_tolerance = GJK_TOUCH_TOLERANCE * F32_EPSILON;
    u32 tri_index;
    v3 tri[3];
    while (c_MeshContactNext(helper, &tri_index, tri))
    {
        f32 w[3], s;
        const enum voronoi region = TriSegmentVoronoi(w, &s, tri[0], tri[1], tri[2], core.p[0], core.p[1]);
        ds_AssertString(region != VORONOI_INVALID, "TriSegmentVoronoi: NaN input or distances beyond f32");

        const v3 c_t = V3Add(V3Add(V3Scale(tri[0], w[0]), V3Scale(tri[1], w[1])), V3Scale(tri[2], w[2]));
        const v3 c_s = SegmentBc(&core, s);
        const f32 distance_sq = V3DistanceSquared(c_t, c_s);
        if (radius*radius < distance_sq)
        {
            continue;
        }

        v3 n_t;
        const u32 has_area = TriCcwNormalTryCompute(&n_t, tri[0], tri[1], tri[2]);
        ds_AssertString(has_area, "mesh triangle without area (c_Shape precondition)");

        struct c_MeshTri *r = c_MeshContactPush(helper, tri_index);
        struct c_Manifold *m = &r->manifold;
        r->contact = 1;
        u32 delayed_region = region;

        /* GJK's touch band, L = max of |t_i| and the core's half height (the capsule's center is the origin) */
        const f32 l_sq = F32Max(F32Max(F32Max(V3LengthSquared(tri[0]), V3LengthSquared(tri[1])), V3LengthSquared(tri[2])),
                                capsule->half_height*capsule->half_height);
        /* shallow penetration */
        if (touch_tolerance*touch_tolerance*l_sq < distance_sq)
        {
            const f32 distance = F32Sqrt(distance_sq);
            const v3 n = V3Scale(V3Sub(c_t, c_s), 1.0f / distance);

            /*
             * Two points if n is within face_parallel_max_degrees of the face normal facing the capsule (f, from the
             * triangle to the capsule: -n ~ f) and both ends of the clipped core touch (d_i <= r); otherwise one
             * point. Each end has its own depth. As for hulls, the closest feature may be an edge (a core parallel
             * to the face is as close to it as to the edge it crosses).
             */
            const v3 f = (V3Dot(n, n_t) < 0.0f) ? n_t : V3Scale(n_t, -1.0f);
            struct segment clip;
            if (g_numerics_config->face_parallel_cos < -V3Dot(n, f)
                && c_TriSegmentSideClip(&clip, tri, n_t, &core))
            {
                const f32 d0 = V3Dot(f, V3Sub(clip.p[0], tri[0]));
                const f32 d1 = V3Dot(f, V3Sub(clip.p[1], tri[0]));
                if (d0 <= radius && d1 <= radius)
                {
                    m->v_count = 2;
                    m->n = V3Scale(f, -1.0f);
                    m->v[0] = V3Add(clip.p[0], V3Scale(m->n, radius));
                    m->v[1] = V3Add(clip.p[1], V3Scale(m->n, radius));
                    m->depth[0] = radius - d0;
                    m->depth[1] = radius - d1;
                    r->depth = c_ManifoldDeepest(m);
                    continue;
                }
            }

            m->v_count = 1;
            m->n = n;
            m->v[0] = V3Add(c_s, V3Scale(n, radius));
            m->depth[0] = radius - distance;
            r->depth = m->depth[0];
            c_MeshTriRegionMaskDelayed(r, delayed_region);
            continue;
        }

        /* deep penetration (the core touches the triangle): faces +-n_t, the core's deeper end */
        const f32 h0 = V3Dot(n_t, V3Sub(core.p[0], tri[0]));
        const f32 h1 = V3Dot(n_t, V3Sub(core.p[1], tri[0]));
        /* front: min(h0, h1); back: min(-h0, -h1); the larger separates more (tie: front) */
        const v3 f = (-F32Max(h0, h1) <= F32Min(h0, h1)) ? n_t : V3Scale(n_t, -1.0f);

        /* face contact: the core clipped to the triangle; empty if nothing is left or no clipped end touches */
        struct segment clip = core;
        const u32 clipped = c_TriSegmentSideClip(&clip, tri, n_t, &core);
        const f32 depth[2] =
        {
            V3Dot(f, V3Sub(clip.p[0], tri[0])),
            V3Dot(f, V3Sub(clip.p[1], tri[0])),
        };
        const u32 deeper = (depth[0] <= depth[1]) ? 0 : 1;
        const u32 face_contact = clipped && depth[deeper] <= radius;

        /*
         * Edges: a knife edge's Gauss map arc is the half circle from n_t through its outward normal m_e to -n_t, so
         * every edge not parallel to the core is a candidate; the axis cross(e, core) points out along m_e.
         */
        f32 edge_separation = -F32_INFINITY;
        u32 edge = U32_MAX;
        v3 edge_axis = V3(0.0f, 0.0f, 0.0f);
        for (u32 a = 0; a < 3; ++a)
        {
            const u32 b = (a + 1) % 3;
            const v3 e = V3Sub(tri[b], tri[a]);
            const v3 cross = V3Cross(e, core.dir);
            const f32 cross_sq = V3Dot(cross, cross);
            if (cross_sq <= g_numerics_config->edge_parallel_sin_sq*V3Dot(e, e)*V3Dot(core.dir, core.dir))
            {
                continue;
            }

            v3 axis = V3Scale(cross, 1.0f / F32Sqrt(cross_sq));
            if (V3Dot(axis, V3Cross(e, n_t)) < 0.0f)
            {
                axis = V3Scale(axis, -1.0f);
            }
            /* the axis is perpendicular to both lines, so any pair of points measures the separation */
            const f32 separation = V3Dot(axis, V3Sub(core.p[0], tri[a]));
            if (edge_separation < separation)
            {
                edge_separation = separation;
                edge = a;
                edge_axis = axis;
            }
        }

        /* edge contact as in c_HullCapsuleContact: within both segments, separating more by sat_face_bias */
        if (edge != U32_MAX && (!face_contact || depth[deeper] + g_numerics_config->sat_face_bias < edge_separation))
        {
            const struct segment e = Segment(tri[edge], tri[(edge + 1) % 3]);
            f32 t_edge, t_core;
            if (SegmentLineClosestParameterCheck(&t_edge, &t_core, &e, &core))
            {
                m->v_count = 1;
                m->n = V3Scale(edge_axis, -1.0f);
                m->v[0] = V3Add(SegmentBc(&core, t_core), V3Scale(m->n, radius));
                m->depth[0] = F32Max(0.0f, radius - edge_separation);
                r->depth = m->depth[0];
                c_MeshTriRegionMaskDelayed(r, (1u << edge) | (1u << ((edge + 1) % 3)));
                continue;
            }
        }

        m->n = V3Scale(f, -1.0f);

        /* as in the shallow path: two points if both clipped ends touch (d_i <= r), else the deeper end */
        if (face_contact)
        {
            if (depth[1 - deeper] <= radius)
            {
                m->v_count = 2;
                m->v[0] = V3Add(clip.p[0], V3Scale(m->n, radius));
                m->v[1] = V3Add(clip.p[1], V3Scale(m->n, radius));
                m->depth[0] = radius - depth[0];
                m->depth[1] = radius - depth[1];
                r->depth = c_ManifoldDeepest(m);
                continue;
            }

            m->v_count = 1;
            m->v[0] = V3Add(clip.p[deeper], V3Scale(m->n, radius));
            m->depth[0] = radius - depth[deeper];
            r->depth = m->depth[0];
            continue;
        }

        /*
         * No face or edge contact: only when the core touches the triangle within GJK's touch band (the clip can be
         * empty and the realizing edge skipped as parallel). Touching: depth r at the closest core point.
         */
        m->v_count = 1;
        m->v[0] = V3Add(c_s, V3Scale(m->n, radius));
        m->depth[0] = radius;
        r->depth = radius;
        c_MeshTriRegionMaskDelayed(r, delayed_region);
    }
}

/*
 * The delayed vertices of a hull-triangle contact (the cache tells which): an edge contact waits on the triangle edge's
 * vertices; a face contact on the hull's face (the triangle incident) on the triangle features in the manifold (a
 * vertex, or an edge's two vertices; three or more points, or all three vertices: none); a face contact on the
 * triangle's face does not wait. The fallback contact (SAT_CACHE_NONE) as a face contact.
 */
static void c_MeshHullTriDelayed(struct c_MeshTri *r, const struct dcel *stub, const sat_FeatureId features[4])
{
    if (r->cache.type == SAT_CACHE_CONTACT_EE)
    {
        const u32 a = sat_FeatureIdIndex(r->cache.feature[1]);
        c_MeshTriRegionMaskDelayed(r, (1u << a) | (1u << ((a + 1) % 3)));
    }
    else if ((r->cache.type == SAT_CACHE_CONTACT_FV || r->cache.type == SAT_CACHE_NONE)
            && !sat_FeatureIdFaceCheck(r->cache.feature[1]) && r->manifold.v_count < 3)
    {
        u32 mask = 0;
        for (u32 i = 0; i < r->manifold.v_count; ++i)
        {
            const u32 index = sat_FeatureIdIndex(features[i]);
            mask |= (sat_FeatureIdVertexCheck(features[i]))
                ? 1u << index
                : (1u << stub->e[index].origin) | (1u << stub->e[stub->e[index].twin].origin);
        }
        c_MeshTriRegionMaskDelayed(r, mask);
    }
}

/*
 * Hull vs triangle (two-sided): sat_PairContact with the hull as s0 (as stored) and the triangle as s1
 * (SAT_PAIR_TRIANGLE: a DcelTriStub with planes +-n_t, in the hull's frame): n from the hull to the mesh, v on the hull
 * (the mesh convention). Every tested triangle keeps a record: its contact, or its separating axis for the cache.
 */
static void c_MeshHullTriangles(struct c_MeshContactHelper *helper, const struct c_ContactResult *cached_result, const struct dcel *hull)
{
    struct ds_DynamicsMetrics *metrics = &g_dynamics_worker[ds_ThreadSelfIndex()].metrics;
    u32 tri_index;
    v3 tri[3];
    while (c_MeshContactNext(helper, &tri_index, tri))
    {
        metrics->mesh_call_count += 1;
        v3 n_t;
        const u32 has_area = TriCcwNormalTryCompute(&n_t, tri[0], tri[1], tri[2]);
        ds_AssertString(has_area, "mesh triangle without area (c_Shape precondition)");

        /* the triangle as s1: face 0 ccw about n_t, face 1 its back */
        struct plane pl_tri[2] = { Plane(n_t, tri[0]), Plane(V3Negate(n_t), tri[0]) };
        struct dcel stub = DcelTriStub();
        stub.v = tri;
        stub.plane = pl_tri;
        /* center_0 orients HullEdgeCheck's axes; the triangle's edge check needs none */
        const struct sat_Pair pair = { .type = SAT_PAIR_TRIANGLE, .h = { hull, &stub }, .v = { hull->v, tri }, .pl = { hull->plane, pl_tri }, .center_0 = V3(0.0f, 0.0f, 0.0f) };

        /* the triangle's cache: a linear search over at most cache_count_max entries (small next to the SAT) */
        const struct c_SatCache *old_cache = NULL;
        for (u32 ci = 0; ci < cached_result->cache_count; ++ci)
        {
            if (cached_result->cache[ci].tri == tri_index)
            {
                old_cache = cached_result->cache + ci;
                break;
            }
        }

        struct c_MeshTri *r = c_MeshContactPush(helper, tri_index);
        struct arena *mem_tmp = ArenaPushScratch();
        sat_FeatureId features[4];
        const u32 colliding = sat_PairContact(&r->manifold, features, &r->cache, mem_tmp, &pair, old_cache);
        ArenaPopScratch();
        if (colliding)
        {
            c_MeshHullTriDelayed(r, &stub, features);
        }

        r->cache.tri = tri_index;
        r->contact = colliding;
        r->depth = (colliding) ? c_ManifoldDeepest(&r->manifold) : 0.0f;
    }
}

/* One manifold per record that survives the internal-edge filter; caches: every record's (hulls) */
static void c_MeshContactFinish(struct arena *frame, struct c_ContactResult *result, struct c_MeshContactHelper *helper, const u32 cache)
{
    u32 contact_count = 0;
    u32 delayed_count = 0;
    for (u32 i = 0; i < helper->record_count; ++i)
    {
        contact_count += helper->record[i].contact;
        delayed_count += (helper->record[i].contact && helper->record[i].delayed_count);
    }

    const u32 cache_len = (cache) ? ((helper->record_count < g_numerics_config->cache_count_max) ? helper->record_count : g_numerics_config->cache_count_max) : 0;
    result->cache = (cache_len) ? ArenaPushPacked(frame, cache_len*sizeof(struct c_SatCache)) : NULL;
    result->contact_manifold = ArenaPush(frame, contact_count*sizeof(struct c_Manifold));
    result->tri = ArenaPush(frame, contact_count*sizeof(u32));
    u32 *delayed = ArenaPush(helper->tmp, delayed_count*sizeof(u32));
    struct ds_BitSet void_bitset = ds_BitSetAlloc(helper->tmp, helper->mesh->v_count, 0, NOT_GROWABLE);
    if ((contact_count && (!result->contact_manifold || !result->tri)) || (delayed_count && !delayed) || (cache_len && !result->cache))
    {
        Log(T_SYSTEM, S_FATAL, "Out of memory in %s\n", __func__);
        FatalCleanupAndExit();
    }

    /*
     * internal edges (TODO(decision): keep this void/delayed scheme or Box3D's): face contacts are emitted and void
     * their 3 vertices; edge/vertex contacts wait, deepest first, and are dropped if their vertices are voided.
     * TODO(perf, audit C5): insertion sorts (here, c_ContactResultSortTriangles).
     */
    u32 delayed_n = 0;
    for (u32 i = 0; i < helper->record_count; ++i)
    {
        const struct c_MeshTri *r = helper->record + i;
        if (!r->contact)
        {
            if (result->cache_count < cache_len && r->cache.type != SAT_CACHE_NONE)
            {
                result->cache[result->cache_count++] = r->cache;
            }
            continue;
        }

        if (r->delayed_count)
        {
            u32 d = delayed_n++;
            for (; d && helper->record[delayed[d-1]].depth < r->depth; --d)
            {
                delayed[d] = delayed[d-1];
            }
            delayed[d] = i;
            continue;
        }

        result->contact_manifold[result->manifold_count] = r->manifold;
        result->tri[result->manifold_count] = r->tri;
        result->manifold_count += 1;
        if (result->cache_count < cache_len && r->cache.type != SAT_CACHE_NONE)
        {
            result->cache[result->cache_count++] = r->cache;
        }
        const u32 *v = helper->mesh->tri[r->tri].buf;
        ds_BitSetSet(&void_bitset, v[0], 1);
        ds_BitSetSet(&void_bitset, v[1], 1);
        ds_BitSetSet(&void_bitset, v[2], 1);
    }

    for (u32 i = 0; i < delayed_n; ++i)
    {
        const struct c_MeshTri *r = helper->record + delayed[i];
        const u32 *v = helper->mesh->tri[r->tri].buf;
        const u32 voided = (r->delayed_count == 1)
            ? ds_BitSetGet(&void_bitset, v[ r->delayed[0] ])
            : ds_BitSetGet(&void_bitset, v[ r->delayed[0] ]) && ds_BitSetGet(&void_bitset, v[ r->delayed[1] ]);
        if (!voided)
        {
            result->contact_manifold[result->manifold_count] = r->manifold;
            result->tri[result->manifold_count] = r->tri;
            result->manifold_count += 1;
            if (result->cache_count < cache_len && r->cache.type != SAT_CACHE_NONE)
            {
                result->cache[result->cache_count++] = r->cache;
            }
        }
        ds_BitSetSet(&void_bitset, v[0], 1);
        ds_BitSetSet(&void_bitset, v[1], 1);
        ds_BitSetSet(&void_bitset, v[2], 1);
    }

    ArenaPopScratch();
    ArenaPopScratch();
    c_ContactResultSortTriangles(frame, result);
}

/*
 * Mesh vs convex shape (sphere, capsule, hull): one manifold per contacting triangle, local to the convex shape (n from
 * it to the mesh, v on its surface).
 */
struct c_ContactResult c_MeshContact(struct arena *frame, const struct c_ContactResult *cached_result, const struct c_Shape *mesh, const ds_TransformWorld *t_mesh, const struct c_Shape *convex, const ds_TransformWorld *t_convex)
{
    ds_Assert(convex->type < C_SHAPE_TRI_MESH);
    struct c_ContactResult result = { 0 };
    struct c_MeshContactHelper helper;
    c_MeshContactInit(&helper, mesh, t_mesh, convex, t_convex);
    switch (convex->type)
    {
        case C_SHAPE_SPHERE:    { c_MeshSphereTriangles(&helper, &convex->sphere); } break;
        case C_SHAPE_CAPSULE:   { c_MeshCapsuleTriangles(&helper, &convex->capsule); } break;
        default:                { c_MeshHullTriangles(&helper, cached_result, &convex->hull); } break;
    }
    c_MeshContactFinish(frame, &result, &helper, convex->type == C_SHAPE_CONVEX_HULL);
    return result;
}
