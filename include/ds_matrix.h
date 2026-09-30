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

#ifndef __DS_MATRIX_H__
#define __DS_MATRIX_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

#include "ds_define.h"
#include "ds_types.h"
#include "ds_float.h"
#include "ds_vector.h"

static inline void M2Print(const char *text, const m2 m)
{
	printf("%s:\n| %f %f |\n| %f %f |\n", text,
		m.col[0].x, m.col[1].x,
		m.col[0].y, m.col[1].y);
}

static inline void M3Print(const char *text, const m3 m)
{
	printf("%s:\n| %f %f %f |\n| %f %f %f |\n| %f %f %f |\n", text,
		m.col[0].x, m.col[1].x, m.col[2].x,
		m.col[0].y, m.col[1].y, m.col[2].y,
		m.col[0].z, m.col[1].z, m.col[2].z);
}

static inline void M4Print(const char *text, const m4 m)
{
	printf("%s:\n| %f %f %f %f |\n| %f %f %f %f |\n| %f %f %f %f |\n| %f %f %f %f |\n", text,
		m.col[0].x, m.col[1].x, m.col[2].x, m.col[3].x,
		m.col[0].y, m.col[1].y, m.col[2].y, m.col[3].y,
		m.col[0].z, m.col[1].z, m.col[2].z, m.col[3].z,
		m.col[0].w, m.col[1].w, m.col[2].w, m.col[3].w);
}

static ds_ForceInline m2 M2Identity(void)
{
	return M2(1.0f, 0.0f,
		  0.0f, 1.0f);
}

static ds_ForceInline m3 M3Identity(void)
{
	return M3(1.0f, 0.0f, 0.0f,
		  0.0f, 1.0f, 0.0f,
		  0.0f, 0.0f, 1.0f);
}

static ds_ForceInline m4 M4Identity(void)
{
	return M4(1.0f, 0.0f, 0.0f, 0.0f,
		  0.0f, 1.0f, 0.0f, 0.0f,
		  0.0f, 0.0f, 1.0f, 0.0f,
		  0.0f, 0.0f, 0.0f, 1.0f);
}

static ds_ForceInline m2 M2Columns(const v2 c1, const v2 c2)
{
	return (m2) { .col = { c1, c2 } };
}

static ds_ForceInline m3 M3Columns(const v3 c1, const v3 c2, const v3 c3)
{
	return (m3) { .col = { c1, c2, c3 } };
}

static ds_ForceInline m4 M4Columns(const v4 c1, const v4 c2, const v4 c3, const v4 c4)
{
	return (m4) { .col = { c1, c2, c3, c4 } };
}

static ds_ForceInline m2 M2Rows(const v2 r1, const v2 r2)
{
	return M2(r1.x, r2.x,
		  r1.y, r2.y);
}

static ds_ForceInline m3 M3Rows(const v3 r1, const v3 r2, const v3 r3)
{
	return M3(r1.x, r2.x, r3.x,
		  r1.y, r2.y, r3.y,
		  r1.z, r2.z, r3.z);
}

static ds_ForceInline m4 M4Rows(const v4 r1, const v4 r2, const v4 r3, const v4 r4)
{
	return M4(r1.x, r2.x, r3.x, r4.x,
		  r1.y, r2.y, r3.y, r4.y,
		  r1.z, r2.z, r3.z, r4.z,
		  r1.w, r2.w, r3.w, r4.w);
}

/* vec*mat */
static ds_ForceInline v2 V2M2Mul(const v2 v, const m2 m)
{
	return V2(V2Dot(m.col[0], v),
		  V2Dot(m.col[1], v));
}

static ds_ForceInline v3 V3M3Mul(const v3 v, const m3 m)
{
	return V3(V3Dot(m.col[0], v),
		  V3Dot(m.col[1], v),
		  V3Dot(m.col[2], v));
}

static ds_ForceInline v4 V4M4Mul(const v4 v, const m4 m)
{
	return V4(V4Dot(m.col[0], v),
		  V4Dot(m.col[1], v),
		  V4Dot(m.col[2], v),
		  V4Dot(m.col[3], v));
}

/* mat*vec */
static ds_ForceInline v2 M2V2Mul(const m2 m, const v2 v)
{
	return V2(v.x * m.col[0].x + v.y * m.col[1].x,
		  v.x * m.col[0].y + v.y * m.col[1].y);
}

static ds_ForceInline v3 M3V3Mul(const m3 m, const v3 v)
{
	return V3(v.x * m.col[0].x + v.y * m.col[1].x + v.z * m.col[2].x,
		  v.x * m.col[0].y + v.y * m.col[1].y + v.z * m.col[2].y,
		  v.x * m.col[0].z + v.y * m.col[1].z + v.z * m.col[2].z);
}

static ds_ForceInline v4 M4V4Mul(const m4 m, const v4 v)
{
	return V4(v.x * m.col[0].x + v.y * m.col[1].x + v.z * m.col[2].x + v.w * m.col[3].x,
		  v.x * m.col[0].y + v.y * m.col[1].y + v.z * m.col[2].y + v.w * m.col[3].y,
		  v.x * m.col[0].z + v.y * m.col[1].z + v.z * m.col[2].z + v.w * m.col[3].z,
		  v.x * m.col[0].w + v.y * m.col[1].w + v.z * m.col[2].w + v.w * m.col[3].w);
}

/* a + b */
static ds_ForceInline m2 M2Add(const m2 a, const m2 b)
{
	return M2Columns(V2Add(a.col[0], b.col[0]),
			 V2Add(a.col[1], b.col[1]));
}

static ds_ForceInline m3 M3Add(const m3 a, const m3 b)
{
	return M3Columns(V3Add(a.col[0], b.col[0]),
			 V3Add(a.col[1], b.col[1]),
			 V3Add(a.col[2], b.col[2]));
}

static ds_ForceInline m4 M4Add(const m4 a, const m4 b)
{
	return M4Columns(V4Add(a.col[0], b.col[0]),
			 V4Add(a.col[1], b.col[1]),
			 V4Add(a.col[2], b.col[2]),
			 V4Add(a.col[3], b.col[3]));
}

/* a - b */
static ds_ForceInline m2 M2Sub(const m2 a, const m2 b)
{
	return M2Columns(V2Sub(a.col[0], b.col[0]),
			 V2Sub(a.col[1], b.col[1]));
}

static ds_ForceInline m3 M3Sub(const m3 a, const m3 b)
{
	return M3Columns(V3Sub(a.col[0], b.col[0]),
			 V3Sub(a.col[1], b.col[1]),
			 V3Sub(a.col[2], b.col[2]));
}

static ds_ForceInline m4 M4Sub(const m4 a, const m4 b)
{
	return M4Columns(V4Sub(a.col[0], b.col[0]),
			 V4Sub(a.col[1], b.col[1]),
			 V4Sub(a.col[2], b.col[2]),
			 V4Sub(a.col[3], b.col[3]));
}

/* m*scale */
static ds_ForceInline m2 M2Scale(const m2 m, const f32 scale)
{
	return M2Columns(V2Scale(m.col[0], scale),
			 V2Scale(m.col[1], scale));
}

static ds_ForceInline m3 M3Scale(const m3 m, const f32 scale)
{
	return M3Columns(V3Scale(m.col[0], scale),
			 V3Scale(m.col[1], scale),
			 V3Scale(m.col[2], scale));
}

static ds_ForceInline m4 M4Scale(const m4 m, const f32 scale)
{
	return M4Columns(V4Scale(m.col[0], scale),
			 V4Scale(m.col[1], scale),
			 V4Scale(m.col[2], scale),
			 V4Scale(m.col[3], scale));
}

/* a*b */
static ds_ForceInline m2 M2Mul(const m2 a, const m2 b)
{
	return M2Columns(M2V2Mul(a, b.col[0]),
			 M2V2Mul(a, b.col[1]));
}

static ds_ForceInline m3 M3Mul(const m3 a, const m3 b)
{
	return M3Columns(M3V3Mul(a, b.col[0]),
			 M3V3Mul(a, b.col[1]),
			 M3V3Mul(a, b.col[2]));
}

static ds_ForceInline m4 M4Mul(const m4 a, const m4 b)
{
	return M4Columns(M4V4Mul(a, b.col[0]),
			 M4V4Mul(a, b.col[1]),
			 M4V4Mul(a, b.col[2]),
			 M4V4Mul(a, b.col[3]));
}

/* m^T */
static ds_ForceInline m2 M2Transpose(const m2 m)
{
	return M2Rows(m.col[0], m.col[1]);
}

static ds_ForceInline m3 M3Transpose(const m3 m)
{
	return M3Rows(m.col[0], m.col[1], m.col[2]);
}

static ds_ForceInline m4 M4Transpose(const m4 m)
{
	return M4Rows(m.col[0], m.col[1], m.col[2], m.col[3]);
}

/* min |a_ij| */
static ds_ForceInline f32 M2AbsMin(const m2 m)
{
	const f32 a1 = F32Min(F32Abs(m.col[0].x), F32Abs(m.col[0].y));
	const f32 a2 = F32Min(F32Abs(m.col[1].x), F32Abs(m.col[1].y));
	return F32Min(a1, a2);
}

static ds_ForceInline f32 M3AbsMin(const m3 m)
{
	const f32 a1 = F32Min(F32Abs(m.col[0].x), F32Min(F32Abs(m.col[0].y), F32Abs(m.col[0].z)));
	const f32 a2 = F32Min(F32Abs(m.col[1].x), F32Min(F32Abs(m.col[1].y), F32Abs(m.col[1].z)));
	const f32 a3 = F32Min(F32Abs(m.col[2].x), F32Min(F32Abs(m.col[2].y), F32Abs(m.col[2].z)));
	return F32Min(a1, F32Min(a2, a3));
}

static ds_ForceInline f32 M4AbsMin(const m4 m)
{
	const f32 a1 = F32Min(F32Abs(m.col[0].x), F32Min(F32Abs(m.col[0].y), F32Min(F32Abs(m.col[0].z), F32Abs(m.col[0].w))));
	const f32 a2 = F32Min(F32Abs(m.col[1].x), F32Min(F32Abs(m.col[1].y), F32Min(F32Abs(m.col[1].z), F32Abs(m.col[1].w))));
	const f32 a3 = F32Min(F32Abs(m.col[2].x), F32Min(F32Abs(m.col[2].y), F32Min(F32Abs(m.col[2].z), F32Abs(m.col[2].w))));
	const f32 a4 = F32Min(F32Abs(m.col[3].x), F32Min(F32Abs(m.col[3].y), F32Min(F32Abs(m.col[3].z), F32Abs(m.col[3].w))));
	return F32Min(a1, F32Min(a2, F32Min(a3, a4)));
}

/* max |a_ij| */
static ds_ForceInline f32 M2AbsMax(const m2 m)
{
	const f32 a1 = F32Max(F32Abs(m.col[0].x), F32Abs(m.col[0].y));
	const f32 a2 = F32Max(F32Abs(m.col[1].x), F32Abs(m.col[1].y));
	return F32Max(a1, a2);
}

static ds_ForceInline f32 M3AbsMax(const m3 m)
{
	const f32 a1 = F32Max(F32Abs(m.col[0].x), F32Max(F32Abs(m.col[0].y), F32Abs(m.col[0].z)));
	const f32 a2 = F32Max(F32Abs(m.col[1].x), F32Max(F32Abs(m.col[1].y), F32Abs(m.col[1].z)));
	const f32 a3 = F32Max(F32Abs(m.col[2].x), F32Max(F32Abs(m.col[2].y), F32Abs(m.col[2].z)));
	return F32Max(a1, F32Max(a2, a3));
}

static ds_ForceInline f32 M4AbsMax(const m4 m)
{
	const f32 a1 = F32Max(F32Abs(m.col[0].x), F32Max(F32Abs(m.col[0].y), F32Max(F32Abs(m.col[0].z), F32Abs(m.col[0].w))));
	const f32 a2 = F32Max(F32Abs(m.col[1].x), F32Max(F32Abs(m.col[1].y), F32Max(F32Abs(m.col[1].z), F32Abs(m.col[1].w))));
	const f32 a3 = F32Max(F32Abs(m.col[2].x), F32Max(F32Abs(m.col[2].y), F32Max(F32Abs(m.col[2].z), F32Abs(m.col[2].w))));
	const f32 a4 = F32Max(F32Abs(m.col[3].x), F32Max(F32Abs(m.col[3].y), F32Max(F32Abs(m.col[3].z), F32Abs(m.col[3].w))));
	return F32Max(a1, F32Max(a2, F32Max(a3, a4)));
}

/* a * b^T */
static ds_ForceInline m2 M2OuterProduct(const v2 a, const v2 b)
{
	return M2Columns(V2Scale(a, b.x),
			 V2Scale(a, b.y));
}

static ds_ForceInline m3 M3OuterProduct(const v3 a, const v3 b)
{
	return M3Columns(V3Scale(a, b.x),
			 V3Scale(a, b.y),
			 V3Scale(a, b.z));
}

static ds_ForceInline m4 M4OuterProduct(const v4 a, const v4 b)
{
	return M4Columns(V4Scale(a, b.x),
			 V4Scale(a, b.y),
			 V4Scale(a, b.z),
			 V4Scale(a, b.w));
}

/* a is a normalised quaternion representing a CCW rotation. */
static ds_ForceInline m3 M3Q(const q a)
{
	const f32 tr_part = 2.0f*a.w*a.w - 1.0f;
	const f32 q12 = 2.0f*a.x*a.y;
	const f32 q13 = 2.0f*a.x*a.z;
	const f32 q10 = 2.0f*a.x*a.w;
	const f32 q23 = 2.0f*a.y*a.z;
	const f32 q20 = 2.0f*a.y*a.w;
	const f32 q30 = 2.0f*a.z*a.w;
	return M3(tr_part + 2.0f*a.x*a.x, q12 + q30, q13 - q20,
		  q12 - q30, tr_part + 2.0f*a.y*a.y, q23 + q10,
		  q13 + q20, q23 - q10, tr_part + 2.0f*a.z*a.z);
}

/* a is a normalised quaternion representing a CCW rotation. */
static ds_ForceInline m4 M4Q(const q a)
{
	const f32 tr_part = 2.0f*a.w*a.w - 1.0f;
	const f32 q12 = 2.0f*a.x*a.y;
	const f32 q13 = 2.0f*a.x*a.z;
	const f32 q10 = 2.0f*a.x*a.w;
	const f32 q23 = 2.0f*a.y*a.z;
	const f32 q20 = 2.0f*a.y*a.w;
	const f32 q30 = 2.0f*a.z*a.w;
	return M4(tr_part + 2.0f*a.x*a.x, q12 + q30, q13 - q20, 0.0f,
		  q12 - q30, tr_part + 2.0f*a.y*a.y, q23 + q10, 0.0f,
		  q13 + q20, q23 - q10, tr_part + 2.0f*a.z*a.z, 0.0f,
		  0.0f, 0.0f, 0.0f, 1.0f);
}

/* Returns determinant of a, and sets inverse, or garbage if it does not exist. (Implemented in ds_matrix.c) */
f32 M2Inverse(m2 *inv, const m2 a);
f32 M3Inverse(m3 *inv, const m3 a);
f32 M4Inverse(m4 *inv, const m4 a);

#ifdef __cplusplus
}
#endif

#endif
