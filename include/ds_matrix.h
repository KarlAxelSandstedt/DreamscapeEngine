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
ds_matrix.h
===========
3x3 and 4x4 f32 matrices (m3, m4 in ds_types.h; the m2 type has only its constructor there): construction,
products with vectors and matrices, arithmetic, rotation matrices from quaternions, inverses. Column-major: aij is row i, column j, col[j-1] is column
j, and M2/M3/M4(...) take their arguments column by column (ds_types.h). Included by ds_math.h.

Everything is plain IEEE arithmetic in a fixed order: the same bits on every platform and compiler. Nothing is
checked; the inverses report a zero determinant by returning it.

Sections: construction; products with vectors; arithmetic; rotations; inverses (ds_matrix.c); printing. Inline definitions at the bottom, in the same order.
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

/*********************************** construction ***********************************/

/* identity matrix */
static ds_ForceInline m3 M3Identity(void);
static ds_ForceInline m4 M4Identity(void);
/* the matrix with the given columns */
static ds_ForceInline m3 M3Columns(const v3 c1, const v3 c2, const v3 c3);
static ds_ForceInline m4 M4Columns(const v4 c1, const v4 c2, const v4 c3, const v4 c4);
/* the matrix with the given rows */
static ds_ForceInline m3 M3Rows(const v3 r1, const v3 r2, const v3 r3);
static ds_ForceInline m4 M4Rows(const v4 r1, const v4 r2, const v4 r3, const v4 r4);

/****************************** products with vectors *******************************/

/* mat*vec: m v (each component summed left to right) */
static ds_ForceInline v3 M3V3Mul(const m3 m, const v3 v);
static ds_ForceInline v4 M4V4Mul(const m4 m, const v4 v);
/* vec*mat: v^T m (= m^T v) */
static ds_ForceInline v3 V3M3Mul(const v3 v, const m3 m);

/************************************ arithmetic ************************************/

/* a + b */
static ds_ForceInline m3 M3Add(const m3 a, const m3 b);
/* a - b */
static ds_ForceInline m3 M3Sub(const m3 a, const m3 b);
/* m*scale */
static ds_ForceInline m3 M3Scale(const m3 m, const f32 scale);
/* a*b */
static ds_ForceInline m3 M3Mul(const m3 a, const m3 b);
static ds_ForceInline m4 M4Mul(const m4 a, const m4 b);
/* m^T */
static ds_ForceInline m3 M3Transpose(const m3 m);
static ds_ForceInline m4 M4Transpose(const m4 m);
/* a * b^T */
static ds_ForceInline m3 M3OuterProduct(const v3 a, const v3 b);


/************************************ rotations *************************************/

/*
 * a is a normalised quaternion representing a CCW rotation (not checked: otherwise the diagonal is off by
 * |a|^2 - 1). M3V3Mul(M3QUnit(a), v) is QUnitRotate(a, v) up to rounding.
 */
static ds_ForceInline m3 M3QUnit(const q a);
/* a is a normalised quaternion representing a CCW rotation; M3QUnit as a 4x4 matrix (last row and column 0, 0, 0, 1) */
static ds_ForceInline m4 M4QUnit(const q a);

/****************************** inverses (ds_matrix.c) ******************************/

/*
 * Returns determinant of a, and sets inverse (the adjugate over the determinant), or garbage (inf/NaN) if it does
 * not exist: check the determinant. No pivoting: nearly singular matrices give large, inaccurate inverses.
 * (Implemented in ds_matrix.c)
 */
f32 M3Inverse(m3 *inv, const m3 a);
f32 M4Inverse(m4 *inv, const m4 a);

/************************************* printing *************************************/

/* print text and the matrix, row by row, to stdout (debugging) */
static inline void M3Print(const char *text, const m3 m);
static inline void M4Print(const char *text, const m4 m);

/************************* construction: inline definitions *************************/

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

static ds_ForceInline m3 M3Columns(const v3 c1, const v3 c2, const v3 c3)
{
	return (m3) { .col = { c1, c2, c3 } };
}

static ds_ForceInline m4 M4Columns(const v4 c1, const v4 c2, const v4 c3, const v4 c4)
{
	return (m4) { .col = { c1, c2, c3, c4 } };
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

/******************** products with vectors: inline definitions *********************/

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

static ds_ForceInline v3 V3M3Mul(const v3 v, const m3 m)
{
	return V3(V3Dot(m.col[0], v),
		  V3Dot(m.col[1], v),
		  V3Dot(m.col[2], v));
}

/************************** arithmetic: inline definitions **************************/

static ds_ForceInline m3 M3Add(const m3 a, const m3 b)
{
	return M3Columns(V3Add(a.col[0], b.col[0]),
			 V3Add(a.col[1], b.col[1]),
			 V3Add(a.col[2], b.col[2]));
}

static ds_ForceInline m3 M3Sub(const m3 a, const m3 b)
{
	return M3Columns(V3Sub(a.col[0], b.col[0]),
			 V3Sub(a.col[1], b.col[1]),
			 V3Sub(a.col[2], b.col[2]));
}

static ds_ForceInline m3 M3Scale(const m3 m, const f32 scale)
{
	return M3Columns(V3Scale(m.col[0], scale),
			 V3Scale(m.col[1], scale),
			 V3Scale(m.col[2], scale));
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

static ds_ForceInline m3 M3Transpose(const m3 m)
{
	return M3Rows(m.col[0], m.col[1], m.col[2]);
}

static ds_ForceInline m4 M4Transpose(const m4 m)
{
	return M4Rows(m.col[0], m.col[1], m.col[2], m.col[3]);
}

static ds_ForceInline m3 M3OuterProduct(const v3 a, const v3 b)
{
	return M3Columns(V3Scale(a, b.x),
			 V3Scale(a, b.y),
			 V3Scale(a, b.z));
}

/************************** rotations: inline definitions ***************************/

static ds_ForceInline m3 M3QUnit(const q a)
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

static ds_ForceInline m4 M4QUnit(const q a)
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

/*************************** printing: inline definitions ***************************/

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

#ifdef __cplusplus
}
#endif

#endif
