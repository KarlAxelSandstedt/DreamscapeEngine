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

#include "ds_math.h"

void Mat2Print(const char *text, mat2 m)
{
	printf("%s:\n| %f %f |\n| %f %f |\n", text,
		       	m[0][0], m[1][0],
		       	m[0][1], m[1][1]
			);
}
void Mat3Print(const char *text, mat3 m)
{
	printf("%s:\n| %f %f %f |\n| %f %f %f |\n| %f %f %f |\n", text,
		       	m[0][0], m[1][0], m[2][0],
		       	m[0][1], m[1][1], m[2][1],
		       	m[0][2], m[1][2], m[2][2] 
			);
}
void Mat4Print(const char *text, mat4 m)
{
	printf("%s:\n| %f %f %f %f |\n| %f %f %f %f |\n| %f %f %f %f |\n| %f %f %f %f |\n", text,
		       	m[0][0], m[1][0], m[2][0], m[3][0],
		       	m[0][1], m[1][1], m[2][1], m[3][1],
		       	m[0][2], m[1][2], m[2][2], m[3][2],
		       	m[0][3], m[1][3], m[2][3], m[3][3]
			);
}

void Mat2Set(mat2 dst, const f32 a11, const f32 a21,
			const f32 a12, const f32 a22)
{
	dst[0][0] = a11; dst[0][1] = a21;
	dst[1][0] = a12; dst[1][1] = a22;
}

void Mat3Set(mat3 dst, const f32 a11, const f32 a21, const f32 a31,
			const f32 a12, const f32 a22, const f32 a32,  
			const f32 a13, const f32 a23, const f32 a33)
{
	dst[0][0] = a11; dst[0][1] = a21; dst[0][2] = a31;
	dst[1][0] = a12; dst[1][1] = a22; dst[1][2] = a32;
	dst[2][0] = a13; dst[2][1] = a23; dst[2][2] = a33;
}

void Mat4Set(mat4 dst, const f32 a11, const f32 a21, const f32 a31, const f32 a41,
			const f32 a12, const f32 a22, const f32 a32, const f32 a42, 
			const f32 a13, const f32 a23, const f32 a33, const f32 a43, 
			const f32 a14, const f32 a24, const f32 a34, const f32 a44)
{
	dst[0][0] = a11; dst[0][1] = a21; dst[0][2] = a31; dst[0][3] = a41;
	dst[1][0] = a12; dst[1][1] = a22; dst[1][2] = a32; dst[1][3] = a42;
	dst[2][0] = a13; dst[2][1] = a23; dst[2][2] = a33; dst[2][3] = a43;
	dst[3][0] = a14; dst[3][1] = a24; dst[3][2] = a34; dst[3][3] = a44;
}


void Mat2SetColumns(mat2 dst, const vec2 c1, const vec2 c2)
{
	dst[0][0] = c1[0];
	dst[0][1] = c1[1];
	dst[1][0] = c2[0];
	dst[1][1] = c2[1];
}

void Mat3SetColumns(mat3 dst, const vec3 c1, const vec3 c2, const vec3 c3)
{

	dst[0][0] = c1[0];
	dst[0][1] = c1[1];
	dst[0][2] = c1[2];
	dst[1][0] = c2[0];
	dst[1][1] = c2[1];
	dst[1][2] = c2[2];
	dst[2][0] = c3[0];
	dst[2][1] = c3[1];
	dst[2][2] = c3[2];
}

void Mat4SetColumns(mat4 dst, const vec4 c1, const vec4 c2, const vec4 c3, const vec4 c4)
{
	dst[0][0] = c1[0];
	dst[0][1] = c1[1];
	dst[0][2] = c1[2];
	dst[0][3] = c1[3];
	dst[1][0] = c2[0];
	dst[1][1] = c2[1];
	dst[1][2] = c2[2];
	dst[1][3] = c2[3];
	dst[2][0] = c3[0];
	dst[2][1] = c3[1];
	dst[2][2] = c3[2];
	dst[2][3] = c3[3];
	dst[3][0] = c4[0];
	dst[3][1] = c4[1];
	dst[3][2] = c4[2];
	dst[3][3] = c4[3];
}

void Mat2SetRows(mat2 dst, const vec2 r1, const vec2 r2)
{
	dst[0][0] = r1[0];
	dst[1][0] = r1[1];
	dst[0][1] = r2[0];
	dst[1][1] = r2[1];
}

void Mat3SetRows(mat3 dst, const vec3 r1, const vec3 r2, const vec3 r3)
{
	dst[0][0] = r1[0];
	dst[1][0] = r1[1];
	dst[2][0] = r1[2];
	dst[0][1] = r2[0];
	dst[1][1] = r2[1];
	dst[2][1] = r2[2];
	dst[0][2] = r3[0];
	dst[1][2] = r3[1];
	dst[2][2] = r3[2];
}

void Mat4SetRows(mat4 dst, const vec4 r1, const vec4 r2, const vec4 r3, const vec4 r4)
{
	dst[0][0] = r1[0];
	dst[1][0] = r1[1];
	dst[2][0] = r1[2];
	dst[3][0] = r1[3];
	dst[0][1] = r2[0];
	dst[1][1] = r2[1];
	dst[2][1] = r2[2];
	dst[3][1] = r2[3];
	dst[0][2] = r3[0];
	dst[1][2] = r3[1];
	dst[2][2] = r3[2];
	dst[3][2] = r3[3];
	dst[0][3] = r4[0];
	dst[1][3] = r4[1];
	dst[2][3] = r4[2];
	dst[3][3] = r4[3];
}

void Mat2Identity(mat2 dst)
{
	Mat2Set(dst, 1,0,0,1);
}

void Mat3Identity(mat3 dst)
{
	Mat3Set(dst, 1,0,0,0,1,0,0,0,1);
}

void Mat4Identity(mat4 dst)
{
	Mat4Set(dst, 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1);
}


void Vec2MatMul(vec2 dst, const vec2 vec, mat2 mat)
{
	dst[0] = vec[0] * mat[0][0] + vec[1] *  mat[0][1];
	dst[1] = vec[0] * mat[1][0] + vec[1] *  mat[1][1];
}

void Vec3MatMul(vec3 dst, const vec3 vec, mat3 mat)
{
	dst[0] = vec[0] * mat[0][0] + vec[1] *  mat[0][1] + vec[2] * mat[0][2];
	dst[1] = vec[0] * mat[1][0] + vec[1] *  mat[1][1] + vec[2] * mat[1][2];
	dst[2] = vec[0] * mat[2][0] + vec[1] *  mat[2][1] + vec[2] * mat[2][2];
}

void Vec4MatMul(vec4 dst, const vec4 vec, mat4 mat)
{
	dst[0] = vec[0] * mat[0][0] + vec[1] *  mat[0][1] + vec[2] * mat[0][2] + vec[3] * mat[0][3];
	dst[1] = vec[0] * mat[1][0] + vec[1] *  mat[1][1] + vec[2] * mat[1][2] + vec[3] * mat[1][3];
	dst[2] = vec[0] * mat[2][0] + vec[1] *  mat[2][1] + vec[2] * mat[2][2] + vec[3] * mat[2][3];
	dst[3] = vec[0] * mat[3][0] + vec[1] *  mat[3][1] + vec[2] * mat[3][2] + vec[3] * mat[3][3];
}

void Mat2VecMul(vec2 dst, mat2 mat, const vec2 vec)
{
	dst[0] = vec[0] * mat[0][0] + vec[1] *  mat[1][0];
	dst[1] = vec[0] * mat[0][1] + vec[1] *  mat[1][1];
}

void Mat3VecMul(vec3 dst, mat3 mat, const vec3 vec)
{
	dst[0] = vec[0] * mat[0][0] + vec[1] *  mat[1][0] + vec[2] * mat[2][0];
	dst[1] = vec[0] * mat[0][1] + vec[1] *  mat[1][1] + vec[2] * mat[2][1];
	dst[2] = vec[0] * mat[0][2] + vec[1] *  mat[1][2] + vec[2] * mat[2][2];
}

void Mat4VecMul(vec4 dst, mat4 mat, const vec4 vec)
{
	dst[0] = vec[0] * mat[0][0] + vec[1] *  mat[1][0] + vec[2] * mat[2][0] + vec[3] * mat[3][0];
	dst[1] = vec[0] * mat[0][1] + vec[1] *  mat[1][1] + vec[2] * mat[2][1] + vec[3] * mat[3][1];
	dst[2] = vec[0] * mat[0][2] + vec[1] *  mat[1][2] + vec[2] * mat[2][2] + vec[3] * mat[3][2];
	dst[3] = vec[0] * mat[0][3] + vec[1] *  mat[1][3] + vec[2] * mat[2][3] + vec[3] * mat[3][3];
}

// dst = [a][b]
void Mat2Mul(mat2 dst, mat2 a, mat2 b)
{
	dst[0][0] = a[0][0]*b[0][0] + a[1][0]*b[0][1];
	dst[0][1] = a[0][1]*b[0][0] + a[1][1]*b[0][1];

	dst[1][0] = a[0][0]*b[1][0] + a[1][0]*b[1][1];
	dst[1][1] = a[0][1]*b[1][0] + a[1][1]*b[1][1];
}

// dst = [a][b]
void Mat3Mul(mat3 dst, mat3 a, mat3 b)
{
	dst[0][0] = a[0][0]*b[0][0] + a[1][0]*b[0][1] + a[2][0]*b[0][2];
	dst[0][1] = a[0][1]*b[0][0] + a[1][1]*b[0][1] + a[2][1]*b[0][2];
	dst[0][2] = a[0][2]*b[0][0] + a[1][2]*b[0][1] + a[2][2]*b[0][2];

	dst[1][0] = a[0][0]*b[1][0] + a[1][0]*b[1][1] + a[2][0]*b[1][2];
	dst[1][1] = a[0][1]*b[1][0] + a[1][1]*b[1][1] + a[2][1]*b[1][2];
	dst[1][2] = a[0][2]*b[1][0] + a[1][2]*b[1][1] + a[2][2]*b[1][2];

	dst[2][0] = a[0][0]*b[2][0] + a[1][0]*b[2][1] + a[2][0]*b[2][2];
	dst[2][1] = a[0][1]*b[2][0] + a[1][1]*b[2][1] + a[2][1]*b[2][2];
	dst[2][2] = a[0][2]*b[2][0] + a[1][2]*b[2][1] + a[2][2]*b[2][2];
}

// dst = [a][b]
void Mat4Mul(mat4 dst, mat4 a, mat4 b)
{
	dst[0][0] = a[0][0]*b[0][0] + a[1][0]*b[0][1] + a[2][0]*b[0][2] + a[3][0]*b[0][3];
	dst[0][1] = a[0][1]*b[0][0] + a[1][1]*b[0][1] + a[2][1]*b[0][2] + a[3][1]*b[0][3];
	dst[0][2] = a[0][2]*b[0][0] + a[1][2]*b[0][1] + a[2][2]*b[0][2] + a[3][2]*b[0][3];
	dst[0][3] = a[0][3]*b[0][0] + a[1][3]*b[0][1] + a[2][3]*b[0][2] + a[3][3]*b[0][3];

	dst[1][0] = a[0][0]*b[1][0] + a[1][0]*b[1][1] + a[2][0]*b[1][2] + a[3][0]*b[1][3];
	dst[1][1] = a[0][1]*b[1][0] + a[1][1]*b[1][1] + a[2][1]*b[1][2] + a[3][1]*b[1][3];
	dst[1][2] = a[0][2]*b[1][0] + a[1][2]*b[1][1] + a[2][2]*b[1][2] + a[3][2]*b[1][3];
	dst[1][3] = a[0][3]*b[1][0] + a[1][3]*b[1][1] + a[2][3]*b[1][2] + a[3][3]*b[1][3];

	dst[2][0] = a[0][0]*b[2][0] + a[1][0]*b[2][1] + a[2][0]*b[2][2] + a[3][0]*b[2][3];
	dst[2][1] = a[0][1]*b[2][0] + a[1][1]*b[2][1] + a[2][1]*b[2][2] + a[3][1]*b[2][3];
	dst[2][2] = a[0][2]*b[2][0] + a[1][2]*b[2][1] + a[2][2]*b[2][2] + a[3][2]*b[2][3];
	dst[2][3] = a[0][3]*b[2][0] + a[1][3]*b[2][1] + a[2][3]*b[2][2] + a[3][3]*b[2][3];

	dst[3][0] = a[0][0]*b[3][0] + a[1][0]*b[3][1] + a[2][0]*b[3][2] + a[3][0]*b[3][3];
	dst[3][1] = a[0][1]*b[3][0] + a[1][1]*b[3][1] + a[2][1]*b[3][2] + a[3][1]*b[3][3];
	dst[3][2] = a[0][2]*b[3][0] + a[1][2]*b[3][1] + a[2][2]*b[3][2] + a[3][2]*b[3][3];
	dst[3][3] = a[0][3]*b[3][0] + a[1][3]*b[3][1] + a[2][3]*b[3][2] + a[3][3]*b[3][3];
}

void Mat2Add(mat2 dst, mat2 a, mat2 b)
{
	dst[0][0] = a[0][0] + b[0][0];  
        dst[0][1] = a[0][1] + b[0][1];

        dst[1][0] = a[1][0] + b[1][0];
        dst[1][1] = a[1][1] + b[1][1];
}

void Mat3Add(mat3 dst, mat3 a, mat3 b)
{
	dst[0][0] = a[0][0] + b[0][0];  
        dst[0][1] = a[0][1] + b[0][1];
        dst[0][2] = a[0][2] + b[0][2];

        dst[1][0] = a[1][0] + b[1][0];
        dst[1][1] = a[1][1] + b[1][1];
        dst[1][2] = a[1][2] + b[1][2];

        dst[2][0] = a[2][0] + b[2][0];
        dst[2][1] = a[2][1] + b[2][1];
        dst[2][2] = a[2][2] + b[2][2];
}

void Mat4Add(mat4 dst, mat4 a, mat4 b)
{
	dst[0][0] = a[0][0] + b[0][0];  
        dst[0][1] = a[0][1] + b[0][1];
        dst[0][2] = a[0][2] + b[0][2];
        dst[0][3] = a[0][3] + b[0][3];

        dst[1][0] = a[1][0] + b[1][0];
        dst[1][1] = a[1][1] + b[1][1];
        dst[1][2] = a[1][2] + b[1][2];
        dst[1][3] = a[1][3] + b[1][3];

        dst[2][0] = a[2][0] + b[2][0];
        dst[2][1] = a[2][1] + b[2][1];
        dst[2][2] = a[2][2] + b[2][2];
        dst[2][3] = a[2][3] + b[2][3];
	
        dst[3][0] = a[3][0] + b[3][0];
        dst[3][1] = a[3][1] + b[3][1];
        dst[3][2] = a[3][2] + b[3][2];
        dst[3][3] = a[3][3] + b[3][3];
}

void Mat2Transpose(mat2 dst, mat2 src)
{
	dst[0][0] = src[0][0];
	dst[1][0] = src[0][1];
	
	dst[0][1] = src[1][0];
	dst[1][1] = src[1][1];
}

void Mat3Transpose(mat3 dst, mat3 src)
{
	dst[0][0] = src[0][0];
	dst[1][0] = src[0][1];
	dst[2][0] = src[0][2];
	
	dst[0][1] = src[1][0];
	dst[1][1] = src[1][1];
	dst[2][1] = src[1][2];

	dst[0][2] = src[2][0];
	dst[1][2] = src[2][1];
	dst[2][2] = src[2][2];
}

void Mat4Transpose(mat4 dst, mat4 src)
{
	dst[0][0] = src[0][0];
	dst[1][0] = src[0][1];
	dst[2][0] = src[0][2];
	dst[3][0] = src[0][3];
	
	dst[0][1] = src[1][0];
	dst[1][1] = src[1][1];
	dst[2][1] = src[1][2];
	dst[3][1] = src[1][3];
	
	dst[0][2] = src[2][0];
	dst[1][2] = src[2][1];
	dst[2][2] = src[2][2];
	dst[3][2] = src[2][3];
	
	dst[0][3] = src[3][0];
	dst[1][3] = src[3][1];
	dst[2][3] = src[3][2];
	dst[3][3] = src[3][3];
}



f32 Mat2Inverse(mat2 dst, mat2 src)
{
	const f32 det = (src[0][0]*src[1][1] - src[1][0]*src[0][1]);
	const f32 div = 1.0f / det;
	Mat2Set(dst, div*src[1][1], -div*src[1][0], -div*src[0][1], div*src[0][0]);

	return det;
}

f32 Mat3Inverse(mat3 dst, mat3 src)
{
	const f32 s11 = src[0][0];
	const f32 s12 = src[1][0];
	const f32 s13 = src[2][0];
	const f32 s21 = src[0][1];
	const f32 s22 = src[1][1];
	const f32 s23 = src[2][1];
	const f32 s31 = src[0][2];
	const f32 s32 = src[1][2];
	const f32 s33 = src[2][2];

	/* co-factors */
	const f32 c11 = s22*s33 - s23*s32;
	const f32 c12 = -(s21*s33 - s23*s31);
	const f32 c13 = s21*s32 - s31*s22;

	const f32 c21 = -(s12*s33 - s32*s13);
	const f32 c22 = s11*s33 - s31*s13;
	const f32 c23 = -(s11*s32 - s31*s12);

	const f32 c31 = s12*s23 - s22*s13;
	const f32 c32 = -(s11*s23 - s21*s13);
	const f32 c33 = s11*s22 - s21*s12;

	const f32 det = (s11*c11 + s12*c12 + s13*c13);
	const f32 det_inv = 1.0f / det;

	/* inv = det^-1 * transpose[Cofactor matrix]*/
	Mat3Set(dst,
		c11*det_inv, c12*det_inv, c13*det_inv,
		c21*det_inv, c22*det_inv, c23*det_inv,
		c31*det_inv, c32*det_inv, c33*det_inv);

	return det;
}

f32 Mat4Inverse(mat4 dst, mat4 src)
{
	const f32 s11 = src[0][0];
	const f32 s12 = src[1][0];
	const f32 s13 = src[2][0];
	const f32 s14 = src[3][0];

	const f32 s21 = src[0][1];
	const f32 s22 = src[1][1];
	const f32 s23 = src[2][1];
	const f32 s24 = src[3][1];

	const f32 s31 = src[0][2];
	const f32 s32 = src[1][2];
	const f32 s33 = src[2][2];
	const f32 s34 = src[3][2];

	const f32 s41 = src[0][3];
	const f32 s42 = src[1][3];
	const f32 s43 = src[2][3];
	const f32 s44 = src[3][3];

	const f32 d1  = s31*s42 - s41*s32;
	const f32 d2  = s31*s43 - s41*s33;
	const f32 d3  = s31*s44 - s41*s34;
	const f32 d4  = s32*s43 - s42*s33;
	const f32 d5  = s32*s44 - s42*s34;
	const f32 d6  = s33*s44 - s43*s34;

	const f32 d7  = s11*s22 - s21*s12;
	const f32 d8  = s11*s23 - s21*s13;
	const f32 d9  = s11*s24 - s21*s14;
	const f32 d10 = s12*s23 - s22*s13;
	const f32 d11 = s12*s24 - s22*s14;
	const f32 d12 = s13*s24 - s23*s14;

	/* 3x3 co-factors */
	const f32 c11 =   s22*d6  - s23*d5  + s24*d4;
	const f32 c12 = -(s21*d6  - s23*d3  + s24*d2);
	const f32 c13 =   s21*d5  - s22*d3  + s24*d1;
	const f32 c14 = -(s21*d4  - s22*d2  + s23*d1);

	const f32 c21 = -(s12*d6  - s13*d5  + s14*d4);
	const f32 c22 =   s11*d6  - s13*d3  + s14*d2 ;
	const f32 c23 = -(s11*d5  - s12*d3  + s14*d1);
	const f32 c24 =   s11*d4  - s12*d2  + s13*d1 ;

	const f32 c31 =   s42*d12 - s43*d11 + s44*d10;
	const f32 c32 = -(s41*d12 - s43*d9  + s44*d8);
	const f32 c33 =   s41*d11 - s42*d9  + s44*d7;
	const f32 c34 = -(s41*d10 - s42*d8  + s43*d7);

	const f32 c41 = -(s32*d12 - s33*d11 + s34*d10);
	const f32 c42 =   s31*d12 - s33*d9  + s34*d8;
	const f32 c43 = -(s31*d11 - s32*d9  + s34*d7);
	const f32 c44 =   s31*d10 - s32*d8  + s33*d7;

	const f32 det = (s11*c11 + s12*c12 + s13*c13 + s14*c14);
	f32 det_inv = 1.0f / det;

	/* inv = det^-1 * transpose[Cofactor matrix] */
	Mat4Set(dst,
		c11*det_inv, c12*det_inv, c13*det_inv, c14*det_inv,
		c21*det_inv, c22*det_inv, c23*det_inv, c24*det_inv,
		c31*det_inv, c32*det_inv, c33*det_inv, c34*det_inv,
		c41*det_inv, c42*det_inv, c43*det_inv, c44*det_inv);

	return det;
}

void Mat2Copy(mat2 dst, mat2 src)
{
	dst[0][0] = src[0][0];
	dst[0][1] = src[0][1];
	dst[1][0] = src[1][0];
	dst[1][1] = src[1][1];
}

void Mat3Copy(mat3 dst, mat3 src)
{
	dst[0][0] = src[0][0];
	dst[0][1] = src[0][1];
	dst[0][2] = src[0][2];
	dst[1][0] = src[1][0];
	dst[1][1] = src[1][1];
	dst[1][2] = src[1][2];
	dst[2][0] = src[2][0];
	dst[2][1] = src[2][1];
	dst[2][2] = src[2][2];
}

void Mat4Copy(mat4 dst, mat4 src)
{
	dst[0][0] = src[0][0];
	dst[0][1] = src[0][1];
	dst[0][2] = src[0][2];
	dst[0][3] = src[0][3];
	dst[1][0] = src[1][0];
	dst[1][1] = src[1][1];
	dst[1][2] = src[1][2];
	dst[1][3] = src[1][3];
	dst[2][0] = src[2][0];
	dst[2][1] = src[2][1];
	dst[2][2] = src[2][2];
	dst[2][3] = src[2][3];
	dst[3][0] = src[3][0];
	dst[3][1] = src[3][1];
	dst[3][2] = src[3][2];
	dst[3][3] = src[3][3];
}

f32 Mat2AbsMin(mat2 src)
{
	const f32 a1 = f32_min(f32_abs(src[0][0]), f32_abs(src[0][1]));
	const f32 a2 = f32_min(f32_abs(src[1][0]), f32_abs(src[1][1]));
	return f32_min(a1, a2);
}

f32 Mat3AbsMin(mat3 src)
{
	const f32 a1 = f32_min(f32_abs(src[0][0]), f32_min(f32_abs(src[0][1]), f32_abs(src[0][2])));
	const f32 a2 = f32_min(f32_abs(src[1][0]), f32_min(f32_abs(src[1][1]), f32_abs(src[1][2])));
	const f32 a3 = f32_min(f32_abs(src[2][0]), f32_min(f32_abs(src[2][1]), f32_abs(src[2][2])));
	return f32_min(a1, f32_min(a2, a3));
}

f32 Mat4AbsMin(mat4 src)
{
	const f32 a1 = f32_min(f32_abs(src[0][0]), f32_min(f32_abs(src[0][1]), f32_min(f32_abs(src[0][2]), f32_abs(src[0][3]))));
	const f32 a2 = f32_min(f32_abs(src[1][0]), f32_min(f32_abs(src[1][1]), f32_min(f32_abs(src[1][2]), f32_abs(src[1][3]))));
	const f32 a3 = f32_min(f32_abs(src[2][0]), f32_min(f32_abs(src[2][1]), f32_min(f32_abs(src[2][2]), f32_abs(src[2][3]))));
	const f32 a4 = f32_min(f32_abs(src[3][0]), f32_min(f32_abs(src[3][1]), f32_min(f32_abs(src[3][2]), f32_abs(src[3][3]))));
	return f32_min(a1, f32_min(a2, f32_min(a3, a4)));
}


f32 Mat2AbsMax(mat2 src)
{
	const f32 a1 = f32_max(f32_abs(src[0][0]), f32_abs(src[0][1]));
	const f32 a2 = f32_max(f32_abs(src[1][0]), f32_abs(src[1][1]));
	return f32_max(a1, a2);
}

f32 Mat3AbsMax(mat3 src)
{
	const f32 a1 = f32_max(f32_abs(src[0][0]), f32_max(f32_abs(src[0][1]), f32_abs(src[0][2])));
	const f32 a2 = f32_max(f32_abs(src[1][0]), f32_max(f32_abs(src[1][1]), f32_abs(src[1][2])));
	const f32 a3 = f32_max(f32_abs(src[2][0]), f32_max(f32_abs(src[2][1]), f32_abs(src[2][2])));
	return f32_max(a1, f32_max(a2, a3));
}

f32 Mat4AbsMax(mat4 src)
{
	const f32 a1 = f32_max(f32_abs(src[0][0]), f32_max(f32_abs(src[0][1]), f32_max(f32_abs(src[0][2]), f32_abs(src[0][3]))));
	const f32 a2 = f32_max(f32_abs(src[1][0]), f32_max(f32_abs(src[1][1]), f32_max(f32_abs(src[1][2]), f32_abs(src[1][3]))));
	const f32 a3 = f32_max(f32_abs(src[2][0]), f32_max(f32_abs(src[2][1]), f32_max(f32_abs(src[2][2]), f32_abs(src[2][3]))));
	const f32 a4 = f32_max(f32_abs(src[3][0]), f32_max(f32_abs(src[3][1]), f32_max(f32_abs(src[3][2]), f32_abs(src[3][3]))));
	return f32_max(a1, f32_max(a2, f32_max(a3, a4)));
}
