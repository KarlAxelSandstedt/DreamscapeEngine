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
#include "ds_matrix.h"

f32 M2Inverse(m2 *inv, const m2 a)
{
	const f32 det = (a.a11*a.a22 - a.a12*a.a21);
	const f32 div = 1.0f / det;
	*inv = M2(div*a.a22, -div*a.a21, -div*a.a12, div*a.a11);

	return det;
}

f32 M3Inverse(m3 *inv, const m3 a)
{
	/* co-factors */
	const f32 c11 = a.a22*a.a33 - a.a23*a.a32;
	const f32 c12 = -(a.a21*a.a33 - a.a23*a.a31);
	const f32 c13 = a.a21*a.a32 - a.a31*a.a22;

	const f32 c21 = -(a.a12*a.a33 - a.a32*a.a13);
	const f32 c22 = a.a11*a.a33 - a.a31*a.a13;
	const f32 c23 = -(a.a11*a.a32 - a.a31*a.a12);

	const f32 c31 = a.a12*a.a23 - a.a22*a.a13;
	const f32 c32 = -(a.a11*a.a23 - a.a21*a.a13);
	const f32 c33 = a.a11*a.a22 - a.a21*a.a12;

	const f32 det = (a.a11*c11 + a.a12*c12 + a.a13*c13);
	const f32 det_inv = 1.0f / det;

	/* inv = det^-1 * transpose[Cofactor matrix]*/
	*inv = M3(
		c11*det_inv, c12*det_inv, c13*det_inv,
		c21*det_inv, c22*det_inv, c23*det_inv,
		c31*det_inv, c32*det_inv, c33*det_inv);

	return det;
}

f32 M4Inverse(m4 *inv, const m4 a)
{
	const f32 d1  = a.a31*a.a42 - a.a41*a.a32;
	const f32 d2  = a.a31*a.a43 - a.a41*a.a33;
	const f32 d3  = a.a31*a.a44 - a.a41*a.a34;
	const f32 d4  = a.a32*a.a43 - a.a42*a.a33;
	const f32 d5  = a.a32*a.a44 - a.a42*a.a34;
	const f32 d6  = a.a33*a.a44 - a.a43*a.a34;

	const f32 d7  = a.a11*a.a22 - a.a21*a.a12;
	const f32 d8  = a.a11*a.a23 - a.a21*a.a13;
	const f32 d9  = a.a11*a.a24 - a.a21*a.a14;
	const f32 d10 = a.a12*a.a23 - a.a22*a.a13;
	const f32 d11 = a.a12*a.a24 - a.a22*a.a14;
	const f32 d12 = a.a13*a.a24 - a.a23*a.a14;

	/* 3x3 co-factors */
	const f32 c11 =   a.a22*d6  - a.a23*d5  + a.a24*d4;
	const f32 c12 = -(a.a21*d6  - a.a23*d3  + a.a24*d2);
	const f32 c13 =   a.a21*d5  - a.a22*d3  + a.a24*d1;
	const f32 c14 = -(a.a21*d4  - a.a22*d2  + a.a23*d1);

	const f32 c21 = -(a.a12*d6  - a.a13*d5  + a.a14*d4);
	const f32 c22 =   a.a11*d6  - a.a13*d3  + a.a14*d2 ;
	const f32 c23 = -(a.a11*d5  - a.a12*d3  + a.a14*d1);
	const f32 c24 =   a.a11*d4  - a.a12*d2  + a.a13*d1 ;

	const f32 c31 =   a.a42*d12 - a.a43*d11 + a.a44*d10;
	const f32 c32 = -(a.a41*d12 - a.a43*d9  + a.a44*d8);
	const f32 c33 =   a.a41*d11 - a.a42*d9  + a.a44*d7;
	const f32 c34 = -(a.a41*d10 - a.a42*d8  + a.a43*d7);

	const f32 c41 = -(a.a32*d12 - a.a33*d11 + a.a34*d10);
	const f32 c42 =   a.a31*d12 - a.a33*d9  + a.a34*d8;
	const f32 c43 = -(a.a31*d11 - a.a32*d9  + a.a34*d7);
	const f32 c44 =   a.a31*d10 - a.a32*d8  + a.a33*d7;

	const f32 det = (a.a11*c11 + a.a12*c12 + a.a13*c13 + a.a14*c14);
	f32 det_inv = 1.0f / det;

	/* inv = det^-1 * transpose[Cofactor matrix] */
	*inv = M4(
		c11*det_inv, c12*det_inv, c13*det_inv, c14*det_inv,
		c21*det_inv, c22*det_inv, c23*det_inv, c24*det_inv,
		c31*det_inv, c32*det_inv, c33*det_inv, c34*det_inv,
		c41*det_inv, c42*det_inv, c43*det_inv, c44*det_inv);

	return det;
}
