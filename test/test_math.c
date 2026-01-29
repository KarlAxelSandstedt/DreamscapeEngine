/*
==========================================================================
    Copyright (C) 2025 Axel Sandstedt 

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

#include <stdio.h>
#include <math.h>
#include <float.h>
#include <fenv.h>

#include "test_local.h"
#include "kas_math.h"
#include "matrix.h"

static struct test_output matrix_inverse_assert(struct test_environment *env)
{
	struct test_output output = { .success = 1, .id = __func__ };

	mat3 I3, A, A_inv;
	Mat3Set(A, 2,0,1,1,2,1,3,4,2);
	Mat3Inverse(A_inv, A);
	Mat3Mul(I3, A, A_inv);

	const f32 eps = 0.0001f;

	for (u32 i = 0; i < 3; ++i)
	{
		assert(1.0f - eps <= I3[i][i] && I3[i][i] <= 1.0f + eps);
		for (u32 j = i+1; j < 3; ++j)
		{
			assert(-eps <= I3[i][j] && I3[j][i] <= eps);
		}
	}

	mat4 I4, B, B_inv;
	Mat4Set(B,
			5,2,6,2, 
			6,2,6,3,
			6,2,2,6,
			8,8,8,7);
	Mat4Inverse(B_inv, B);
	Mat4Mul(I4, B, B_inv);

	for (u32 i = 0; i < 4; ++i)
	{
		assert(1.0f - eps <= I4[i][i] && I4[i][i] <= 1.0f + eps);
		for (u32 j = i+1; j < 4; ++j)
		{
			assert(-eps <= I3[i][j] && I3[j][i] <= eps);
		}
	}


	return output;
}

static struct test_output (*math_tests[])(struct test_environment *) =
{
	matrix_inverse_assert,
};

struct suite m_math_suite =
{
	.id = "math",
	.unit_test = math_tests,
	.unit_test_count = sizeof(math_tests) / sizeof(math_tests[0]),
};

struct suite *math_suite = &m_math_suite;
