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

#include <stdio.h>
#include <math.h>
#include <float.h>
#include <fenv.h>

#include "ds_test.h"
#include "ds_matrix.h"

static struct test_Output matrix_inverse_assert(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

	const m3 A = M3(2,0,1,1,2,1,3,4,2);
	m3 A_inv;
	M3Inverse(&A_inv, A);
	const m3 I3 = M3Mul(A, A_inv);

	const f32 eps = 0.0001f;

	for (u32 i = 0; i < 3; ++i)
	{
		ds_assert(1.0f - eps <= I3.col[i].buf[i] && I3.col[i].buf[i] <= 1.0f + eps);
		for (u32 j = i+1; j < 3; ++j)
		{
			assert(-eps <= I3.col[i].buf[j] && I3.col[j].buf[i] <= eps);
		}
	}

	const m4 B = M4(5,2,6,2, 
			6,2,6,3,
			6,2,2,6,
			8,8,8,7);
	m4 B_inv;
	M4Inverse(&B_inv, B);
	const m4 I4 = M4Mul(B, B_inv);

	for (u32 i = 0; i < 4; ++i)
	{
		assert(1.0f - eps <= I4.col[i].buf[i] && I4.col[i].buf[i] <= 1.0f + eps);
		for (u32 j = i+1; j < 4; ++j)
		{
			assert(-eps <= I3.col[i].buf[j] && I3.col[j].buf[i] <= eps);	/* bug 13: should be I4 (and reads past I3 for i = 3) */
		}
	}


	return output;
}

static struct test_Output (*math_tests[])(struct test_Environment *) =
{
	matrix_inverse_assert,
};

struct suite_Correctness m_math_suite =
{
	.id = "math",
	.unit_test = math_tests,
	.unit_test_count = sizeof(math_tests) / sizeof(math_tests[0]),
};

struct suite_Correctness *math_correctness_suite = &m_math_suite;
