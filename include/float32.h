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

#ifndef __MG_FLOAT32_H__
#define __MG_FLOAT32_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_common.h"

#undef  F32_PI
#undef  F32_PI

#define F32_PI				3.14159274101257324f
#define F32_PI2    			(2.0f * PI)

#define	F32_SIGN_LENGTH			1
#define	F32_EXPONENT_LENGTH		8
#define	F32_SIGNIFICAND_LENGTH		23

#define	F32_SIGN_MASK			0x80000000
#define	F32_EXPONENT_MASK		0x7f800000
#define	F32_SIGNIFICAND_MASK		0x007fffff

#define	F32_BIAS			127
#define	F32_MAX_EXPONENT		127
#define	F32_MIN_EXPONENT		-126

#define	F32_MAX_POSITIVE_SUBNORMAL	f32_max_positive_subnormal()
#define	F32_MIN_POSITIVE_SUBNORMAL	f32_min_positive_subnormal()
#define	F32_MAX_NEGATIVE_SUBNORMAL	f32_max_negative_subnormal()
#define	F32_MIN_NEGATIVE_SUBNORMAL	f32_min_negative_subnormal()

#define	F32_MAX_POSITIVE_NORMAL		f32_max_positive_normal()
#define	F32_MIN_POSITIVE_NORMAL		f32_min_positive_normal()
#define	F32_MAX_NEGATIVE_NORMAL		f32_max_negative_normal()
#define	F32_MIN_NEGATIVE_NORMAL		f32_min_negative_normal()

#define	F32_INFINITY			f32_inf(0)
#define	F32_EPSILON			1.1920929e-7f

enum ieee_type
{
	IEEE_NAN,
	IEEE_INF,
	IEEE_ZERO,
	IEEE_NORMAL,
	IEEE_SUBNORMAL,
};

union ieee32
{
	f32 f;
	u32 bits;		/* SIGN_BIT(1) | EXPONENT(8) | SIGNIFICAND(23) */
};

void		f32_bits_print(FILE *file, const f32 f);

/* return bit value of sign in FP f or ieee32 f */
f32 		f32_construct(const u32 sign_bit, const u32 exponent_bits, const u32 mantissa_bits);
/* return sign bit shifted down */
u32 		f32_sign_bit(const f32 f);
/* return exponent bits  shifted down */
u32 		f32_exponent_bits(const f32 f);
/* return mantissa bits */
u32 		f32_mantissa_bits(const f32 f);
/* return sign 1.0f or -1.0f */
f32 		f32_sign(const f32 f);

f32 		f32_abs(const f32 f);
f32 		f32_max(const f32 a, const f32 b);
f32 		f32_min(const f32 a, const f32 b);
f32 		f32_clamp(const f32 val, const f32 min, const f32 max);
f32 		f32_round(const f32 val);
f32 		f32_sqrt(const f32 f);
f32		f32_pow(const f32 f, const f32 power);

f32		f32_cos(const f32 f);
f32		f32_acos(const f32 f);
f32		f32_sin(const f32 f);
f32		f32_asin(const f32 f);
f32		f32_tan(const f32 f);
f32		f32_atan(const f32 f);

/* Return 1 if f is prefix, otherwise 0 */
enum ieee_type 	f32_classify(const f32 f);
u32 		f32_test_nan(const f32 f);
u32 		f32_test_positive_inf(const f32 f);
u32 		f32_test_negative_inf(const f32 f);
u32 		f32_test_positive_zero(const f32 f);
u32 		f32_test_negative_zero(const f32 f);
u32 		f32_test_normal(const f32 f);
u32 		f32_test_subnormal(const f32 f);

f32 		f32_nan(void);
f32 		f32_inf(const u32 sign);
f32 		f32_zero(const u32 sign);
f32 		f32_subnormal(const u32 sign, const u32 significand);

f32 		f32_max_positive_subnormal(void);
f32 		f32_min_positive_subnormal(void);
f32 		f32_max_negative_subnormal(void);
f32 		f32_min_negative_subnormal(void);

f32 		f32_max_positive_normal(void);
f32 		f32_min_positive_normal(void);
f32 		f32_max_negative_normal(void);
f32 		f32_min_negative_normal(void);

#ifdef __cplusplus
} 
#endif

#endif
