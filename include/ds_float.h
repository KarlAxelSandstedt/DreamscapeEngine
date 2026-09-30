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

#ifndef __DS_FLOAT_H__
#define __DS_FLOAT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <math.h>

#include "ds_define.h"
#include "ds_types.h"
#include "ds_base.h"

#define F32_PI				3.14159274101257324f
#define F32_PI2    			(2.0f * F32_PI)

#define	F32_SIGN_LENGTH			1
#define	F32_EXPONENT_LENGTH		8
#define	F32_SIGNIFICAND_LENGTH		23

#define	F32_SIGN_MASK			0x80000000
#define	F32_EXPONENT_MASK		0x7f800000
#define	F32_SIGNIFICAND_MASK		0x007fffff

#define	F32_BIAS			127
#define	F32_MAX_EXPONENT		127
#define	F32_MIN_EXPONENT		-126

#define	F32_MAX_POSITIVE_SUBNORMAL	F32MaxPositiveSubnormal()
#define	F32_MIN_POSITIVE_SUBNORMAL	F32MinPositiveSubnormal()
#define	F32_MAX_NEGATIVE_SUBNORMAL	F32MaxNegativeSubnormal()
#define	F32_MIN_NEGATIVE_SUBNORMAL	F32MinNegativeSubnormal()

#define	F32_MAX_POSITIVE_NORMAL		F32MaxPositiveNormal()
#define	F32_MIN_POSITIVE_NORMAL		F32MinPositiveNormal()
#define	F32_MAX_NEGATIVE_NORMAL		F32MaxNegativeNormal()
#define	F32_MIN_NEGATIVE_NORMAL		F32MinNegativeNormal()

#define	F32_INFINITY			F32Inf(0)
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

static ds_ForceInline f32 F32Construct(const u32 sign_bit, const u32 exponent_bits, const u32 mantissa_bits)
{
	union ieee32 b;
	b.bits = (sign_bit << (F32_EXPONENT_LENGTH + F32_SIGNIFICAND_LENGTH))
		| (exponent_bits << F32_SIGNIFICAND_LENGTH)
		| mantissa_bits;
	return b.f;
}

/* return sign bit shifted down */
static ds_ForceInline u32 F32SignBit(const f32 f)
{
	const union ieee32 val = { .f = f };
	return (val.bits & F32_SIGN_MASK) >> (F32_EXPONENT_LENGTH + F32_SIGNIFICAND_LENGTH);
}

/* return exponent bits  shifted down */
static ds_ForceInline u32 F32ExponentBits(const f32 f)
{
	const union ieee32 val = { .f = f };
	return (val.bits & F32_EXPONENT_MASK) >> (F32_SIGNIFICAND_LENGTH);
}

/* return mantissa bits */
static ds_ForceInline u32 F32MantissaBits(const f32 f)
{
	const union ieee32 val = { .f = f };
	return (val.bits & F32_SIGNIFICAND_MASK);
}

/* return sign 1.0f or -1.0f */
static ds_ForceInline f32 F32Sign(const f32 f)
{
	return -2.0f * F32SignBit(f) + 1.0f;
}

static ds_ForceInline f32 F32Abs(const f32 f)
{
	union ieee32 val = { .f = f };
	val.bits &= F32_EXPONENT_MASK | F32_SIGNIFICAND_MASK;
	return val.f;
}

static ds_ForceInline f32 F32Max(const f32 a, const f32 b)
{
	return fmaxf(a, b);
}

static ds_ForceInline f32 F32Min(const f32 a, const f32 b)
{
	return fminf(a, b);
}

static ds_ForceInline f32 F32Clamp(const f32 val, const f32 min, const f32 max)
{
	if (val <= min) return min;
	if (val >= max) return max;

	return val;
}

static ds_ForceInline f32 F32Round(const f32 val)
{
	return nearbyintf(val);
}

static ds_ForceInline f32 F32Sqrt(const f32 f)
{
	return sqrtf(f);
}

static ds_ForceInline f32 F32Pow(const f32 f, const f32 power)
{
	return powf(f, power);
}

static ds_ForceInline f32 F32Cos(const f32 f)
{
	return cosf(f);
}

static ds_ForceInline f32 F32Acos(const f32 f)
{
	return acosf(f);
}

static ds_ForceInline f32 F32Sin(const f32 f)
{
	return sinf(f);
}

static ds_ForceInline f32 F32Asin(const f32 f)
{
	return asinf(f);
}

static ds_ForceInline f32 F32Tan(const f32 f)
{
	return tanf(f);
}

static ds_ForceInline f32 F32Atan(const f32 f)
{
	return atanf(f);
}

static ds_ForceInline u32 F32TestNan(const f32 f)
{
	const union ieee32 val = { .f = f };
	return ((val.bits & F32_EXPONENT_MASK) == F32_EXPONENT_MASK) && ((val.bits & F32_SIGNIFICAND_MASK) > 0);
}

static ds_ForceInline u32 F32TestPositiveInf(const f32 f)
{
	const union ieee32 val = { .f = f };
	return val.bits == F32_EXPONENT_MASK;
}

static ds_ForceInline u32 F32TestNegativeInf(const f32 f)
{
	const union ieee32 val = { .f = f };
	return val.bits == (F32_SIGN_MASK | F32_EXPONENT_MASK);
}

static ds_ForceInline u32 F32TestPositiveZero(const f32 f)
{
	const union ieee32 val = { .f = f };
	return val.bits == 0;
}

static ds_ForceInline u32 F32TestNegativeZero(const f32 f)
{
	const union ieee32 val = { .f = f };
	return val.bits == F32_SIGN_MASK;
}

static ds_ForceInline u32 F32TestNormal(const f32 f)
{
	const union ieee32 val = { .f = f };
	const i32 s = (i32) ((val.bits & F32_EXPONENT_MASK) >> F32_SIGNIFICAND_LENGTH) - F32_BIAS;
	return F32_MIN_EXPONENT <= s && s <= F32_MAX_EXPONENT;
}

static ds_ForceInline u32 F32TestSubnormal(const f32 f)
{
	const union ieee32 val = { .f = f };
	return ((val.bits & F32_EXPONENT_MASK) == 0) && ((val.bits & F32_SIGNIFICAND_MASK) > 0);
}

static ds_ForceInline enum ieee_type F32Classify(const f32 f)
{
	if (F32TestNan(f)) return IEEE_NAN;
	else if (F32TestPositiveInf(f) || F32TestNegativeInf(f)) return IEEE_INF;
	else if (F32TestPositiveZero(f) || F32TestNegativeZero(f)) return IEEE_ZERO;
	else if (F32TestSubnormal(f)) return IEEE_SUBNORMAL;
	else return IEEE_NORMAL;
}

static ds_ForceInline f32 F32Nan(void)
{
	const union ieee32 val = { .bits = F32_EXPONENT_MASK | F32_SIGNIFICAND_MASK };
	return val.f;
}

static ds_ForceInline f32 F32Inf(const u32 sign)
{
	union ieee32 val = { .bits = F32_EXPONENT_MASK };
	val.bits |= (sign) ? F32_SIGN_MASK : 0;
	return val.f;
}

static ds_ForceInline f32 F32Zero(const u32 sign)
{
	union ieee32 val;
	val.bits = (sign) ? F32_SIGN_MASK : 0;
	return val.f;
}

static ds_ForceInline f32 F32MaxPositiveSubnormal(void)
{
	const union ieee32 val = { .bits = F32_SIGNIFICAND_MASK };
	return val.f;
}

static ds_ForceInline f32 F32MinPositiveSubnormal(void)
{
	const union ieee32 val = { .bits = 0x1 };
	return val.f;
}

static ds_ForceInline f32 F32MaxNegativeSubnormal(void)
{
	const union ieee32 val = { .bits = F32_SIGN_MASK | 0x1 };
	return val.f;
}

static ds_ForceInline f32 F32MinNegativeSubnormal(void)
{
	const union ieee32 val = { .bits = F32_SIGN_MASK | F32_SIGNIFICAND_MASK };
	return val.f;
}

static ds_ForceInline f32 F32MaxPositiveNormal(void)
{
	const union ieee32 val = { .bits = (F32_EXPONENT_MASK & 0x7f000000) | F32_SIGNIFICAND_MASK };
	return val.f;
}

static ds_ForceInline f32 F32MinPositiveNormal(void)
{
	const union ieee32 val = { .bits = (F32_EXPONENT_MASK & 0x00800000) };
	return val.f;
}

static ds_ForceInline f32 F32MaxNegativeNormal(void)
{
	const union ieee32 val = { .bits = F32_SIGN_MASK | (F32_EXPONENT_MASK & 0x00800000) };
	return val.f;
}

static ds_ForceInline f32 F32MinNegativeNormal(void)
{
	const union ieee32 val = { .bits = F32_SIGN_MASK | (F32_EXPONENT_MASK & 0x7f000000) | F32_SIGNIFICAND_MASK };
	return val.f;
}

static inline void F32BitsPrint(FILE *file, const f32 f)
{
	const union ieee32 val = { .f = f };
	fprintf(file, "ieee32:\t");
	fprintf(file, "%u", (val.bits >> 31) & 0x1);
	fprintf(file, " ");
	for (i32 i = 30; 22 < i; --i) { fprintf(file, "%u", (val.bits >> i) & 0x1); }
	fprintf(file, " ");
	for (i32 i = 22; 0 <= i; --i) { fprintf(file, "%u", (val.bits >> i) & 0x1); }
	fprintf(file, "\n");
}

#ifdef __cplusplus
}
#endif

#endif
