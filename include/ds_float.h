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
ds_float.h
==========
IEEE 754 binary32 (f32): constants, bit access and classification, sign/min/max/clamp/round, the C library's
math functions under engine names, and f64 to f32 rounding toward -inf or +inf. Platform layer: the engine
includes math.h only here.

Determinism: everything here gives the same bits on every platform and compiler, except F32Pow and the
trigonometric functions. Those are the C library's, not correctly rounded, and differ in the last bit between
platforms (UCRT, glibc, musl): nothing the simulation computes may depend on them.

Sections: constants; bit access and classification; sign, min, max, clamp, round; C library functions;
directed rounding from f64; special values; printing. Inline definitions at the bottom, in the same order.
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

/******************************** constants *********************************/

/*
F32_PI is pi rounded to f32; F32_PI2 is 2 pi (not pi/2). F32_EPSILON is FLT_EPSILON, 2^-23. Exponents are
unbiased (normal f32 have F32_MIN_EXPONENT..F32_MAX_EXPONENT). The extreme values are function calls (special
values below): MAX_POSITIVE and MIN_POSITIVE are the largest and smallest positive value of their class,
MAX_NEGATIVE the negative value closest to zero, MIN_NEGATIVE the most negative one.
*/

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

/*********************** bit access and classification ***********************/

/*
union ieee32 views an f32 as its bits. The field functions return a field shifted down to bit 0;
F32Construct assembles one without masking, so sign_bit must be 0 or 1, exponent_bits < 256 and
mantissa_bits < 2^23.

The tests and F32Classify read the bits only: they never raise, signaling NaNs included. They return 1 or 0;
F32FiniteCheck is 1 unless f is NaN or +-inf. Normal and subnormal include both signs.
*/

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

static ds_ForceInline f32 F32Construct(const u32 sign_bit, const u32 exponent_bits, const u32 mantissa_bits);
static ds_ForceInline u32 F32SignBit(const f32 f);
static ds_ForceInline u32 F32ExponentBits(const f32 f);
static ds_ForceInline u32 F32MantissaBits(const f32 f);

static ds_ForceInline u32 F32TestNan(const f32 f);
static ds_ForceInline u32 F32TestPositiveInf(const f32 f);
static ds_ForceInline u32 F32TestNegativeInf(const f32 f);
static ds_ForceInline u32 F32TestPositiveZero(const f32 f);
static ds_ForceInline u32 F32TestNegativeZero(const f32 f);
static ds_ForceInline u32 F32TestNormal(const f32 f);
static ds_ForceInline u32 F32TestSubnormal(const f32 f);
static ds_ForceInline enum ieee_type F32Classify(const f32 f);
static ds_ForceInline u32 F32FiniteCheck(const f32 f);

/********************** sign, min, max, clamp, round ***********************/

/*
F32Sign is -1.0f when the sign bit is set (-0 and NaNs with the sign bit included), else 1.0f. F32Abs clears
the sign bit (NaN stays NaN).

F32Max and F32Min return the larger and the smaller argument by one comparison: a NaN first argument gives
the second, a NaN second argument is returned. F32Clamp returns min if val <= min, max if val >= max, else
val, so NaN passes through; min <= max is not checked.

F32Round rounds to the nearest integer, halves to even (0.5 -> 0, 1.5 -> 2, -0.5 -> -0).
*/

static ds_ForceInline f32 F32Sign(const f32 f);
static ds_ForceInline f32 F32Abs(const f32 f);
static ds_ForceInline f32 F32Max(const f32 a, const f32 b);
static ds_ForceInline f32 F32Min(const f32 a, const f32 b);
static ds_ForceInline f32 F32Clamp(const f32 val, const f32 min, const f32 max);
static ds_ForceInline f32 F32Round(const f32 val);

/************************* C library functions **************************/

/*
The C library's sqrtf, powf, cosf, ... Angles are in radians. F32Sqrt is correctly rounded (IEEE), the same
bits everywhere. F32Pow and the trigonometric functions are not (see the top of the file).
*/

static ds_ForceInline f32 F32Sqrt(const f32 f);
static ds_ForceInline f32 F32Pow(const f32 f, const f32 power);
static ds_ForceInline f32 F32Cos(const f32 f);
static ds_ForceInline f32 F32Acos(const f32 f);
static ds_ForceInline f32 F32Sin(const f32 f);
static ds_ForceInline f32 F32Asin(const f32 f);
static ds_ForceInline f32 F32Tan(const f32 f);
static ds_ForceInline f32 F32Atan(const f32 f);

/*********************** directed rounding from f64 ************************/

/*
x rounded to f32 toward -inf (Downwards) or +inf (Upwards): the largest f32 <= x, the smallest f32 >= x.
Beyond the f32 range that is +-FLT_MAX or +-inf; NaN stays NaN. Same bits everywhere.
*/

static ds_ForceInline f32 F64DemoteDownwards(const f64 x);
static ds_ForceInline f32 F64DemoteUpwards(const f64 x);

/****************************** special values ******************************/

/*
F32Nan is a quiet NaN (all significand bits set). F32Inf and F32Zero take the sign: 0 positive, nonzero
negative. The extreme values behind the F32_..._NORMAL and F32_..._SUBNORMAL constants.
*/

static ds_ForceInline f32 F32Nan(void);
static ds_ForceInline f32 F32Inf(const u32 sign);
static ds_ForceInline f32 F32Zero(const u32 sign);
static ds_ForceInline f32 F32MaxPositiveSubnormal(void);
static ds_ForceInline f32 F32MinPositiveSubnormal(void);
static ds_ForceInline f32 F32MaxNegativeSubnormal(void);
static ds_ForceInline f32 F32MinNegativeSubnormal(void);
static ds_ForceInline f32 F32MaxPositiveNormal(void);
static ds_ForceInline f32 F32MinPositiveNormal(void);
static ds_ForceInline f32 F32MaxNegativeNormal(void);
static ds_ForceInline f32 F32MinNegativeNormal(void);

/********************************* printing *********************************/

/* Print f's sign, exponent and significand bits to file, e.g. "ieee32:\t0 01111111 000...0\n" for 1.0f */
static inline void F32BitsPrint(FILE *file, const f32 f);

/************** bit access and classification: inline definitions **************/

static ds_ForceInline f32 F32Construct(const u32 sign_bit, const u32 exponent_bits, const u32 mantissa_bits)
{
	union ieee32 b;
	b.bits = (sign_bit << (F32_EXPONENT_LENGTH + F32_SIGNIFICAND_LENGTH))
		| (exponent_bits << F32_SIGNIFICAND_LENGTH)
		| mantissa_bits;
	return b.f;
}

static ds_ForceInline u32 F32SignBit(const f32 f)
{
	const union ieee32 val = { .f = f };
	return (val.bits & F32_SIGN_MASK) >> (F32_EXPONENT_LENGTH + F32_SIGNIFICAND_LENGTH);
}

static ds_ForceInline u32 F32ExponentBits(const f32 f)
{
	const union ieee32 val = { .f = f };
	return (val.bits & F32_EXPONENT_MASK) >> (F32_SIGNIFICAND_LENGTH);
}

static ds_ForceInline u32 F32MantissaBits(const f32 f)
{
	const union ieee32 val = { .f = f };
	return (val.bits & F32_SIGNIFICAND_MASK);
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

static ds_ForceInline u32 F32FiniteCheck(const f32 f)
{
	const union ieee32 val = { .f = f };
	return (val.bits & F32_EXPONENT_MASK) != F32_EXPONENT_MASK;
}

/************** sign, min, max, clamp, round: inline definitions **************/

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
	return (a > b) ? a : b;
}

static ds_ForceInline f32 F32Min(const f32 a, const f32 b)
{
	return (a < b) ? a : b;
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

/***************** C library functions: inline definitions *****************/

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

/*************** directed rounding from f64: inline definitions ***************/

/* the nearest f32, stepped once outward if the cast rounded the other way; nextafterf is exact */
static ds_ForceInline f32 F64DemoteDownwards(const f64 x)
{
	const f32 f = (f32) x;
	return ((f64) f > x) ? nextafterf(f, -F32_INFINITY) : f;
}

static ds_ForceInline f32 F64DemoteUpwards(const f64 x)
{
	const f32 f = (f32) x;
	return ((f64) f < x) ? nextafterf(f, F32_INFINITY) : f;
}

/******************** special values: inline definitions ********************/

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

/*********************** printing: inline definitions ***********************/

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
