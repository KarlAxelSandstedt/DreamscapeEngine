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

#define F32_PI				3.14159274101257324f	/* pi rounded to f32 */
#define F32_PI2    			(2.0f * F32_PI)	/* 2 pi (not pi/2) */

#define	F32_SIGN_LENGTH			1
#define	F32_EXPONENT_LENGTH		8
#define	F32_SIGNIFICAND_LENGTH		23

#define	F32_SIGN_MASK			0x80000000
#define	F32_EXPONENT_MASK		0x7f800000
#define	F32_SIGNIFICAND_MASK		0x007fffff

#define	F32_BIAS			127	/* exponent bias */
#define	F32_MAX_EXPONENT		127	/* unbiased; normal f32 have F32_MIN_EXPONENT .. F32_MAX_EXPONENT */
#define	F32_MIN_EXPONENT		-126

#define	F32_MAX_POSITIVE_SUBNORMAL	F32MaxPositiveSubnormal()	/* MAX/MIN_POSITIVE: largest/smallest positive of the class */
#define	F32_MIN_POSITIVE_SUBNORMAL	F32MinPositiveSubnormal()
#define	F32_MAX_NEGATIVE_SUBNORMAL	F32MaxNegativeSubnormal()	/* MAX_NEGATIVE: the negative closest to 0; MIN_NEGATIVE: the most negative */
#define	F32_MIN_NEGATIVE_SUBNORMAL	F32MinNegativeSubnormal()

#define	F32_MAX_POSITIVE_NORMAL		F32MaxPositiveNormal()
#define	F32_MIN_POSITIVE_NORMAL		F32MinPositiveNormal()
#define	F32_MAX_NEGATIVE_NORMAL		F32MaxNegativeNormal()
#define	F32_MIN_NEGATIVE_NORMAL		F32MinNegativeNormal()

#define	F32_INFINITY			F32Inf(0)	/* +inf */
#define	F32_EPSILON			1.1920929e-7f	/* FLT_EPSILON, 2^-23 */

/*********************** bit access and classification ***********************/

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

/* assemble an f32 from its fields, not masked: sign_bit 0 or 1, exponent_bits < 256, mantissa_bits < 2^23 */
static ds_ForceInline f32 F32Construct(const u32 sign_bit, const u32 exponent_bits, const u32 mantissa_bits);
/* return sign bit shifted down */
static ds_ForceInline u32 F32SignBit(const f32 f);
/* return exponent bits  shifted down */
static ds_ForceInline u32 F32ExponentBits(const f32 f);
/* return mantissa bits */
static ds_ForceInline u32 F32MantissaBits(const f32 f);

/* 1 if f is NaN (signaling NaNs included); like all tests here it reads the bits only and never raises */
static ds_ForceInline u32 F32TestNan(const f32 f);
/* 1 if f is +inf */
static ds_ForceInline u32 F32TestPositiveInf(const f32 f);
/* 1 if f is -inf */
static ds_ForceInline u32 F32TestNegativeInf(const f32 f);
/* 1 if f is +0 */
static ds_ForceInline u32 F32TestPositiveZero(const f32 f);
/* 1 if f is -0 */
static ds_ForceInline u32 F32TestNegativeZero(const f32 f);
/* 1 if f is normal (either sign) */
static ds_ForceInline u32 F32TestNormal(const f32 f);
/* 1 if f is subnormal (either sign) */
static ds_ForceInline u32 F32TestSubnormal(const f32 f);
/* return f's class (NaN, inf, zero, subnormal, normal) */
static ds_ForceInline enum ieee_type F32Classify(const f32 f);
/* 1 if f is neither NaN nor Inf */
static ds_ForceInline u32 F32FiniteCheck(const f32 f);

/********************** sign, min, max, clamp, round ***********************/

/* return sign 1.0f or -1.0f: -1.0f when the sign bit is set (-0 and NaNs with the sign bit included) */
static ds_ForceInline f32 F32Sign(const f32 f);
/* |f|: clears the sign bit (NaN stays NaN) */
static ds_ForceInline f32 F32Abs(const f32 f);
/* the larger of a and b by one comparison: a NaN a gives b, a NaN b is returned */
static ds_ForceInline f32 F32Max(const f32 a, const f32 b);
/* the smaller of a and b by one comparison: a NaN a gives b, a NaN b is returned */
static ds_ForceInline f32 F32Min(const f32 a, const f32 b);
/* min if val <= min, max if val >= max, else val (NaN passes through; min <= max is not checked) */
static ds_ForceInline f32 F32Clamp(const f32 val, const f32 min, const f32 max);
/* the nearest integer, halves to even (0.5 -> 0, 1.5 -> 2, -0.5 -> -0) */
static ds_ForceInline f32 F32Round(const f32 val);

/************************* C library functions **************************/

/* sqrtf: correctly rounded, the same bits on every platform */
static ds_ForceInline f32 F32Sqrt(const f32 f);
/* powf: not correctly rounded, may differ in the last bit between platforms */
static ds_ForceInline f32 F32Pow(const f32 f, const f32 power);
/* cosf (radians): not correctly rounded, may differ in the last bit between platforms */
static ds_ForceInline f32 F32Cos(const f32 f);
/* acosf (radians): not correctly rounded, may differ in the last bit between platforms */
static ds_ForceInline f32 F32Acos(const f32 f);
/* sinf (radians): not correctly rounded, may differ in the last bit between platforms */
static ds_ForceInline f32 F32Sin(const f32 f);
/* asinf (radians): not correctly rounded, may differ in the last bit between platforms */
static ds_ForceInline f32 F32Asin(const f32 f);
/* tanf (radians): not correctly rounded, may differ in the last bit between platforms */
static ds_ForceInline f32 F32Tan(const f32 f);
/* atanf (radians): not correctly rounded, may differ in the last bit between platforms */
static ds_ForceInline f32 F32Atan(const f32 f);
/* atan2f(y, x) (radians, in [-pi, pi]; (0, 0) gives 0): not correctly rounded, may differ in the last bit between platforms */
static ds_ForceInline f32 F32Atan2(const f32 y, const f32 x);

/*********************** directed rounding from f64 ************************/

/*
 * x rounded to f32 toward -inf (Downwards) or +inf (Upwards): the nearest f32, stepped once outward if the cast
 * rounded the other way. nextafterf is an exact IEEE operation, so this is deterministic. NaN stays NaN.
 * Beyond the f32 range: +-FLT_MAX or +-inf.
 */
static ds_ForceInline f32 F64DemoteDownwards(const f64 x);
static ds_ForceInline f32 F64DemoteUpwards(const f64 x);

/****************************** special values ******************************/

/* a quiet NaN (all significand bits set) */
static ds_ForceInline f32 F32Nan(void);
/* +inf if sign is 0, else -inf */
static ds_ForceInline f32 F32Inf(const u32 sign);
/* +0 if sign is 0, else -0 */
static ds_ForceInline f32 F32Zero(const u32 sign);
/* largest positive subnormal */
static ds_ForceInline f32 F32MaxPositiveSubnormal(void);
/* smallest positive subnormal */
static ds_ForceInline f32 F32MinPositiveSubnormal(void);
/* negative subnormal closest to 0 */
static ds_ForceInline f32 F32MaxNegativeSubnormal(void);
/* most negative subnormal */
static ds_ForceInline f32 F32MinNegativeSubnormal(void);
/* largest positive normal (FLT_MAX) */
static ds_ForceInline f32 F32MaxPositiveNormal(void);
/* smallest positive normal (FLT_MIN) */
static ds_ForceInline f32 F32MinPositiveNormal(void);
/* negative normal closest to 0 (-FLT_MIN) */
static ds_ForceInline f32 F32MaxNegativeNormal(void);
/* most negative normal (-FLT_MAX) */
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

static ds_ForceInline f32 F32Atan2(const f32 y, const f32 x)
{
	return atan2f(y, x);
}

/*************** directed rounding from f64: inline definitions ***************/

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
