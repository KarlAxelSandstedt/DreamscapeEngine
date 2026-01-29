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

#include <math.h>
#include <float.h>

#include "ds_base.h"
#include "float32.h"

static void f32_static_assert(void)
{
	ds_StaticAssert(F32_EPSILON == FLT_EPSILON, "Our machine epsilon does not equate to FLT_EPSILON");
}

static u32 ieee32_bit(const union ieee32 f, const u32 bit)
{
	assert(bit <= 31);

	return (f.bits & (1 << bit)) >> bit;
}

static void ieee32_print(FILE *file, const union ieee32 val)
{
	i32 i = 31;
	fprintf(file, "ieee32:\t");
	fprintf(file, "%u", ieee32_bit(val, i--));
	fprintf(file, " ");
	for (; 22 < i; --i) { fprintf(file, "%u", ieee32_bit(val, i)); }
	fprintf(file, " ");
	for (; 0 <= i; --i) { fprintf(file, "%u", ieee32_bit(val, i)); }
	fprintf(file, "\n");
}

static u32 ieee32_sign_bit(const union ieee32 f)
{
	return (f.bits & F32_SIGN_MASK) >> (F32_EXPONENT_LENGTH + F32_SIGNIFICAND_LENGTH);
}

static u32 ieee32_exponent_bits(const union ieee32 f)
{
	return (f.bits & F32_EXPONENT_MASK) >> (F32_SIGNIFICAND_LENGTH);
}

static u32 ieee32_mantissa_bits(const union ieee32 f)
{
	return (f.bits & F32_SIGNIFICAND_MASK);
}

f32 f32_construct(const u32 sign_bit, const u32 exponent_bits, const u32 mantissa_bits)
{
	union ieee32 b;

	b.bits = (sign_bit << (F32_EXPONENT_LENGTH + F32_SIGNIFICAND_LENGTH))
		| (exponent_bits << F32_SIGNIFICAND_LENGTH)
		| mantissa_bits;

	return b.f;
}

static u32 ieee32_test_nan(const f32 f)
{
	union ieee32 val = { .f = f };	

	const u32 is_nan = (((val.bits & F32_EXPONENT_MASK) == F32_EXPONENT_MASK) 
			 && ((val.bits & F32_SIGNIFICAND_MASK) > 0))
		? 1
		: 0;

	return is_nan;
}

static u32 ieee32_test_positive_inf(const f32 f)
{
	union ieee32 val = { .f = f };

	const u32 is_positive_inf = (val.bits == F32_EXPONENT_MASK) ? 1 : 0;

	return is_positive_inf;
}

static u32 ieee32_test_negative_inf(const f32 f)
{
	union ieee32 val = { .f = f };

	const u32 is_negative_inf = (val.bits == (F32_SIGN_MASK | F32_EXPONENT_MASK)) ? 1 : 0;

	return is_negative_inf;
}

static u32 ieee32_test_subnormal(const f32 f)
{
	union ieee32 val = { .f = f };

	return (((val.bits & F32_EXPONENT_MASK) == 0) && ((val.bits & F32_SIGNIFICAND_MASK) > 0))
		? 1
		: 0;
}

static u32 ieee32_test_positive_zero(const f32 f)
{
	union ieee32 val = { .f = f };

	const u32 is_positive_zero = (val.bits == 0) ? 1 : 0;

	return is_positive_zero;
}

static u32 ieee32_test_negative_zero(const f32 f)
{
	union ieee32 val = { .f = f };

	const u32 is_negative_zero = (val.bits == F32_SIGN_MASK) ? 1 : 0;

	return is_negative_zero;
}


u32 f32_sign_bit(const f32 f)
{
	union ieee32 val = { .f = f };
	return ieee32_sign_bit(val);
}

u32 f32_exponent_bits(const f32 f)
{
	union ieee32 val = { .f = f };
	return ieee32_exponent_bits(val);
}

u32 f32_mantissa_bits(const f32 f)
{
	union ieee32 val = { .f = f };
	return ieee32_mantissa_bits(val);
}

f32 f32_sign(const f32 f)
{
	union ieee32 val = { .f = f };
	return -2.0f * ieee32_sign_bit(val) + 1.0f;
}

f32 f32_abs(const f32 f)
{
	union ieee32 val = { .f = f };
	val.bits &= F32_EXPONENT_MASK | F32_SIGNIFICAND_MASK;
	return val.f;
}

u32 f32_test_nan(const f32 f)
{
	union ieee32 val = { .f = f };	

	const u32 is_nan = (((val.bits & F32_EXPONENT_MASK) == F32_EXPONENT_MASK) 
			 && ((val.bits & F32_SIGNIFICAND_MASK) > 0))
		? 1
		: 0;

	return is_nan;
}

u32 f32_test_positive_inf(const f32 f)
{
	union ieee32 val = { .f = f };

	const u32 is_positive_inf = (val.bits == F32_EXPONENT_MASK) ? 1 : 0;

	return is_positive_inf;
}

u32 f32_test_negative_inf(const f32 f)
{
	union ieee32 val = { .f = f };

	const u32 is_negative_inf = (val.bits == (F32_SIGN_MASK | F32_EXPONENT_MASK)) ? 1 : 0;

	return is_negative_inf;
}

u32 f32_test_normal(const f32 f)
{
	union ieee32 val = { .f = f };

	const i32 s = (i32) ((val.bits & F32_EXPONENT_MASK) >> F32_SIGNIFICAND_LENGTH) - F32_BIAS;

	return (F32_MIN_EXPONENT <= s && s <= F32_MAX_EXPONENT) ? 1 : 0;
}

u32 f32_test_subnormal(const f32 f)
{
	union ieee32 val = { .f = f };

	return (((val.bits & F32_EXPONENT_MASK) == 0) && ((val.bits & F32_SIGNIFICAND_MASK) > 0))
		? 1
		: 0;
}

u32 f32_test_positive_zero(const f32 f)
{
	union ieee32 val = { .f = f };

	const u32 is_positive_zero = (val.bits == 0) ? 1 : 0;

	return is_positive_zero;
}

u32 f32_test_negative_zero(const f32 f)
{
	union ieee32 val = { .f = f };

	const u32 is_negative_zero = (val.bits == F32_SIGN_MASK) ? 1 : 0;

	return is_negative_zero;
}

f32 f32_inf(const u32 sign)
{
	union ieee32 val = { .bits = F32_EXPONENT_MASK };

	val.bits |= (sign) ? F32_SIGN_MASK : 0;

	return val.f;
}

f32 f32_zero(const u32 sign)
{
	union ieee32 val;

	val.bits = (sign) ? F32_SIGN_MASK : 0;

	return val.f;
}

f32 f32_nan(void)
{
	union ieee32 val = { .bits = F32_EXPONENT_MASK | F32_SIGNIFICAND_MASK };

	return val.f;
}

f32 f32_max_positive_subnormal(void)
{
	union ieee32 val = { .bits = F32_SIGNIFICAND_MASK };

	return val.f;
}

f32 f32_min_positive_subnormal(void)
{
	union ieee32 val = { .bits = 0x1 };

	return val.f;
}

f32 f32_max_negative_subnormal(void)
{
	union ieee32 val = { .bits = F32_SIGN_MASK | 0x1 };

	return val.f;
}

f32 f32_min_negative_subnormal(void)
{
	union ieee32 val = { .bits = F32_SIGN_MASK | F32_SIGNIFICAND_MASK };

	return val.f;
}

f32 f32_max_positive_normal(void)
{
	union ieee32 val = { .bits = (F32_EXPONENT_MASK & 0x7f000000) | F32_SIGNIFICAND_MASK };

	return val.f;
}

f32 f32_min_positive_normal(void)
{
	union ieee32 val = { .bits = (F32_EXPONENT_MASK & 0x00800000) };

	return val.f;
}

f32 f32_max_negative_normal(void)
{
	union ieee32 val = { .bits = F32_SIGN_MASK | (F32_EXPONENT_MASK & 0x00800000) };

	return val.f;
}

f32 f32_min_negative_normal(void)
{
	union ieee32 val = { .bits = F32_SIGN_MASK | (F32_EXPONENT_MASK & 0x7f000000) | F32_SIGNIFICAND_MASK };

	return val.f;
}

enum ieee_type f32_classify(const f32 f)
{
	if (ieee32_test_nan(f)) return IEEE_NAN;	
	else if (ieee32_test_positive_inf(f) || ieee32_test_negative_inf(f)) return IEEE_INF;	
	else if (ieee32_test_positive_zero(f) || ieee32_test_negative_zero(f)) return IEEE_ZERO;	
	else if (ieee32_test_subnormal(f)) return IEEE_SUBNORMAL;	
	else return IEEE_NORMAL;	
}

void f32_bits_print(FILE *file, const f32 f)
{
	const union ieee32 val = { .f = f };
	ieee32_print(file, val);
}

f32 f32_clamp(const f32 val, const f32 min, const f32 max)
{
	if (val <= min) return min;
	if (val >= max) return max;

	return val;
}

f32 f32_max(const f32 a, const f32 b)
{
	return fmaxf(a, b);
}

f32 f32_min(const f32 a, const f32 b)
{
	return fminf(a, b);
}

f32 f32_round(const f32 val)
{
	return nearbyintf(val);
}

f32 f32_sqrt(const f32 f)
{
	return sqrtf(f);
}

f32 f32_cos(const f32 f)
{
	return cosf(f);
}

f32 f32_acos(const f32 f)
{
	return acosf(f);
}

f32 f32_sin(const f32 f)
{
	return sinf(f);
}

f32 f32_asin(const f32 f)
{
	return asinf(f);
}

f32 f32_tan(const f32 f)
{
	return tanf(f);
}

f32 f32_atan(const f32 f)
{
	return atanf(f);
}

f32 f32_pow(const f32 f, const f32 power)
{
	return powf(f, power);
}
