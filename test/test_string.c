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
#include <string.h>

#include "test_local.h"
#include "kas_string.h"
#include "dtoa.h"

static utf8 utf8_substring(struct arena *mem, const utf8 *string, const u32 start, const u32 len)
{
	utf8 substring;
	if (len)
	{
		substring.buf = ArenaPushMemcpy(mem, string->buf + start, len+1);
		substring.size = len+1;
		substring.len = len;
		substring.buf[len] = '\0';
	}
	else
	{
		substring = utf8_empty();
	}
	return substring;
}

static utf8 utf8_ascii_random(struct arena *mem, const u32 len)
{
	utf8 substring =
	{
		.buf = ArenaPush(mem, len+1),
		.size = len+1,
		.len = len,
	};
	for (u32 i = 0; i < len; ++i)
	{
		substring.buf[i] = (u8) RngU64Range((u64) 'A', (u64) 'Z');
	}
	substring.buf[len] = '\0';
	return substring;
}

static u32 utf8_ascii_substring_naive(const utf8 *string, const utf8 *substring)
{
	if (substring->len > string->len)
	{
		return 0;
	}
	else if (substring->len == 0)
	{
		return 1;
	}

	u32 found = 0;
	for (u32 start = 0; start < string->len - substring->len + 1; ++start)
	{
		found = 1;
		for (u32 i = 0; i < substring->len; ++i)
		{
			if (string->buf[start + i] != substring->buf[i])
			{
				found = 0;
				break;
			}	
		}

		if (found)
		{
			break;
		}
	}

	return found;
}	

static struct test_output utf8_lookup_substring_randomizer(struct test_environment *env)
{
	struct test_output output = { .success = 1, .id = __func__ };

	for (u32 i = 0; i < 1000000; ++i)
	{
		ArenaPushRecord(env->mem_1);

		const u32 string_len = (u32) RngU64Range(0, 20);
		const u32 random_len = (u32) RngU64Range(2, 4);
		const u32 substring_len = (u32) RngU64Range(0, string_len);
		const u32 substring_start = (substring_len)
					? (u32) RngU64Range(0, string_len-substring_len)
					: U32_MAX;

		const utf8 string = utf8_ascii_random(env->mem_1, string_len);
		const utf8 random = utf8_ascii_random(env->mem_1, random_len);
		const utf8 substring = utf8_substring(env->mem_1, &string, substring_start, substring_len);

		struct kmp_substring kmp_random = utf8_lookup_substring_init(env->mem_1, random);
		struct kmp_substring kmp_substring = utf8_lookup_substring_init(env->mem_1, substring);

		//if (substring_start != U32_MAX)
		//{
		//	for (u32 i = 0; i < substring_start; ++i)
		//	{
		//		fprintf(stderr, " ");
		//	}
		//}
		//for (u32 i = 0; i < substring_len; ++i)
		//{
		//	if (kmp_substring.backtrack[i] != U32_MAX)
		//	{
		//		fprintf(stderr, "%u", kmp_substring.backtrack[i]);
		//	}
		//	else
		//	{
		//		fprintf(stderr, " ");
		//	}
		//}
		//fprintf(stderr, "\n");
		//if (substring_start != U32_MAX)
		//{
		//	for (u32 i = 0; i < substring_start; ++i)
		//	{
		//		fprintf(stderr, " ");
		//	}
		//}
		//fprintf(stderr, "%s\n", (char *) substring.buf);
		//fprintf(stderr, "%s\n", (char *) string.buf);
		//fprintf(stderr, "%s\n", (char *) random.buf);

		const u32 random_found = utf8_lookup_substring(&kmp_random, string);	
		const u32 substring_found = utf8_lookup_substring(&kmp_substring, string);	

		TEST_EQUAL(substring_found, 1);
		TEST_EQUAL(random_found, utf8_ascii_substring_naive(&string, &random));

		ArenaPopRecord(env->mem_1);
	}

	return output;
}

static struct test_output DmgDtoa_functionallity_check(struct test_environment *env)
{
	struct test_output output = { .success = 1, .id = __func__ };

	char *str;
	i32 decpt, sign;

	str = DmgDtoa(1.25, 0, 0, &decpt, &sign, NULL);
	TEST_EQUAL(sign, 0);
	TEST_EQUAL(decpt, 1)
	TEST_EQUAL(str[0], '1');
	TEST_EQUAL(str[1], '2');
	TEST_EQUAL(str[2], '5');
	TEST_EQUAL(str[3], '\0');
	DmgDtoaFree(str);

	str = DmgDtoa(-1.25, 0, 0, &decpt, &sign, NULL);
	TEST_EQUAL(sign, 1);
	TEST_EQUAL(decpt, 1)
	TEST_EQUAL(str[0], '1');
	TEST_EQUAL(str[1], '2');
	TEST_EQUAL(str[2], '5');
	TEST_EQUAL(str[3], '\0');
	DmgDtoaFree(str);


	str = DmgDtoa(1.25, 2, 3, &decpt, &sign, NULL);
	TEST_EQUAL(sign, 0);
	TEST_EQUAL(decpt, 1)
	TEST_EQUAL(str[0], '1');
	TEST_EQUAL(str[1], '2');
	TEST_EQUAL(str[2], '5');
	TEST_EQUAL(str[3], '\0');
	DmgDtoaFree(str);

	str = DmgDtoa(-1.25, 2, 3, &decpt, &sign, NULL);
	TEST_EQUAL(sign, 1);
	TEST_EQUAL(decpt, 1)
	TEST_EQUAL(str[0], '1');
	TEST_EQUAL(str[1], '2');
	TEST_EQUAL(str[2], '5');
	TEST_EQUAL(str[3], '\0');
	DmgDtoaFree(str);

	str = DmgDtoa(1.0625, 3, 3, &decpt, &sign, NULL);
	TEST_EQUAL(sign, 0);
	TEST_EQUAL(decpt, 1)
	TEST_EQUAL(str[0], '1');
	TEST_EQUAL(str[1], '0');
	TEST_EQUAL(str[2], '6');
	TEST_EQUAL(str[3], '2');
	DmgDtoaFree(str);

	str = DmgDtoa(1.0, 0, 0, &decpt, &sign, NULL);
	TEST_EQUAL(sign, 0);
	TEST_EQUAL(decpt, 1)
	TEST_EQUAL(str[0], '1');
	TEST_EQUAL(str[1], '\0');
	DmgDtoaFree(str);

	str = DmgDtoa(0.125, 0, 0, &decpt, &sign, NULL);
	TEST_EQUAL(sign, 0);
	TEST_EQUAL(decpt, 0)
	TEST_EQUAL(str[0], '1');
	TEST_EQUAL(str[1], '2');
	TEST_EQUAL(str[2], '5');
	TEST_EQUAL(str[3], '\0');
	DmgDtoaFree(str);

	str = DmgDtoa(0.0625, 0, 0, &decpt, &sign, NULL);
	TEST_EQUAL(sign, 0);
	TEST_EQUAL(decpt, -1)
	TEST_EQUAL(str[0], '6');
	TEST_EQUAL(str[1], '2');
	TEST_EQUAL(str[2], '5');
	TEST_EQUAL(str[3], '\0');
	DmgDtoaFree(str);

	return output;
}

static struct test_output DmgStrtod_dtoa_equivalence(struct test_environment *env)
{
	struct test_output output = { .success = 1, .id = __func__ };

	char text[4096];
	char *buf = text;

	int decpt, sign;
	char *tmp;
	u32 passed = 0;
	u32 skipped = 0;
	for (u32 i = 0; i < U32_MAX / 100; ++i)
	{
		const f64 d = ((b64) { .u = RngU64() }).f;
		char *str = DmgDtoa(d, 0, 0, &decpt, &sign, NULL);
		const int pw = decpt-1;
		//if (pw < 0)
		{
			if (decpt == 9999)
			{
				(sign)
					? snprintf(buf, sizeof(text), "-%s", str)
					: snprintf(buf, sizeof(text),  "%s", str);
			}
			else
			{
				(sign)
					? snprintf(buf, sizeof(text), "-%c.%se%i", str[0], str+1, pw)
					: snprintf(buf, sizeof(text),  "%c.%se%i", str[0], str+1, pw);
			}
		}
		//else
		//{
		//	buf = str;
		//}
		//fprintf(stderr, "Expected:\t%.21e\nReturned:\t%s\n", d, buf);
		const f64 ret = DmgStrtod(buf, &tmp);
		if (-F32_INFINITY < ret && ret < F32_INFINITY)
		{
			TEST_EQUAL(d, ret);
			passed += 1;
		}
		else
		{
			skipped += 1;
		}
		DmgDtoaFree(str);
	}

	//fprintf(stderr, "Passed: %u, Skipped: %u\n", passed, skipped);

	return output;
}

static struct test_output DmgStrtod_utf8_f64_equivalence(struct test_environment *env)
{
	struct test_output output = { .success = 1, .id = __func__ };

	char text[4096];
	char *buf = text;

	int decpt, sign;
	char *tmp;
	u32 passed = 0;
	u32 skipped = 0;
	for (u32 i = 0; i < U32_MAX / 1000; ++i)
	{
		ArenaPushRecord(env->mem_1);

		const f64 d = ((b64) { .u = RngU64() }).f;
		const utf8 str = utf8_f64(env->mem_1, 0, d);
		const f64 ret = f64_utf8(env->mem_1, str);

		//fprintf(stderr, "expected:\t%.27e\ngot:     \t%.27e\n", d, ret);

		if (-F32_INFINITY < ret && ret < F32_INFINITY)
		{
			TEST_EQUAL(d, ret);
			passed += 1;
		}
		else
		{
			skipped += 1;
		}

		ArenaPopRecord(env->mem_1);
	}

	//fprintf(stderr, "Passed: %u, Skipped: %u\n", passed, skipped);

	return output;
}

static struct test_output DmgStrtod_utf32_f64_equivalence(struct test_environment *env)
{
	struct test_output output = { .success = 1, .id = __func__ };

	char text[4096];
	char *buf = text;

	int decpt, sign;
	char *tmp;
	u32 passed = 0;
	u32 skipped = 0;
	for (u32 i = 0; i < U32_MAX / 1000; ++i)
	{
		ArenaPushRecord(env->mem_1);

		const f64 d = ((b64) { .u = RngU64() }).f;
		const utf32 str = utf32_f64(env->mem_1, 0, d);
		const f64 ret = f64_utf32(env->mem_1, str);

		//fprintf(stderr, "expected:\t%.27e\ngot:     \t%.27e\n", d, ret);

		if (-F32_INFINITY < ret && ret < F32_INFINITY)
		{
			TEST_EQUAL(d, ret);
			passed += 1;
		}
		else
		{
			skipped += 1;
		}

		ArenaPopRecord(env->mem_1);
	}

	//fprintf(stderr, "Passed: %u, Skipped: %u\n", passed, skipped);

	return output;
}

static struct test_output utf8_utf32_u64_i64_equivalence(struct test_environment *env)
{
	struct test_output output = { .success = 1, .id = __func__ };

	char text[4096];
	char *buf = text;

	int decpt, sign;
	char *tmp;

	TEST_EQUAL(0, u64_utf8(utf8_u64(env->mem_1, 0)).u64);
	TEST_EQUAL(0, i64_utf8(utf8_i64(env->mem_1, 0)).i64);
	TEST_EQUAL(0, u64_utf32(utf32_u64(env->mem_1, 0)).u64);
	TEST_EQUAL(0, u64_utf32(utf32_i64(env->mem_1, 0)).i64);

	TEST_EQUAL(U64_MAX, u64_utf8(utf8_u64(env->mem_1,   U64_MAX)).u64);
	TEST_EQUAL(I64_MAX, i64_utf8(utf8_i64(env->mem_1,   I64_MAX)).i64);
	TEST_EQUAL(U64_MAX, u64_utf32(utf32_u64(env->mem_1, U64_MAX)).u64);
	TEST_EQUAL(I64_MAX, i64_utf32(utf32_i64(env->mem_1, I64_MAX)).i64);

	TEST_EQUAL(I64_MIN, i64_utf8(utf8_i64(env->mem_1,   I64_MIN)).u64);
	TEST_EQUAL(I64_MIN, i64_utf32(utf32_i64(env->mem_1, I64_MIN)).i64);
	TEST_EQUAL(-1, i64_utf8(utf8_i64(env->mem_1,   -1)).u64);
	TEST_EQUAL(-1, i64_utf32(utf32_i64(env->mem_1, -1)).i64);
	
	TEST_EQUAL(PARSE_SUCCESS, u64_utf8(utf8_u64(env->mem_1, 0)).op_result);
	TEST_EQUAL(PARSE_SUCCESS, i64_utf8(utf8_i64(env->mem_1, 0)).op_result);
	TEST_EQUAL(PARSE_SUCCESS, u64_utf32(utf32_u64(env->mem_1, 0)).op_result);
	TEST_EQUAL(PARSE_SUCCESS, u64_utf32(utf32_i64(env->mem_1, 0)).op_result);

	TEST_EQUAL(PARSE_SUCCESS,   u64_utf8( utf8_u64(env->mem_1,  U64_MAX)).op_result);
	TEST_EQUAL(PARSE_SUCCESS,   i64_utf8( utf8_i64(env->mem_1,  I64_MAX)).op_result);
	TEST_EQUAL(PARSE_SUCCESS, u64_utf32(utf32_u64(env->mem_1, U64_MAX)).op_result);
	TEST_EQUAL(PARSE_SUCCESS, i64_utf32(utf32_i64(env->mem_1, I64_MAX)).op_result);

	TEST_EQUAL(PARSE_SUCCESS, i64_utf8(utf8_i64(env->mem_1,   I64_MIN)).op_result);
	TEST_EQUAL(PARSE_SUCCESS, i64_utf32(utf32_i64(env->mem_1, I64_MIN)).op_result);
	TEST_EQUAL(PARSE_SUCCESS, i64_utf8(utf8_i64(env->mem_1,   -1)).op_result);
	TEST_EQUAL(PARSE_SUCCESS, i64_utf32(utf32_i64(env->mem_1, -1)).op_result);
	
	utf8 str;
	utf32 str32;

	str = (utf8) { .buf = "18446744073709551616", .len = strlen("18446744073709551616") };
	str32 = utf32_utf8(env->mem_1, str);
	TEST_EQUAL(PARSE_OVERFLOW, u64_utf8(str).op_result);
	TEST_EQUAL(PARSE_OVERFLOW, u64_utf32(str32).op_result);

	str = (utf8) { .buf = "18446744073709551616000", .len = strlen("18446744073709551616000") };
	str32 = utf32_utf8(env->mem_1, str);
	TEST_EQUAL(PARSE_OVERFLOW, u64_utf8(str).op_result);
	TEST_EQUAL(PARSE_OVERFLOW, u64_utf32(str32).op_result);
	  
	str = (utf8) { .buf = "-0", .len = strlen("-0") };
	str32 = utf32_utf8(env->mem_1, str);
	TEST_EQUAL(PARSE_STRING_INVALID, u64_utf8(str).op_result);
	TEST_EQUAL(PARSE_STRING_INVALID, u64_utf32(str32).op_result);

	str = (utf8) { .buf = "-0", .len = strlen("-0") };
	str32 = utf32_utf8(env->mem_1, str);
	TEST_EQUAL(PARSE_SUCCESS, i64_utf8(str).op_result);
	TEST_EQUAL(PARSE_SUCCESS, i64_utf32(str32).op_result);
	TEST_EQUAL(0, i64_utf8(str).i64);
	TEST_EQUAL(0, i64_utf32(str32).i64);

	str = (utf8) { .buf = "-9223372036854775809", .len = strlen("-9223372036854775809") };
	str32 = utf32_utf8(env->mem_1, str);
	TEST_EQUAL(PARSE_UNDERFLOW, i64_utf8(str).op_result);
	TEST_EQUAL(PARSE_UNDERFLOW, i64_utf32(str32).op_result);

	str = (utf8) { .buf = "-9223372036854775809000", .len = strlen("-9223372036854775809000") };
	str32 = utf32_utf8(env->mem_1, str);
	TEST_EQUAL(PARSE_UNDERFLOW, i64_utf8(str).op_result);
	TEST_EQUAL(PARSE_UNDERFLOW, i64_utf32(str32).op_result);

	str = (utf8) { .buf = "9223372036854775808", .len = strlen("9223372036854775808") };
	str32 = utf32_utf8(env->mem_1, str);
	TEST_EQUAL(PARSE_OVERFLOW, i64_utf8(str).op_result);
	TEST_EQUAL(PARSE_OVERFLOW, i64_utf32(str32).op_result);
	  
	str = (utf8) { .buf = "92233720368547758080", .len = strlen("92233720368547758080") };
	str32 = utf32_utf8(env->mem_1, str);
	TEST_EQUAL(PARSE_OVERFLOW, i64_utf8(str).op_result);
	TEST_EQUAL(PARSE_OVERFLOW, i64_utf32(str32).op_result);

	for (u32 i = 0; i < U32_MAX / 1000; ++i)
	{
		ArenaPushRecord(env->mem_1);

		const b64 b = { .u = RngU64() };

		const utf8 uf8_u64 = utf8_u64(env->mem_1, b.u);
		const utf8 uf8_i64 = utf8_i64(env->mem_1, b.i);
		const utf32 uf32_u64 = utf32_u64(env->mem_1, b.u);
		const utf32 uf32_i64 = utf32_i64(env->mem_1, b.i);

		const struct parse_retval ret1 = u64_utf8(uf8_u64);
		const struct parse_retval ret2 = i64_utf8(uf8_i64);
		const struct parse_retval ret3 = u64_utf32(uf32_u64);
		const struct parse_retval ret4 = i64_utf32(uf32_i64);

		const u64 u1 = ret1.u64;
		const u64 i1 = ret2.i64;
		const u64 u2 = ret3.u64;
		const u64 i2 = ret4.i64;
		
		TEST_EQUAL(ret1.op_result, PARSE_SUCCESS);
		TEST_EQUAL(ret2.op_result, PARSE_SUCCESS);
		TEST_EQUAL(ret3.op_result, PARSE_SUCCESS);
		TEST_EQUAL(ret4.op_result, PARSE_SUCCESS);

		TEST_EQUAL(u1, b.u);
		TEST_EQUAL(i1, b.i);
		TEST_EQUAL(u2, b.u);
		TEST_EQUAL(i2, b.i);

		ArenaPopRecord(env->mem_1);
	}

	return output;
}

static struct test_output(*kas_string_tests[])(struct test_environment *) =
{
	DmgStrtod_utf32_f64_equivalence,
	DmgStrtod_utf8_f64_equivalence,
	DmgDtoa_functionallity_check,
	DmgStrtod_dtoa_equivalence,
	utf8_utf32_u64_i64_equivalence,
	utf8_lookup_substring_randomizer,
};

struct suite m_kas_string_suite =
{
	.id = "string",
	.unit_test = kas_string_tests,
	.unit_test_count = sizeof(kas_string_tests) / sizeof(kas_string_tests[0]),
};

struct suite *kas_string_suite = &m_kas_string_suite;
