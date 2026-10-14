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
#include <string.h>

#include "ds_test.h"
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
		substring = Utf8Empty();
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

static u32 Utf8AsciiSubstringNaive(const utf8 *string, const utf8 *substring)
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

static struct test_Output utf8_lookup_substring_randomizer(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

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

		struct kmpSubstring kmp_random = Utf8LookupSubstringInit(env->mem_1, random);
		struct kmpSubstring kmpSubstring = Utf8LookupSubstringInit(env->mem_1, substring);

		//if (substring_start != U32_MAX)
		//{
		//	for (u32 i = 0; i < substring_start; ++i)
		//	{
		//		fprintf(stderr, " ");
		//	}
		//}
		//for (u32 i = 0; i < substring_len; ++i)
		//{
		//	if (kmpSubstring.backtrack[i] != U32_MAX)
		//	{
		//		fprintf(stderr, "%u", kmpSubstring.backtrack[i]);
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

		const u32 random_found = Utf8LookupSubstring(&kmp_random, string);	
		const u32 substring_found = Utf8LookupSubstring(&kmpSubstring, string);	

		TEST_EQUAL(substring_found, 1);
		TEST_EQUAL(random_found, Utf8AsciiSubstringNaive(&string, &random));

		ArenaPopRecord(env->mem_1);
	}

	return output;
}

static struct test_Output DmgDtoa_functionallity_check(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

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

static struct test_Output DmgStrtod_dtoa_equivalence(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

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

static struct test_Output DmgStrtod_utf8_f64_equivalence(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

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
		const utf8 str = Utf8F64(env->mem_1, 0, d);
		const f64 ret = F64Utf8(env->mem_1, str);

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

static struct test_Output DmgStrtod_utf32_f64_equivalence(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

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
		const utf32 str = Utf32F64(env->mem_1, 0, d);
		const f64 ret = F64Utf32(env->mem_1, str);

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

static struct test_Output utf8_Utf32U64_i64_equivalence(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

	char text[4096];
	char *buf = text;

	int decpt, sign;
	char *tmp;

	TEST_EQUAL(0, U64Utf8(Utf8U64(env->mem_1, 0)).u64);
	TEST_EQUAL(0, I64Utf8(Utf8I64(env->mem_1, 0)).i64);
	TEST_EQUAL(0, U64Utf32(Utf32U64(env->mem_1, 0)).u64);
	TEST_EQUAL(0, U64Utf32(Utf32I64(env->mem_1, 0)).i64);

	TEST_EQUAL(U64_MAX, U64Utf8(Utf8U64(env->mem_1,   U64_MAX)).u64);
	TEST_EQUAL(I64_MAX, I64Utf8(Utf8I64(env->mem_1,   I64_MAX)).i64);
	TEST_EQUAL(U64_MAX, U64Utf32(Utf32U64(env->mem_1, U64_MAX)).u64);
	TEST_EQUAL(I64_MAX, I64Utf32(Utf32I64(env->mem_1, I64_MAX)).i64);

	TEST_EQUAL(I64_MIN, I64Utf8(Utf8I64(env->mem_1,   I64_MIN)).i64);
	TEST_EQUAL(I64_MIN, I64Utf32(Utf32I64(env->mem_1, I64_MIN)).i64);
	TEST_EQUAL(-1,      I64Utf8(Utf8I64(env->mem_1,   -1)).i64);
	TEST_EQUAL(-1,      I64Utf32(Utf32I64(env->mem_1, -1)).i64);
	
	TEST_EQUAL(PARSE_SUCCESS, U64Utf8(Utf8U64(env->mem_1, 0)).op_result);
	TEST_EQUAL(PARSE_SUCCESS, I64Utf8(Utf8I64(env->mem_1, 0)).op_result);
	TEST_EQUAL(PARSE_SUCCESS, U64Utf32(Utf32U64(env->mem_1, 0)).op_result);
	TEST_EQUAL(PARSE_SUCCESS, U64Utf32(Utf32I64(env->mem_1, 0)).op_result);

	TEST_EQUAL(PARSE_SUCCESS, U64Utf8(Utf8U64(env->mem_1,  U64_MAX)).op_result);
	TEST_EQUAL(PARSE_SUCCESS, I64Utf8(Utf8I64(env->mem_1,  I64_MAX)).op_result);
	TEST_EQUAL(PARSE_SUCCESS, U64Utf32(Utf32U64(env->mem_1, U64_MAX)).op_result);
	TEST_EQUAL(PARSE_SUCCESS, I64Utf32(Utf32I64(env->mem_1, I64_MAX)).op_result);

	TEST_EQUAL(PARSE_SUCCESS, I64Utf8(Utf8I64(env->mem_1,   I64_MIN)).op_result);
	TEST_EQUAL(PARSE_SUCCESS, I64Utf32(Utf32I64(env->mem_1, I64_MIN)).op_result);
	TEST_EQUAL(PARSE_SUCCESS, I64Utf8(Utf8I64(env->mem_1,   -1)).op_result);
	TEST_EQUAL(PARSE_SUCCESS, I64Utf32(Utf32I64(env->mem_1, -1)).op_result);
	
	utf8 str;
	utf32 str32;

	str = Utf8Inline("18446744073709551616");
	str32 = Utf32Utf8(env->mem_1, str);
	TEST_EQUAL(PARSE_OVERFLOW, U64Utf8(str).op_result);
	TEST_EQUAL(PARSE_OVERFLOW, U64Utf32(str32).op_result);

	str = Utf8Inline("18446744073709551616000");
	str32 = Utf32Utf8(env->mem_1, str);
	TEST_EQUAL(PARSE_OVERFLOW, U64Utf8(str).op_result);
	TEST_EQUAL(PARSE_OVERFLOW, U64Utf32(str32).op_result);
	  
	str = Utf8Inline("-0");
	str32 = Utf32Utf8(env->mem_1, str);
	TEST_EQUAL(PARSE_STRING_INVALID, U64Utf8(str).op_result);
	TEST_EQUAL(PARSE_STRING_INVALID, U64Utf32(str32).op_result);

	str = Utf8Inline("-0");
	str32 = Utf32Utf8(env->mem_1, str);
	TEST_EQUAL(PARSE_SUCCESS, I64Utf8(str).op_result);
	TEST_EQUAL(PARSE_SUCCESS, I64Utf32(str32).op_result);
	TEST_EQUAL(0, I64Utf8(str).i64);
	TEST_EQUAL(0, I64Utf32(str32).i64);

	str = Utf8Inline("-9223372036854775809");
	str32 = Utf32Utf8(env->mem_1, str);
	TEST_EQUAL(PARSE_UNDERFLOW, I64Utf8(str).op_result);
	TEST_EQUAL(PARSE_UNDERFLOW, I64Utf32(str32).op_result);

	str = Utf8Inline("-9223372036854775809000");
	str32 = Utf32Utf8(env->mem_1, str);
	TEST_EQUAL(PARSE_UNDERFLOW, I64Utf8(str).op_result);
	TEST_EQUAL(PARSE_UNDERFLOW, I64Utf32(str32).op_result);

	str = Utf8Inline("9223372036854775808");
	str32 = Utf32Utf8(env->mem_1, str);
	TEST_EQUAL(PARSE_OVERFLOW, I64Utf8(str).op_result);
	TEST_EQUAL(PARSE_OVERFLOW, I64Utf32(str32).op_result);
	  
	str = Utf8Inline("92233720368547758080");
	str32 = Utf32Utf8(env->mem_1, str);
	TEST_EQUAL(PARSE_OVERFLOW, I64Utf8(str).op_result);
	TEST_EQUAL(PARSE_OVERFLOW, I64Utf32(str32).op_result);

	for (u32 i = 0; i < U32_MAX / 1000; ++i)
	{
		ArenaPushRecord(env->mem_1);

		const b64 b = { .u = RngU64() };

		const utf8 uf8_u64 = Utf8U64(env->mem_1, b.u);
		const utf8 uf8_i64 = Utf8I64(env->mem_1, b.i);
		const utf32 uf32_u64 = Utf32U64(env->mem_1, b.u);
		const utf32 uf32_i64 = Utf32I64(env->mem_1, b.i);

		const struct parseRetval ret1 = U64Utf8(uf8_u64);
		const struct parseRetval ret2 = I64Utf8(uf8_i64);
		const struct parseRetval ret3 = U64Utf32(uf32_u64);
		const struct parseRetval ret4 = I64Utf32(uf32_i64);

		const u64 u1 = ret1.u64;
		const i64 i1 = ret2.i64;
		const u64 u2 = ret3.u64;
		const i64 i2 = ret4.i64;
		
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

static u32 FormatTestUtf32Equal(struct arena *mem, const utf32 str, const char *expected)
{
	const utf32 exp = Utf32Utf8(mem, Utf8Cstr(mem, expected));
	if (exp.len != str.len) { return 0; }
	for (u64 i = 0; i < exp.len; ++i)
	{
		if (exp.buf[i] != str.buf[i]) { return 0; }
	}
	return 1;
}

/* str has len codepoints spelled by exactly the bytes of expected (a C string) */
static u32 FormatTestBytes(const utf8 str, const u32 len, const char *expected)
{
	const u64 bytes = strlen(expected);
	if (str.len != len || Utf8SizeRequired(str) != bytes)
	{
		return 0;
	}
	for (u64 i = 0; i < bytes; ++i)
	{
		if (str.buf[i] != (u8) expected[i]) { return 0; }
	}
	return 1;
}

/*
 * Formatter regressions: decimals within the leading zeros (exact-size buffer, no overrun), multi-digit
 * decimal counts, an empty %s, "%%", and a '%' inside a %s argument (what LogString now relies on).
 */
static struct test_Output utf8_format_regressions(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

	u8 buf[64];
	u32 buf32[64];

	/* 0.0001: four leading decimal zeroes; asking for <= 4 decimals writes only zeroes */
	TEST_TRUE(Utf8Equivalence(Utf8Inline("0.00"), Utf8F64Buffered(buf, sizeof(buf), 2, 0.0001)));
	TEST_TRUE(Utf8Equivalence(Utf8Inline("0.000"), Utf8F64Buffered(buf, sizeof(buf), 3, 0.0001)));
	TEST_TRUE(Utf8Equivalence(Utf8Inline("-0.0"), Utf8F64Buffered(buf, sizeof(buf), 1, -0.0001)));
	TEST_TRUE(Utf8Equivalence(Utf8Inline("0.01"), Utf8F64Buffered(buf, sizeof(buf), 2, 0.0123)));
	memset(buf, 'x', sizeof(buf));
	TEST_TRUE(Utf8Equivalence(Utf8Inline("0.00"), Utf8F64Buffered(buf, 4, 2, 0.0001)));
	TEST_EQUAL('x', buf[4]);

	TEST_TRUE(FormatTestUtf32Equal(env->mem_1, Utf32F64Buffered(buf32, 64, 2, 0.0001), "0.00"));
	TEST_TRUE(FormatTestUtf32Equal(env->mem_1, Utf32F64Buffered(buf32, 64, 2, 0.0123), "0.01"));
	for (u32 i = 0; i < 64; ++i) { buf32[i] = 'x'; }
	TEST_TRUE(FormatTestUtf32Equal(env->mem_1, Utf32F64Buffered(buf32, 4, 2, 0.0001), "0.00"));
	TEST_EQUAL('x', buf32[4]);

	/* multi-digit decimal counts */
	TEST_TRUE(Utf8Equivalence(Utf8Inline("1.5000000000"), Utf8FormatBuffered(buf, sizeof(buf), "%10f", 1.5)));
	TEST_TRUE(Utf8Equivalence(Utf8Inline("1.500000000000"), Utf8FormatBuffered(buf, sizeof(buf), "%12f", 1.5)));

	/* an empty %s writes nothing and formatting goes on */
	TEST_TRUE(Utf8Equivalence(Utf8Inline("a  b 7"), Utf8FormatBuffered(buf, sizeof(buf), "a %s b %u", "", 7)));
	TEST_TRUE(Utf8Equivalence(Utf8Inline("x"), Utf8FormatBuffered(buf, sizeof(buf), "%sx%s", "", "")));

	/* "%%" and a '%' in an argument */
	TEST_TRUE(Utf8Equivalence(Utf8Inline("100%"), Utf8FormatBuffered(buf, sizeof(buf), "100%%")));
	TEST_TRUE(Utf8Equivalence(Utf8Inline("%5%"), Utf8FormatBuffered(buf, sizeof(buf), "%%%u%%", 5)));
	TEST_TRUE(Utf8Equivalence(Utf8Inline("50% done %s"), Utf8FormatBuffered(buf, sizeof(buf), "%s", "50% done %s")));

	/* non-ASCII %s arguments and format characters: offsets advance by bytes, len counts codepoints */
	TEST_TRUE(FormatTestBytes(Utf8FormatBuffered(buf, sizeof(buf), "%s|%u", "\xC3\xA9\xE2\x82\xAC", 7), 4, "\xC3\xA9\xE2\x82\xAC|7"));
	TEST_TRUE(FormatTestBytes(Utf8FormatBuffered(buf, sizeof(buf), "\xC3\xA9=%u\xE2\x82\xAC", 5), 4, "\xC3\xA9=5\xE2\x82\xAC"));
	TEST_TRUE(FormatTestBytes(Utf8FormatBuffered(buf, sizeof(buf), "\xF0\x9F\x98\x80%s\xF0\x9F\x98\x80", "\xC3\xA9"), 3, "\xF0\x9F\x98\x80\xC3\xA9\xF0\x9F\x98\x80"));
	/*
	 * a bad byte in the format is one (bad) codepoint, as the decoder reads it back; the '\0' after the
	 * output keeps stale continuation bytes from completing a cut-off sequence
	 */
	memset(buf, 0xAC, sizeof(buf));
	TEST_TRUE(FormatTestBytes(Utf8FormatBuffered(buf, sizeof(buf), "\xC3" "A%u", 1), 3, "\xC3" "A1"));
	TEST_TRUE(FormatTestBytes(Utf8FormatBuffered(buf, sizeof(buf), "x\xE2\x82", 0), 3, "x\xE2\x82"));
	/* a sequence that does not fit is not split */
	TEST_EQUAL(0, Utf8FormatBuffered(buf, 1, "\xC3\xA9").len);
	TEST_TRUE(FormatTestBytes(Utf8FormatBuffered(buf, 2, "\xC3\xA9"), 1, "\xC3\xA9"));
	/* 300 x U+00E4, twice (was cut to half: each %s advanced by codepoints) */
	char umlauts[601];
	for (u32 i = 0; i < 300; ++i) { umlauts[2*i] = (char) 0xC3; umlauts[2*i+1] = (char) 0xA4; }
	umlauts[600] = '\0';
	u8 big[2048];
	const utf8 twice = Utf8FormatBuffered(big, sizeof(big), "%s%s", umlauts, umlauts);
	TEST_EQUAL(600, twice.len);
	TEST_EQUAL(1200, Utf8SizeRequired(twice));
	TEST_TRUE(Utf8Equivalence(twice, Utf8Format(env->mem_1, "%s%s", umlauts, umlauts)));

	/* Utf8Format keeps exactly the bytes */
	const utf8 formatted = Utf8Format(env->mem_1, "%s-\xE2\x82\xAC-%s", "\xC3\xA9", "\xC3\xA9");
	TEST_EQUAL(5, formatted.len);
	TEST_EQUAL(9, formatted.size);

	return output;
}

/* Decode bytes[0, size) at offset; check the codepoint and the new offset. */
static u32 DecodeTestCheck(const char *bytes, const u32 size, const u64 offset, const u32 expected, const u64 expected_offset)
{
	const utf8 str = { .buf = (u8 *) bytes, .size = size, .len = 0 };
	u64 new_offset = 0;
	const u32 codepoint = Utf8ReadCodepoint(&new_offset, &str, offset);
	return codepoint == expected && new_offset == expected_offset;
}

/*
 * Utf8ReadCodepoint on valid, truncated and invalid input (overlong, surrogate, > U+10FFFF, bad lead and
 * continuation bytes; bad sequences advance one byte), Utf8WriteCodepoint's refusals, random bytes
 * (decoding stays within size; every good codepoint re-encodes to its bytes), integer formatting into
 * minimal buffers, integer parsing of "" and "-", F64 rounding and F64Utf8's arena use, KMP start index.
 */
static struct test_Output utf8_decode_and_parse_regressions(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

	TEST_TRUE(DecodeTestCheck("A", 1, 0, 'A', 1));
	TEST_TRUE(DecodeTestCheck("\xC3\xA9", 2, 0, 0xE9, 2));
	TEST_TRUE(DecodeTestCheck("\xE2\x82\xAC", 3, 0, 0x20AC, 3));
	TEST_TRUE(DecodeTestCheck("\xF0\x9F\x98\x80", 4, 0, 0x1F600, 4));
	TEST_TRUE(DecodeTestCheck("\xF4\x8F\xBF\xBF", 4, 0, 0x10FFFF, 4));

	/* cut off by size: the bytes after it are never read */
	TEST_TRUE(DecodeTestCheck("\xE2\x82\xAC", 2, 0, UTF8_BAD_CODEPOINT, 1));
	TEST_TRUE(DecodeTestCheck("\xE2\x82\xAC", 2, 1, UTF8_BAD_CODEPOINT, 2));
	TEST_TRUE(DecodeTestCheck("\xE2\x82\xAC", 2, 2, UTF8_BAD_CODEPOINT, 3));
	TEST_TRUE(DecodeTestCheck("\xF0\x9F\x98\x80", 3, 0, UTF8_BAD_CODEPOINT, 1));

	/* overlong, surrogate, beyond U+10FFFF, bad lead bytes */
	TEST_TRUE(DecodeTestCheck("\xC0\x80", 2, 0, UTF8_BAD_CODEPOINT, 1));
	TEST_TRUE(DecodeTestCheck("\xC1\xBF", 2, 0, UTF8_BAD_CODEPOINT, 1));
	TEST_TRUE(DecodeTestCheck("\xE0\x9F\xBF", 3, 0, UTF8_BAD_CODEPOINT, 1));
	TEST_TRUE(DecodeTestCheck("\xF0\x8F\xBF\xBF", 4, 0, UTF8_BAD_CODEPOINT, 1));
	TEST_TRUE(DecodeTestCheck("\xED\xA0\x80", 3, 0, UTF8_BAD_CODEPOINT, 1));
	TEST_TRUE(DecodeTestCheck("\xED\xBF\xBF", 3, 0, UTF8_BAD_CODEPOINT, 1));
	TEST_TRUE(DecodeTestCheck("\xF4\x90\x80\x80", 4, 0, UTF8_BAD_CODEPOINT, 1));
	TEST_TRUE(DecodeTestCheck("\xF5\x80\x80\x80", 4, 0, UTF8_BAD_CODEPOINT, 1));
	TEST_TRUE(DecodeTestCheck("\xF8\x88\x80\x80\x80", 5, 0, UTF8_BAD_CODEPOINT, 1));
	TEST_TRUE(DecodeTestCheck("\xFF", 1, 0, UTF8_BAD_CODEPOINT, 1));
	TEST_TRUE(DecodeTestCheck("\x80", 1, 0, UTF8_BAD_CODEPOINT, 1));

	/* a bad continuation byte is read again as a lead byte */
	TEST_TRUE(DecodeTestCheck("\xC3" "A", 2, 0, UTF8_BAD_CODEPOINT, 1));
	TEST_TRUE(DecodeTestCheck("\xC3" "A", 2, 1, 'A', 2));

	/* Utf8CstrBuffered counts only the copied bytes, not stale continuation bytes after them */
	u8 buf[64];
	memset(buf, 0xAC, sizeof(buf));
	/* size stays the capacity; the '\0' after the copy keeps a cut-off sequence bad */
	const utf8 cut = Utf8CstrBuffered(buf, sizeof(buf), "A\xE2\x82");
	TEST_EQUAL(3, cut.len);
	TEST_EQUAL(sizeof(buf), cut.size);
	TEST_EQUAL(3, Utf8SizeRequired(cut));
	memset(buf + 8, 0xAC, sizeof(buf) - 8);
	const utf8 cut_copy = Utf8CopyBuffered(buf + 8, sizeof(buf) - 8, cut);
	TEST_EQUAL(3, cut_copy.len);
	TEST_EQUAL(3, Utf8SizeRequired(cut_copy));
	memset(buf, 0xAC, sizeof(buf));
	const utf8 cut_format = Utf8FormatBuffered(buf, sizeof(buf), "%u\xE2\x82", 7);
	TEST_EQUAL(3, cut_format.len);
	TEST_EQUAL(3, Utf8SizeRequired(cut_format));

	/* a len that claims more codepoints than size holds: the size, and so a copy, stops at size */
	const utf8 lying = { .buf = buf, .size = 2, .len = 10 };
	TEST_EQUAL(2, Utf8SizeRequired(lying));
	u64 copied_bytes;
	Utf8CopyBufferedAndReturnRequiredSize(&copied_bytes, buf + 16, 32, lying);
	TEST_EQUAL(2, copied_bytes);

	TEST_EQUAL(0, Utf8WriteCodepoint(buf, 4, 0xD800));
	TEST_EQUAL(0, Utf8WriteCodepoint(buf, 4, 0xDFFF));
	TEST_EQUAL(0, Utf8WriteCodepoint(buf, 4, 0x110000));
	TEST_EQUAL(0, Utf8WriteCodepoint(buf, 4, UTF8_BAD_CODEPOINT));
	TEST_EQUAL(4, Utf8WriteCodepoint(buf, 4, 0x10FFFF));
	TEST_EQUAL(3, Utf8WriteCodepoint(buf, 4, 0xE000));

	/* random bytes, biased towards lead and continuation bytes */
	for (u32 iter = 0; iter < 20000; ++iter)
	{
		u8 bytes[16];
		const u32 size = (u32) RngU64Range(0, sizeof(bytes) - 4);
		for (u32 i = 0; i < sizeof(bytes); ++i)
		{
			const u64 r = RngU64();
			bytes[i] = (r & 1) ? (u8) (0x80 | ((r >> 1) & 0x3f)) : (u8) (r >> 8);
		}

		const utf8 str = { .buf = bytes, .size = size, .len = 0 };
		u64 offset = 0;
		while (offset < size)
		{
			u64 new_offset;
			const u32 codepoint = Utf8ReadCodepoint(&new_offset, &str, offset);
			TEST_TRUE(offset < new_offset && new_offset <= size);
			if (codepoint == UTF8_BAD_CODEPOINT)
			{
				TEST_EQUAL(offset + 1, new_offset);
			}
			else
			{
				u8 encoded[4];
				const u32 len = Utf8WriteCodepoint(encoded, sizeof(encoded), codepoint);
				TEST_EQUAL(new_offset - offset, len);
				for (u32 i = 0; i < len; ++i)
				{
					TEST_EQUAL(encoded[i], bytes[offset + i]);
				}
			}
			offset = new_offset;
		}
	}

	/* a sign and one digit fit exactly */
	TEST_TRUE(Utf8Equivalence(Utf8Inline("7"), Utf8I64Buffered(buf, 1, 7)));
	TEST_EQUAL(0, Utf8I64Buffered(buf, 1, -7).len);
	TEST_TRUE(Utf8Equivalence(Utf8Inline("-7"), Utf8I64Buffered(buf, 2, -7)));
	u32 buf32[4];
	TEST_EQUAL(1, Utf32I64Buffered(buf32, 1, 7).len);
	TEST_EQUAL('7', buf32[0]);
	TEST_EQUAL(0, Utf32I64Buffered(buf32, 1, -7).len);
	TEST_EQUAL(2, Utf32I64Buffered(buf32, 2, -7).len);

	/* no digits */
	TEST_EQUAL(PARSE_STRING_INVALID, U64Utf8(Utf8Inline("")).op_result);
	TEST_EQUAL(PARSE_STRING_INVALID, I64Utf8(Utf8Inline("")).op_result);
	TEST_EQUAL(PARSE_STRING_INVALID, I64Utf8(Utf8Inline("-")).op_result);
	TEST_EQUAL(PARSE_STRING_INVALID, U64Utf32(Utf32Utf8(env->mem_1, Utf8Inline(""))).op_result);
	TEST_EQUAL(PARSE_STRING_INVALID, I64Utf32(Utf32Utf8(env->mem_1, Utf8Inline(""))).op_result);
	TEST_EQUAL(PARSE_STRING_INVALID, I64Utf32(Utf32Utf8(env->mem_1, Utf8Inline("-"))).op_result);
	TEST_EQUAL(-12, I64Utf8(Utf8Inline("-12")).i64);

	/* rounded, not truncated (dtoa mode 3: exact binary value, ties to even) */
	TEST_TRUE(Utf8Equivalence(Utf8Inline("0.13"), Utf8F64Buffered(buf, sizeof(buf), 2, 0.129)));
	TEST_TRUE(Utf8Equivalence(Utf8Inline("1.00"), Utf8F64Buffered(buf, sizeof(buf), 2, 0.999)));
	TEST_TRUE(Utf8Equivalence(Utf8Inline("0.12"), Utf8F64Buffered(buf, sizeof(buf), 2, 0.125)));
	TEST_TRUE(Utf8Equivalence(Utf8Inline("-2.50"), Utf8F64Buffered(buf, sizeof(buf), 2, -2.4999999)));
	TEST_TRUE(Utf8Equivalence(Utf8Inline("100000000000000000000.00"), Utf8F64Buffered(buf, sizeof(buf), 2, 1e20)));
	TEST_TRUE(Utf8Equivalence(Utf8Inline("2.5"), Utf8F64Buffered(buf, sizeof(buf), 0, 2.5)));
	u32 buf32f[32];
	TEST_TRUE(FormatTestUtf32Equal(env->mem_1, Utf32F64Buffered(buf32f, 32, 2, 0.129), "0.13"));

	/* F64Utf8 gives its copy back */
	const u64 mem_left = env->mem_1->mem_left;
	for (u32 i = 0; i < 1000; ++i)
	{
		TEST_EQUAL(1.5, F64Utf8(env->mem_1, Utf8Inline("1.5")));
	}
	TEST_EQUAL(mem_left, env->mem_1->mem_left);

	/* start is a codepoint index */
	struct kmpSubstring kmp = Utf8LookupSubstringInit(env->mem_1, Utf8Inline("b"));
	TEST_TRUE(Utf8LookupSubstring(&kmp, Utf8Inline("\xC3\xA9" "b")));
	TEST_EQUAL(1, kmp.start);

	return output;
}

static struct test_Output(*kas_string_tests[])(struct test_Environment *) =
{
	utf8_format_regressions,
	utf8_decode_and_parse_regressions,
	DmgStrtod_utf32_f64_equivalence,
	DmgStrtod_utf8_f64_equivalence,
	DmgDtoa_functionallity_check,
	DmgStrtod_dtoa_equivalence,
	utf8_Utf32U64_i64_equivalence,
	utf8_lookup_substring_randomizer,
};

struct suite_Correctness m_kas_string_suite =
{
	.id = "string",
	.unit_test = kas_string_tests,
	.unit_test_count = sizeof(kas_string_tests) / sizeof(kas_string_tests[0]),
};

struct suite_Correctness *kas_string_correctness_suite = &m_kas_string_suite;
