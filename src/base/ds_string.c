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
#include "ds_base.h"

#include "dtoa.h"
#define XXH_INLINE_ALL
#include "xxhash.h"

u32 WordbreakCheck(const u32 codepoint)
{
	u32 wordbreak = 0;
	switch (codepoint)
	{
		case ' ':
		case '\t':
		case '\n':
		case '=':
		case '-':
		case ':':
		case ';':
		case '\\':
		case '/':
		{
			wordbreak = 1;
		} break;

		default: { } break;
	}

	return wordbreak;
}

const utf8 empty_utf8 = { .len = 0, .size = 0, .buf = (u8 *) "" };
const utf32 empty_utf32 = { .len = 0, .max_len = 0, .buf = NULL };

utf32 Utf32StreamConsumeWhitespace(utf32 *stream)
{
	utf32 sub = { .len = 0, .buf = stream->buf };
	for (; sub.len < stream->len; ++sub.len)
	{
		if (stream->buf[sub.len] != ' ' && stream->buf[sub.len] != '\n' && stream->buf[sub.len] != '\t')
		{
			break;
		}
	}
	sub.max_len = sub.len;

	stream->len -= sub.len;
	stream->max_len -= sub.len;
	stream->buf += sub.len;
	return sub;
}

utf32 Utf32StreamConsumeNonWhitespace(utf32 *stream)
{
	utf32 sub = { .len = 0, .buf = stream->buf };
	for (; sub.len < stream->len; ++sub.len)
	{
		if (stream->buf[sub.len] == ' ' || stream->buf[sub.len] == '\n' || stream->buf[sub.len] == '\t')
		{
			break;
		}
	}
	sub.max_len = sub.len;

	stream->len -= sub.len;
	stream->max_len -= sub.len;
	stream->buf += sub.len;
	return sub;
}

u32 Utf8ReadCodepoint(u64 *new_offset, const utf8 *str, const u64 offset)
{
	/* a bad sequence consumes one byte, so decoding resynchronizes at the next lead byte */
	*new_offset = offset + 1;
	if (offset >= str->size)
	{
		return UTF8_BAD_CODEPOINT;
	}

	const u8 *b = str->buf + offset;
	const u32 b0 = b[0];
	if (b0 < 0x80)
	{
		return b0;
	}

	/*
	 * Each case reads only within str->size and folds its checks into one test: continuation bytes are
	 * 10xx xxxx, and overlong forms, UTF-16 surrogates and values beyond U+10FFFF (F4 90+, F5-F7) are
	 * bad. Measured faster than a branch-free form (clamped reads, blended result) on ASCII, Latin and
	 * mixed text: the sequence size is well predicted.
	 */
	const u64 left = str->size - offset;
	/* never 0: the low 24 bits are ones */
	const u32 sequence_size = Clz32(~(b0 << 24));
	u32 decoded;
	switch (sequence_size)
	{
		case 2:
		{
			if (left < 2) { return UTF8_BAD_CODEPOINT; }
			const u32 b1 = b[1];
			decoded = (b0 & 0x1f) << 6 | (b1 & 0x3f);
			if (((b1 & 0xc0) ^ 0x80) | (decoded < 0x80))
			{
				return UTF8_BAD_CODEPOINT;
			}
		} break;

		case 3:
		{
			if (left < 3) { return UTF8_BAD_CODEPOINT; }
			const u32 b1 = b[1];
			const u32 b2 = b[2];
			decoded = (b0 & 0x0f) << 12 | (b1 & 0x3f) << 6 | (b2 & 0x3f);
			if (((b1 & 0xc0) ^ 0x80) | ((b2 & 0xc0) ^ 0x80) | (decoded < 0x800) | (decoded - 0xd800 < 0x800))
			{
				return UTF8_BAD_CODEPOINT;
			}
		} break;

		case 4:
		{
			if (left < 4) { return UTF8_BAD_CODEPOINT; }
			const u32 b1 = b[1];
			const u32 b2 = b[2];
			const u32 b3 = b[3];
			decoded = (b0 & 0x07) << 18 | (b1 & 0x3f) << 12 | (b2 & 0x3f) << 6 | (b3 & 0x3f);
			/* decoded - 0x10000 wraps for overlong forms */
			if (((b1 & 0xc0) ^ 0x80) | ((b2 & 0xc0) ^ 0x80) | ((b3 & 0xc0) ^ 0x80) | (decoded - 0x10000 > 0xfffff))
			{
				return UTF8_BAD_CODEPOINT;
			}
		} break;

		/* 1: a continuation byte as lead; 5-8: F8-FF */
		default:
		{
			return UTF8_BAD_CODEPOINT;
		}
	}

	*new_offset = offset + sequence_size;
	return decoded;
}

char *CstrUtf8(struct arena *mem, const utf8 utf8)
{	
	const u64 size = Utf8SizeRequired(utf8);
	char *ret = ArenaPush(mem, size+1); 
	if (ret)
	{
		memcpy(ret, utf8.buf, size);
		ret[size] = '\0';
	}	

	return (ret) ? ret : "";
}

f32 F32Cstr(char **new_offset, const char *str)
{
	return (f32) DmgStrtod(str, new_offset);
}

f64 F64Cstr(char **new_offset, const char *str)
{
	return DmgStrtod(str, new_offset);
}

f32 F32Utf8(struct arena *tmp, const utf8 str)
{
	return (f32) F64Utf8(tmp, str);
}

f64 F64Utf8(struct arena *tmp, const utf8 str)
{
	if (str.len == 0)
	{
		return 0;
	}

	/* the null-terminated copy is popped again, as in F64Utf32 */
	f64 val = 0.0;
	const u64 size = Utf8SizeRequired(str);
	char *cstr = ArenaPushPacked(tmp, size+1);
	if (cstr)
	{
		memcpy(cstr, str.buf, size);
		cstr[size] = '\0';
		val = DmgStrtod(cstr, NULL);
		ArenaPopPacked(tmp, size+1);
	}

	return val;
}

f32 F32Utf32(struct arena *tmp, const utf32 str)
{
	return (f32) F64Utf32(tmp, str);
}

f64 F64Utf32(struct arena *tmp, const utf32 str)
{
	f64 ret = 0.0f;
	char *buf = ArenaPushPacked(tmp, str.len+1);
	if (buf)
	{
		for (u32 i = 0; i < str.len; ++i)
		{
			buf[i] = (char) str.buf[i];	
		}
		buf[str.len] = '\0';
		ret = DmgStrtod(buf, NULL);
		ArenaPopPacked(tmp, str.len+1);
	}
	return ret;
}

u64 Utf8SizeRequired(const utf8 utf8)
{
	u64 size = 0;
	for (u32 i = 0; i < utf8.len; ++i)
	{
		Utf8ReadCodepoint(&size, &utf8, size);
	}
	/* len codepoints past size (a malformed utf8) read as bad bytes beyond it: never count those */
	return (size < utf8.size) ? size : utf8.size;
}

utf8 Utf8F32Buffered(u8 buf[], const u64 bufsize, const u32 decimals, const f32 val)
{                                                                                  
	return Utf8F64Buffered(buf, bufsize, decimals, (f64) val);
}                                                                                  
                                                                                   
utf8 Utf8F64Buffered(u8 buf[], const u64 bufsize, const u32 decimals, const f64 val)
{
	utf8 str = Utf8Empty();

	i32 sign;
	i32 decpt;
	/* decimals: correctly rounded to that many (mode 3); 0: shortest round-trip digits (mode 0) */
	char *dmg_str = (decimals)
		? DmgDtoa((f64) val, 3, (int) decimals, &decpt, &sign, NULL)
		: DmgDtoa((f64) val, 0, 0, &decpt, &sign, NULL);

	/* INF / Nan */
	if (decpt == 9999)
	{
		str = Utf8CstrBuffered(buf, bufsize, dmg_str);
	}
	else
	{
		const u64 dmg_len = strlen(dmg_str);
		u64 integers_in_dmg = 0; 
		u64 integer_zero_leading = 0; 
		u64 integer_zeroes_trailing = 0;
		u64 decimals_in_dmg = 0;
		u64 decimal_zeroes_leading = 0;
		u64 decimal_zeroes_trailing = 0;	
		u64 decimals_in_dmg_wanted = 0;
		u64 decimals_wanted = decimals;

		/* 0.**** */
		if (decpt < 1)
		{
			decimals_in_dmg = dmg_len;
			integer_zero_leading = 1;
			decimal_zeroes_leading = (u64) (-decpt);
		}
		/* ****.0 */
		else if ((u64) decpt >= dmg_len)
		{
			integers_in_dmg = dmg_len;
			integer_zeroes_trailing = (u64) decpt - dmg_len;
		}
		/* ****.**** */
		else
		{
			integers_in_dmg = (u64) decpt;
			decimals_in_dmg = dmg_len - (u64) decpt;
		}

		if (decimals == 0)
		{
			decimals_wanted = decimal_zeroes_leading + decimals_in_dmg;
			decimals_in_dmg_wanted = decimals_in_dmg;
		}
		else if (decimals <= decimal_zeroes_leading)
		{
			/* the wanted decimals are all leading zeroes: write only those */
			decimals_in_dmg_wanted = 0;
			decimal_zeroes_leading = decimals;
		}
		else if (decimals <= decimal_zeroes_leading + decimals_in_dmg)
		{
			decimals_in_dmg_wanted = decimals - decimal_zeroes_leading;
		}
		else
		{
			decimals_in_dmg_wanted = decimals_in_dmg;
			decimal_zeroes_trailing = decimals - decimal_zeroes_leading - decimals_in_dmg;
		}

		const u64 req_size = sign
			+ integer_zero_leading 
			+ integers_in_dmg
			+ integer_zeroes_trailing
			+ ((decimals_wanted) ? 1 + decimals_wanted : 0);

		if (req_size <= bufsize)
		{
			u32 i = 0;
			u32 dmg_i = 0;

			if (sign)
			{
				buf[i++] = '-';
			}

			if (integer_zero_leading)
			{
				buf[i++] = '0';			
			}

			for (u32 k = 0; k < integers_in_dmg; ++k)
			{
				buf[i++] = dmg_str[dmg_i++];			
			}

			for (u32 k = 0; k < integer_zeroes_trailing; ++k)
			{
				buf[i++] = '0';			
			}

			if (decimals_wanted)
			{
				buf[i++] = '.';			

				for (u32 k = 0; k < decimal_zeroes_leading; ++k)
				{
					buf[i++] = '0';			
				}

				for (u32 k = 0; k < decimals_in_dmg_wanted; ++k)
				{
					buf[i++] = dmg_str[dmg_i++];			
				}

				for (u32 k = 0; k < decimal_zeroes_trailing; ++k)
				{
					buf[i++] = '0';			
				}
			}

			ds_Assert(i == req_size);
			str.buf = buf;
			str.len = i;
			str.size = bufsize;
		}
	}

	DmgDtoaFree(dmg_str);

	return str;
}

utf8 Utf8U64Buffered(u8 buf[], const u64 bufsize, const u64 val)
{
	utf8 ret = Utf8Empty();

	u8 tmp[64];

	if (bufsize >= 1)
	{
		u64 v = val;
		u32 len = 0;
		do
		{
			tmp[64-1-len++] = '0' + (v % 10);
			v /= 10;
		} while (v && (len < bufsize));

		if (v == 0)
		{
			ret = (utf8) { .buf = buf, .len = len, .size = bufsize, };
			for (u32 i = 0; i < len; ++i)
			{
				buf[i] = tmp[64-len+i];
			}
		}
	}

	return ret;
}

utf8 Utf8I64Buffered(u8 buf[], const u64 bufsize, const i64 val)
{
	utf8 ret = Utf8Empty();

	b64 b = { .i = val };
	const u64 sign = b.u >> 63;
	b.u = (1-sign)*b.u + sign*((~b.u) + 1);
	/* the sign and at least one digit */
	if (sign + 1 <= bufsize)
	{
		const utf8 str = Utf8U64Buffered(buf+sign, bufsize-sign, b.u);
		if (str.len)
		{
			if (sign)
			{
				buf[0] = '-';
			}
			ret.buf = buf;
			ret.len = str.len+sign;
			ret.size = bufsize;
		}
	}

	return ret;
}

struct parseRetval I64Utf8(const utf8 str)
{
	/* "" and "-" have no digits */
	struct parseRetval ret = { .op_result = PARSE_STRING_INVALID, .i64 = 0 };
	utf8 tmp = str;
	if (tmp.len)
	{
		u64 sign = 0;
		if (tmp.buf[0] == '-')
		{
			tmp.len -= 1;
			tmp.buf += 1;
			sign = 1;
		}

		ret = U64Utf8(tmp);
		if (!ret.op_result)
		{
			if (sign)
			{
				/* signed integer underflow */
				if (ret.u64 > (u64) I64_MAX + 1)
				{
					ret = (struct parseRetval) { .op_result = PARSE_UNDERFLOW, .i64 = 0 };
				}
				else
				{
					ret.u64 = (~ret.u64) + 1;
				}
			}
			/* signed integer overflow */
			else if (ret.u64 > I64_MAX)
			{
				ret = (struct parseRetval) { .op_result = PARSE_OVERFLOW, .i64 = 0 };
			}
		} 
		else if (sign && ret.op_result == PARSE_OVERFLOW)
		{
			ret = (struct parseRetval) { .op_result = PARSE_UNDERFLOW, .i64 = 0 };
		}

	}

	return ret;
}

struct parseRetval U64Utf8(const utf8 str)
{
	if (str.len == 0)
	{
		return (struct parseRetval) { .op_result = PARSE_STRING_INVALID, .u64 = 0 };
	}

	u64 overflow = 0;
	u64 ret = 0;
	for (u32 i = 0; i < str.len; ++i)
	{
		const u64 c = str.buf[i];
		if (c < '0' || '9' < c)
		{
			return (struct parseRetval) { .op_result = PARSE_STRING_INVALID, .u64 = 0 };
		}

		overflow |= U64MulReturnOverflow(&ret, ret, 10);
		overflow |= U64AddReturnOverflow(&ret, ret, c-'0');
	}

	return (overflow) 
		? (struct parseRetval) { .op_result = PARSE_OVERFLOW, .u64 = 0   }
		: (struct parseRetval) { .op_result = PARSE_SUCCESS,  .u64 = ret };
}

struct parseRetval I64Utf32(const utf32 str)
{
	/* "" and "-" have no digits */
	struct parseRetval ret = { .op_result = PARSE_STRING_INVALID, .i64 = 0 };
	utf32 tmp = str;
	if (tmp.len)
	{
		u64 sign = 0;
		if (tmp.buf[0] == '-')
		{
			tmp.len -= 1;
			tmp.buf += 1;
			sign = 1;
		}

		ret = U64Utf32(tmp);
		if (!ret.op_result)
		{
			if (sign)
			{
				/* signed integer underflow */
				if (ret.u64 > (u64) I64_MAX + 1)
				{
					ret = (struct parseRetval) { .op_result = PARSE_UNDERFLOW, .i64 = 0 };
				}
				else
				{
					ret.u64 = (~ret.u64) + 1;
				}
			}
			/* signed integer overflow */
			else if (ret.u64 > I64_MAX)
			{
				ret = (struct parseRetval) { .op_result = PARSE_OVERFLOW, .i64 = 0 };
			}
		} 
		else if (sign && ret.op_result == PARSE_OVERFLOW)
		{
			ret = (struct parseRetval) { .op_result = PARSE_UNDERFLOW, .i64 = 0 };
		}

	}

	return ret;
}

struct parseRetval U64Utf32(const utf32 str)
{
	if (str.len == 0)
	{
		return (struct parseRetval) { .op_result = PARSE_STRING_INVALID, .u64 = 0 };
	}

	u64 overflow = 0;
	u64 ret = 0;
	for (u32 i = 0; i < str.len; ++i)
	{
		const u64 c = str.buf[i];
		if (c < '0' || '9' < c)
		{
			return (struct parseRetval) { .op_result = PARSE_STRING_INVALID, .u64 = 0 };
		}

		overflow |= U64MulReturnOverflow(&ret, ret, 10);
		overflow |= U64AddReturnOverflow(&ret, ret, c-'0');
	}

	return (overflow) 
		? (struct parseRetval) { .op_result = PARSE_OVERFLOW, .u64 = 0   }
		: (struct parseRetval) { .op_result = PARSE_SUCCESS,  .u64 = ret };
}
utf8 Utf8F32(struct arena *mem, const u32 decimals, const f32 val)
{                                                               
	return Utf8F64(mem, decimals, (f64) val);
}                                                               
                                                                
utf8 Utf8F64(struct arena *mem, const u32 decimals, const f64 val)
{
	const u64 bufsize = mem->mem_left;
	u8 *buf = ArenaPushPacked(mem, bufsize);

	utf8 str = Utf8F64Buffered(buf, bufsize, decimals, val);
	if (str.len)
	{
		str.size = str.len;
		ArenaPopPacked(mem, bufsize - str.size);
	}
	else
	{
		ArenaPopPacked(mem, bufsize);
	}

	return str;
}

utf8 Utf8U64(struct arena *mem, const u64 val)
{
	const u64 bufsize = mem->mem_left;
	u8 *buf = ArenaPushPacked(mem, bufsize);

	utf8 str = Utf8U64Buffered(buf, bufsize, val);
	if (str.len)
	{
		str.size = str.len;
		ArenaPopPacked(mem, bufsize - str.size);
	}
	else
	{
		ArenaPopPacked(mem, bufsize);
	}

	return str;
}

utf8 Utf8I64(struct arena *mem, const i64 val)
{
	const u64 bufsize = mem->mem_left;
	u8 *buf = ArenaPushPacked(mem, bufsize);

	utf8 str = Utf8I64Buffered(buf, bufsize, val);
	if (str.len)
	{
		str.size = str.len;
		ArenaPopPacked(mem, bufsize - str.size);
	}
	else
	{
		ArenaPopPacked(mem, bufsize);
	}

	return str;
}

utf32 Utf32F32Buffered(u32 buf[], const u64 buflen, const u32 decimals, const f32 val)
{                                                                                    
	return Utf32F64Buffered(buf, buflen, decimals, (f64) val);
}                                                                                    
                                                                                     
utf32 Utf32F64Buffered(u32 buf[], const u64 buflen, const u32 decimals, const f64 val)
{
	utf32 str = Utf32Empty();

	i32 sign;
	i32 decpt;
	/* decimals: correctly rounded to that many (mode 3); 0: shortest round-trip digits (mode 0) */
	char *dmg_str = (decimals)
		? DmgDtoa((f64) val, 3, (int) decimals, &decpt, &sign, NULL)
		: DmgDtoa((f64) val, 0, 0, &decpt, &sign, NULL);

	/* INF / Nan */
	if (decpt == 9999)
	{
		str = Utf32CstrBuffered(buf, buflen, dmg_str);
	}
	else
	{
		const u64 dmg_len = strlen(dmg_str);
		u64 integers_in_dmg = 0; 
		u64 integer_zero_leading = 0; 
		u64 integer_zeroes_trailing = 0;
		u64 decimals_in_dmg = 0;
		u64 decimal_zeroes_leading = 0;
		u64 decimal_zeroes_trailing = 0;	
		u64 decimals_in_dmg_wanted = 0;
		u64 decimals_wanted = decimals;

		/* 0.**** */
		if (decpt < 1)
		{
			decimals_in_dmg = dmg_len;
			integer_zero_leading = 1;
			decimal_zeroes_leading = (u64) (-decpt);
		}
		/* ****.0 */
		else if ((u64) decpt >= dmg_len)
		{
			integers_in_dmg = dmg_len;
			integer_zeroes_trailing = (u64) decpt - dmg_len;
		}
		/* ****.**** */
		else
		{
			integers_in_dmg = (u64) decpt;
			decimals_in_dmg = dmg_len - (u64) decpt;
		}

		if (decimals == 0)
		{
			decimals_wanted = decimal_zeroes_leading + decimals_in_dmg;
			decimals_in_dmg_wanted = decimals_in_dmg;
		}
		else if (decimals <= decimal_zeroes_leading)
		{
			/* the wanted decimals are all leading zeroes: write only those */
			decimals_in_dmg_wanted = 0;
			decimal_zeroes_leading = decimals;
		}
		else if (decimals <= decimal_zeroes_leading + decimals_in_dmg)
		{
			decimals_in_dmg_wanted = decimals - decimal_zeroes_leading;
		}
		else
		{
			decimals_in_dmg_wanted = decimals_in_dmg;
			decimal_zeroes_trailing = decimals - decimal_zeroes_leading - decimals_in_dmg;
		}

		const u64 req_len = sign
			+ integer_zero_leading 
			+ integers_in_dmg
			+ integer_zeroes_trailing
			+ ((decimals_wanted) ? 1 + decimals_wanted : 0);

		if (req_len <= buflen)
		{
			u32 i = 0;
			u32 dmg_i = 0;

			if (sign)
			{
				buf[i++] = '-';
			}

			if (integer_zero_leading)
			{
				buf[i++] = '0';			
			}

			for (u32 k = 0; k < integers_in_dmg; ++k)
			{
				buf[i++] = dmg_str[dmg_i++];			
			}

			for (u32 k = 0; k < integer_zeroes_trailing; ++k)
			{
				buf[i++] = '0';			
			}

			if (decimals_wanted)
			{
				buf[i++] = '.';			

				for (u32 k = 0; k < decimal_zeroes_leading; ++k)
				{
					buf[i++] = '0';			
				}

				for (u32 k = 0; k < decimals_in_dmg_wanted; ++k)
				{
					buf[i++] = dmg_str[dmg_i++];			
				}

				for (u32 k = 0; k < decimal_zeroes_trailing; ++k)
				{
					buf[i++] = '0';			
				}
			}

			ds_Assert(i == req_len);
			str.buf = buf;
			str.len = i;
			str.max_len = i;
		}
	}

	DmgDtoaFree(dmg_str);

	return str;
}

utf32 Utf32U64Buffered(u32 buf[], const u64 bufsize, const u64 val)
{
	utf32 ret = Utf32Empty();

	u8 tmp[64];
	if (bufsize >= 1)
	{
		u64 v = val;
		u32 len = 0;
		do
		{
			tmp[64-1-len++] = '0' + (v % 10);
			v /= 10;
		} while (v && (len < bufsize));

		if (v == 0)
		{
			ret = (utf32) { .buf = buf, .len = len, .max_len = bufsize, };
			for (u32 i = 0; i < len; ++i)
			{
				buf[i] = tmp[64-len+i];
			}
		}
	}

	return ret;
}

utf32 Utf32I64Buffered(u32 buf[], const u64 bufsize, const i64 val)
{
	utf32 ret = Utf32Empty();

	b64 b = { .i = val };
	const u64 sign = b.u >> 63;
	b.u = (1-sign)*b.u + sign*((~b.u) + 1);
	/* the sign and at least one digit */
	if (sign + 1 <= bufsize)
	{
		const utf32 str = Utf32U64Buffered(buf+sign, bufsize-sign, b.u);
		if (str.len)
		{
			if (sign)
			{
				buf[0] = '-';
			}
			ret.buf = buf;
			ret.len = str.len+sign;
			ret.max_len = bufsize;		
		}
	}

	return ret;

}

utf32 Utf32F32(struct arena *mem, const u32 decimals, const f32 val)
{                                                                 
	return Utf32F64(mem, decimals, (f64) val);
}                                                                 
                                                                  
utf32 Utf32F64(struct arena *mem, const u32 decimals, const f64 val)
{
	struct memArray alloc = ArenaPushAlignedAll(mem, sizeof(u32), sizeof(u32));

	utf32 str = Utf32F64Buffered(alloc.addr, alloc.len, decimals, val);
	if (str.len)
	{
		str.max_len = str.len;
		ArenaPopPacked(mem, (alloc.len-str.len)*sizeof(u32));
	}
	else
	{
		ArenaPopPacked(mem, alloc.memPushed);
	}
	
	return str;
}

utf32 Utf32U64(struct arena *mem, const u64 val)
{
	struct memArray alloc = ArenaPushAlignedAll(mem, sizeof(u32), sizeof(u32));

	utf32 str = Utf32U64Buffered(alloc.addr, alloc.len, val);
	if (str.len)
	{
		str.max_len = str.len;
		ArenaPopPacked(mem, (alloc.len-str.len)*sizeof(u32));
	}
	else
	{
		ArenaPopPacked(mem, alloc.memPushed);
	}
	
	return str;

}

utf32 Utf32I64(struct arena *mem, const i64 val)
{
	struct memArray alloc = ArenaPushAlignedAll(mem, sizeof(u32), sizeof(u32));

	utf32 str = Utf32I64Buffered(alloc.addr, alloc.len, val);
	if (str.len)
	{
		str.max_len = str.len;
		ArenaPopPacked(mem, (alloc.len-str.len)*sizeof(u32));
	}
	else
	{
		ArenaPopPacked(mem, alloc.memPushed);
	}
	
	return str;

}

utf8 Utf8Empty(void)
{
	return empty_utf8;
}


utf32 Utf32Empty(void)
{
	return empty_utf32;
}

utf8 Utf8Alloc(struct arena *mem, const u64 bufsize)
{
	void *buf = ArenaPush(mem, bufsize);
	return (buf)
		? (utf8) { .len = 0, .size = bufsize, .buf = buf }
		: Utf8Empty();
}

utf8 Utf8Buffered(u8 buf[], const u64 bufsize)
{
	return (utf8) { .len = 0, .size = bufsize, .buf = buf };
}

utf32 Utf32Alloc(struct arena *mem, const u32 len)
{
	u32 *buf = ArenaPush(mem, len*sizeof(u32));
	return (buf)
		? (utf32) { .len = 0, .max_len = len, .buf = buf }
		: Utf32Empty();
}

utf32 Utf32Buffered(u32 buf[], const u32 len)
{
	return (utf32) { .len = 0, .max_len = len, .buf = buf };
}

void Utf8DebugPrint(const utf8 str)
{
	u64 offset = 0;
	for (u64 i = 0; i < str.len; ++i)
	{
		fprintf(stderr, "%c", Utf8ReadCodepoint(&offset, &str, offset));
	}
	fprintf(stderr, "\n");
}

void Utf32DebugPrint(const utf32 str)
{
	for (u64 i = 0; i < str.len; ++i)
	{
		fprintf(stderr, "%c", str.buf[i]);
	}
	fprintf(stderr, "\n");
}

enum string_token
{
	STRING_TOKEN_INVALID,
	STRING_TOKEN_NULL,
	STRING_TOKEN_CHAR,
	STRING_TOKEN_F32,
	STRING_TOKEN_U32,
	STRING_TOKEN_U64,
	STRING_TOKEN_I32,
	STRING_TOKEN_I64,
	STRING_TOKEN_POINTER,
	STRING_TOKEN_C_STRING,
	STRING_TOKEN_KAS_STRING,
};

/* set extra depending on % parameter, token_length is always valid */
static enum string_token internal_determine_format_parameter(const char *format, u32 *token_length, u32 *extra)
{
	enum string_token type = STRING_TOKEN_INVALID;
	*token_length = 0;
	switch (format[(*token_length)++])
	{
		case '\0':
		{
			type = STRING_TOKEN_NULL;
		} break;

		case '%':
		{
			switch (format[(*token_length)++])
			{
				case 'l':
				{
					switch(format[(*token_length)++])
					{
						case 'u':
						{
							type = STRING_TOKEN_U64;
						} break;

						case 'i':
						{
							type = STRING_TOKEN_I64;
						} break;

						default:
						{
							type = STRING_TOKEN_INVALID;
						} break;
					}
				} break;

				case 'u':
				{
					type = STRING_TOKEN_U32;
				} break;

				case 'i':
				{
					type = STRING_TOKEN_I32;
				} break;

				/* decimal count into f32 (OR f64 in future) */
				case '0': 
				case '1':
				case '2':
				case '3':
				case '4':
				case '5':
				case '6':
				case '7':
				case '8':
				case '9':
				{
					*extra = format[*token_length-1] - '0';
					for (; '0' <= format[*token_length] && format[*token_length] <= '9'; *token_length += 1)
					{
						*extra = (*extra * 10) + (format[*token_length] - '0');
					}

					 type = (format[(*token_length)++] == 'f')
						 ? STRING_TOKEN_F32
						 : STRING_TOKEN_INVALID;
				} break; 

				case 'f':
				{
					*extra = 7;
					type = STRING_TOKEN_F32;
				} break;

				case 'p':
				{
					type = STRING_TOKEN_POINTER;
				} break;

				case 's':
				{
					type = STRING_TOKEN_C_STRING;
				} break;

				case 'k':
				{
					type =  STRING_TOKEN_KAS_STRING;
				} break;

				/* "%%": a literal '%' (STRING_TOKEN_CHAR writes extra bytes from format[0]) */
				case '%':
				{
					*extra = 1;
					type = STRING_TOKEN_CHAR;
				} break;
			}
		} break;

		/*
		 * one codepoint of the format: its whole UTF-8 sequence (extra bytes), or a single byte if it is
		 * bad, so the output decodes to as many codepoints as were counted
		 */
		default:
		{
			u32 available = 1;
			while (available < 4 && format[available] != '\0')
			{
				available += 1;
			}
			const utf8 sequence = { .buf = (u8 *) format, .size = available, .len = 0 };
			u64 sequence_end;
			Utf8ReadCodepoint(&sequence_end, &sequence, 0);
			*token_length = (u32) sequence_end;
			*extra = (u32) sequence_end;
			type = STRING_TOKEN_CHAR;
		} break;
	}

	return type;
}

utf8 Utf8FormatBufferedVariadic(u64 *reqsize, u8 *buf, const u64 bufsize, const char *format, va_list args) 
{
	*reqsize = 0;
	if (bufsize == 0)
	{
		return Utf8Empty();
	}

	utf8 pstr;

	enum string_token token;
	u32 token_length;
	u32 len = 0;
	u32 offset = 0;
	u32 extra = 0;
	u32 cont = 1;
	while (cont)
	{
		u64 size = 0;
		token = internal_determine_format_parameter(format, &token_length, &extra);
		switch (token)
		{
			case STRING_TOKEN_NULL:
			{
				cont = 0;
			} break;

			case STRING_TOKEN_F32:
			{
				const f64 val = va_arg(args, f64);
				pstr = Utf8F64Buffered(buf + offset, bufsize - offset, extra, val);	
				cont = pstr.len;
			} break;

			case STRING_TOKEN_U32:
			{
				const u32 val = va_arg(args, u32);
				pstr = Utf8U64Buffered(buf + offset, bufsize - offset, val);	
				cont = pstr.len;
			} break;

			case STRING_TOKEN_U64:
			{
				const u64 val = va_arg(args, u64);
				pstr = Utf8U64Buffered(buf + offset, bufsize - offset, val);
				cont = pstr.len;
			} break;

			case STRING_TOKEN_I32:
			{
				const i32 val = va_arg(args, i32);
				pstr = Utf8I64Buffered(buf + offset, bufsize - offset, val);	
				cont = pstr.len;
			} break;

			case STRING_TOKEN_I64:
			{
				const i64 val = va_arg(args, i64);
				pstr = Utf8I64Buffered(buf + offset, bufsize - offset, val);	
				cont = pstr.len;
			} break;

			case STRING_TOKEN_POINTER:
			{
				/* read a pointer, not a u64: pointers are 4 bytes on wasm32 */
				const u64 val = (u64) (uintptr_t) va_arg(args, void *);
				pstr = Utf8U64Buffered(buf + offset, bufsize - offset, val);
				cont = pstr.len;
			} break;

			case STRING_TOKEN_C_STRING:
			{
				const char *cstr = va_arg(args, char *);
				/* len 0 means "did not fit" below: an empty string writes nothing and goes on */
				if (cstr[0])
				{
					pstr = Utf8CstrBuffered(buf + offset, bufsize - offset, cstr);
					cont = pstr.len;
					/* advance by bytes, not codepoints */
					size = strlen(cstr);
				}
				else
				{
					pstr = Utf8Empty();
				}
			} break;

			case STRING_TOKEN_KAS_STRING:
			{
				const utf8 *kfstr = va_arg(args, utf8 *);
				if (kfstr->len)
				{
					pstr = Utf8CopyBufferedAndReturnRequiredSize(&size, buf + offset, bufsize - offset, *kfstr);
					cont = pstr.len;
				}
				else
				{
					pstr = Utf8Empty();
				}
			} break;

			case STRING_TOKEN_INVALID:
			{
				cont = 0;
			} break;

			case STRING_TOKEN_CHAR:
			{
				/* one codepoint (or one bad byte) of extra bytes */
				cont = (extra <= bufsize - offset);
				if (cont)
				{
					memcpy(buf + offset, format, extra);
					pstr = (utf8) { .buf = buf + offset, .len = 1, .size = extra };
					size = extra;
				}
			} break;
		}

		if (!cont)
		{
			break;
		}

		if (size == 0)
		{
			size = pstr.len;
		}

		len += pstr.len;
		offset += size;
		format += token_length;
	}

	/* the terminator ends any cut-off sequence left by a bad byte in the format (size stays the capacity) */
	if (offset < bufsize)
	{
		buf[offset] = '\0';
	}

	*reqsize = offset;
	utf8 kstr =
	{
		.len = len,
		.size = bufsize,
		.buf = buf,
	};

	return kstr;
}

utf8 Utf8FormatVariadic(struct arena *mem, const char *format, va_list args)
{
	const u64 mem_left = mem->mem_left;
	void *buf = ArenaPushPacked(mem, mem->mem_left);
	u64 reqsize;
	utf8 kstr = Utf8FormatBufferedVariadic(&reqsize, buf, mem_left, format, args);

	if (kstr.len == 0)
       	{
		ArenaPopPacked(mem, mem_left);
		return Utf8Empty();
	}
	
	kstr.size = reqsize;
	ArenaPopPacked(mem, mem_left - reqsize);
	return kstr;
}

utf8 Utf8Format(struct arena *mem, const char *format, ...)
{
	const u64 mem_left = mem->mem_left;
	va_list args;
	va_start(args, format);
	void *buf = ArenaPushPacked(mem, mem->mem_left);
	u64 reqsize;
	utf8 kstr = Utf8FormatBufferedVariadic(&reqsize, buf, mem_left, format, args);
	va_end(args);

	if (kstr.len == 0)
       	{
		ArenaPopPacked(mem, mem_left);
		return Utf8Empty();
	}
	
	kstr.size = reqsize;
	ArenaPopPacked(mem, mem_left - reqsize);
	return kstr;
}

utf8 Utf8FormatBuffered(u8 *buf, const u64 bufsize, const char *format, ...)
{
	u64 reqsize; 
	va_list args;
	va_start(args, format);
	utf8 kstr = Utf8FormatBufferedVariadic(&reqsize, buf, bufsize, format, args);
	va_end(args);

	return kstr;
}

utf8 Utf8CstrBuffered(u8 buf[], const u64 bufsize, const char *cstr)
{
	const u64 bytes = strlen(cstr);	
	utf8 ret = Utf8Empty();
	if (bytes <= bufsize)
	{
		ret = (utf8) { .len = 0, .size = (u32) bufsize, .buf = buf };
		memcpy(buf, cstr, bytes);
		/* decode only the copied bytes, and terminate them if there is room: a sequence cut off at the end stays bad */
		const utf8 copied = { .buf = buf, .size = (u32) bytes, .len = 0 };
		for (u64 offset = 0; offset < bytes; ret.len += 1)
		{
			Utf8ReadCodepoint(&offset, &copied, offset);
		}
		if (bytes < bufsize)
		{
			buf[bytes] = '\0';
		}
	}

	return ret;
}

utf8 Utf8Cstr(struct arena *mem, const char *cstr)
{
	const u64 bytes = strlen(cstr);	
	utf8 ret = Utf8Alloc(mem, bytes);
	if (ret.size)
	{
		/* ret.size is bytes */
		memcpy(ret.buf, cstr, bytes);
		for (u64 offset = 0; offset < bytes; ret.len += 1)
		{
			Utf8ReadCodepoint(&offset, &ret, offset);
		}
	}

	return ret;
}

utf32 Utf32CstrBuffered(u32 buf[], const u64 buflen, const char *cstr)
{
	const u64 bytes = strlen(cstr);	
	const utf8 str = { .len = 0, .size = bytes, .buf = (u8 *) cstr };
	utf32 ret = (utf32) { .len = 0, .max_len = buflen, .buf = buf };

	for (u64 offset = 0; offset < bytes; ret.len += 1)
	{
		if (ret.len == ret.max_len)
		{
			ret = Utf32Empty();
			break;
		}
		ret.buf[ret.len] = Utf8ReadCodepoint(&offset, &str, offset);
	}

	return ret;
}

utf32 Utf32Cstr(struct arena *mem, const char *cstr)
{
	struct memArray alloc = ArenaPushAlignedAll(mem, sizeof(u32), sizeof(u32));	
	utf32 ret = Utf32CstrBuffered(alloc.addr, alloc.len, cstr);
	
	(ret.len)
		? ArenaPopPacked(mem, sizeof(u32) * (ret.max_len - ret.len))
		: ArenaPopPacked(mem, alloc.memPushed);

	ret.max_len = ret.len;
	return ret;
}

utf8 Utf8Copy(struct arena *mem, const utf8 str)
{
	const u64 bufsize_req = Utf8SizeRequired(str);
	utf8 copy = Utf8Alloc(mem, bufsize_req);
	if (copy.size)
	{
		memcpy(copy.buf, str.buf, bufsize_req);
		copy.len = str.len;
	}

	return copy;
}

utf8 Utf8CopyBuffered(u8 buf[], const u64 bufsize, const utf8 str)
{	
	u64 tmp;
	return Utf8CopyBufferedAndReturnRequiredSize(&tmp, buf, bufsize, str);
}


utf8 Utf8CopyBufferedAndReturnRequiredSize(u64 *reqsize, u8 buf[], const u64 bufsize, const utf8 str)
{
	*reqsize = 0;
	const u64 bufsize_req = Utf8SizeRequired(str);

	utf8 copy = Utf8Empty();
	if (bufsize_req <= bufsize)
	{
		*reqsize = bufsize_req;
		memcpy(buf, str.buf, bufsize_req);
		/* terminated if there is room: a sequence cut off at the end stays bad (size stays the capacity) */
		if (bufsize_req < bufsize)
		{
			buf[bufsize_req] = '\0';
		}
		copy = (utf8) { .len = str.len, .size = bufsize, .buf = buf };
	}

	return copy;
}

utf32 Utf32Copy(struct arena *mem, const utf32 str)
{
	utf32 copy = Utf32Alloc(mem, str.len);
	if (copy.max_len)
	{
		memcpy(copy.buf, str.buf, str.len * sizeof(u32));
		copy.len = str.len;
	}
	
	return copy;
}

utf32 Utf32CopyBuffered(u32 buf[], const u64 buflen, const utf32 str)
{
	utf32 copy = Utf32Empty();
	if (str.len <= buflen)
	{
		memcpy(buf, str.buf, str.len * sizeof(u32));
		copy = (utf32) { .len = str.len, .max_len = buflen, .buf = buf };
	}
	
	return copy;
}

utf32 Utf32Utf8(struct arena *mem, const utf8 str)
{
	utf32 conv = Utf32Empty();
	u32 *buf = ArenaPush(mem, str.len*sizeof(u32));
	if (buf)
	{
		u64 offset = 0;
		for (u32 i = 0; i < str.len; ++i)
		{
			buf[i] = Utf8ReadCodepoint(&offset, &str, offset);	
		}
		conv = (utf32) { .len = str.len, .max_len = str.len, .buf = buf, };
	}

	return conv;
}

utf32 Utf32Utf8Buffered(u32 buf[], const u64 buflen, const utf8 str)
{
	utf32 conv = Utf32Empty();
	if (str.len <= buflen)
	{
		u64 offset = 0;
		for (u32 i = 0; i < str.len; ++i)
		{
			buf[i] = Utf8ReadCodepoint(&offset, &str, offset);	
		}

		conv = (utf32) { .len = str.len, .max_len = buflen, .buf = buf, };
	}

	return conv;
}

u32 Utf8WriteCodepoint(u8 *buf, const u32 bufsize, const u32 codepoint)
{
	u32 len = 0;

	/* surrogates have no UTF-8 form: Utf8ReadCodepoint would read them back as bad bytes */
	if (0xd800 <= codepoint && codepoint <= 0xdfff)
	{
		return 0;
	}

	if (codepoint <= 0x7f && bufsize >= 1)
	{
		len = 1;
		buf[0] = (u8) (codepoint & 0x7f);
	}	
	else if (codepoint <= 0x7ff && bufsize >= 2)
	{
		len = 2;
		buf[0] = 0xc0 | (u8) ((codepoint >> 6) & 0x1f);
		buf[1] = 0x80 | (u8) (codepoint & 0x3f);
	}
	else if (codepoint <= 0xffff && bufsize >= 3)
	{
		len = 3;
		buf[0] = 0xe0 | (u8) ((codepoint >> 12) & 0x0f);
		buf[1] = 0x80 | (u8) ((codepoint >> 6) & 0x3f);
		buf[2] = 0x80 | (u8) (codepoint & 0x3f);
	}
	else if (codepoint <= 0x10ffff && bufsize >= 4)
	{
		len = 4;
		buf[0] = 0xf0 | (u8) ((codepoint >> 18) & 0x07);
		buf[1] = 0x80 | (u8) ((codepoint >> 12) & 0x3f);
		buf[2] = 0x80 | (u8) ((codepoint >> 6) & 0x3f);
		buf[3] = 0x80 | (u8) (codepoint & 0x3f);
	}

	return len;
}

utf8 Utf8Utf32BufferedAndReturnRequiredSize(u64 *reqsize, u8 buf[], const u64 bufsize, const utf32 str)
{
	*reqsize = 0;
	utf8 ret = { .buf = buf, .size = bufsize, .len = str.len };

	for (u32 i = 0; i < str.len; ++i)
	{
		const u32 len = Utf8WriteCodepoint(buf + *reqsize, bufsize - *reqsize, str.buf[i]);
		if (len == 0)
		{
			ret = Utf8Empty();
			*reqsize = 0;
			break;
		}

		*reqsize += len;
	}

	return ret;
}

utf8 Utf8Utf32Buffered(u8 buf[], const u64 bufsize, const utf32 str)
{
	u64 reqsize;
	return Utf8Utf32BufferedAndReturnRequiredSize(&reqsize, buf, bufsize, str);

}

utf8 Utf8Utf32BufferedNullTerminatedAndReturnRequiredSize(u64 *reqsize, u8 buf[], const u64 bufsize, const utf32 str)
{
	utf8 ret = Utf8Utf32BufferedAndReturnRequiredSize(reqsize, buf, bufsize, str);
	if (ret.len && *reqsize < bufsize)
	{
		ret.buf[*reqsize] = '\0';
		*reqsize += 1;
	}
	else
	{
		ret = Utf8Empty();
		*reqsize = 0;
	}
	return ret;
}

utf8 Utf8Utf32BufferedNullTerminated(u8 buf[], const u64 bufsize, const utf32 str)
{
	u64 reqsize;
	return Utf8Utf32BufferedNullTerminatedAndReturnRequiredSize(&reqsize, buf, bufsize, str);
}

utf8 Utf8Utf32(struct arena *mem, const utf32 str32)
{
	const u64 bufsize = mem->mem_left;
	u8 *buf = ArenaPushPacked(mem, bufsize);

	u64 reqsize;
	utf8 str = Utf8Utf32BufferedAndReturnRequiredSize(&reqsize, buf, bufsize, str32);
	if (str.len)
	{
		str.size = reqsize;
		ArenaPopPacked(mem, bufsize - reqsize);
	}
	else
	{
		str = Utf8Empty();
		ArenaPopPacked(mem, bufsize);
	}

	return str;
}

utf8 Utf8Utf32NullTerminated(struct arena *mem, const utf32 str32)
{
	const u64 bufsize = mem->mem_left;
	u8 *buf = ArenaPushPacked(mem, bufsize);

	u64 reqsize;
	utf8 str = Utf8Utf32BufferedNullTerminatedAndReturnRequiredSize(&reqsize, buf, bufsize, str32);
	if (str.len)
	{
		str.size = reqsize;
		ArenaPopPacked(mem, bufsize - reqsize);
	}
	else
	{
		str = Utf8Empty();
		ArenaPopPacked(mem, bufsize);
	}

	return str;
}

u32 CstrHash(const char *cstr)
{
	const u64 len = strlen(cstr);
	return (u32) XXH3_64bits(cstr, len);
}

u32 Utf8Hash(const utf8 str)
{
	const u64 size = Utf8SizeRequired(str);
	return (u32) XXH3_64bits(str.buf, size);
}

u32 Utf8Equivalence(const utf8 str1, const utf8 str2)
{
	u32 equivalence = 1;
	u64 offset1 = 0; 
	u64 offset2 = 0;
	if (str1.len == str2.len)
	{
		for (u32 i = 0; i < str1.len; ++i)
		{
			const u32 codepoint1 = Utf8ReadCodepoint(&offset1, &str1, offset1);
			const u32 codepoint2 = Utf8ReadCodepoint(&offset2, &str2, offset2);
			if (codepoint1 != codepoint2)
			{
				equivalence = 0;
				break;
			}
		}
	}
	else
	{
		equivalence = 0;
	}

	return equivalence;
}

struct kmpSubstring Utf8LookupSubstringInit(struct arena *mem, const utf8 str)
{
	struct kmpSubstring kmp;
	kmp.substring = Utf32Utf8(mem, str);
	kmp.backtrack = ArenaPush(mem, kmp.substring.len*sizeof(u32));

	if (kmp.substring.len)
	{
		u32 b = U32_MAX;
		kmp.backtrack[0] = U32_MAX;
		for (u32 i = 1; i < kmp.substring.len; ++i)
		{
			while (b != U32_MAX)
			{
				if (kmp.substring.buf[i] == kmp.substring.buf[b+1])
				{
					break;
				}
				b = kmp.backtrack[b];
			}	
			
			if (kmp.substring.buf[i] == kmp.substring.buf[b+1])
			{
				b += 1;
			}

			kmp.backtrack[i] = b;
		}
	}

 	return kmp;	
}

u32 Utf8LookupSubstring(struct kmpSubstring *kmp_substring, const utf8 str) 
{
	if (kmp_substring->substring.len == 0)
	{
		kmp_substring->start = U32_MAX;
 		return 1;	
	}
	else if (str.len < kmp_substring->substring.len)
	{
		return 0;
	}

	u32 found = 0;
	u64 offset = 0;
	u32 si = U32_MAX;
	for (u32 i = 0; i < str.len; ++i)
	{
		const u32 codepoint = Utf8ReadCodepoint(&offset, &str, offset);
		while (si != U32_MAX && codepoint != kmp_substring->substring.buf[si+1])
		{
			si = kmp_substring->backtrack[si];
		}

		if (codepoint == kmp_substring->substring.buf[si+1])
		{
			si += 1;
		}

		if (si+1 == kmp_substring->substring.len)
		{
			kmp_substring->start = i - (kmp_substring->substring.len) + 1;	
			found = 1;
			break;
		}
	}

	return found;
}
