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

#ifndef __DS_STRING_H__
#define __DS_STRING_H__

#include <stdarg.h>
#include "ds_define.h"
#include "ds_allocator.h"

#ifdef __cplusplus
extern "C" { 
#endif

/*
 *	String Library 
 *
 *	utf8 - NOT null-terminated unicode string
 *	utf32 - 32bit unicode string
 */


#define Utf8Inline(cstr) (utf8)  		\
	{					\
		.len =	sizeof(cstr)-1,		\
		.size =	sizeof(cstr),		\
		.buf =  (u8 *) cstr,		\
	}

typedef struct utf8 utf8;
struct utf8 
{
	u8 *	buf;	
	u32 	size;		/* Number of bytes in buffer */
	u32 	len;		/* Length of string (not bytes) (not including **POSSIBLE** '\0') */
};

typedef struct utf32 utf32;
struct utf32
{
	u32 *	buf;		/* buf[max_len] */
	u32 	len;		/* number of codepoints in buf */
	u32 	max_len;	/* buffer length in (u32) */
};

/************************************** Helpers ***************************************/

enum parseRetvalType
{
	PARSE_SUCCESS = 0,
	PARSE_UNDERFLOW,
	PARSE_OVERFLOW,
	PARSE_STRING_INVALID,
	PARSE_NO_OP,
	PARSE_COUNT
};

struct parseRetval
{
	enum parseRetvalType 	op_result;	/* 0 on success, >= 1 on failure */
	union 
	{
		u8	u8;
		u16	u16;
		u32	u32;
		u64	u64;
		i8	i8;
		i16	i16;
		i32	i32;
		i64	i64;
		f32	f32;
	};
};

/* return true if codepoint is whitespace (' ', '\t' or \n') or any of ('=', '-', ':', ';'. '\\', '/') */
u32	WordbreakCheck(const u32 codepoint);

/* 
 * Return utf32 (Possibly of length 0) with any initial whitespace, and adjust stream (Possibly with new 
 * length 0) * to point just after initial whitespace: 
 *
 * 	stream(Whitespace | Non-Whitespace | Rest)  => return(Whitespace) | stream_adjusted(Non-Whitespace | rest)
 */
utf32 	Utf32StreamConsumeWhitespace(utf32 *stream);
/* 
 * Return utf32 (Possibly of length 0) with any initial non-whitespace, and adjust stream (Possibly with new 
 * length 0) * to point just after initial non-whitespace: 
 *
 * 	stream(Non-Whitespace | Whitespace | Rest)  => return(Non-Whitespace) | stream_adjusted(Whitespace | rest)
 */
utf32 	Utf32StreamConsumeNonWhitespace(utf32 *stream);

/************************************** C-Strings ***************************************/

/* Hash null-terminated C string */
u32			CstrHash(const char *cstr);

/* Convert utf8 to null-terminated C string */
char *			CstrUtf8(struct arena *mem, const utf8 utf8);

/* Convert null-terminated C string to f32/f64. new_offset is set to be just past the end of str. */
f32			F32Cstr(char **new_offset, const char *str);
f64			F64Cstr(char **new_offset, const char *str);

/************************************** UTF8 ***************************************/

/* return required size to hold ut8.buf[len] */
u64 			Utf8SizeRequired(const utf8 utf8);
/* Hash utf8 string */
u32			Utf8Hash(const utf8 utf8);
/* return 1 if utf8 contents are equivalent, 0 otherwise */
u32			Utf8Equivalence(const utf8 str1, const utf8 str2);	

#define			UTF8_BAD_CODEPOINT	U32_MAX
/* return utf32 codepoint on success, or UTF8_BAD_CODEPOINT on bad offset */
u32 			Utf8ReadCodepoint(u64 *new_offset, const utf8 *str, const u64 offset);	
/* return length written into buf, or 0 of failure. */
u32			Utf8WriteCodepoint(u8 *buf, const u32 bufsize, const u32 codepoint);

void			Utf8DebugPrint(const utf8 str);

utf8 			Utf8Empty(void);
utf8			Utf8Alloc(struct arena *mem, const u64 bufsize);
utf8			Utf8Buffered(u8 buf[], const u64 bufsize);

utf8			Utf8CopyBuffered(u8 buf[], const u64 bufsize, const utf8 str);
utf8			Utf8CopyBufferedAndReturnRequiredSize(u64 *reqsize, u8 buf[], const u64 bufsize, const utf8 str);
utf8			Utf8CstrBuffered(u8 buf[], const u64 bufsize, const char *cstr);
utf8			Utf8F32Buffered(u8 buf[], const u64 bufsize, const u32 decimals, const f32 val);
utf8			Utf8F64Buffered(u8 buf[], const u64 bufsize, const u32 decimals, const f64 val);
utf8			Utf8U64Buffered(u8 buf[], const u64 bufsize, const u64 val);
utf8			Utf8I64Buffered(u8 buf[], const u64 bufsize, const i64 val);
utf8			Utf8Utf32Buffered(u8 buf[], const u64 bufsize, const utf32 str);
utf8			Utf8Utf32BufferedNullTerminated(u8 buf[], const u64 bufsize, const utf32 str);
utf8			Utf8Utf32BufferedAndReturnRequiredSize(u64 *reqsize, u8 buf[], const u64 bufsize, const utf32 str);
utf8 			Utf8Utf32BufferedNullTerminatedAndReturnRequiredSize(u64 *reqsize, u8 buf[], const u64 bufsize, const utf32 str);

utf8			Utf8Copy(struct arena *mem, const utf8 str);
utf8			Utf8Cstr(struct arena *mem, const char *cstr);
utf8			Utf8F32(struct arena *mem, const u32 decimals, const f32 val);
utf8			Utf8F64(struct arena *mem, const u32 decimals, const f64 val);
utf8			Utf8U64(struct arena *mem, const u64 val);
utf8			Utf8I64(struct arena *mem, const i64 val);
utf8			Utf8Utf32(struct arena *mem, const utf32 str);
utf8			Utf8Utf32NullTerminated(struct arena *mem, const utf32 str);

f32			F32Utf8(struct arena *tmp, const utf8 str);
f64			F64Utf8(struct arena *tmp, const utf8 str);
struct parseRetval 	U64Utf8(const utf8 str);	/* on overflow, return u64 value 0 		*/
struct parseRetval 	I64Utf8(const utf8 str); 	/* on under/overflow, return i64 value 0 	*/

/******************************** UTF8 substring lookup *******************************/

struct kmpSubstring
{
	utf32	substring;
	u32 *	backtrack;	/* backtrack[substring.len] : KMP backtracking indices  */
	u32	start;		/* if utf8, (start byte) else if utf32 (start index) */
};
/* initialize substring helper for substring matching using Knuth-Morris-Pratt algorithm (KMP) */
struct kmpSubstring	Utf8LookupSubstringInit(struct arena *mem, const utf8 str);
/* KMP algorithm: return true on substring found, false otherwise. If true, set kmp_substring to first encounter of substring  */
u32			Utf8LookupSubstring(struct kmpSubstring *kmp_substring, const utf8 str);

/*********************************** UTF8 Formatting ***********************************/

/*
 * Utf8Format - takes format string and applies the following rules to generate a utf8. The final generated
 * 		 string will have its length set in len, which will be 0 on failure.
 *
 * Format follow the utf8 formatting rules:
 * 	%s   	- C string 
 * 	%k 	- utf8 ptr
 * 	%i	- i32
 * 	%u	- u32
 * 	%(d)f  	- f64, optionally replace (d) with number of decimals, defaults to 7
 * 	%li 	- i64
 * 	%lu 	- u64
 * 	%p	- Pointer
 * Note: 
 * 	As variadic arguments will promote themselves (or self-promote), dangers arise when the function expects 64 bit values and the user gives 32 bit values or less.
 * 	The smaller values such as char, unsigned int, floats, will be promoted to some type (perhaps self-promotions) and generate storage for that type. Thus, formatting
 * 	chars or perhaps ints as %lli will result in undefined behaviour as the function will grab 64 bits from a 32 bit storage, generating garbage. Explicit casting removes this error.
 */
utf8 			Utf8Format(struct arena *mem, const char *format, ...); 
utf8 			Utf8FormatVariadic(struct arena *mem, const char *format, va_list args); 
utf8 			Utf8FormatBuffered(u8 *buf, const u64 bufsize, const char *format, ...); 
utf8 			Utf8FormatBufferedVariadic(u64 *reqsize, u8 *buf, const u64 bufsize, const char *format, va_list args);

/************************************** UTF32 ***************************************/

void			Utf32DebugPrint(const utf32 str);

utf32 			Utf32Empty(void);
utf32			Utf32Alloc(struct arena *mem, const u32 len);
utf32			Utf32Buffered(u32 buf[], const u32 len);

utf32			Utf32CopyBuffered(u32 buf[], const u64 buflen, const utf32 str);
utf32			Utf32CstrBuffered(u32 buf[], const u64 buflen, const char *cstr);
utf32			Utf32F32Buffered(u32 buf[], const u64 buflen, const u32 decimals, const f32 val);
utf32			Utf32F64Buffered(u32 buf[], const u64 buflen, const u32 decimals, const f64 val);
utf32			Utf32U64Buffered(u32 buf[], const u64 buflen, const u64 val);
utf32			Utf32I64Buffered(u32 buf[], const u64 buflen, const i64 val);
utf32			Utf32Utf8Buffered(u32 buf[], const u64 buflen, const utf8 str);

utf32			Utf32Copy(struct arena *mem, const utf32 str);
utf32			Utf32Cstr(struct arena *mem, const char *cstr);
utf32			Utf32F32(struct arena *mem, const u32 decimals, const f32 val);
utf32			Utf32F64(struct arena *mem, const u32 decimals, const f64 val);
utf32			Utf32U64(struct arena *mem, const u64 val);
utf32			Utf32I64(struct arena *mem, const i64 val);
utf32			Utf32Utf8(struct arena *mem, const utf8 str);

f32			F32Utf32(struct arena *tmp, const utf32 str);
f64			F64Utf32(struct arena *tmp, const utf32 str);
struct parseRetval	U64Utf32(const utf32 str);	/* on overflow, return u64 value 0 		*/
struct parseRetval	I64Utf32(const utf32 str); 	/* on under/overflow, return i64 value 0 	*/

/************************************* Text Font Layout *************************************/

//TODO move this stuff into font library
//struct textGlyph
//{
//	f32 	x;		/* x[i] = offset of codepoint[i] glyph on line (in whole pixels) 	*/
//	u32 	codepoint;	/* text line codepoints 						*/
//};
//
//struct textLine
//{
//	struct textLine *	next;
//	u32			glyph_count;
//	struct textGlyph *	glyph;
//};
//
//struct textLayout
//{
//	struct textLine *	line;	
//	u32			line_count;
//	f32			width;		/* max line width */
//};
//
//struct font;
//
///* process utf32 according to font rules and construct required lines. Each line baseline starts at x = 0, with an 
// * height calculated according to linespace = ascent - descent + linegap. */
//struct textLayout *Utf32TextLayout(struct arena *mem, const utf32 *str, const f32 line_width, const u32 tab_size, const struct font *font);
//struct textLayout *Utf32TextLayoutIncludeWhitespace(struct arena *mem, const utf32 *str, const f32 line_width, const u32 tab_size, const struct font *font);

#ifdef __cplusplus
} 
#endif

#endif
