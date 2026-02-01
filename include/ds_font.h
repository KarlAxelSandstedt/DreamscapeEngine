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

#ifndef __DS_FONT_H__
#define __DS_FONT_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "asset_public.h"

struct textGlyph
{
	f32 	x;		/* x[i] = offset of codepoint[i] glyph on line (in whole pixels) 	*/
	u32 	codepoint;	/* text line codepoints 						*/
};

struct textLine
{
	struct textLine *	next;
	u32			glyph_count;
	struct textGlyph *	glyph;
};

struct textLayout
{
	struct textLine *	line;	
	u32			line_count;
	f32			width;		/* max line width */
};

struct font;

/* process utf32 according to font rules and construct required lines. Each line baseline starts at x = 0, with an 
 * height calculated according to linespace = ascent - descent + linegap. */
struct textLayout *	Utf32TextLayout(struct arena *mem, const utf32 *str, const f32 line_width, const u32 tab_size, const struct font *font);
struct textLayout *	Utf32TextLayoutIncludeWhitespace(struct arena *mem, const utf32 *str, const f32 line_width, const u32 tab_size, const struct font *font);

#ifdef __cplusplus
} 
#endif

#endif
