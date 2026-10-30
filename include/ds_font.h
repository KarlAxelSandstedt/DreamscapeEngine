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

#include "ds_asset.h"

/*
Text layout: a utf32 string becomes lines of glyph positions for a font (struct font, ds_asset.h), ready for the
renderer (r_scene.c) and the UI's cursors and selections (ui_core.c).

Lines are filled word by word (whitespace: ' ', '\t', '\n'; everything else is a word). A word that does not fit
after the line's pen starts the next line; a word longer than a line is split where it stops fitting, with a '-'
appended when the hyphen fits; a glyph wider than the line overflows it (one glyph per line, nothing is dropped).
A '\n' ends its line at once, so a text ending in '\n' ends with an empty line. Whitespace past the end of the
line fills it. Kerning is ignored.

Sections: layout structs, the layout functions.
*/

/********************************** layout structs **********************************/

struct textGlyph
{
	f32 	x;		/* pen position of the glyph on its line, whole pixels; the glyph's bearing is the caller's */
	u32 	codepoint;	/* text line codepoints 						*/
};

struct textLine
{
	struct textLine *	next;
	u32			glyph_count;
	struct textGlyph *	glyph;	/* glyph[glyph_count], contiguous */
};

struct textLayout
{
	struct textLine *	line;	/* line_count >= 1 lines, in order */
	u32			line_count;
	f32			width;		/* the widest line's extent: its pen after the last glyph or whitespace; <= line_width */
};

struct font;

/********************************** the layout functions **********************************/

/*
 * Lay str out on lines at most line_width pixels wide (F32_INFINITY: no wrapping; 0: one glyph per line), tabs
 * tab_size spaces wide. The layout, its lines and glyphs are pushed on mem. Each line's pen starts at x = 0;
 * the caller places lines font->linespace apart and adds each glyph's bearing. Utf32TextLayout only advances
 * the pen over whitespace; Utf32TextLayoutIncludeWhitespace emits a glyph per whitespace codepoint too (at the
 * pen before it, the '\n' on the line it ends), so every codepoint of str has a glyph, for cursors and
 * selections. Both give the same lines and widths.
 */
struct textLayout *	Utf32TextLayout(struct arena *mem, const utf32 *str, const f32 line_width, const u32 tab_size, const struct font *font);
struct textLayout *	Utf32TextLayoutIncludeWhitespace(struct arena *mem, const utf32 *str, const f32 line_width, const u32 tab_size, const struct font *font);

#ifdef __cplusplus
} 
#endif

#endif
