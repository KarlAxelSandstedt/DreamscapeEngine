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

#include "ds_font.h"
#include "ds_float.h"

static utf32 FontStreamSubstringOnRow(utf32 *text, u32 *x_new_offset, const struct font *font, const u32 x_offset, const u32 line_width)
{
	utf32 sub = { .len = 0, .buf = text->buf };

	const u32 pixels_left = line_width - x_offset;
	u32 substring_pixels = 0;

	const struct fontGlyph *linebreak = GlyphLookup(font, (u32) '-');
	u32 substring_with_wordbreak_len = 0;
	u32 substring_pixels_with_wordbreak = 0;
	
	for (; sub.len < text->len; ++sub.len)
	{
		const struct fontGlyph *glyph = GlyphLookup(font, text->buf[sub.len]);
		if (substring_pixels + glyph->bearing.x + glyph->size.x > pixels_left)
		{
			break;
		}

		substring_pixels += glyph->advance;
		if (substring_pixels + linebreak->bearing.x + linebreak->size.x <= pixels_left)
		{
			substring_with_wordbreak_len += 1;
			substring_pixels_with_wordbreak += glyph->advance;
		}
	}
	
	if (0 < sub.len && sub.len < text->len)
	{
		sub.len = substring_with_wordbreak_len;
		substring_pixels = (substring_with_wordbreak_len) ? substring_pixels_with_wordbreak : 0;
	}
	/* nothing fits an empty line: one glyph overflows it, so every glyph is placed and the word advances */
	if (sub.len == 0 && x_offset == 0 && text->len)
	{
		sub.len = 1;
		substring_pixels = GlyphLookup(font, text->buf[0])->advance;
	}

	*x_new_offset = x_offset + substring_pixels;
	text->len -= sub.len;
	text->buf += sub.len;
	return sub;
}

/* append a line; its glyphs are pushed packed right behind it, so nothing else may push on mem meanwhile */
static struct textLine *FontLayoutLineNew(struct arena *mem, struct textLayout *layout, struct textLine *line)
{
	struct textLine *next = ArenaPush(mem, sizeof(struct textLine));
	next->next = NULL;
	next->glyph_count = 0;
	next->glyph = (void *) mem->stack_ptr;
	if (line)
	{
		line->next = next;
	}
	else
	{
		layout->line = next;
	}
	layout->line_count += 1;
	return next;
}

static void FontLayoutGlyphPush(struct arena *mem, struct textLine *line, const u32 x, const u32 codepoint)
{
	ArenaPushPacked(mem, sizeof(struct textGlyph));
	line->glyph[line->glyph_count].x = (f32) x;
	line->glyph[line->glyph_count].codepoint = codepoint;
	line->glyph_count += 1;
}

/*
 * Lines are filled word by word. A word that does not fit after the line's pen starts the next line; a word
 * longer than a line is split where it stops fitting, with a '-' when the hyphen fits. Whitespace advances the
 * pen (and, with include_whitespace, becomes glyphs); a '\n' ends its line at once, so a text ending in '\n'
 * ends with an empty line. The width is the widest line's extent: the pen after its last glyph or whitespace,
 * never past line_width.
 */
static struct textLayout *FontLayout(struct arena *mem, const utf32 *str, const f32 line_width, const u32 tab_size, const struct font *font, const u32 include_whitespace)
{
	struct textLayout *layout = ArenaPush(mem, sizeof(struct textLayout));
	layout->line = NULL;
	layout->line_count = 0;
	layout->width = 0.0f;
	struct textLine *line = FontLayoutLineNew(mem, layout, NULL);

	const u32 line_pixels = (line_width == F32_INFINITY) ? U32_MAX : (u32) line_width;
	const u32 space_pixels = GlyphLookup(font, (u32) ' ')->advance;
	const u32 tab_pixels = tab_size*space_pixels;
	const struct fontGlyph *hyphen = GlyphLookup(font, (u32) '-');

	u32 x_offset = 0;	/* the line's pen */
	u32 widest = 0;		/* the widest line's extent */
	u32 wrap = 0;		/* the line is full: the next word starts a new one */
	utf32 stream = *str;
	while (stream.len)
	{
		utf32 whitespace = Utf32StreamConsumeWhitespace(&stream);
		for (u32 i = 0; i < whitespace.len; ++i)
		{
			if (include_whitespace)
			{
				FontLayoutGlyphPush(mem, line, x_offset, whitespace.buf[i]);
			}
			switch (whitespace.buf[i])
			{
				case ' ':
				{
					x_offset += space_pixels;
				} break;

				case '\t':
				{
					x_offset += tab_pixels;
				} break;

				case '\n':
				{
					widest = (x_offset > widest) ? x_offset : widest;
					line = FontLayoutLineNew(mem, layout, line);
					x_offset = 0;
					wrap = 0;
				} break;

				default:
				{
					ds_Assert(0 && "whitespace string contains non-whitespace");
				}
			}
			/* whitespace past the end of the line fills it: the next word wraps */
			x_offset = (x_offset > line_pixels) ? line_pixels : x_offset;
		}
		widest = (x_offset > widest) ? x_offset : widest;

		utf32 word = Utf32StreamConsumeNonWhitespace(&stream);
		/* fill row(s) with word */
		while (word.len)
		{
			if (wrap)
			{
				line = FontLayoutLineNew(mem, layout, line);
				x_offset = 0;
				wrap = 0;
			}

			u32 x = x_offset;
			/* find substring of word that fits on row, advance substring */
			const utf32 sub = FontStreamSubstringOnRow(&word, &x_offset, font, x_offset, line_pixels);
			for (u32 i = 0; i < sub.len; ++i)
			{
				FontLayoutGlyphPush(mem, line, x, sub.buf[i]);
				x += GlyphLookup(font, sub.buf[i])->advance;
			}

			/* the rest of the word goes to the next line */
			if (word.len)
			{
				wrap = 1;
				if (sub.len)
				{
					FontLayoutGlyphPush(mem, line, x_offset, (u32) '-');
					x_offset += hyphen->advance;
				}
			}
			widest = (x_offset > widest) ? x_offset : widest;
		}
	}

	layout->width = (f32) widest;
	return layout;
}

struct textLayout *Utf32TextLayout(struct arena *mem, const utf32 *str, const f32 line_width, const u32 tab_size, const struct font *font)
{
	return FontLayout(mem, str, line_width, tab_size, font, 0);
}

struct textLayout *Utf32TextLayoutIncludeWhitespace(struct arena *mem, const utf32 *str, const f32 line_width, const u32 tab_size, const struct font *font)
{
	return FontLayout(mem, str, line_width, tab_size, font, 1);
}
