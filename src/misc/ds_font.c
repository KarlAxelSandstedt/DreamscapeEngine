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
#include "float32.h"

static u32 FontUtf32WhitespaceWidth(const struct font *font, const utf32 *whitespace, const u32 tab_size)
{
	u32 pixels = 0;

	const struct fontGlyph *glyph = GlyphLookup(font, (u32) ' ');
	const u32 space_pixels = glyph->advance;
	const u32 tab_pixels = tab_size*space_pixels;
	u32 new_line = 0;
	for (u32 i = 0; i < whitespace->len; ++i)
	{
		switch (whitespace->buf[i])
		{
			case ' ':
			{
				pixels += space_pixels;
			} break;

			case '\t':
			{
				pixels += tab_pixels;
			} break;

			case '\n':
			{
				new_line = 1;
			} break;

			default:
			{
				ds_Assert(0 && "whitespace string contains non-whitespace");
			}
		}
	}

	return (new_line) ? U32_MAX : pixels;
}

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
		if (substring_pixels + glyph->bearing[0] + glyph->size[0] > pixels_left)
		{
			break;
		}

		substring_pixels += glyph->advance;
		if (substring_pixels + linebreak->bearing[0] + linebreak->size[0] <= pixels_left)
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

	*x_new_offset = x_offset + substring_pixels;
	text->len -= sub.len;
	text->buf += sub.len;
	return sub;
}

struct textLayout *Utf32TextLayout(struct arena *mem, const utf32 *str, const f32 line_width, const u32 tab_size, const struct font *font)
{
	struct textLayout *layout = ArenaPush(mem, sizeof(struct textLayout));
	layout->line_count = 1;
	layout->line = ArenaPush(mem, sizeof(struct textLine));
	layout->line->next = NULL;
	layout->line->glyph_count = 0;
	layout->line->glyph = (void *) mem->stack_ptr;
	struct textLine *line = layout->line;

	const u32 line_pixels = (line_width == F32_INFINITY) ? U32_MAX : (u32) line_width;

	u32 x_offset = 0;
	u32 begin_new_line = 0;
	utf32 stream = *str;
	while (stream.len)
	{
		utf32 whitespace = Utf32StreamConsumeWhitespace(&stream);
		const u32 pixels = FontUtf32WhitespaceWidth(font, &whitespace, tab_size);
		x_offset = (pixels == U32_MAX || x_offset + pixels > (u32) line_pixels) ? line_pixels : x_offset + pixels;

		utf32 word = Utf32StreamConsumeNonWhitespace(&stream);
		/* fill row(s) with word */
		while (word.len)
		{
			if (begin_new_line)
			{
				layout->line_count += 1;
				line->next = ArenaPush(mem, sizeof(struct textLine));
				line = line->next;
				line->next = NULL;
				line->glyph_count = 0;
				line->glyph = (void *) mem->stack_ptr;
				begin_new_line = 0;
			}

			u32 x = x_offset;
			/* find substring of word that fits on row, advance substring */
			utf32 sub = FontStreamSubstringOnRow(&word, &x_offset, font, x_offset, line_pixels);
			for (u32 i = 0; i < sub.len; ++i)
			{
				ArenaPushPacked(mem, sizeof(struct textGlyph));
				line->glyph[line->glyph_count].x = x;
				line->glyph[line->glyph_count].codepoint = sub.buf[i];
				line->glyph_count += 1;
				x += GlyphLookup(font, sub.buf[i])->advance;
			}

			/* couldn't fit whole word on row */
			if (word.len)
			{
				begin_new_line = 1;
				if (sub.len == 0)
				{
					if (x_offset == 0)
					{
						break;
					}
				}
				else
				{
					ArenaPushPacked(mem, sizeof(struct textGlyph));
					line->glyph[line->glyph_count].x = x_offset;
					line->glyph[line->glyph_count].codepoint = (u32) '-';
					line->glyph_count += 1;
				}
				x_offset = 0;	
			}
		}
	}

	layout->width = (layout->line_count > 1)
		? line_width
		: (f32) x_offset;
	return layout;
}

struct textLayout *Utf32TextLayoutIncludeWhitespace(struct arena *mem, const utf32 *str, const f32 line_width, const u32 tab_size, const struct font *font)
{
	struct textLayout *layout = ArenaPush(mem, sizeof(struct textLayout));
	layout->line_count = 1;
	layout->line = ArenaPush(mem, sizeof(struct textLine));
	layout->line->next = NULL;
	layout->line->glyph_count = 0;
	layout->line->glyph = (void *) mem->stack_ptr;
	struct textLine *line = layout->line;

	const u32 line_pixels = (line_width == F32_INFINITY) ? U32_MAX : (u32) line_width;

	const struct fontGlyph *glyph = GlyphLookup(font, (u32) ' ');
	const u32 space_pixels = glyph->advance;
	const u32 tab_pixels = tab_size*space_pixels;

	u32 x_offset = 0;
	u32 begin_new_line = 0;
	utf32 stream = *str;
	while (stream.len)
	{
		utf32 whitespace = Utf32StreamConsumeWhitespace(&stream);

		u32 pixels = 0;
		u32 new_line = 0;
		for (u32 i = 0; i < whitespace.len; ++i)
		{
			ArenaPushPacked(mem, sizeof(struct textGlyph));
			line->glyph[line->glyph_count].x = x_offset;
			line->glyph[line->glyph_count].codepoint = whitespace.buf[i];
			line->glyph_count += 1;

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
					new_line = 1;
				} break;
		
				default:
				{
					ds_Assert(0 && "whitespace string contains non-whitespace");
				}
			}
		}
		x_offset = (new_line || x_offset > (u32) line_pixels) ? line_pixels : x_offset;

		utf32 word = Utf32StreamConsumeNonWhitespace(&stream);
		/* fill row(s) with word */
		while (word.len)
		{
			if (begin_new_line)
			{	
				layout->line_count += 1;
				line->next = ArenaPush(mem, sizeof(struct textLine));
				line = line->next;
				line->next = NULL;
				line->glyph_count = 0;
				line->glyph = (void *) mem->stack_ptr;
				begin_new_line = 0;
			}

			u32 x = x_offset;
			/* find substring of word that fits on row, advance substring */
			utf32 sub = FontStreamSubstringOnRow(&word, &x_offset, font, x_offset, line_pixels);
			for (u32 i = 0; i < sub.len; ++i)
			{
				ArenaPushPacked(mem, sizeof(struct textGlyph));
				line->glyph[line->glyph_count].x = x;
				line->glyph[line->glyph_count].codepoint = sub.buf[i];
				line->glyph_count += 1;
				x += GlyphLookup(font, sub.buf[i])->advance;
			}

			/* couldn't fit whole word on row */
			if (word.len)
			{
				begin_new_line = 1;
				if (sub.len == 0)
				{
					if (x_offset == 0)
					{
						break;
					}
				}
				else
				{
					ArenaPushPacked(mem, sizeof(struct textGlyph));
					line->glyph[line->glyph_count].x = x_offset;
					line->glyph[line->glyph_count].codepoint = (u32) '-';
					line->glyph_count += 1;
				}
				x_offset = 0;	
			}
		}
	}

	layout->width = (layout->line_count > 1)
		? line_width
		: (f32) x_offset;
	return layout;
}
