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
#include "ds_asset.h"
#include "ds_font.h"
#include "ds_float.h"

/*
Request every sprite sheet. In a DS_DEV build the request rebuilds the sheet from its PNGs and saves it
(assets/sprites, the .ssff files: this is how the sheets are regenerated after a stream format change), then loads and
decodes it; the decode's asserts check the sheet against its header.
*/
struct test_Output SsffRequestTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

	const enum ssffId id[] = { SSFF_LED_ID, SSFF_DYNAMIC_ID };
	for (u32 i = 0; i < sizeof(id) / sizeof(id[0]); ++i)
	{
		struct assetSsff *asset = AssetRequestSsff(env->mem_1, id[i]);
		TEST_NOT_ZERO(asset->loaded);
		TEST_NOT_ZERO(asset->width && asset->height);
		TEST_NOT_ZERO(asset->pixel != NULL);
	}

	return output;
}

/* Request every font: in a DS_DEV build the request rebuilds it from its TTF with freetype and saves it */
struct test_Output FontRequestTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

	const enum fontId id[] = { FONT_DEFAULT_SMALL, FONT_DEFAULT_MEDIUM };
	for (u32 i = 0; i < sizeof(id) / sizeof(id[0]); ++i)
	{
		struct assetFont *asset = AssetRequestFont(env->mem_1, id[i]);
		TEST_NOT_ZERO(asset->loaded);
		TEST_NOT_ZERO(asset->font != NULL);
	}

	return output;
}

/*
With both sheets loaded, every sprite id maps to a sheet of the table (and so to a texture id), and every sprite but
SPRITE_NONE belongs to a loaded sheet and has a size. An entry a sheet's parameter setter (asset_init.c) leaves unset
keeps whatever the arena held.
*/
struct test_Output SpriteTableTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

	AssetRequestSsff(env->mem_1, SSFF_LED_ID);
	AssetRequestSsff(env->mem_1, SSFF_DYNAMIC_ID);
	for (u32 s = 0; s < SPRITE_COUNT; ++s)
	{
		const struct sprite *spr = g_sprite + s;
		if (spr->ssff_id >= SSFF_COUNT)
		{
			fprintf(stderr, "sprite %u: ssff_id %u, the table has %u sheets\n", s, spr->ssff_id, SSFF_COUNT);
			TEST_FAILURE;
		}
		TEST_NOT_ZERO(AssetSpriteGetTextureId((enum spriteId) s) < TEXTURE_COUNT);
		if (s != SPRITE_NONE)
		{
			TEST_NOT_ZERO(spr->ssff_id != SSFF_NONE_ID && g_asset_db->ssff[spr->ssff_id]->loaded);
			TEST_NOT_ZERO(spr->pixel_size.x && spr->pixel_size.y);
		}
	}

	return output;
}

/* A flush unloads the sheets; the next request loads them again with the same sprite parameters */
struct test_Output AssetFlushReloadTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

	const enum ssffId id[] = { SSFF_LED_ID, SSFF_DYNAMIC_ID };
	struct sprite before[SPRITE_COUNT];
	for (u32 i = 0; i < sizeof(id) / sizeof(id[0]); ++i)
	{
		AssetRequestSsff(env->mem_1, id[i]);
	}
	memcpy(before, g_sprite, sizeof(before));

	AssetFlush();
	for (u32 i = 0; i < sizeof(id) / sizeof(id[0]); ++i)
	{
		TEST_NOT_ZERO(!g_asset_db->ssff[id[i]]->loaded);
	}

	for (u32 i = 0; i < sizeof(id) / sizeof(id[0]); ++i)
	{
		struct assetSsff *asset = AssetRequestSsff(env->mem_1, id[i]);
		TEST_NOT_ZERO(asset->loaded);
		TEST_NOT_ZERO(asset->pixel != NULL);
	}
	for (u32 s = 0; s < SPRITE_COUNT; ++s)
	{
		/* the same sheet decoded the same way: bit-identical parameters */
		TEST_NOT_ZERO(before[s].pixel_size.x == g_sprite[s].pixel_size.x && before[s].pixel_size.y == g_sprite[s].pixel_size.y);
		TEST_NOT_ZERO(before[s].bl.x == g_sprite[s].bl.x && before[s].bl.y == g_sprite[s].bl.y);
		TEST_NOT_ZERO(before[s].tr.x == g_sprite[s].tr.x && before[s].tr.y == g_sprite[s].tr.y);
	}

	return output;
}

/*
The glyph atlas of each font: every glyph's rectangle lies inside the pixmap and no two overlap (a glyph taller than
the row stride bleeds into the next row), and the unknown glyph has pixels and an advance (an unmapped codepoint shows
a box, not nothing).
*/
struct test_Output FontAtlasTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

	const enum fontId id[] = { FONT_DEFAULT_SMALL, FONT_DEFAULT_MEDIUM };
	for (u32 k = 0; k < sizeof(id) / sizeof(id[0]); ++k)
	{
		const struct font *font = AssetRequestFont(env->mem_1, id[k])->font;
		const f32 w = (f32) font->pixmap_width;
		const f32 h = (f32) font->pixmap_height;
		u32 overlaps = 0;
		for (u32 i = 0; i < font->glyph_count; ++i)
		{
			const struct fontGlyph *g = font->glyph + i;
			TEST_NOT_ZERO(g->bl.x >= 0.0f && g->bl.y >= 0.0f && g->tr.x <= 1.0f && g->tr.y <= 1.0f);
			TEST_NOT_ZERO(g->tr.x - g->bl.x >= 0.0f && g->tr.y - g->bl.y >= 0.0f);
			for (u32 j = i + 1; j < font->glyph_count; ++j)
			{
				const struct fontGlyph *o = font->glyph + j;
				/* strict overlap in pixels; shared edges are fine */
				if (g->bl.x*w < o->tr.x*w - 0.5f && o->bl.x*w < g->tr.x*w - 0.5f
					&& g->bl.y*h < o->tr.y*h - 0.5f && o->bl.y*h < g->tr.y*h - 0.5f)
				{
					if (overlaps == 0)
					{
						fprintf(stderr, "font %u: glyph U+%04X (%i x %i) overlaps glyph U+%04X (%i x %i) in the atlas\n",
							id[k], g->codepoint, g->size.x, g->size.y, o->codepoint, o->size.x, o->size.y);
					}
					overlaps += 1;
				}
			}
		}
		if (overlaps)
		{
			fprintf(stderr, "font %u: %u overlapping glyph pairs\n", id[k], overlaps);
			TEST_FAILURE;
		}

		const struct fontGlyph *unknown = font->glyph + font->glyph_unknown_index;
		TEST_NOT_ZERO(unknown->size.x > 0 && unknown->size.y > 0 && unknown->advance > 0);
		TEST_NOT_ZERO(GlyphLookup(font, 0x10FFFF) == unknown);
	}

	return output;
}

/* A layout's width is its widest line's extent, whatever the line width and the trailing whitespace */
struct test_Output TextLayoutWidthTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

	const struct font *font = AssetRequestFont(env->mem_1, FONT_DEFAULT_SMALL)->font;
	const f32 abc = (f32) (3 * GlyphLookup(font, (u32) 'a')->advance);	/* monospace */
	const struct { const char *text; f32 line_width; f32 width; u32 lines; } c[] =
	{
		{ "abc",      F32_INFINITY, abc, 1 },
		{ "abc\n",    F32_INFINITY, abc, 1 },
		{ "abc\n",    200.0f,       abc, 1 },
		{ "ab\ncd",   F32_INFINITY, abc * 2.0f / 3.0f, 2 },
		{ "abc\nabcd", F32_INFINITY, abc * 4.0f / 3.0f, 2 },
	};
	for (u32 i = 0; i < sizeof(c) / sizeof(c[0]); ++i)
	{
		const utf32 str = Utf32Cstr(env->mem_1, c[i].text);
		const struct textLayout *layout = Utf32TextLayout(env->mem_1, &str, c[i].line_width, 4, font);
		if (layout->line_count != c[i].lines || layout->width != c[i].width)
		{
			fprintf(stderr, "layout %u: %u lines, width %g (expected %u lines, width %g)\n", i, layout->line_count, layout->width, c[i].lines, c[i].width);
			TEST_FAILURE;
		}
	}

	return output;
}

static struct test_Output (*asset_tests[])(struct test_Environment *) =
{
	SsffRequestTest,
	FontRequestTest,
	SpriteTableTest,
	AssetFlushReloadTest,
	FontAtlasTest,
	TextLayoutWidthTest,
};

struct suite_Correctness m_asset_suite =
{
	.id = "Asset",
	.unit_test = asset_tests,
	.unit_test_count = sizeof(asset_tests) / sizeof(asset_tests[0]),
};

struct suite_Correctness *asset_correctness_suite = &m_asset_suite;
