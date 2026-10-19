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

static struct test_Output (*asset_tests[])(struct test_Environment *) =
{
	SsffRequestTest,
	FontRequestTest,
	SpriteTableTest,
	AssetFlushReloadTest,
};

struct suite_Correctness m_asset_suite =
{
	.id = "Asset",
	.unit_test = asset_tests,
	.unit_test_count = sizeof(asset_tests) / sizeof(asset_tests[0]),
};

struct suite_Correctness *asset_correctness_suite = &m_asset_suite;
