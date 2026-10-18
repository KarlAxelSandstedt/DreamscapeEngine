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

static struct test_Output (*asset_tests[])(struct test_Environment *) =
{
	SsffRequestTest,
};

struct suite_Correctness m_asset_suite =
{
	.id = "Asset",
	.unit_test = asset_tests,
	.unit_test_count = sizeof(asset_tests) / sizeof(asset_tests[0]),
};

struct suite_Correctness *asset_correctness_suite = &m_asset_suite;
