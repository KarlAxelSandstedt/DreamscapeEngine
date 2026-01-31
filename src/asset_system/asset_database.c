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

#include "asset_local.h"

void AssetFlush(void)
{
	/* id 0 == SSFF_NONE */
	for (u32 id = 1; id < SSFF_COUNT; ++id)
	{
		struct assetSsff *asset = g_asset_db->ssff[id];
#ifdef DS_DEV
		asset->valid = 0;
#endif
		if (asset->loaded)
		{
			free(asset->pixel);
		}
		asset->loaded = 0;
	}
}
