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

#include "ds_float.h"
#include "ds_vector.h"

struct ds_NumericsConfig *g_numerics_config = NULL;

struct ds_NumericsConfig ds_NumericsConfigDefault(void)
{
    struct ds_NumericsConfig config = { 0 };
    config.dbvh_reinsert_threshold = 0.25f;
    config.cache_count_max = 32;
    config.gjk_max_iterations = 32;
    /* tan(2.86 deg) ~ 0.05 */
    config.capsule_parallel_max_degrees = 2.86f;
    /* Box3D: cos(3.62 deg) ~ 0.998 */
    config.face_parallel_max_degrees = 3.62f;
    /* hull-hull's former hard-coded cut-off (sin^2(0.5 deg) ~ 7.62e-5); Box3D: ~0.29 deg */
    config.edge_parallel_max_degrees = 0.5f;
    config.sat_face_bias = 5.0f * DS_UNIT_MM;
    config.sat_cache_tolerance = 5.0f * DS_UNIT_MM;
    ds_NumericsConfigDerive(&config);

    return config;
}

void ds_NumericsConfigDerive(struct ds_NumericsConfig *config)
{
    const f32 capsule_tan = F32Tan(config->capsule_parallel_max_degrees * F32_PI2 / 360);
    config->capsule_parallel_tan_sq = capsule_tan*capsule_tan;

    config->face_parallel_cos = F32Cos(config->face_parallel_max_degrees * F32_PI2 / 360);

    const f32 edge_sin = F32Sin(config->edge_parallel_max_degrees * F32_PI2 / 360);
    config->edge_parallel_sin_sq = edge_sin*edge_sin;
}

void ds_NumericsConfigPush(struct ds_NumericsConfig *config)
{
    ds_NumericsConfigDerive(config);
    g_numerics_config = config;
}

void ds_NumericsConfigPop(void)
{
    ds_Assert(g_numerics_config);
    g_numerics_config = NULL;
}
