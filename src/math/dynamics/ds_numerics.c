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
    config.dbvh_reinsert_threshold_pending = 0.25f;
    config.cache_count_max_pending = 32;
    config.gjk_max_iterations_pending = 32;
    /* tan(2.86 deg) ~ 0.05 */
    config.capsule_parallel_max_degrees_pending = 2.86f;
    /* Box3D: cos(3.62 deg) ~ 0.998 */
    config.face_parallel_max_degrees_pending = 3.62f;
    /* hull-hull's former hard-coded cut-off (sin^2(0.5 deg) ~ 7.62e-5); Box3D: ~0.29 deg */
    config.edge_parallel_max_degrees_pending = 0.5f;
    config.sat_face_bias_pending = 5.0f * DS_UNIT_MM;
    config.sat_cache_tolerance_pending = 5.0f * DS_UNIT_MM;

    return config;
}

void ds_NumericsConfigPush(struct ds_NumericsConfig *config)
{
    g_numerics_config = config;

    config->dbvh_reinsert_threshold_pending = F32Clamp(config->dbvh_reinsert_threshold_pending, 0.0f, 1.0f);
    config->dbvh_reinsert_threshold = config->dbvh_reinsert_threshold_pending;

    config->cache_count_max = config->cache_count_max_pending;

    config->gjk_max_iterations_pending = (config->gjk_max_iterations_pending > 0) ? config->gjk_max_iterations_pending : 1;
    config->gjk_max_iterations = config->gjk_max_iterations_pending;

    config->capsule_parallel_max_degrees_pending = F32Clamp(config->capsule_parallel_max_degrees_pending, 0.0f, 45.0f);
    config->capsule_parallel_max_degrees = config->capsule_parallel_max_degrees_pending;
    const f32 capsule_tan = F32Tan(config->capsule_parallel_max_degrees * F32_PI2 / 360);
    config->capsule_parallel_tan_sq = capsule_tan*capsule_tan;

    config->face_parallel_max_degrees_pending = F32Clamp(config->face_parallel_max_degrees_pending, 0.0f, 45.0f);
    config->face_parallel_max_degrees = config->face_parallel_max_degrees_pending;
    config->face_parallel_cos = F32Cos(config->face_parallel_max_degrees * F32_PI2 / 360);

    config->edge_parallel_max_degrees_pending = F32Clamp(config->edge_parallel_max_degrees_pending, 0.0f, 45.0f);
    config->edge_parallel_max_degrees = config->edge_parallel_max_degrees_pending;
    const f32 edge_sin = F32Sin(config->edge_parallel_max_degrees * F32_PI2 / 360);
    config->edge_parallel_sin_sq = edge_sin*edge_sin;

    config->sat_face_bias_pending = F32Clamp(config->sat_face_bias_pending, 0.0f, F32_INFINITY);
    config->sat_face_bias = config->sat_face_bias_pending;

    config->sat_cache_tolerance_pending = F32Clamp(config->sat_cache_tolerance_pending, 0.0f, F32_INFINITY);
    config->sat_cache_tolerance = config->sat_cache_tolerance_pending;
}

void ds_NumericsConfigPop(void)
{
    ds_Assert(g_numerics_config);
    g_numerics_config = NULL;
}



