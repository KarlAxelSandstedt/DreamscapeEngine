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

struct ds_NumericsConfig *g_numerics_config = NULL;

struct ds_NumericsConfig ds_NumericsConfigDefault(void)
{
    struct ds_NumericsConfig config = { 0 };
    config.vec3_parallel_check_max_degrees_pending = 0.5f;
    config.manifold_cache_normal_parallel_check_max_degrees_pending = 10.0f;
    config.manifold_cache_depth_max_diff_allowed_pending = DS_UNIT_CM;
    config.manifold_cache_linear_velocity_max_diff_allowed_pending = 10.0f * DS_UNIT_M;
    config.dbvh_reinsert_threshold_pending = 0.25f;

    return config;
}

void ds_NumericsConfigPush(struct ds_NumericsConfig *config)
{
    g_numerics_config = config;

    config->dbvh_reinsert_threshold = f32_clamp(config->dbvh_reinsert_threshold_pending, 0.0f, 1.0f);

    config->vec3_parallel_check_max_degrees_pending = f32_clamp(config->vec3_parallel_check_max_degrees_pending, 0.0f, 45.0f);
    config->vec3_parallel_check_max_degrees = config->vec3_parallel_check_max_degrees_pending;
    config->vec3_parallel_check_eps = Vec3ParallelCheckEpsilon(config->vec3_parallel_check_max_degrees);

    /* Since we are dealing with normals, easy test becomes Dot(a,b) > eps, eps = cos(max_degrees_in_radian) */
    config->manifold_cache_normal_parallel_check_max_degrees_pending = f32_clamp(config->manifold_cache_normal_parallel_check_max_degrees_pending, 0.0f, 45.0f);
    config->manifold_cache_normal_parallel_check_max_degrees = config->manifold_cache_normal_parallel_check_max_degrees_pending;
    config->manifold_cache_normal_parallel_check_eps = f32_cos(config->manifold_cache_normal_parallel_check_max_degrees * F32_PI2 / 360);

    config->manifold_cache_depth_max_diff_allowed_pending = f32_clamp(config->manifold_cache_depth_max_diff_allowed_pending, 0.0f, F32_INFINITY);
    config->manifold_cache_depth_max_diff_allowed = config->manifold_cache_depth_max_diff_allowed_pending;

    config->manifold_cache_linear_velocity_max_diff_allowed_pending = f32_clamp(config->manifold_cache_linear_velocity_max_diff_allowed_pending, 0.0f, F32_INFINITY);
    config->manifold_cache_linear_velocity_max_diff_allowed = config->manifold_cache_linear_velocity_max_diff_allowed_pending;
}

void ds_NumericsConfigPop(void)
{
    ds_Assert(g_numerics_config);
    g_numerics_config = NULL;
}



