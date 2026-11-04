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

struct ds_NumericsConfig ds_NumericsConfigDerive(const struct ds_NumericsConfig *pending)
{
    struct ds_NumericsConfig config = *pending;

    config.dbvh_reinsert_threshold = F32Clamp(config.dbvh_reinsert_threshold, 0.0f, 1.0f);
    config.cache_count_max = U32Max(config.cache_count_max, 1);
    config.gjk_max_iterations = U32Max(config.gjk_max_iterations, 1);
    config.capsule_parallel_max_degrees = F32Clamp(config.capsule_parallel_max_degrees, 0.0f, 45.0f);
    config.face_parallel_max_degrees = F32Clamp(config.face_parallel_max_degrees, 0.0f, 45.0f);
    config.edge_parallel_max_degrees = F32Clamp(config.edge_parallel_max_degrees, 0.0f, 45.0f);
    config.sat_face_bias = F32Clamp(config.sat_face_bias, 0.0f, F32MaxPositiveNormal());
    config.sat_cache_tolerance = F32Clamp(config.sat_cache_tolerance, 0.0f, F32MaxPositiveNormal());
    
    const f32 capsule_tan = F32Tan(config.capsule_parallel_max_degrees * F32_PI2 / 360);
    config.capsule_parallel_tan_sq = capsule_tan*capsule_tan;

    config.face_parallel_cos = F32Cos(config.face_parallel_max_degrees * F32_PI2 / 360);

    const f32 edge_sin = F32Sin(config.edge_parallel_max_degrees * F32_PI2 / 360);
    config.edge_parallel_sin_sq = edge_sin*edge_sin;
   
    return config;
}

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

    config = ds_NumericsConfigDerive(&config);
    return config;
}

void ds_NumericsConfigPush(struct ds_NumericsConfig *config)
{
    g_numerics_config = config;
}

void ds_NumericsConfigPop(void)
{
    ds_Assert(g_numerics_config);
    g_numerics_config = NULL;
}

#define DS_NUMERICS_CONFIG_VERSION  1u
/* version + 8 set values (the derived ones are recomputed on read) */
#define DS_NUMERICS_CONFIG_SIZE     (4 + 8*4)

u64 ds_NumericsConfigSerializeSize(const struct ds_NumericsConfig *config)
{
    return DS_NUMERICS_CONFIG_SIZE;
}

void ds_NumericsConfigSerialize(struct SS *ss, const struct ds_NumericsConfig *config)
{
    ds_Assert(ss->bit_index % 8 == 0);
    ds_Assert(ds_NumericsConfigSerializeSize(config) <= SSBytesLeft(ss));

    SSWriteU32Le(ss, DS_NUMERICS_CONFIG_VERSION);
    SSWriteF32Le(ss, config->dbvh_reinsert_threshold);
    SSWriteU32Le(ss, config->cache_count_max);
    SSWriteF32Le(ss, config->capsule_parallel_max_degrees);
    SSWriteF32Le(ss, config->face_parallel_max_degrees);
    SSWriteF32Le(ss, config->edge_parallel_max_degrees);
    SSWriteF32Le(ss, config->sat_face_bias);
    SSWriteF32Le(ss, config->sat_cache_tolerance);
    SSWriteU32Le(ss, config->gjk_max_iterations);
}

u32 ds_NumericsConfigTryDeserialize(struct SS *ss, struct ds_NumericsConfig *config)
{
    const u64 bit_index = ss->bit_index;
    u32 success = 0;

    if (ss->bit_index % 8 != 0 || SSBytesLeft(ss) < DS_NUMERICS_CONFIG_SIZE)
    {
        goto end;
    }

    if (SSReadU32Le(ss) != DS_NUMERICS_CONFIG_VERSION)
    {
        goto end;
    }

    *config = (struct ds_NumericsConfig) { 0 };
    config->dbvh_reinsert_threshold = SSReadF32Le(ss);
    config->cache_count_max = SSReadU32Le(ss);
    config->capsule_parallel_max_degrees = SSReadF32Le(ss);
    config->face_parallel_max_degrees = SSReadF32Le(ss);
    config->edge_parallel_max_degrees = SSReadF32Le(ss);
    config->sat_face_bias = SSReadF32Le(ss);
    config->sat_cache_tolerance = SSReadF32Le(ss);
    config->gjk_max_iterations = SSReadU32Le(ss);
    *config = ds_NumericsConfigDerive(config);
    success = 1;

end:
    if (!success)
    {
        ss->bit_index = bit_index;
        *config = (struct ds_NumericsConfig) { 0 };
    }

    return success;
}
