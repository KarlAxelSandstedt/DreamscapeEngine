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
    config.proxy_margin_min = F32Clamp(config.proxy_margin_min, 0.0f, F32MaxPositiveNormal());
    config.proxy_margin_scale = F32Clamp(config.proxy_margin_scale, 0.0f, F32MaxPositiveNormal());
    
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
    config.proxy_margin_min = 0.02f;
    config.proxy_margin_scale = 0.05f;
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

#define DS_NUMERICS_CONFIG_VERSION  2u
#define DS_NUMERICS_CONFIG_BITS     (32 + 10*32)

u64 ds_NumericsConfigSerializeBitPackedSize(const struct ds_NumericsConfig *config)
{
    return DS_NUMERICS_CONFIG_BITS;
}

void ds_NumericsConfigSerializeBitPacked(struct SS *ss, const struct ds_NumericsConfig *config)
{
    ds_Assert(DS_NUMERICS_CONFIG_BITS <= SSBitsLeft(ss));

    SSWriteU64Partial(ss, DS_NUMERICS_CONFIG_VERSION, 32);
    SSWriteU64Partial(ss, (b32) { .f = config->dbvh_reinsert_threshold }.u, 32);
    SSWriteU64Partial(ss, config->cache_count_max, 32);
    SSWriteU64Partial(ss, (b32) { .f = config->capsule_parallel_max_degrees }.u, 32);
    SSWriteU64Partial(ss, (b32) { .f = config->face_parallel_max_degrees }.u, 32);
    SSWriteU64Partial(ss, (b32) { .f = config->edge_parallel_max_degrees }.u, 32);
    SSWriteU64Partial(ss, (b32) { .f = config->sat_face_bias }.u, 32);
    SSWriteU64Partial(ss, (b32) { .f = config->sat_cache_tolerance }.u, 32);
    SSWriteU64Partial(ss, config->gjk_max_iterations, 32);
    SSWriteU64Partial(ss, (b32) { .f = config->proxy_margin_min }.u, 32);
    SSWriteU64Partial(ss, (b32) { .f = config->proxy_margin_scale }.u, 32);
}

u32 ds_NumericsConfigTryDeserializeBitPacked(struct SS *ss, struct ds_NumericsConfig *config)
{
    struct SS ss_mut = *ss;
    *config = (struct ds_NumericsConfig) { 0 };
    if (SSBitsLeft(&ss_mut) < DS_NUMERICS_CONFIG_BITS || SSReadU64Partial(&ss_mut, 32) != DS_NUMERICS_CONFIG_VERSION)
    {
        return 0;
    }

    config->dbvh_reinsert_threshold = (b32) { .u = (u32) SSReadU64Partial(&ss_mut, 32) }.f;
    config->cache_count_max = (u32) SSReadU64Partial(&ss_mut, 32);
    config->capsule_parallel_max_degrees = (b32) { .u = (u32) SSReadU64Partial(&ss_mut, 32) }.f;
    config->face_parallel_max_degrees = (b32) { .u = (u32) SSReadU64Partial(&ss_mut, 32) }.f;
    config->edge_parallel_max_degrees = (b32) { .u = (u32) SSReadU64Partial(&ss_mut, 32) }.f;
    config->sat_face_bias = (b32) { .u = (u32) SSReadU64Partial(&ss_mut, 32) }.f;
    config->sat_cache_tolerance = (b32) { .u = (u32) SSReadU64Partial(&ss_mut, 32) }.f;
    config->gjk_max_iterations = (u32) SSReadU64Partial(&ss_mut, 32);
    config->proxy_margin_min = (b32) { .u = (u32) SSReadU64Partial(&ss_mut, 32) }.f;
    config->proxy_margin_scale = (b32) { .u = (u32) SSReadU64Partial(&ss_mut, 32) }.f;
    *config = ds_NumericsConfigDerive(config);
    *ss = ss_mut;
    return 1;
}

