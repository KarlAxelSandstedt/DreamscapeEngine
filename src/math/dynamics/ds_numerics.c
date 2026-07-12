#include "dynamics.h"

struct ds_NumericsConfig *g_numerics_config = NULL;

struct ds_NumericsConfig ds_NumericsConfigDefault(void)
{
    struct ds_NumericsConfig config = { 0 };
    config.vec3_parallel_check_max_degrees_pending = 0.5f;
    config.manifold_cached_normal_parallel_check_max_degrees_pending = 10.0f;

    return config;
}

void ds_NumericsConfigPush(struct ds_NumericsConfig *config)
{
    g_numerics_config = config;

    config->vec3_parallel_check_max_degrees_pending = f32_clamp(config->vec3_parallel_check_max_degrees_pending, 0.0f, 45.0f);
    config->vec3_parallel_check_max_degrees = config->vec3_parallel_check_max_degrees_pending;
    config->vec3_parallel_check_eps = Vec3ParallelCheckEpsilon(config->vec3_parallel_check_max_degrees);

    /* Since we are dealing with normals, easy test becomes Dot(a,b) > eps, eps = cos(max_degrees_in_radian) */
    config->manifold_cached_normal_parallel_check_max_degrees_pending = f32_clamp(config->manifold_cached_normal_parallel_check_max_degrees_pending, 0.0f, 45.0f);
    config->manifold_cached_normal_parallel_check_max_degrees = config->manifold_cached_normal_parallel_check_max_degrees_pending;
    config->manifold_cached_normal_parallel_check_eps = f32_cos(config->manifold_cached_normal_parallel_check_max_degrees * F32_PI2 / 360);
}

void ds_NumericsConfigPop(void)
{
    ds_Assert(g_numerics_config);
    g_numerics_config = NULL;
}



