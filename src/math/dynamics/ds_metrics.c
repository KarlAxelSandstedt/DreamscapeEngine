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

void ds_DynamicsMetricsFlush(struct ds_DynamicsMetrics *metrics)
{
    metrics->hull_call_count = 0;
    metrics->hull_cache_probe_count = 0;
    metrics->hull_cache_eviction_count = 0;

    metrics->mesh_call_count = 0;
    metrics->mesh_cache_probe_count = 0;
    metrics->mesh_cache_eviction_count = 0;

    metrics->warm_point_count = 0;
    metrics->warm_match_id = 0;
}

void ds_DynamicsMetricsAdd(struct ds_DynamicsMetrics *sum, const struct ds_DynamicsMetrics *metrics)
{
   sum->hull_call_count += metrics->hull_call_count;
   sum->hull_cache_probe_count += metrics->hull_cache_probe_count;
   sum->hull_cache_eviction_count += metrics->hull_cache_eviction_count;

   sum->mesh_call_count += metrics->mesh_call_count;
   sum->mesh_cache_probe_count += metrics->mesh_cache_probe_count;
   sum->mesh_cache_eviction_count += metrics->mesh_cache_eviction_count;

   sum->warm_point_count += metrics->warm_point_count;
   sum->warm_match_id += metrics->warm_match_id;
}

void ds_DynamicsMetricsPrint(FILE *file, const struct ds_DynamicsMetrics *metrics)
{
    fprintf(file, "================= Statistics ==============\n");

    if (metrics->mesh_call_count)
    {
        const f32 valid_cache_probe_count = metrics->mesh_cache_probe_count - metrics->mesh_cache_eviction_count;
        const f32 mesh_full_compute_rate = 100.0f - 100.0f * valid_cache_probe_count / metrics->mesh_call_count;
        fprintf(file, "mesh_full_compute_rate: %.2f%%\n", mesh_full_compute_rate);
    }

    if (metrics->mesh_cache_probe_count)
    {
        const f32 mesh_cache_hit_rate = 100.0f * (f32) (metrics->mesh_cache_probe_count - metrics->mesh_cache_eviction_count) / metrics->mesh_cache_probe_count;
        fprintf(file, "mesh_cache_hit_rate:    %.2f%%\n", mesh_cache_hit_rate);
    }

    if (metrics->hull_call_count)
    {
        const f32 valid_cache_probe_count = metrics->hull_cache_probe_count - metrics->hull_cache_eviction_count;
        const f32 hull_full_compute_rate = 100.0f - 100.0f * valid_cache_probe_count / metrics->hull_call_count;
        fprintf(file, "hull_full_compute_rate: %.2f%%\n", hull_full_compute_rate);
    }

    if (metrics->hull_cache_probe_count)
    {
        const f32 hull_cache_hit_rate = 100.0f * (f32) (metrics->hull_cache_probe_count - metrics->hull_cache_eviction_count) / metrics->hull_cache_probe_count;
        fprintf(file, "hull_cache_hit_rate:    %.2f%%\n", hull_cache_hit_rate);
    }

    if (metrics->warm_point_count)
    {
        const f32 warm_match_rate = 100.0f * (f32) metrics->warm_match_id / (f32) metrics->warm_point_count;
        fprintf(file, "warm_point_count:       %u\n", metrics->warm_point_count);
        fprintf(file, "warm_match_rate:        %.2f%%\n", warm_match_rate);
    }
}
