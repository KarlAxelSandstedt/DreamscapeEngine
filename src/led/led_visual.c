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

#include "led_local.h"

struct led_Visual 	visual_storage = { 0 };
struct led_Visual *	g_visual = &visual_storage;

void led_VisualInitDefaults(const u32 window)
{
	const struct ds_Window *sys_win = ds_WindowAddress(window);
	Vec4Set(g_visual->unit_grid_color, 0.8f, 0.8f, 0.8f, 0.8f);
	g_visual->unit_grid_equidistance = 1.0f;
	g_visual->unit_grid_lines_per_axis = 100;
	g_visual->unit_grid_draw = 1;

        Vec4Set(g_visual->border_color, 0.4f, 0.4f, 0.7f, 1.0f);
	Vec4Set(g_visual->background_color, 0.0625f, 0.0625f, 0.0625f, 1.0f);
        Vec4Set(g_visual->background_highlight_color, 0.125f, 0.125f, 0.125f, 1.0f);
        Vec4Set(g_visual->background_invalid_color, 0.6f, 0.3f, 0.3, 1.0f);
        Vec4Set(g_visual->text_color, 0.7f, 0.7f, 0.9f, 1.0f);

	g_visual->border_size = 2;
	g_visual->edge_softness = 2.0f;
	g_visual->corner_radius = 3.0f;

	//TODO
	//g_visual->unit_r_handle = 

	g_visual->axes_draw = 1;
	//TODO
	//g_visual->axes_r_handle = 

	const vec3 cam_position = { 5.0f, 5.0f, 5.0f };
	const vec3 cam_direction = { -1.0f, -1.0f, -1.0f };
	const f32 fz_near = 0.0125f;
	const f32 fz_far =  512.0f;
	const f32 aspect_ratio = (f32) sys_win->size[0] / sys_win->size[1];
	const f32 fov_x = F32_PI / 2.0f;
	g_visual->cam = r_CameraInit(cam_position, cam_direction, fz_near, fz_far, aspect_ratio, fov_x);
}
