/*
==========================================================================
    Copyright (C) 2025 Axel Sandstedt 

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

static void led_project_menu_window_interaction(struct led *led)
{
}

void led_project_menu_main(struct led *led)
{
	struct led_project_menu *menu = &led->project_menu;
	struct system_window *sys_win;
	if (menu->window == HI_NULL_INDEX)
	{
		menu->window = system_window_alloc("Project Menu", Vec2u32Inline(0,0), Vec2u32Inline(400, 400), g_process_root_window);
		menu->popup_new_project = ui_popup_null();

		struct system_window *sys_win = system_window_address(menu->window);
		menu->input_line_new_project = ui_text_input_alloc(&sys_win->mem_persistent, 32);
		menu->utf8_new_project = Utf8Alloc(&sys_win->mem_persistent, 32*sizeof(u32));
	}

	sys_win = system_window_address(menu->window);
	if (menu->window != HI_NULL_INDEX && sys_win->tagged_for_destruction)
	{
		menu->window = HI_NULL_INDEX;
		menu->input_line_new_project = ui_text_input_empty();
	}

	if (menu->projects_folder_refresh || !menu->projects_folder_allocated)
	{
		enum fs_error ret = directory_navigator_enter_and_alias_path(&menu->dir_nav, led->root_folder.path);
		if (ret == FS_SUCCESS)
		{
			menu->projects_folder_allocated = 1;
			menu->projects_folder_refresh = 0;
		}
		else if (ret == FS_PATH_INVALID)
		{
			Log(T_SYSTEM, S_ERROR, "Could not enter folder %k, bad path.", &led->root_folder.path);
		}
		else
		{
			Log(T_SYSTEM, S_ERROR, "Unhandled error when entering folder %k.", &led->root_folder.path);
		}
	}

	if (led->project.initialized)
	{
		struct system_window *win = system_window_address(menu->window);
		system_window_tag_sub_hierarchy_for_destruction(menu->window);
		menu->window = HI_NULL_INDEX;
		menu->input_line_new_project = ui_text_input_empty();
	}
}


void led_main(struct led *led, const u64 ns_delta)
{
	led->ns_delta = ns_delta * led->ns_delta_modifier;
	led->ns += led->ns_delta;
	ArenaFlush(&led->frame);

	if (!led->project.initialized)
	{
		//led_project_menu_main(led);		
	}

	/*
	 * (1) process user input => (2) build ui => (3) led_core(): process systems in order
	 */
	led_core(led);
}
