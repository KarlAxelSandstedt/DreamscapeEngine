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

#include <string.h>

#include "sys_public.h"
#include "sys_local.h"

void system_window_event_handler(struct system_window *sys_win)
{
	struct system_event *event = NULL;
	for (u32 i = sys_win->ui->event_list.first; i != DLL_NULL; i = dll_Next(event))
	{
		event = PoolAddress(&sys_win->ui->event_pool, i);
		switch (event->keycode)
		{
			case DS_L:
			{
				(cursor_is_locked(sys_win))
					? cursor_unlock(sys_win)
					: cursor_lock(sys_win);
			} break;

			case DS_F10: 
			{
				(cursor_is_visible(sys_win))
				      ? cursor_hide(sys_win) 
				      : cursor_show(sys_win);
			} break;

			case DS_F11: 
			{
				(native_window_is_fullscreen(sys_win->native))
				      ? native_window_windowed(sys_win->native) 
				      : native_window_fullscreen(sys_win->native);
			} break;

			case DS_F12:
			{
				(native_window_is_bordered(sys_win->native))
				      ? native_window_borderless(sys_win->native) 
				      : native_window_bordered(sys_win->native);
			} break;

			case DS_C: 
			{
				system_window_tag_sub_hierarchy_for_destruction(system_window_index(sys_win));
			} break;	

			default:
			{
				fprintf(stderr, "Unhandled Press: %s\n", ds_keycode_to_string(event->keycode));
			} break;
		}
	}
}

void system_process_events(void)
{
	const u32 key_modifiers = system_key_modifiers();
	struct system_event event;
	while (system_event_consume(&event)) 
	{
		struct slot slot = system_window_lookup(event.native_handle);
		struct system_window *sys_win = slot.address;
		if (!sys_win)
		{
			continue;
		}

		u32 text_edit_event = 0;
		switch (event.type)
		{
			case SYSTEM_TEXT_INPUT:
			{
				text_edit_event = 1;
				sys_win->cmd_queue->regs[0].u32 = 0;
				sys_win->cmd_queue->regs[1].u32 = key_modifiers;
				sys_win->cmd_queue->regs[2].utf8 = Utf8Copy(sys_win->ui->mem_frame, event.utf8);
			} break;

			case SYSTEM_SCROLL:
			{
				if (event.scroll.direction == MOUSE_SCROLL_UP)
				{
					sys_win->ui->inter.scroll_up_count += event.scroll.count;
					fprintf(stderr, "Scroll up\n");
				}
				else
				{
					sys_win->ui->inter.scroll_down_count += event.scroll.count;
					fprintf(stderr, "Scroll down\n");
				}
			} break;

			case SYSTEM_BUTTON_PRESSED:
			{
				if (event.button < MOUSE_BUTTON_NONMAPPED)
				{
					const u64 ns_delta = event.ns_timestamp - sys_win->ui->inter.ns_button_time_since_last_pressed[event.button];
					sys_win->ui->inter.ns_button_time_since_last_pressed[event.button] = event.ns_timestamp;
					if (ns_delta < sys_win->ui->inter.ns_double_click) 
					{
						sys_win->ui->inter.button_double_clicked[event.button] = 1;
					}
					sys_win->ui->inter.button_clicked[event.button] = 1;
					sys_win->ui->inter.button_pressed[event.button] = 1;
					//fprintf(stderr, "Button Pressed: %s\n", ds_button_to_string(event.button));
				}
			} break;

			case SYSTEM_BUTTON_RELEASED:
			{
				if (event.button < MOUSE_BUTTON_NONMAPPED)
				{
					sys_win->ui->inter.button_pressed[event.button] = 0;
					sys_win->ui->inter.button_released[event.button] = 1;
					//fprintf(stderr, "%p, Button Released: %s\n", sys_win, ds_button_to_string(event.button));
				}
			} break;

			case SYSTEM_KEY_PRESSED:
			{
				//TODO remove 
				sys_win->ui->inter.key_clicked[event.keycode] = 1;
				sys_win->ui->inter.key_pressed[event.keycode] = 1;

				if (sys_win->ui->inter.text_edit_mode)
				{
					switch (event.scancode)
					{
						case DS_RIGHT:
						{
							text_edit_event = 1;
							sys_win->cmd_queue->regs[0].u32 = DS_RIGHT;
							sys_win->cmd_queue->regs[1].u32 = key_modifiers;
							sys_win->cmd_queue->regs[2].utf8 = Utf8Empty();
						} break;
					
						case DS_LEFT:
						{
							text_edit_event = 1;
							sys_win->cmd_queue->regs[0].u32 = DS_LEFT;
							sys_win->cmd_queue->regs[1].u32 = key_modifiers;
							sys_win->cmd_queue->regs[2].utf8 = Utf8Empty();
						} break;

						case DS_DELETE:
						{
							text_edit_event = 1;
							sys_win->cmd_queue->regs[0].u32 = DS_DELETE;
							sys_win->cmd_queue->regs[1].u32 = key_modifiers;
							sys_win->cmd_queue->regs[2].utf8 = Utf8Empty();
						} break;

						case DS_HOME:
						{
							text_edit_event = 1;
							sys_win->cmd_queue->regs[0].u32 = DS_HOME;
							sys_win->cmd_queue->regs[1].u32 = key_modifiers;
							sys_win->cmd_queue->regs[2].utf8 = Utf8Empty();
						} break;

						case DS_END:
						{
							text_edit_event = 1;
							sys_win->cmd_queue->regs[0].u32 = DS_END;
							sys_win->cmd_queue->regs[1].u32 = key_modifiers;
							sys_win->cmd_queue->regs[2].utf8 = Utf8Empty();
						} break;

						case DS_BACKSPACE:
						{
							text_edit_event = 1;
							sys_win->cmd_queue->regs[0].u32 = DS_BACKSPACE;
							sys_win->cmd_queue->regs[1].u32 = key_modifiers;
							sys_win->cmd_queue->regs[2].utf8 = Utf8Empty();
						} break;

						case DS_X:
						{
							if (key_modifiers & KEY_MOD_CTRL)
							{
								text_edit_event = 1;
								sys_win->cmd_queue->regs[0].u32 = DS_X;
								sys_win->cmd_queue->regs[1].u32 = key_modifiers;
								sys_win->cmd_queue->regs[2].utf8 = Utf8Empty();
							}
						} break;

						case DS_C:
						{
							if (key_modifiers & KEY_MOD_CTRL)
							{
								text_edit_event = 1;
								sys_win->cmd_queue->regs[0].u32 = DS_C;
								sys_win->cmd_queue->regs[1].u32 = key_modifiers;
								sys_win->cmd_queue->regs[2].utf8 = Utf8Empty();
							}	
						} break;

						case DS_V:
						{
							if (key_modifiers & KEY_MOD_CTRL)
							{
								text_edit_event = 1;
								sys_win->cmd_queue->regs[0].u32 = 0;
								sys_win->cmd_queue->regs[1].u32 = key_modifiers;
								sys_win->cmd_queue->regs[2].utf8 = utf8_get_clipboard(sys_win->ui->mem_frame);
							}	
						} break;


						case DS_ESCAPE: 
						{ 
							cmd_queue_submit_f(sys_win->ui->mem_frame, sys_win->cmd_queue, "ui_text_input_mode_disable \"%k\"", &sys_win->ui->inter.text_edit_id);
						 } break;

						default: { } break;
					}		
				}
				else
				{
					struct slot slot = PoolAdd(&sys_win->ui->event_pool);
					dll_Append(&sys_win->ui->event_list, sys_win->ui->event_pool.buf, slot.index);
					struct system_event *new = slot.address;
					new->scancode = event.scancode;
					new->keycode = event.keycode;
					new->ns_timestamp = event.ns_timestamp;
					new->type = event.type;
				}
			} break;

			case SYSTEM_KEY_RELEASED:
			{
				sys_win->ui->inter.key_released[event.keycode] = 1;
				sys_win->ui->inter.key_pressed[event.keycode] = 0;
				if (sys_win->ui->inter.text_edit_mode)
				{
				}
				else
				{
					struct slot slot = PoolAdd(&sys_win->ui->event_pool);
					dll_Append(&sys_win->ui->event_list, sys_win->ui->event_pool.buf, slot.index);
					struct system_event *new = slot.address;
					new->scancode = event.scancode;
					new->keycode = event.keycode;
					new->ns_timestamp = event.ns_timestamp;
					new->type = event.type;
				}
			} break;

			case SYSTEM_CURSOR_POSITION:
			{
				vec2 cursor_delta;
				window_position_native_to_system(sys_win->ui->inter.cursor_position, sys_win->native, event.native_cursor_window_position);
				Vec2Set(cursor_delta, event.native_cursor_window_delta[0], -event.native_cursor_window_delta[1]);
				Vec2Translate(sys_win->ui->inter.cursor_delta, cursor_delta);
			} break;

			case SYSTEM_WINDOW_CLOSE:
			{
				system_window_tag_sub_hierarchy_for_destruction(slot.index);
			} break;

			case SYSTEM_WINDOW_CURSOR_ENTER:
			{
				//fprintf(stderr, "cursor_enter\n");
			} break;

			case SYSTEM_WINDOW_CURSOR_LEAVE:
			{
				//fprintf(stderr, "cursor_leave\n");
			} break;

			case SYSTEM_WINDOW_FOCUS_IN:
			{
				//fprintf(stderr, "window_focus_in\n");
			} break;

			case SYSTEM_WINDOW_FOCUS_OUT:
			{
				//fprintf(stderr, "window_focus_out\n");
			} break;

			//case SYSTEM_WINDOW_EXPOSE:
			//{
			//	//gc->win.screen_aspect_ratio = (f32) gc->win.screen_size[0] / gc->win.screen_size[1];
			//	//gc->win.aspect_ratio = (f32) gc->win.size[0] / gc->win.size[1];
			//	//gc->win.visible = 1;
			//	////fprintf(stderr, "visible\n");
			//	//Vec2u32Print("size:", gc->win.size);
			//	//struct r_cmd cmd_viewport = 
			//	//{
			//	//	.type = R_CMD_VIEWPORT,
			//	//};
			//	//r_cmd_add(&cmd_viewport);
			//} break;

			case SYSTEM_WINDOW_CONFIG:
			{
				system_window_config_update(array_list_index(g_window_hierarchy->list, sys_win));
			} break;

			//case SYSTEM_WINDOW_MINIMIZE:
			//{
			//	gc->win.visible = 0;
			//	fprintf(stderr, "minimized\n");
			//} break;

			case SYSTEM_NO_EVENT:
			{
				fprintf(stderr, "SYSTEM_NO_EVENT\n");
			} break;
		}

		if (text_edit_event)
		{
			cmd_queue_submit(sys_win->cmd_queue, cmd_ui_text_op);
		}
	}
}
