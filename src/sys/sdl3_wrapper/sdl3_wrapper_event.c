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

#include "sdl3_wrapper_local.h"
#include "sys_public.h"

u32 sdl3_wrapper_event_consume(struct system_event *event)
{
	u32 event_exists = 0; 

	SDL_Event ev;
	while (SDL_PollEvent(&ev))
	{
		event_exists = 1;
		event->native_handle = (u64) SDL_GetWindowFromEvent(&ev);
		event->ns_timestamp = ev.common.timestamp;
		switch (ev.type)
		{
			case SDL_EVENT_TEXT_INPUT:
			{
				event->type = SYSTEM_TEXT_INPUT;
				//event->utf32 = decode_utf8_null_terminated_buffered(ThreadAlloc256B(), 256 / 4, (u8 *) ev.text.text);
				event->utf8.buf = (u8 *) ev.text.text;
				event->utf8.len = 0;
				u64 offset = 0;
				while (Utf8ReadCodepoint(&offset, &event->utf8, offset))
				{
					event->utf8.len += 1;
				}
				event->utf8.size = event->utf8.len + 1;
			} break;

			case SDL_EVENT_WINDOW_MOUSE_ENTER:
			{
				event->type = SYSTEM_WINDOW_CURSOR_ENTER;
			} break;

			case SDL_EVENT_WINDOW_MOUSE_LEAVE:
			{
				event->type = SYSTEM_WINDOW_CURSOR_LEAVE;
			} break;

			case SDL_EVENT_WINDOW_FOCUS_GAINED:
			{
				event->type = SYSTEM_WINDOW_FOCUS_IN;
			} break;

			case SDL_EVENT_WINDOW_FOCUS_LOST:
			{
				event->type = SYSTEM_WINDOW_FOCUS_OUT;
			} break;

			case SDL_EVENT_WINDOW_MOVED:
			case SDL_EVENT_WINDOW_RESIZED:
			{
				event->type = SYSTEM_WINDOW_CONFIG;
			} break;

			case SDL_EVENT_MOUSE_MOTION:
			{
				event->type = SYSTEM_CURSOR_POSITION;
				Vec2Set(event->native_cursor_window_position, ev.motion.x, ev.motion.y);
				Vec2Set(event->native_cursor_window_delta,  ev.motion.xrel, ev.motion.yrel);
			} break;

			case SDL_EVENT_MOUSE_WHEEL:
			{
				event->type = SYSTEM_SCROLL;
				event->scroll.direction = (ev.wheel.y > 0.0f) ? MOUSE_SCROLL_UP : MOUSE_SCROLL_DOWN;
				event->scroll.count= (ev.wheel.integer_y > 0) ? ev.wheel.integer_y : -ev.wheel.integer_y;
			} break;

			case SDL_EVENT_MOUSE_BUTTON_UP:
			{
				event->type = SYSTEM_BUTTON_RELEASED;
				event->button = sdl3_wrapper_to_system_mouse_button(ev.button.button);
			} break;

			case SDL_EVENT_MOUSE_BUTTON_DOWN:
			{
				event->type = SYSTEM_BUTTON_PRESSED;
				event->button = sdl3_wrapper_to_system_mouse_button(ev.button.button);
			} break;

			case SDL_EVENT_KEY_UP:
			{
				event->type = SYSTEM_KEY_RELEASED;
				event->keycode = sdl3_wrapper_to_system_keycode(ev.key.key);
				event->scancode = sdl3_wrapper_to_system_scancode(ev.key.scancode);
			} break;

			case SDL_EVENT_KEY_DOWN:
			{
				event->type = SYSTEM_KEY_PRESSED;
				event->keycode = sdl3_wrapper_to_system_keycode(ev.key.key);
				event->scancode = sdl3_wrapper_to_system_scancode(ev.key.scancode);
			} break;

			case SDL_EVENT_WINDOW_CLOSE_REQUESTED:
			{
				event->type = SYSTEM_WINDOW_CLOSE;
			} break;

			default:
			{
				event_exists = 0;
			} break;
		}

		if (event_exists)
		{
			break;
		}
	}

	return event_exists;
}
