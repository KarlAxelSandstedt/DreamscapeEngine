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

#include "sdl3_wrapper_local.h"

u32 sdl3_KeyModifiers(void)
{
	const u32 sdl_modifiers = SDL_GetModState();
	return (KEY_MOD_LSHIFT * !!(sdl_modifiers & SDL_KMOD_LSHIFT))
		| (KEY_MOD_RSHIFT * !!(sdl_modifiers & SDL_KMOD_RSHIFT))
		| (KEY_MOD_LCTRL  * !!(sdl_modifiers & SDL_KMOD_LCTRL))
		| (KEY_MOD_RCTRL  * !!(sdl_modifiers & SDL_KMOD_RCTRL))
		| (KEY_MOD_LALT   * !!(sdl_modifiers & SDL_KMOD_LALT))
		| (KEY_MOD_RALT   * !!(sdl_modifiers & SDL_KMOD_RALT))
		| (KEY_MOD_LGUI   * !!(sdl_modifiers & SDL_KMOD_LGUI))
		| (KEY_MOD_RGUI   * !!(sdl_modifiers & SDL_KMOD_RGUI))
		| (KEY_MOD_NUM    * !!(sdl_modifiers & SDL_KMOD_NUM))
		| (KEY_MOD_CAPS   * !!(sdl_modifiers & SDL_KMOD_CAPS))
		| (KEY_MOD_ALTGR  * !!(sdl_modifiers & SDL_KMOD_MODE))
		| (KEY_MOD_SCROLL * !!(sdl_modifiers & SDL_KMOD_SCROLL));
}

enum mouseButton sdl3_DsMouseButton(const u8 mouse_button)
{
	enum mouseButton button = MOUSE_BUTTON_NONMAPPED;
	switch (mouse_button)
	{
		case SDL_BUTTON_LEFT: { button = MOUSE_BUTTON_LEFT; } break;
		case SDL_BUTTON_RIGHT: { button = MOUSE_BUTTON_RIGHT; } break;
		case SDL_BUTTON_MIDDLE: { button = MOUSE_BUTTON_SCROLL; } break;
	}

	return button;
}

enum dsKeycode sdl3_DsKeycode(const SDL_Keycode sdl_key)
{
	enum dsKeycode key = DS_NO_SYMBOL;

	if (SDLK_0 <= sdl_key && sdl_key <= SDLK_9)
	{
		key = DS_0 + (sdl_key - SDLK_0);
	}
	else if (SDLK_A <= sdl_key && sdl_key <= SDLK_Z)
	{
		key = DS_A + (sdl_key - SDLK_A);
	}
	else if (SDLK_F1 <= sdl_key && sdl_key <= SDLK_F12)
	{
		key = DS_F1 + (sdl_key - SDLK_F1);
	}
	else if (sdl_key == SDLK_LSHIFT) { key = DS_SHIFT; }
	else if (sdl_key == SDLK_SPACE) { key = DS_SPACE; }
	else if (sdl_key == SDLK_BACKSPACE) { key = DS_BACKSPACE; }
	else if (sdl_key == SDLK_ESCAPE) { key = DS_ESCAPE; }
	else if (sdl_key == SDLK_TAB) { key = DS_TAB; }
	else if (sdl_key == SDLK_RETURN) { key = DS_ENTER; }
	else if (sdl_key == SDLK_LCTRL) { key = DS_CTRL; }
	else if (sdl_key == SDLK_DELETE) { key = DS_DELETE; }
	else if (sdl_key == SDLK_HOME) { key = DS_HOME; }
	else if (sdl_key == SDLK_UP) { key = DS_UP; }
	else if (sdl_key == SDLK_DOWN) { key = DS_DOWN; }
	else if (sdl_key == SDLK_LEFT) { key = DS_LEFT; }
	else if (sdl_key == SDLK_RIGHT) { key = DS_RIGHT; }
	else if (sdl_key == SDLK_PLUS) { key = DS_PLUS; }
	else if (sdl_key == SDLK_MINUS) { key = DS_MINUS; }
	else if (sdl_key == SDL_SCANCODE_END) { key = DS_END; }

	return key;
}

enum dsKeycode sdl3_DsScancode(const SDL_Scancode sdl_key)
{
	enum dsKeycode key = DS_NO_SYMBOL;

	if (SDL_SCANCODE_0 <= sdl_key && sdl_key <= SDL_SCANCODE_9)
	{
		key = DS_0 + (sdl_key - SDL_SCANCODE_0);
	}
	else if (SDL_SCANCODE_A <= sdl_key && sdl_key <= SDL_SCANCODE_Z)
	{
		key = DS_A + (sdl_key - SDL_SCANCODE_A);
	}
	else if (SDL_SCANCODE_F1 <= sdl_key && sdl_key <= SDL_SCANCODE_F12)
	{
		key = DS_F1 + (sdl_key - SDL_SCANCODE_F1);
	}
	else if (sdl_key == SDL_SCANCODE_LSHIFT) { key = DS_SHIFT; }
	else if (sdl_key == SDL_SCANCODE_SPACE) { key = DS_SPACE; }
	else if (sdl_key == SDL_SCANCODE_BACKSPACE) { key = DS_BACKSPACE; }
	else if (sdl_key == SDL_SCANCODE_ESCAPE) { key = DS_ESCAPE; }
	else if (sdl_key == SDL_SCANCODE_TAB) { key = DS_TAB; }
	else if (sdl_key == SDL_SCANCODE_RETURN) { key = DS_ENTER; }
	else if (sdl_key == SDL_SCANCODE_LCTRL) { key = DS_CTRL; }
	else if (sdl_key == SDL_SCANCODE_DELETE) { key = DS_DELETE; }
	else if (sdl_key == SDL_SCANCODE_HOME) { key = DS_HOME; }
	else if (sdl_key == SDL_SCANCODE_UP) { key = DS_UP; }
	else if (sdl_key == SDL_SCANCODE_DOWN) { key = DS_DOWN; }
	else if (sdl_key == SDL_SCANCODE_LEFT) { key = DS_LEFT; }
	else if (sdl_key == SDL_SCANCODE_RIGHT) { key = DS_RIGHT; }
	else if (sdl_key == SDL_SCANCODE_END) { key = DS_END; }
	else if (sdl_key == SDL_SCANCODE_KP_PLUS) { key = DS_PLUS; }
	else if (sdl_key == SDL_SCANCODE_KP_MINUS) { key = DS_MINUS; }

	return key;
}
