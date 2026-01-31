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

#include "ds_platform.h"

const char *ds_button_string_map[] =
{
	"MOUSE_BUTTON_LEFT",
	"MOUSE_BUTTON_RIGHT",
	"MOUSE_BUTTON_SCROLL",
	"MOUSE_BUTTON_NONMAPPED",
	"MOUSE_BUTTON_COUNT",
};

const char *ds_keycode_string_map[] = 
{
	"DS_SHIFT",
	"DS_CTRL",
	"DS_SPACE",
	"DS_BACKSPACE",
	"DS_ESCAPE",
	"DS_ENTER",
	"DS_PLUS",
	"DS_MINUS",
	"DS_F1",
	"DS_F2",
	"DS_F3",
	"DS_F4",
	"DS_F5",
	"DS_F6",
	"DS_F7",
	"DS_F8",
	"DS_F9",
	"DS_F10",
	"DS_F11",
	"DS_F12",
	"DS_TAB",
	"DS_UP",
	"DS_DOWN",
	"DS_LEFT",
	"DS_RIGHT",
	"DS_DELETE",
	"DS_HOME",
	"DS_END",
	"DS_0",
	"DS_1",
	"DS_2",
	"DS_3",
	"DS_4",
	"DS_5",
	"DS_6",
	"DS_7",
	"DS_8",
	"DS_9",
	"DS_A",
	"DS_B", 
	"DS_C", 
	"DS_D", 
	"DS_E", 
	"DS_F", 
	"DS_G", 
	"DS_H", 
	"DS_I", 
	"DS_J", 
	"DS_K", 
	"DS_L", 
	"DS_M", 
	"DS_N", 
	"DS_O", 
	"DS_P", 
	"DS_Q", 
	"DS_R", 
	"DS_S", 
	"DS_T", 
	"DS_U", 
	"DS_V", 
	"DS_W", 
	"DS_X", 
	"DS_Y", 
	"DS_Z", 
	"DS_NO_SYMBOL",
	"DS_KEY_COUN"
};

const char *CstrDsKeycode(const enum dsKeycode key)
{
	return ds_keycode_string_map[key];
}

const char *CstrButton(const enum mouseButton button)
{
	return ds_button_string_map[button];
}
