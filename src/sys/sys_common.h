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

#ifndef __SYSTEM_DEFINITION_H__
#define __SYSTEM_DEFINITION_H__

#include "ds_common.h"
#include "ds_string.h"
#include "ds_vector.h"
#include "list.h"

/* MACROS, GLOBALS and SYSTEM-LEVEL STRUCTS */
#if __DS_PLATFORM__ == __DS_LINUX__
#include "linux_public.h"
#elif __DS_PLATFORM__ == __DS_WEB__
#include "wasm_public.h"
#elif __DS_PLATFORM__ == __DS_WIN64__
#include "win_public.h"
#endif


/************************************************************************/
/* 			system mouse/keyboard handling 			*/
/************************************************************************/

#define	KEY_MOD_NONE	0
#define	KEY_MOD_LSHIFT	(1 << 0)
#define	KEY_MOD_RSHIFT	(1 << 1)
#define	KEY_MOD_LCTRL	(1 << 2)
#define	KEY_MOD_RCTRL	(1 << 3)
#define	KEY_MOD_LALT	(1 << 4)
#define	KEY_MOD_RALT	(1 << 5)	/* Alt Gr? 		*/
#define	KEY_MOD_LGUI	(1 << 6)	/* Left windows-key?	*/
#define	KEY_MOD_RGUI	(1 << 7)	/* Right windows-key? 	*/
#define KEY_MOD_NUM	(1 << 8)	/* Num-lock  		*/ 
#define KEY_MOD_CAPS	(1 << 9)	
#define KEY_MOD_ALTGR	(1 << 10)	
#define KEY_MOD_SCROLL	(1 << 11)	/* Scroll-lock */

#define KEY_MOD_SHIFT	(KEY_MOD_LSHIFT | KEY_MOD_RSHIFT)
#define KEY_MOD_CTRL	(KEY_MOD_LCTRL | KEY_MOD_RCTRL)
#define KEY_MOD_ALT	(KEY_MOD_LALT | KEY_MOD_RALT)
#define KEY_MOD_GUI	(KEY_MOD_LGUI | KEY_MOD_RGUI)

enum ds_keycode
{
	DS_SHIFT,
	DS_CTRL,
	DS_SPACE,
	DS_BACKSPACE,
	DS_ESCAPE,
	DS_ENTER,
	DS_F1,
	DS_F2,
	DS_F3,
	DS_F4,
	DS_F5,
	DS_F6,
	DS_F7,
	DS_F8,
	DS_F9,
	DS_F10,
	DS_F11,
	DS_F12,
	DS_TAB,
	DS_UP,
	DS_DOWN,
	DS_LEFT,
	DS_RIGHT,
	DS_DELETE,
	DS_PLUS,
	DS_MINUS,
	DS_HOME,
	DS_END,
	DS_0,
	DS_1,
	DS_2,
	DS_3,
	DS_4,
	DS_5,
	DS_6,
	DS_7,
	DS_8,
	DS_9,
	DS_A,
	DS_B,  
	DS_C, 
	DS_D, 
	DS_E, 
	DS_F, 
	DS_G, 
	DS_H, 
	DS_I, 
	DS_J, 
	DS_K, 
	DS_L, 
	DS_M, 
	DS_N, 
	DS_O, 
	DS_P, 
	DS_Q, 
	DS_R, 
	DS_S, 
	DS_T, 
	DS_U, 
	DS_V, 
	DS_W, 
	DS_X, 
	DS_Y, 
	DS_Z, 
	DS_NO_SYMBOL,
	DS_KEY_COUNT
};

enum mouse_button
{
	MOUSE_BUTTON_LEFT,
	MOUSE_BUTTON_RIGHT,
	MOUSE_BUTTON_SCROLL,
	MOUSE_BUTTON_NONMAPPED,
	MOUSE_BUTTON_COUNT
};

enum mouse_scroll
{
	MOUSE_SCROLL_UP,
	MOUSE_SCROLL_DOWN,
	MOUSE_SCROLL_COUNT
};

/************************************************************************/
/* 			      system events 				*/
/************************************************************************/

enum system_event_type 
{
	SYSTEM_SCROLL,
	SYSTEM_KEY_PRESSED,
	SYSTEM_KEY_RELEASED,
	SYSTEM_BUTTON_PRESSED,
	SYSTEM_BUTTON_RELEASED,
	SYSTEM_CURSOR_POSITION,
	SYSTEM_TEXT_INPUT,
	SYSTEM_WINDOW_CLOSE,
	SYSTEM_WINDOW_CURSOR_ENTER,
	SYSTEM_WINDOW_CURSOR_LEAVE,
	SYSTEM_WINDOW_FOCUS_IN,
	SYSTEM_WINDOW_FOCUS_OUT,
	SYSTEM_WINDOW_EXPOSE,
	SYSTEM_WINDOW_CONFIG,
	SYSTEM_WINDOW_MINIMIZE,
	SYSTEM_NO_EVENT,
};

struct system_event 
{
	POOL_SLOT_STATE;
	DLL_SLOT_STATE;
	u64			native_handle;	/* window handle 			*/
	u64			ns_timestamp;	/* external event time; NOT OUR CLOCK 	*/
	enum system_event_type 	type;

	/* Input key */
	enum ds_keycode keycode; 	
	enum ds_keycode scancode;

	/* Input Mouse button */
	enum mouse_button button; 

	/* mouse scrolling */
	struct 
	{
		enum mouse_scroll direction;
		u32 count;
	} scroll;

	vec2 native_cursor_window_position;	/* native window coordinate space cursor position */
	vec2 native_cursor_window_delta;	/* native window coordinate space cursor delta */

	utf8	utf8;
};

#endif
