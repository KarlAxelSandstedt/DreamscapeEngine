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

#ifndef __SYSTEM_LOCAL_H__
#define __SYSTEM_LOCAL_H__

#include "sys_public.h"
#include "Log.h"

#if __GAPI__ == __DS_SDL3__
#include "sdl3_wrapper_public.h"
#endif

/************************************************************************/
/* 				System Graphics 			*/
/************************************************************************/

/*TODO: Transform native screen position into our system coordinate system */
extern void 	(*screen_position_native_to_system)(vec2 sys_pos, struct native_window *native, const vec2 nat_pos);
/*TODO: Transform system screen position into native screen position */
extern void 	(*screen_position_system_to_native)(vec2 nat_pos, struct native_window *native, const vec2 sys_pos);
/* Transform native window position into our system coordinate system, return 1 if position is inside window, 0 otherwise */
extern void 	(*window_position_native_to_system)(vec2 sys_pos, struct native_window *native, const vec2 nat_pos);
/* Transform system window position into native coordinate system return 1 if position is inside window, 0 otherwise */
extern void 	(*window_position_system_to_native)(vec2 nat_pos, struct native_window *native, const vec2 sys_pos);

/* setup system window */
extern struct native_window *	(*native_window_create)(struct arena *mem, const char *title, const vec2u32 position, const vec2u32 size);
/* destroy system window */
extern void 			(*native_window_destroy)(struct native_window *native);
/* Return the native window handle of the system window */
extern u64			(*native_window_get_native_handle)(const struct native_window *native);
/* set global gl context to work on window */
extern void 			(*native_window_gl_set_current)(struct native_window *native);
/* opengl swap window */	
extern void 			(*native_window_gl_swap_buffers)(struct native_window *native);
/* set config variables of native window */
extern void 			(*native_window_config_update)(vec2u32 position, vec2u32 size, struct native_window *native);
/* set window fullscreen */
extern void 			(*native_window_fullscreen)(struct native_window *native);
/* set window windowed */
extern void 			(*native_window_windowed)(struct native_window *native);
/* set window border */ 
extern void 			(*native_window_bordered)(struct native_window *native);
/* set window borderless */ 
extern void 			(*native_window_borderless)(struct native_window *native);
/* return true if window is fullscreen  */ 
extern u32			(*native_window_is_fullscreen)(const struct native_window *native);
/* return true if window is bordered  */ 
extern u32			(*native_window_is_bordered)(const struct native_window *native);

/* show cursor  */
extern void 			(*native_cursor_show)(struct native_window *native);
/* hide cursor  */
extern void 			(*native_cursor_hide)(struct native_window *native);
/* return 1 if cursor is hidden, 0 otherwise */
extern u32  			(*native_cursor_is_visible)(struct native_window *native);
/* return 1 if cursor is locked, 0 otherwise */
extern u32  			(*native_cursor_is_locked)(struct native_window *native);
/* return 1 on success, 0 otherwise */
extern u32 			(*native_cursor_lock)(struct native_window *native);
/* return 1 on success, 0 otherwise */
extern u32 			(*native_cursor_unlock)(struct native_window *native);
/* set rectangle within window that cursor is restricted to */
extern void 			(*native_cursor_set_rect)(struct native_window *sys_win, const vec2 nat_position, const vec2 size);
/* release any rectangle restriction */
extern void 			(*native_cursor_unset_rect)(struct native_window *native);



/************************************************************************/
/* 				System Events 				*/
/************************************************************************/

/* If native event exist, consume event into a system event and return 1. otherwise return 0 */
extern u32 	(*system_event_consume)(struct system_event *event);

/************************************************************************/
/* 			system mouse/keyboard handling 			*/
/************************************************************************/

/* Enable text input system events */
extern u32 	(*system_enter_text_input_mode)(struct native_window *native);
/* Disable text input system events */
extern u32 	(*system_exit_text_input_mode)(struct native_window *native);

#endif
