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

#ifndef __SYS_INFO_H__
#define __SYS_INFO_H__

#include <stdio.h>
#include "sys_common.h"
#include "ds_common.h"
#include "memory.h"
#include "ds_math.h"
#include "bitVector.h"
#include "ds_string.h"
#include "hash_map.h"
#include "hierarchy_index.h"
#include "ds_vector.h"

#if __DS_PLATFORM__ == __DS_LINUX__
#include "linux_public.h"
#elif __DS_PLATFORM__ == __DS_WIN64__
#include "win_public.h"
#elif __DS_PLATFORM__ == __DS_WEB__
#include "wasm_public.h"
#endif

/************************************************************************/
/* 			Graphics abstraction layer 			*/
/************************************************************************/

/*
 * system window coordinate system:
 *
 *  (0,Y) ------------------------- (X,Y)
 *    |				      |
 *    |				      |
 *    |				      |
 *    |				      |
 *    |				      |
 *  (0,0) ------------------------- (X,0)
 *
 *  Since we are using a right handed coordinate system to describe the world, and the camera
 *  looks down the +Z axis, an increase in X or Y in the screen space means an "increase" from
 *  the camera's perspective as well. We must ensure that the underlying platform events that
 *  contains window coordinates are translated into this format.
 *
 *		A (Y)
 *		|
 *		|	(X)
 *		|------->
 *	       /
 *            / 
 *	     V (Z)
 */
#include "ui_public.h"
#include "array_list.h"
#include "cmd.h"

struct ui;
struct r_scene;
struct nativeWindow;

/* init function pointers */
void 	system_graphics_init(void);
/* free any graphics resources */
void 	system_graphics_destroy(void);

extern struct hi *	g_window_hierarchy;
extern u32 			g_process_root_window;
extern u32 			g_window;

struct system_window
{
	struct hiNode 	header;			/* DO NOT MOVE */
	struct nativeWindow *		native;			/* native graphics handle */
	struct ui *			ui;			/* local ui */
	struct cmd_queue *		cmd_queue;		/* local command queue */
	struct cmd_console *		cmd_console;		/* console */
	struct r_scene *		r_scene;
	struct arena 			mem_persistent;		/* peristent 1MB arena */

	u32				tagged_for_destruction; /* If tagged, free on next start of frame */
	u32				text_input_mode;	/* If on, window is receiving text input events */ 
	vec2u32				position;
	vec2u32				size;

	u32				gl_state;
};

/* alloc system_window resources, if no gl context exist, allocate context as well. */
u32 			system_window_alloc(const char *title, const vec2u32 position, const vec2u32 size, const u32 parent);
/* alloc system_window resources AND set window to global root process window, if no gl context exist, allocate context as well. */
u32 			system_process_root_window_alloc(const char *title, const vec2u32 position, const vec2u32 size);
/* handle sys_win->ui events */
void 			DsWindowEventHandler(struct system_window *sys_win);
/* Tag sub-hierachy of root (including root itself) for destruction on next frame. */
void system_window_tag_sub_hierarchy_for_destruction(const u32 root);
/* free system windows tagged for destruction */
void			system_free_tagged_windows(void);
/* get system window index  */
u32			system_window_index(const struct system_window *sys_win);
/* get system window address  */
struct system_window *	system_window_address(const u32 index);
/* Return system window containing the given native window handle, or an empty allocation slot if no window found */
struct slot		system_window_lookup(const u64 native_handle);
/* Set window to current (global pointers to window, ui and cmd_queue is set) */
void			system_window_set_global(const u32 window);
/* enable text input mode for current window */
void 			system_window_text_input_mode_enable(void);
/* disable text input mode for current window */
void 			system_window_text_input_mode_disable(void);
/* Set system window to be the current gl context */
void			system_window_set_current_gl_context(const u32 window);
/* opengl swap buffers */
void 			system_window_swap_gl_buffers(const u32 window);
/* update system_window configuration */
void			system_window_config_update(const u32 window);
/* get system_window size */
void			system_window_size(vec2u32 size, const u32 window);


/* set rectangle within window that cursor is restricted to */
void 			cursor_set_rect(struct system_window *sys_win, const vec2 sys_position, const vec2 size);
/* release any rectangle restriction */
void 			cursor_unset_rect(struct system_window *sys_win);
/* lock cursor to rectangle */
u32  			cursor_is_locked(struct system_window *sys_win);
/* return 1 on success, 0 otherwise */
u32 			cursor_lock(struct system_window *sys_win);
/* return 1 on success, 0 otherwise */
u32 			cursor_unlock(struct system_window *sys_win);
/* return 1 on visible, 0 on hidden  */
u32 			cursor_is_visible(struct system_window *sys_win);
/* show cursor  */
void 			cursor_show(struct system_window *sys_win);
/* hide cursor  */
void 			cursor_hide(struct system_window *sys_win);

#endif
