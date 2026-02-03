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

#ifndef __DS_GRAPHICS_H__
#define __DS_GRAPHICS_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_base.h"
#include "ds_string.h"
#include "hash_map.h"
#include "hierarchy_index.h"
#include "cmd.h"

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

struct r_Scene;
struct nativeWindow;

/* init function pointers */
void 	ds_GraphicsApiInit(void);
/* free any graphics resources */
void 	ds_GraphicsApiShutdown(void);

extern struct hi *	g_window_hierarchy;
extern u32 		g_process_root_window;
extern u32 		g_window;

struct ds_Window
{
	HI_SLOT_STATE;
	struct nativeWindow *	native;			/* native graphics handle */
	struct ui *		ui;			/* local ui */
	struct cmdQueue 	cmd_queue;		/* local command queue */
	struct ui_CmdConsole *	cmd_console;		/* console */
	struct r_Scene *	r_scene;
	struct arena 		mem_persistent;		/* peristent 1MB arena */

	u32			tagged_for_destruction; /* If tagged, free on next start of frame */
	u32			text_input_mode;	/* If on, window is receiving text input events */ 
	vec2u32			position;
	vec2u32			size;

	u32			gl_state;
};

/* alloc system_window resources, if no gl context exist, allocate context as well. */
u32 			ds_WindowAlloc(const char *title, const vec2u32 position, const vec2u32 size, const u32 parent);
/* alloc system_window resources AND set window to global root process window, if no gl context exist, allocate context as well. */
u32 			ds_RootWindowAlloc(const char *title, const vec2u32 position, const vec2u32 size);
/* handle sys_win->ui events */
void 			ds_WindowEventHandler(struct ds_Window *sys_win);
/* Tag sub-hierachy of root (including root itself) for destruction on next frame. */
void 			ds_WindowTagSubHierarchyForDestruction(const u32 root);
/* free system windows tagged for destruction */
void			ds_DeallocTaggedWindows(void);
/* get system window index  */
u32			ds_WindowIndex(const struct ds_Window *sys_win);
/* get system window address  */
struct ds_Window *	ds_WindowAddress(const u32 index);
/* Return system window containing the given native window handle, or an empty allocation slot if no window found */
struct slot		ds_WindowLookup(const u64 native_handle);
/* Set window to current (global pointers to window, ui and cmd_queue is set) */
void			ds_WindowSetGlobal(const u32 window);
/* enable text input mode for current window */
void 			ds_WindowTextInputModeEnable(void);
/* disable text input mode for current window */
void 			ds_WindowTextInputModeDisable(void);
/* Set system window to be the current gl context */
void			ds_WindowSetCurrentGlContext(const u32 window);
/* opengl swap buffers */
void 			ds_WindowSwapGlBuffers(const u32 window);
/* update system_window configuration */
void			ds_WindowConfigUpdate(const u32 window);
/* get system_window size */
void			ds_WindowSize(vec2u32 size, const u32 window);


/* set rectangle within window that cursor is restricted to */
void 			ds_CursorSetRectangle(struct ds_Window *sys_win, const vec2 sys_position, const vec2 size);
/* release any rectangle restriction */
void 			ds_CursorUnsetRectangle(struct ds_Window *sys_win);
/* return 1 if cursor is locked, 0 otherwise */
u32  			ds_CursorLockedCheck(struct ds_Window *sys_win);
/* return 1 on success, 0 otherwise */
u32 			ds_CursorLock(struct ds_Window *sys_win);
/* return 1 on success, 0 otherwise */
u32 			ds_CursorUnlock(struct ds_Window *sys_win);
/* return 1 if cursor is visible, 0 otherwise */
u32 			ds_CursorVisibleCheck(struct ds_Window *sys_win);
/* show cursor  */
void 			ds_CursorShow(struct ds_Window *sys_win);
/* hide cursor  */
void 			ds_CursorHide(struct ds_Window *sys_win);

#ifdef __cplusplus
} 
#endif

#endif
