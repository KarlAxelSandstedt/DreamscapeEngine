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

#ifndef __SYSTEM_LOCAL_H__
#define __SYSTEM_LOCAL_H__

#include "ds_platform.h"

#if __GAPI__ == __DS_SDL3__
#include "sdl3_wrapper_public.h"
#endif

/************************************************************************/
/* 				System Graphics 			*/
/************************************************************************/

/*TODO: Transform native screen position into our system coordinate system */
extern void 	(*ScreenPositionNativeToEngine)(vec2 sys_pos, struct nativeWindow *native, const vec2 nat_pos);
/*TODO: Transform system screen position into native screen position */
extern void 	(*ScreenPositionEngineToNative)(vec2 nat_pos, struct nativeWindow *native, const vec2 sys_pos);
/* Transform native window position into our system coordinate system, return 1 if position is inside window, 0 otherwise */
extern void 	(*WindowPositionNativeToEngine)(vec2 sys_pos, struct nativeWindow *native, const vec2 nat_pos);
/* Transform system window position into native coordinate system return 1 if position is inside window, 0 otherwise */
extern void 	(*WindowPositionEngineToNative)(vec2 nat_pos, struct nativeWindow *native, const vec2 sys_pos);

/* setup system window */
extern struct nativeWindow *	(*NativeWindowCreate)(struct arena *mem, const char *title, const vec2u32 position, const vec2u32 size);
/* destroy system window */
extern void 			(*NativeWindowDestroy)(struct nativeWindow *native);
/* Return the native window handle of the system window */
extern u64			(*NativeWindowGetNativeHandle)(const struct nativeWindow *native);
/* set global gl context to work on window */
extern void 			(*NativeWindowGlSetCurrent)(struct nativeWindow *native);
/* opengl swap window */	
extern void 			(*NativeWindowGlSwapBuffers)(struct nativeWindow *native);
/* set config variables of native window */
extern void 			(*NativeWindowConfigUpdate)(vec2u32 position, vec2u32 size, struct nativeWindow *native);
/* set window fullscreen */
extern void 			(*NativeWindowFullscreen)(struct nativeWindow *native);
/* set window windowed */
extern void 			(*NativeWindowWindowed)(struct nativeWindow *native);
/* set window border */ 
extern void 			(*NativeWindowBordered)(struct nativeWindow *native);
/* set window borderless */ 
extern void 			(*NativeWindowBorderless)(struct nativeWindow *native);
/* return true if window is fullscreen  */ 
extern u32			(*NativeWindowFullscreenCheck)(const struct nativeWindow *native);
/* return true if window is bordered  */ 
extern u32			(*NativeWindowBorderedCheck)(const struct nativeWindow *native);

/* show cursor  */
extern void 			(*NativeCursorShow)(struct nativeWindow *native);
/* hide cursor  */
extern void 			(*NativeCursorHide)(struct nativeWindow *native);
/* return 1 if cursor is hidden, 0 otherwise */
extern u32  			(*NativeCursorVisibleCheck)(struct nativeWindow *native);
/* return 1 if cursor is locked, 0 otherwise */
extern u32  			(*NativeCursorLockedCheck)(struct nativeWindow *native);
/* return 1 on success, 0 otherwise */
extern u32 			(*NativeCursorLock)(struct nativeWindow *native);
/* return 1 on success, 0 otherwise */
extern u32 			(*NativeCursorUnlock)(struct nativeWindow *native);
/* set rectangle within window that cursor is restricted to */
extern void 			(*NativeCursorSetRectangle)(struct nativeWindow *sys_win, const vec2 nat_position, const vec2 size);
/* release any rectangle restriction */
extern void 			(*NativeCursorUnsetRectangle)(struct nativeWindow *native);



/************************************************************************/
/* 				System Events 				*/
/************************************************************************/

/* If native event exist, consume event into a system event and return 1. otherwise return 0 */
extern u32 	(*EventConsume)(struct dsEvent *event);

/************************************************************************/
/* 			system mouse/keyboard handling 			*/
/************************************************************************/

/* Enable text input system events */
extern u32 	(*EnterTextInputMode)(struct nativeWindow *native);
/* Disable text input system events */
extern u32 	(*ExitTextInputMode)(struct nativeWindow *native);

#endif
