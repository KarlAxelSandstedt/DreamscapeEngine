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

#ifndef __DS_SDL3_WRAPPER_LOCAL_H__
#define __DS_SDL3_WRAPPER_LOCAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "SDL3/SDL.h"

#undef LogWriteMessage

#include "ds_platform.h"
#include "sdl3_wrapper_public.h"

/* load every GL function of func through SDL_GL_GetProcAddress; a missing one is fatal */
void			sdl3_GlFunctionsInit(struct gl_Functions *func);
/*
 * If an SDL event the engine handles is pending, translate it into *event and return 1; skips the others; 0 when
 * none is left. A text input event's utf8 points into SDL's event storage: valid until the next call.
 */
u32 			sdl3_EventConsume(struct ds_Event *event);
/* the current KEY_MOD_* bits (SDL_GetModState) */
u32 			sdl3_KeyModifiers(void);
/* SDL mouse button to the engine's; MOUSE_BUTTON_NONMAPPED for buttons other than left, right and middle */
enum mouseButton	sdl3_DsMouseButton(const u8 mouse_button);
/* SDL keycode (layout dependent) to the engine's key; DS_NO_SYMBOL for keys the engine doesn't map */
enum dsKeycode		sdl3_DsKeycode(const SDL_Keycode sdl_key);
/* SDL scancode (physical key) to the engine's key; DS_NO_SYMBOL for keys the engine doesn't map */
enum dsKeycode 	sdl3_DsScancode(const SDL_Scancode sdl_key);

#ifdef __cplusplus
}
#endif

#endif
