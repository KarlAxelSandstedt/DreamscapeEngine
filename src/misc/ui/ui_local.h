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

#ifndef __UI_LOCAL_H__
#define __UI_LOCAL_H__

#include "ds_ui.h"
#include "cmd.h"

/* UI CMDS */
void 	ui_TimelineDrag(void);
void	ui_TextInputModeEnable(void);
void	ui_TextInputModeDisable(void);
void 	ui_TextInputFlush(void);
void 	ui_TextOp(void);

void 		ui_PopupBuild(void);
extern u32	cmd_ui_popup_build;

/* internal */
struct ui_TextInput *text_edit_stub_ptr(void);

#endif
