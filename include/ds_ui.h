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

#ifndef __UI_PUBLIC_H__
#define __UI_PUBLIC_H__

#ifdef __cplusplus
extern "C" { 
#endif

#include "ds_base.h"
#include "ds_math.h"
#include "ds_graphics.h"
#include "ds_asset.h"
#include "hierarchy_index.h"
#include "ds_vector.h"
#include "list.h"

#define UI_SCOPE(PUSH, POP)	for (i32 __i = ((PUSH), 0); __i < 1; ++__i, (POP))

#define TAB_SIZE	8

/* allocate ui resources */
void	ds_UiApiInit(void);

/*
ui_visual
=========
*/

/* Visual values push at the start of every ui_frame. */
struct ui_Visual
{
	/* default colors */
	vec4			background_color;
	vec4			border_color;
	vec4			gradient_color[BOX_CORNER_COUNT];
	vec4			sprite_color;

	/* default pixel padding */
	f32			pad;

	/* default effects */
	f32			edge_softness;
	f32			corner_radius;
	f32			border_size;

	/* default text visual values */
	enum fontId		font;
	enum alignment_x	text_alignment_x;
	enum alignment_y	text_alignment_y;
	f32			text_pad_x;
	f32			text_pad_y;
};

struct ui_Visual ui_VisualInit(const vec4 background_color
		, const vec4 border_color
		, const vec4 gradient_color[BOX_CORNER_COUNT]
		, const vec4 sprite_color
		, const f32 pad
		, const f32 edge_softness
		, const f32 corner_radius
		, const f32 border_size
		, const enum fontId font
		, const enum alignment_x text_alignment_x
		, const enum alignment_y text_alignment_y
		, const f32 text_pad_x
		, const f32 text_pad_y);

/********************************************************************************************************/
/*					       UI_BUILD							*/
/********************************************************************************************************/

/*
ui_TextInput
=============
ui_TextInput is a helper struct that contains text and text selection state. The ui core stores an ui_TextInput
alias when editing text. This alias either points to a ui_node's internal ui_TextInput (if the flag
UI_TEXT_EDIT_INTER_BUF_ON_FOCUS is set in the node) or an externally provided ui_TextInput. 
If an external structure is provided, make sure the structure cannot be freed or reallocated during the time
from when it was last built in a ui frame to the time of the next command queue execution.

*/

struct ui_TextInput
{
	u32	focused; 	
	u32	cursor;		/* cursor position */
	u32	mark;		/* marked position, selection area is the interval between cursor and mark */
	utf32	text;
};

struct ui_TextInput 	ui_TextInputEmpty(void);
struct ui_TextInput 	ui_TextInputBuffered(u32 buf[], const u32 len);
struct ui_TextInput 	ui_TextInputAlloc(struct arena *mem, const u32 max_len);

struct slot 		ui_TextInputF(struct ui_TextInput *input, const utf32 unfocused_text, const char *fmt, ...);
struct slot 		ui_TextInput(struct ui_TextInput *input, const utf32 unfocused_text, const utf8 id);

/*
ui_field
========
ui_field widgets are Input areas to display various types as strings. If interacted with, you can edit and return
a modification of the input value.
*/

f32	ui_FieldF32(const f32 value, const intv range, const utf8 id);
f32	ui_FieldF32F(const f32 value, const intv range, const char *fmt, ...);

u64	ui_FieldU64(const u64 value, const intvu64 range, const utf8 id);
u64	ui_FieldU64F(const u64 value, const intvu64 range, const char *fmt, ...);

i64	ui_FieldI64(const i64 value, const intvi64 range, const utf8 id);
i64	ui_FieldI64F(const i64 value, const intvi64 range, const char *fmt, ...);

utf8	ui_FieldUtf8(const utf8 id);
utf8	ui_FieldUtf8F(const char *fmt, ...);

/*
ui_list
=======
ui_list widgets are areas that displays selectable rows or columns of a specified size. 
Only rows within the visible range are actually constructed.
*/

enum ui_SelectionType
{
	UI_SELECTION_NONE,	/* Entries or non-selectable. 			  */
	UI_SELECTION_UNIQUE,	/* Only a single entry can be selected at a time. */
	UI_SELECTION_MULTI,	/* Any number of entries can be selected. 	  */
	UI_SELECTION_COUNT
};

struct ui_List
{
	u64			last_build_frame;		/* laft frame list was built */
	u64			last_selection_happened; 	/* Last frame a entry was selected */
	u32			last_selected;			/* last entry in current frame to be selected */
	enum ui_SelectionType	selection_type;			/* MULTI or UNIQUE; If unique, only one entry can
								   be selected  */

	u32			cache_count;			/* cached count from previous frame */
	u32			frame_count;			/* current count in current frame */
	struct ui_Node *	frame_node_address;
	u32			frame_node;

	intv 			visible;		/* visible pixel range in list 
							   : [0 : max(cache_count*entry_pixel_size, list_size)] */
	f32			max_pixel_size;		/* maximum pixel size of list		*/
	f32			entry_pixel_size;	/* entry pixel size in the layout axis 	*/
	enum axis_2		axis;			/* child layout axis 			*/
};

#define ui_list(list, fmt, ...)		UI_SCOPE(ui_ListPush(list, fmt,  __VA_ARGS__), ui_ListPop(list))

struct ui_List 		ui_ListInit(enum axis_2 axis, const f32 max_pixel_size, const f32 entry_pixel_size, const enum ui_SelectionType unique_selection);
void			ui_ListPush(struct ui_List *list, const char *format, ...);
void			ui_ListPop(struct ui_List *list);
struct ui_NodeCache	ui_ListEntryAllocCached(struct ui_List *list, const utf8 id, const struct ui_NodeCache cache);
struct slot 		ui_ListEntryAlloc(struct ui_List *list, const utf8 id);
struct slot 		ui_ListEntryAllocF(struct ui_List *list, const char *format, ...);

/*
ui_DropdownMenu
================
ui_DropdownMenu is a widget that displays a string in its box. When the box is hovered, a list of entries is attached
to some choosen side of the menu.
*/

enum ui_DropdownPosition
{
	UI_DROPDOWN_BELOW,
	UI_DROPDOWN_ABOVE,
	UI_DROPDOWN_LEFT,
	UI_DROPDOWN_RIGHT,
};

struct ui_DropdownMenu
{
	u64				flags;
	enum ui_DropdownPosition	position;

	u32				root;

	vec2				entry_size;
	f32				max_dropdown_height;
	f32				dropdown_x;
	f32				dropdown_y;

	struct ui_List			list;
};

struct ui_DropdownMenu 	ui_DropdownMenuInit(const f32 max_dropdown_height, const vec2 entry_size, const enum ui_DropdownPosition position);
u32			ui_DropdownMenu(struct ui_DropdownMenu *menu, const utf8 id);
u32			ui_DropdownMenuF(struct ui_DropdownMenu *menu, const char *format, ...);
void			ui_DropdownMenuPush(struct ui_DropdownMenu *menu);
void			ui_DropdownMenuPop(struct ui_DropdownMenu *menu);

struct slot		ui_DropdownMenuEntry(struct ui_DropdownMenu *menu, const utf8 id);
struct slot		ui_DropdownMenuEntryF(struct ui_DropdownMenu *menu, const char *format, ...);

/*
ui_tooltip
==========
//TODO
*/

/***************************************** ######TODO *****************************************/

struct ui_CmdConsole
{
	struct ui_TextInput	prompt;
	u32			visible;
};

void	ui_CmdConsoleF(struct ui_CmdConsole *console, const char *fmt, ...);

u64	ui_ButtonF(const u64 flags, const char *fmt, ...);

/***************************************** ui_Timeline ******************************************/

struct ui_TimelineRowConfig
{
	f32	height;		/* pixel_height */
	intv	depth_visible;	/* visible task depth */
};

struct ui_TimelineConfig
{
	u64 	ns_interval_start;	/* start of interval covered by timeline window */
	u64 	ns_interval_end;	/* end of interval covered by timeline window   */
	u64	ns_interval_size;	
	f32	ns_half_pixel;		/* ns per half-pixel of timeline */
	u32	fixed;			/* helper: should the ns interval be continuously updated? */

	u32 	row_count;
	u32	rowPushed;
	struct ui_TimelineRowConfig *row;

	/* display geometry */
	f32	width;			/* pixel width of timeline */
	f32	task_height;		/* task / timeline event height in pixels */
	f32	perc_width_row_title_column; /* width of row column (in percentage of parent)  */
	u32	unit_line_count;	/* actual unit line count */
	f32	unit_line_width;	/* pixel width of unit lines */
	f32	subline_width;		/* pixel wdith of sublines */
	u32	sublines_per_line;	/* How many sublines should be drawn between two unit lines */
	
	/* contains visual info and how to convert nanoseconds to prefered unit */
	utf8	unit;			/* ns, us, ms, s */
	u64	unit_line_first;
	u64 	unit_line_interval;		/* time in given unit between equidistant lines in timeline */
	u64	unit_to_ns_multiplier;		/* unit * multiplier = ns */

	/* preferences */
	u32	unit_line_preferred_count; /* prefered number of unit lines, may not correspond to actual count */

	/* colors */
	vec4 	unit_line_color;	/* color of unit lines */
	vec4	subline_color;		/* color of sub lines */
	vec4	text_color;		/* color of displayed times */
	vec4	background_color;	/* color of timeline background */
	vec4	draggable_color;	/* color of row y-draggable bar */

	vec4	task_gradient_br;	/* gradient color at br of task */
	vec4	task_gradient_tr;	/* gradient color at tr of task */
	vec4	task_gradient_tl;	/* gradient color at tl of task */
	vec4	task_gradient_bl;	/* gradient color at bl of task */

	/* booleans */
	u8	draw_sublines;		/* Draw sublines (less visible lines without units) */
	u8	draw_edgelines;		/* Draw lines (including their unit values) of the two edges of the interval */

	u32	timeline;		/* [INTERNAL] index of row column ui node */
	u32	task_window;		
};

#define ui_TimelineRow(config, row, format, ...) UI_SCOPE(ui_TimelineRowPush(config, row, format, __VA_ARGS__), ui_TimelineRowPop(config))

void 	ui_Timeline(struct ui_TimelineConfig *config);
void 	ui_TimelineRowPush(struct ui_TimelineConfig *config, const u32 row, const char *format, ...);
void 	ui_TimelineRowPop(struct ui_TimelineConfig *config);

/***************************************** Popup Windows *****************************************/

enum ui_PopupType
{
	UI_POPUP_CHOICE,	
	UI_POPUP_UTF8_DISPLAY,
	UI_POPUP_UTF8_INPUT,
	UI_POPUP_COUNT
};

enum ui_PopupState
{
	UI_POPUP_STATE_NULL,			/* popup is not allocated 				*/
	UI_POPUP_STATE_RUNNING,			/* popup is displaying contents				*/
	UI_POPUP_STATE_PENDING_VERIFICATION,	/* popup output ready for verification 			*/
	UI_POPUP_STATE_COMPLETED,		/* popup output has been applied, run cleanup and exit	*/
	UI_POPUP_STATE_COUNT
};

/*
 * TODO Usage: 
 */
struct ui_Popup
{
	u32 			window;
	enum ui_PopupType	type;
	enum ui_PopupState	state;

	utf8			display1;
	utf8			display2;
	utf8			display3;

	struct ui_TextInput	*prompt;
	utf8			*input;

	union
	{
		u32		positive;
		u32		yes;
		u32		accept;
	};
	
	const char *		cstr_negative;
	union
	{
		u32		negative;
		u32		no;
		u32		decline;
		u32		cancel;
	};
};

struct ui_Popup ui_PopupNull(void);
void		ui_PopupTryDestroyAndSetToNull(struct ui_Popup *popup);

void 		ui_PopupUtf8Display(struct ui_Popup *popup, const utf8 display, const char *title, const struct ui_Visual *visual);
void 		ui_PopupUtf8Input(struct ui_Popup *popup, utf8 *input, struct ui_TextInput *line, const utf8 description, const utf8 prefix, const char *title, const struct ui_Visual *visual);
void 		ui_PopupChoice(struct ui_Popup *popup, const utf8 description, const utf8 positive, const utf8 negative, const char *title, const struct ui_Visual *visual);

/********************************************************************************************************/
/*				      UI DRAW COMMANDS AND BUCKETING					*/
/********************************************************************************************************/

#define UI_CMD_TEXTURE_BITS	((u32) 14)
#define UI_CMD_LAYER_BITS	((u32) 2)
#define UI_CMD_DEPTH_BITS	((u32) 32  - UI_CMD_TEXTURE_BITS - UI_CMD_LAYER_BITS)

#define UI_CMD_TEXTURE_LOW_BIT	((u32) 0)
#define UI_CMD_LAYER_LOW_BIT	(UI_CMD_TEXTURE_BITS)
#define UI_CMD_DEPTH_LOW_BIT	(UI_CMD_TEXTURE_BITS + UI_CMD_LAYER_BITS)

#define UI_CMD_TEXTURE_MASK	((((u32) 1 << UI_CMD_TEXTURE_BITS) - (u32) 1) << UI_CMD_TEXTURE_LOW_BIT)
#define UI_CMD_LAYER_MASK	((((u32) 1 << UI_CMD_LAYER_BITS) - (u32) 1) << UI_CMD_LAYER_LOW_BIT)
#define UI_CMD_DEPTH_MASK	((((u32) 1 << UI_CMD_DEPTH_BITS) - (u32) 1) << UI_CMD_DEPTH_LOW_BIT)

#define ui_CmdTextureGet(val32)	((val32 & UI_CMD_TEXTURE_MASK) >> UI_CMD_TEXTURE_LOW_BIT)
#define ui_CmdLayerGet(val32)		((val32 & UI_CMD_LAYER_MASK) >> UI_CMD_LAYER_LOW_BIT)
#define ui_CmdDepthGet(val32)		((val32 & UI_CMD_DEPTH_MASK) >> UI_CMD_DEPTH_LOW_BIT)

#define UI_CMD_LAYER_VISUAL		((u32) 0x3)
#define UI_CMD_LAYER_INTER		((u32) 0x2)
#define UI_CMD_LAYER_TEXT_SELECTION 	((u32) 0x1)
#define UI_CMD_LAYER_TEXT		((u32) 0x0)

#define ui_DrawCommand(depth, layer, texture)	 					\
					(	  ((depth) << UI_CMD_DEPTH_LOW_BIT)	\
						| ((layer) << UI_CMD_LAYER_LOW_BIT)	\
						| ((texture) << UI_CMD_TEXTURE_LOW_BIT)	\
					)
struct ui_Node;
typedef struct ui_TextSelection
{
	const struct ui_Node *	node;
	struct textLayout *	layout;
	vec4			color;
	u32			low;
	u32			high;
} ui_TextSelection;
DECLARE_STACK(ui_TextSelection);

struct ui_TextSelection ui_TextSelectionEmpty(void);

struct ui_DrawNode
{
	struct ui_DrawNode *	next;
	u32 			index;	/* index to node if CMD_LAYER != TEXT_SELECTION, otherwise index to text selection stack. */
};

struct ui_DrawBucket
{
	POOL_SLOT_STATE;
	DLL_SLOT_STATE;
	u32 			cmd;
	u32 			count;
	struct ui_DrawNode *	list;
};

/********************************************************************************************************/
/*				             UI INTERACTIONS						*/
/********************************************************************************************************/

/*
 * arg[0] = keycode (0 if none)
 * arg[1] = key_modifiers (KEY_MOD_***)
 * arg[2] = utf8 encoded input 
 */
extern u32 cmd_ui_text_op;

/*
 * selection = [low, high); if str_replace.len != 0, the text in selection is replaced with the string's contents,
 * 	and any text after the selection, i.e. the contents in [high, end], is either shifted downwards or upwards
 * 	depending on the context. 
 */
struct ui_TextOp
{
	utf32	str_copy;	/* If not empty, we cope contents to clipboard 		*/
	utf32	str_replace;	/* Replace [low, high) with contents (Even if Empty) 	*/
	u32	cursor_new;	/* new cursor position 					*/
	u32	mark_new;	/* new mark position 					*/
	u32	low;		/* lower limit (inclusive) of interval to replace 	*/
	u32	high;		/* upper limit (exclusive) of interval to replace 	*/
};

/* UI Interaction state, contains both persistent and frame state */
struct ui_Interaction
{
	/* ui interactions */
	u64 	interactions;

	/* current mouse hovered node */
	utf8 	node_hovered;

	u32			text_internal_buf[256];
	u32 			text_edit_mode;
	utf8			text_edit_id;		
	struct ui_TextInput *	text_edit;	/* aliasing external ui_TextInput. */

	vec2	cursor_delta;
	vec2 	cursor_position;    /* window bottom left = (0.0f, 0.0f) */

	/* keyboard state */
	u32 	key_clicked[DS_KEY_COUNT];	/* frame : Was key clicked this frame? [DS_KEYCODE_COUNT] 	*/
	u32 	key_released[DS_KEY_COUNT];	/* frame : Was key clicked this frame? [DS_KEYCODE_COUNT] 	*/
	u32 	key_pressed[DS_KEY_COUNT];	/* persistent : Is key currently pressed? [DS_KEYCODE_COUNT] 	*/

	/* mouse state */
	u64	ns_double_click;
	u64	ns_button_time_since_last_pressed[MOUSE_BUTTON_COUNT]; /* persistent : time (ns) since last press */
	u32 	button_double_clicked[MOUSE_BUTTON_COUNT];	/* frame : was button double_click happen this frame? */
	u32 	button_clicked[MOUSE_BUTTON_COUNT];		/* frame : was button pressed this frame? */
	u32 	button_released[MOUSE_BUTTON_COUNT];		/* frame : Was button releasd this frame? */ 
	u32 	button_pressed[MOUSE_BUTTON_COUNT];		/* persistent : Is button still pressed?  */
	u32 	scroll_up_count;				/* frame */
	u32 	scroll_down_count;				/* frame */
};

/********************************************************************************************************/
/*				               UI SIZING 						*/
/********************************************************************************************************/

/* ui node size type for each axis */
enum ui_SizeType
{
	UI_SIZE_NONE,		/* No size type, //TODO Explicitly set hardcoded size??? */
	UI_SIZE_PIXEL,		/* Wanted size is given in pixels on node creation */
	UI_SIZE_PERC_PARENT,	/* Size is determined by the parent's size and the given input percentage */
	UI_SIZE_UNIT,		/* Size is determined by a pushed viewable unit interval and the current
				   pushed node unit interval. the unit is a custom one, so percentages
				   are calculated against the viewable unit inteval. */
	UI_SIZE_CHILDSUM,	/* Size is determined by the childrens' total size; The Children's wanted 
					   positions (in the axis) will be ignored and instead packed. */
	UI_SIZE_TEXT,		/* Size is determined by the text input and the text input draw instruction */
	UI_SIZE_COUNT
};

/* ui_Size: semantic size type per axis of a ui_node. */
typedef struct ui_Size
{
	enum ui_SizeType	type;		/* size type */
	f32			strictness;	/* lower bound of final size in percentage of computed size */
	union
	{
		f32		pixels;		/* pixel size 		*/
		f32		percentage;	/* percentage of parent */ 
		f32		line_width;	/* line width in pixels */
		intv		intv;		/* unit interval 	*/
	};
} ui_Size;
DECLARE_STACK(ui_Size);

/********************************************************************************************************/
/*				               UI STATE 						*/
/********************************************************************************************************/

DECLARE_STACK(utf32);

/* Per window ui struct */
struct ui
{
	struct memSlot		mem_slot;
	struct ui_Interaction	inter;

	struct pool		bucket_pool;
	struct dll		bucket_list;
	struct hashMap 		bucket_map;
	u32			bucket_cache;	/* for quick cmd check */
	u32			bucket_count;

	struct pool		event_pool;
	struct dll		event_list;

	/* node map for all u's  */
	/* Shared allocator for all nodes  */
	struct hi 		node_hierarchy;
	struct hashMap 		node_map;

	stack_ui_TextSelection	frame_stack_text_selection;
	vec4			text_cursor_color;
	vec4			text_selection_color;

	u64		frame;
	struct arena 	mem_frame_arr[2];
	struct arena *	mem_frame;

	vec2u32		window_size;

	u32		node_count_frame;
	u32		node_count_prev_frame;

	u32		root;	/* root node of ui, always allocated on frame begin */

	stack_u32	stack_parent;	
	stack_u32	stack_sprite;
	stack_u64	stack_flags;
	stack_u64 	stack_recursive_interaction_flags;
	stack_ptr	stack_font;

	/* external text usage; used for skipping layout calculations for a string that is used in multiple nodes */
	stack_utf32	stack_external_text;
	stack_ptr	stack_external_text_layout;
	stack_ptr	stack_external_text_input;

	/* push all floating nodes so that we can linear search which floating subtree we are hovering */
	stack_u32	stack_floating_node;
	stack_u32	stack_floating_depth;

	/* text stacks */
	stack_u32	stack_text_alignment_x;
	stack_u32	stack_text_alignment_y;
	stack_f32	stack_text_pad[AXIS_2_COUNT];

	stack_f32	stack_pad;

	stack_u32	stack_fixed_depth;
	stack_f32	stack_floating[AXIS_2_COUNT];	
	stack_ui_Size	stack_ui_Size[AXIS_2_COUNT];
	stack_intv	stack_viewable[AXIS_2_COUNT];
	stack_u32	stack_child_layout_axis;
	struct stackVec4	stack_background_color;
	struct stackVec4	stack_border_color;
	struct stackVec4	stack_gradient_color[BOX_CORNER_COUNT];
	struct stackVec4	stack_sprite_color;
	stack_f32	stack_edge_softness;
	stack_f32	stack_corner_radius;
	stack_f32	stack_border_size;
};

extern struct ui *g_ui;

struct ui *	ui_Alloc(void);					/* allocate a new ui 		*/
void		ui_Dealloc(struct ui *ui);			/* dealloc an ui 		*/
void		ui_Set(struct ui *ui);				/* set global ui within ui_*.c 	*/
void		ui_FrameBegin(const vec2u32 window_size, const struct ui_Visual *base); /* begin new ui frame */
void		ui_FrameEnd(void);				/* end ui frame 		*/

/********************************************************************************************************/
/*					 ui_node internals 						*/
/********************************************************************************************************/

/* 
 * Possible flags for nodes: 
 * 	Draw flags affect rendering and what render code is run within the core. The absence of a draw flag should
 * 	set necessary "zero" defaults.
 *
 * 	Inter flags affect how the node is interacted with.
 */

#define 	UI_FLAG_NONE			((u64) 0)

/******************** Renderpath flags ********************/
#define 	UI_DRAW_BACKGROUND 		((u64) 1 << 0)
#define 	UI_DRAW_BORDER 			((u64) 1 << 1)
#define 	UI_DRAW_EDGE_SOFTNESS		((u64) 1 << 2)
#define 	UI_DRAW_ROUNDED_CORNERS		((u64) 1 << 3)
#define		UI_DRAW_GRADIENT		((u64) 1 << 4)
#define 	UI_DRAW_TEXT			((u64) 1 << 5)
#define 	UI_DRAW_SPRITE			((u64) 1 << 6)
#define 	UI_DRAW_TEXT_FADE		((u64) 1 << 7)
#define		UI_DRAW_FLAGS	(				\
			  UI_DRAW_BACKGROUND 			\
	      		| UI_DRAW_BORDER 			\
	      		| UI_DRAW_EDGE_SOFTNESS 		\
	      		| UI_DRAW_ROUNDED_CORNERS		\
	      		| UI_DRAW_GRADIENT			\
	      		| UI_DRAW_SPRITE			\
	      		| UI_DRAW_TEXT_FADE			\
	)

/******************** Interaction flags ********************/

#define		UI_INTER_RECURSIVE_ROOT		((u64) 1 << 17)	/* When this is set for a node, the node will, 
								   regardless of if we interact with it locally, 
								   allocate an inter_node. the inter_node is then
								   also modified according to the node's recursive
								   interaction flags by any of the node's children.
								  */
#define		UI_INTER_ACTIVE				((u64) 1 << 18)	/* The node can be activated or focused. */
#define		UI_INTER_HOVER				((u64) 1 << 19)
#define 	UI_INTER_LEFT_CLICK			((u64) 1 << 20)
#define 	UI_INTER_LEFT_DOUBLE_CLICK		((u64) 1 << 21) 
#define		UI_INTER_DRAG				((u64) 1 << 22)
#define		UI_INTER_SCROLL				((u64) 1 << 23)
#define		UI_INTER_SELECT				((u64) 1 << 24) /* Selection actions (Lclick, ...), on the
									   node makes the node selected, and a repeat
									   selection of the node deselects it. */
#define		UI_INTER_FOCUS				((u64) 1 << 25) /* Similar to UI_INTER_SELECT, expect that
									   refocusing, or reselecting the node does
									   not remove the focus. Useful for clicking
									   on input lines. Remove focus on ESCAPE */
#define		UI_INTER_FOCUS_IN			((u64) 1 << 26)
#define		UI_INTER_FOCUS_OUT			((u64) 1 << 27)

#define		UI_INTER_FLAGS	(0				\
			| UI_INTER_ACTIVE			\
			| UI_INTER_HOVER			\
			| UI_INTER_LEFT_CLICK			\
			| UI_INTER_LEFT_DOUBLE_CLICK		\
			| UI_INTER_DRAG				\
			| UI_INTER_SCROLL			\
			| UI_INTER_SELECT			\
			| UI_INTER_FOCUS			\
			| UI_INTER_FOCUS_IN			\
			| UI_INTER_FOCUS_OUT			\
		)

#define		UI_INTER_FOCUS_FLAGS (0				\
			| UI_INTER_FOCUS			\
			| UI_INTER_FOCUS_IN			\
			| UI_INTER_FOCUS_OUT			\
		)

#define		UI_INTER_ACTIVATION_FLAGS (0			\
			| UI_INTER_DRAG				\
			| UI_INTER_SELECT			\
			| UI_INTER_FOCUS			\
		)

/* When recursively checking for select, we must specify UI_INTER_LEFT_CLICK to propagate interactions correctly */
#define		UI_INTER_RECURSIVE_SELECT	(UI_INTER_SELECT | UI_INTER_LEFT_CLICK)

/******************** General control flags ********************/
#define		UI_UNIT_POSITIVE_DOWN		((u64) 1 << 34) /* Default Y-layout of ui_SizeUnit's is that y grow
							  	   upwards, by setting this flag, node unit sizes
								   will be interpreted with Y growing downwards.  */
#define		UI_SKIP_HOVER_SEARCH		((u64) 1 << 35) /* Skip searching if cursor is hovering the node 
								   (and subsequently its sub-hierarchy). Useful for
								   when two children occupies the same space, one
								   which we interact with, the other only visual.  */

#define		UI_TEXT_EDIT			((u64) 1 << 36) /* Node sets text edit state on focus */
#define		UI_TEXT_EDIT_INTER_BUF_ON_FOCUS	((u64) 1 << 37) /* Node aliases an cleared internal ui buffer on
								   focus instead of external text buffer. Since we
								   can only text edit a single widget at a time, the
								   whole system can rely upon a single buffer. */
#define		UI_TEXT_EDIT_COPY_ON_FOCUS	((u64) 1 << 38)	/* If UI_TEXT_EDIT, copy whatever text was served 
								   to the node into the text edit buffer and select
								   the whole string. */
#define		UI_TEXT_ATTACHED		((u64) 1 << 39) /* text is attached to node */
#define		UI_TEXT_ALLOW_OVERFLOW		((u64) 1 << 40) /* calculate textLayout using an infinite line width  */
#define		UI_TEXT_EXTERNAL		((u64) 1 << 41) /* Ignore any text given in node identifier string
								   at allocation and instead pick text from top of 
								   stack_text_external */
#define		UI_TEXT_EXTERNAL_LAYOUT		((u64) 1 << 42) /* Ignore text layout paths in ui library and instead
								   use a pre-derived layout. the flag impies the flag
								   UI_TEXT_EXTERNAL and UI_TEXT_ALLOW_OVERFLOW. */
#define		UI_ALLOW_VIOLATION_X		((u64) 1 << 43) /* Allow children to violate node's boundaries in X */
#define		UI_ALLOW_VIOLATION_Y		((u64) 1 << 44) /* Allow children to violate node's boundaries in Y */
#define		UI_FLOATING_X			((u64) 1 << 45)	/* node is "floating", i.e. has a fixed position
								   given at creation time (in X) and is not affected
								   by parent violation solving */
#define		UI_FLOATING_Y			((u64) 1 << 46)	/* node is "floating", i.e. has a fixed position
								   given at creation time (in Y) and is not affected
								   by parent violation solving */
#define		UI_FIXED_X			((u64) 1 << 47)	/* Fixed position in X, ignores position layout */
#define		UI_FIXED_Y			((u64) 1 << 48)	/* Fixed position in Y, ignores position layout */

/******************** Implicit control flags (DO NOT SET THESE YOURSELF!) ********************/
#define		UI_NON_HASHED			((u64) 1 << 55)	/* Non-hashed node; => non-interactable  */
#define		UI_TEXT_LAYOUT_POSTPONED	((u64) 1 << 56) /* textLayout is calculated after violation solving  */
#define		UI_PAD				((u64) 1 << 57) /* padding node */
#define		UI_PAD_FILL			((u64) 1 << 58)	/* pad size such that parent node size is completely 
								   used up by children */
#define		UI_PERC_POSTPONED_X		((u64) 1 << 59) /* perc calculations are postponed (in X) until after 
								   violation solving. (Useful for child perc parent
								   when parent is childsum  */
#define		UI_PERC_POSTPONED_Y		((u64) 1 << 60) /* perc calculations are postponed (in Y) until after 
								   violation solving. */
struct ui_Node
{
	HI_SLOT_STATE;
	utf8			id;		/* unique identifier  */
	struct ui_TextInput	input;		/* text to display OR text to edit */

	u64		flags;			/* interaction, draw flags */
	u64		last_frame_touched;	/* if not touched within new frame, the node is pruned at the end */
	u32		hash;			/* hashed id */
	u32		depth;			/* parent->depth + 1 or fixed depth */

	u64		inter_recursive_mask;	/* union of ancestor and node recursive_flags */
	u64		inter_recursive_flags;	/* recursive interactions checked by children */
	u64		inter;			/* interactions during frame */

	const struct font *font;
	enum spriteId	sprite;

	enum axis_2	child_layout_axis;
	struct ui_Size 	semantic_size[AXIS_2_COUNT];

	/* text layout values */
	enum alignment_x	text_align_x;
	enum alignment_y	text_align_y;
	vec2			text_pad;
	struct textLayout *	layout_text;

	/* building position (relative) and size (in pixels); not taking into account the hierarchy */
	vec2		layout_position;	
	vec2		layout_size;

	/* final window position (absolute) and size (in pixels) */
	vec2		pixel_position;	
	vec2		pixel_size;

	/* visible pixel interval (subset of pixel_position + pixel_size */
	intv		pixel_visible[AXIS_2_COUNT];

	vec4		background_color;
	vec4		border_color;
	vec4		sprite_color;
	vec4		gradient_color[BOX_CORNER_COUNT];
	f32		border_size;
	f32		edge_softness;
	f32		corner_radius;
};

struct ui_NodeCache
{
	u64		last_frame_touched;
	struct ui_Node *frame_node;	
	u32		index;	
};

struct ui_NodeCache	ui_NodeCacheNull(void);

#define UI_NON_CACHED_INDEX 	HI_ORPHAN_STUB_INDEX
struct ui_NodeCache 	ui_NodeAllocCached(const u64 flags, const utf8 id, const utf8 text, const struct ui_NodeCache cache);

/* allocate new node, values are set according to stack values */
struct slot	ui_NodeAlloc(const u64 flags, const utf8 *formatted);
/* format input string and allocate new node, values are set according to stack values */
struct slot	ui_NodeAllocF(const u64 flags, const char *format, ...);
/* allocate a new non-hashed node, values are set according to stack values */
struct slot	ui_NodeAllocNonHashed(const u64 flags);
/* get node address */
struct ui_Node *ui_NodeAddress(const u32 node);
/* lookup node using id; returns (NULL, U32_MAX) on not found. */
struct slot	ui_NodeLookup(const utf8 *id);
/* push node stack; any new node created becomes a decendant */
void 		ui_NodePush(const u32 node);		
/* pop node stack */
void 		ui_NodePop(void);			
/* get top node address (parent address) */
struct ui_Node *ui_NodeTop(void);


/* add layout node padding given number of pixels according to stack_pad (non-cacheable) */
u32	ui_Pad();
/* add layout node padding given number of pixels (non-cacheable) */
u32	ui_PadPixel(const f32 pixels);
/* add layout node padding given percentage of parent (non-cacheable) */
u32	ui_PadPerc(const f32 perc);
/* add layout node padding parent space until it is filled, (non-cacheable)  */
u32	ui_PadFill(void);

/********************************************************************************************************/
/*					 ui_Size initalizers 						*/
/********************************************************************************************************/

#define ui_SizePixel(_pixels, _strictness) 	((struct ui_Size) { .type = UI_SIZE_PIXEL, .pixels = _pixels, .strictness = _strictness })
#define ui_SizePerc(_percentage)		((struct ui_Size) { .type = UI_SIZE_PERC_PARENT, .percentage = _percentage, .strictness = 0.0f })	
#define ui_SizeChildsum(_strictness)		((struct ui_Size) { .type = UI_SIZE_CHILDSUM, .strictness = _strictness })
#define ui_SizeUnit(_intv)			((struct ui_Size) { .type = UI_SIZE_UNIT, .intv = _intv, .strictness = 0.0f })
#define ui_SizeText(_line_width, _strictness)	((struct ui_Size) { .type = UI_SIZE_TEXT, .line_width = _line_width, .strictness = _strictness })

/********************************************************************************************************/
/*					 Push/Pop global state  					*/
/********************************************************************************************************/

#define ui_Parent(parent)			UI_SCOPE(ui_NodePush(parent), ui_NodePop())
#define ui_Size(axis, size)			UI_SCOPE(ui_SizePush(axis, size), ui_SizePop(axis))
#define ui_Width(size)				UI_SCOPE(ui_WidthPush(size), ui_WidthPop())
#define ui_Height(size)				UI_SCOPE(ui_HeightPush(size), ui_HeightPop())
#define ui_Floating(axis, pixel)		UI_SCOPE(ui_FloatingPush(axis, pixel), ui_FloatingPop(axis))
#define ui_FloatingX(pixel)			ui_Floating(AXIS_2_X, pixel)
#define ui_FloatingY(pixel)			ui_Floating(AXIS_2_Y, pixel)
#define ui_FixedX(pixel)			ui_Flags(UI_FIXED_X)		\
						ui_FloatingX(pixel)
#define ui_FixedY(pixel)			ui_Flags(UI_FIXED_Y)		\
						ui_FloatingY(pixel)
#define ui_ChildLayoutAxis(axis)		UI_SCOPE(ui_ChildLayoutAxisPush(axis), ui_ChildLayoutAxisPop())
#define ui_BackgroundColor(color)		UI_SCOPE(ui_BackgroundColorPush(color), ui_BackgroundColorPop())
#define ui_BorderColor(color)			UI_SCOPE(ui_BorderColorPush(color), ui_BorderColorPop())
#define ui_SpriteColor(color)			UI_SCOPE(ui_SpriteColorPush(color), ui_SpriteColorPop())
#define ui_GradientColorBr(color)		UI_SCOPE(ui_GradientColorPush(BOX_CORNER_BR, color), ui_GradientColorPop(BOX_CORNER_BR))
#define ui_GradientColorTr(color)		UI_SCOPE(ui_GradientColorPush(BOX_CORNER_TR, color), ui_GradientColorPop(BOX_CORNER_TR))
#define ui_GradientColorTl(color)		UI_SCOPE(ui_GradientColorPush(BOX_CORNER_TL, color), ui_GradientColorPop(BOX_CORNER_TL))
#define ui_GradientColorBl(color)		UI_SCOPE(ui_GradientColorPush(BOX_CORNER_BL, color), ui_GradientColorPop(BOX_CORNER_BL))
#define ui_EdgeSoftness(val)			UI_SCOPE(ui_EdgeSoftnessPush(val), ui_EdgeSoftnessPop())
#define ui_CornerRadius(val)			UI_SCOPE(ui_CornerRadiusPush(val), ui_CornerRadiusPop())
#define ui_BorderSize(val)			UI_SCOPE(ui_BorderSizePush(val), ui_BorderSizePop())
#define ui_Font(font)				UI_SCOPE(ui_FontPush(font), ui_FontPop())
#define ui_Sprite(sprite)			UI_SCOPE(ui_SpritePush(sprite), ui_SpritePop())
#define ui_SpriteColor(color)			UI_SCOPE(ui_SpriteColorPush(color), ui_SpriteColorPop())
#define ui_IntvViewable(axis, inv)		UI_SCOPE(ui_IntvViewablePush(axis,(inv)), ui_IntvViewablePop(axis))
#define ui_IntvViewableX(inv)			ui_IntvViewable(AXIS_2_X, (inv))
#define ui_IntvViewableY(inv)			ui_IntvViewable(AXIS_2_Y, (inv))
#define ui_TextAlignX(align)			UI_SCOPE(ui_TextAlignXPush(align), ui_TextAlignXPop())
#define ui_TextAlignY(align)			UI_SCOPE(ui_TextAlignYPush(align), ui_TextAlignYPop())
#define ui_TextPad(axis, pad)			UI_SCOPE(ui_TextPadPush(axis, pad), ui_TextPadPop(axis))
#define ui_TextPadX(pad)			ui_TextPad(AXIS_2_X, pad)
#define ui_TextPadY(pad)			ui_TextPad(AXIS_2_Y, pad)
#define ui_Flags(flags)				UI_SCOPE(ui_FlagsPush(flags), ui_FlagsPop())
#define ui_FixedDepth(depth)			UI_SCOPE(ui_FixedDepthPush(depth), ui_FixedDepthPop())
#define ui_RecursiveInteraction(flags) 	UI_SCOPE(ui_RecursiveInteractionPush(flags), ui_RecursiveInteractionPop())
#define ui_ExternalText(text)			UI_SCOPE(ui_ExternalTextPush(text), ui_ExternalTextPop())
#define ui_ExternalTextLayout(layout, text)	UI_SCOPE(ui_ExternalTextLayoutPush(layout, text), ui_ExternalTextLayoutPop())
#define ui_ExternalTextInput(input)		UI_SCOPE(ui_ExternalTextInputPush(input), ui_ExternalTextInputPop())

void 	ui_SizePush(const enum axis_2 axis, const struct ui_Size size);
void 	ui_SizeSet(const enum axis_2 axis, const struct ui_Size size);
void 	ui_SizePop(const enum axis_2 axis);

void 	ui_WidthPush(const struct ui_Size size);
void 	ui_WidthSet(const struct ui_Size size);
void 	ui_WidthPop(void);

void 	ui_HeightPush(const struct ui_Size size);
void 	ui_HeightSet(const struct ui_Size size);
void 	ui_HeightPop(void);

void 	ui_FloatingPush(const enum axis_2 axis, const f32 pixel);
void 	ui_FloatingSet(const enum axis_2 axis, const f32 pixel);
void 	ui_FloatingPop(const enum axis_2 axis);

void 	ui_ChildLayoutAxisPush(const enum axis_2 axis);
void 	ui_ChildLayoutAxisSet(const enum axis_2 axis);
void 	ui_ChildLayoutAxisPop(void);

void 	ui_IntvViewablePush(const enum axis_2 axis, const intv inv);
void 	ui_IntvViewableSet(const enum axis_2 axis, const intv inv);
void 	ui_IntvViewablePop(const enum axis_2 axis);
#define ui_IntvViewableXSet(inv)	ui_IntvViewableSet(AXIS_2_X, inv)
#define ui_IntvViewableYSet(inv)	ui_IntvViewableSet(AXIS_2_Y, inv)

void 	ui_BackgroundColorPush(const vec4 color);
void 	ui_BackgroundColorSet(const vec4 color);
void 	ui_BackgroundColorPop(void);

void 	ui_BorderColorPush(const vec4 color);
void 	ui_BorderColorSet(const vec4 color);
void 	ui_BorderColorPop(void);

void 	ui_SpriteColorPush(const vec4 color);
void 	ui_SpriteColorSet(const vec4 color);
void 	ui_SpriteColorPop(void);

void 	ui_GradientColorPush(const enum box_corner, const vec4 color);
void 	ui_GradientColorSet(const enum box_corner, const vec4 color);
void 	ui_GradientColorPop(const enum box_corner);

void 	ui_FontPush(const enum fontId font);
void 	ui_FontSet(const enum fontId font);
void 	ui_FontPop(void);

void 	ui_EdgeSoftnessPush(const f32 softness);
void 	ui_EdgeSoftnessSet(const f32 softness);
void 	ui_EdgeSoftnessPop(void);

void 	ui_CornerRadiusPush(const f32 radius);
void 	ui_CornerRadiusSet(const f32 radius);
void 	ui_CornerRadiusPop(void);

void 	ui_BorderSizePush(const f32 pixels);
void 	ui_BorderSizeSet(const f32 pixels);
void 	ui_BorderSizePop(void);

void 	ui_SpritePush(const enum spriteId sprite);
void 	ui_SpriteSet(const enum spriteId sprite);
void 	ui_SpritePop(void);

void 	ui_TextAlignXPush(const enum alignment_x align);
void 	ui_TextAlignX_set(const enum alignment_x align);
void 	ui_TextAlignXPop(void);

void 	ui_TextAlignYPush(const enum alignment_y align);
void 	ui_TextAlignY_set(const enum alignment_y align);
void 	ui_TextAlignYPop(void);

void 	ui_TextPadPush(const enum axis_2 axis, const f32 pad);
void 	ui_TextPadSet(const enum axis_2 axis, const f32 pad);
void 	ui_TextPadPop(const enum axis_2 axis);

void 	ui_FlagsPush(const u64 flags);
void 	ui_FlagsSet(const u64 flags);
void 	ui_FlagsPop(void);

void 	ui_RecursiveInteractionPush(const u64 flags);
void 	ui_RecursiveInteractionPop(void);

void 	ui_PaddingPush(const f32 pad);
void 	ui_PaddingSet(const f32 pad);
void 	ui_PaddingPop(void);

void 	ui_FixedDepthPush(const u32 depth);
void 	ui_FixedDepthSet(const u32 depth);
void	ui_FixedDepthPop(void);

void	ui_ExternalTextPush(const utf32 text);
void	ui_ExternalTextSet(const utf32 text);
void	ui_ExternalTextPop();

void	ui_ExternalTextLayoutPush(struct textLayout *textLayout, const utf32 text);
void	ui_ExternalTextLayoutSet(struct textLayout *textLayout, const utf32 text);
void	ui_ExternalTextLayoutPop();

void	ui_ExternalTextInputPush(struct ui_TextInput *text_input);
void	ui_ExternalTextInputPop();

/********************************************************************************************************/
/*					  SIZES AND AUTOLAYOUT						*/
/********************************************************************************************************/
/*
 
Some notes of autolayout and the different size types.  Since we have sizes depending on both children and parents,
we require some ordering of how we compute each node's size. 

	enum ui_Size
	{
		UI_SIZE_NONE,		// No size type 
		UI_SIZE_PIXEL,		// Wanted size is given in pixels on node creation 
		UI_SIZE_PERC_PARENT,	// Size is determined by the parent's size and the given input percentage 
		UI_SIZE_CHILDSUM,	// Size is determined by the childrens' total size. 
		UI_SIZE_TEXT,		// Size determine on the text input and the text input draw instruction 
	};

--- sizes ---

UI_SIZE_PIXEL is easy; if a node is of the type, we simply grab the current global (vec2) preferred_size and preferred
position at node creation, and we are done.

Similarily, For nodes of type UI_SIZE_TEXT, we can compute on node creation the required size for displaying the
node's text.

UI_SIZE_PERC_PARENT is dependent on the parent's final computed size, so we must require that, for a given
axis, a parent may not be of the CHILDSUM type at the same time as at least one of its children is of type
PERC_PARENT. Given that the assumption holds, the parent must be of size PIXEL or TEXT or PERC_PARENT.

For UI_CHILDSUM, we obviously require for the given axis that all children have had their final sizes computed.
By assumption, no child is of type PERC_PARENT, so they must be of size PIXEL or TEXT or UI_CHILDSUM.


--- size calculations ---

Note that whenever we create a node of type PERC_PARENT, the parent is obviously already created. Now,
if we assume that that parent had its size caĺculated at creation, we may calculate the current node's size aswell.
Since the root node has its size already computed, by induction, we may calculate the layout size of any node 
of type PERC_PARENT at node creation. Thus, the only size type we must calculate after all of the frame's
nodes have been created, is the CHILDSUM Type. This must be done bottom up.

 	| NODE_CREATION | LAYOUT_PASS 	|
types	---------------------------------
 	| PIXEL,  	| SUM		|
 	| TEXT,  	|  		|
 	| PERCENTAGE   	|  		|

--- pre layout positions ---
Before we can talk about violation solving, we need to discuss the issue of deriving layout positions. We first
note how we may want to position different nodes:

1. Parent childsum: When the parent is of type childsum, we compact the children at the time of the childsum size calculation.

2. Floating / Fixed: At some point, we must explicitly provide a fixed position in window space at which some node 
starts at. Obviously the root node's fixed position is the upper left pixel of the window. Perhaps we want to hover 
over an in-game entity and display some statistics at the moust point. These kinds of positions are called Floating, 
or Fixed, and are not restricted by some parent layout; instead, the restriction calculations "resets" for any of 
the floating node's children.

3. Now, consider the case of the profiler; we have to put nodes at the correct unit of time, so similar to Floating nodes,
we must provide some fixed unit as a position. This coincidentally gives rise to a solution for the scrolling case as well!
If we define a node flag NODE_*_UNIT and a push/pop interval [unit_min, unit_max], and extend nodes to use the interval if
the flag is set, we should somehow be able to set correct positions for child nodes. Some interesting values are:

Interesting variables/values:
	NODE_*_UNIT ; If defined, the node's children will have their positions and sizes calculated according to their 
		 	interval position and size. Children not intersecting the view_interval are not created, nor
			are children deemed "to small" (saves space and time + small ones (<2px) are not meaniful anyway)

	Interval Full		[min, max] 		; full interval, perhaps [0, current_time]. May not be useful to us.

	Interval viewable 	[view_min, view_max] 	; viewable interval, any children whose interval position and size
							; intersections this interval is created as a node. The child's
							; position and size can at this point be calculated as a percentage
							; offset and size of the parent.

Interesting functions:
	unit_viewable_intervalPush/pop()	(push/pop current unit interval we are working with)


	This problem may be approached from two different sides; one way is to define a new size_type, size_unit.
	We then store the axis position and size as a (f32) unit, and in the layout phase, we can derive actual percentages,
	or pixels, given that we know the visible interval. Since the position is "floaty", or an offset, it would probably
	be simpler to store some sort of percentages instead of pixels. But this begs the question, wouldn't it be simpler to
	have size_type be PARENT_PERCENTAGE? This leads to the second solution:

	In solution 2, we set size_type = PERC_PARENT for any child, and, at creation when we run ui_node_calculate_immediate_size,
        we add a conditional codepath in PERC_PARENT as to differ the fixed percentage path to the interval percentage path. Similarly,
	we can derive a "percentage position" when NODE_*_UNIT is set.

--- visibility masking ---
each node has a visibility mask, and a flag NODE_VISIBILITY_MASK... to denote if we are derive a visibility mask for the node against its parent.
This is useful for children that are under/overflowing a parent and we wish not to draw the under/overflowing part. 

--- violation solver ---

--- Floating/Fixed interactivity ---
Issue: a floating node may cover a non-ancestral region, so we cannot do a recursive mouse position interection to determine
the hierarchy of nodes we may possible interact with.

=========================== PHASES ===========================

We need to be very careful in how and when we convert between a node's different positions and sizes. We have three types; the semantic
layout provided by the user, either explicitly or implicitly, The pre solver layout constituting some arbitrary values shoved into the
solver, and lastly the post solver layout, the final layout which we could state in pixels.

	   semantic size and position 		(semantic)
	=> layout size and position 		(pre solver)
	=> solved violations size and position  (post solver)

There is a nasty trade off between simplicity and iteration time here; depending on the phase size, lets say the semantic size of a
node, we can calculate its layout size immediately for fixed and upward dependent semantic sizes, but require tree traversal for the
downward dependent semantic sizes. The question is, How much do we wish to sacrifice simplicity for less iteration time? We begin
by drawing a table for the size transformations for each size type:

		SEMANTIC PHASE		LAYOUT PHASE			SOLVER_PHASE				
	       	+-----------------------+-------------------------------+-------------------------------------------------------------------------------+
          PIXEL |X     			|lay_size = px 			|sol_size = solve_according_to_context						|
           TEXT	|X   			|lay_size = text_px		|sol_size = solve_according_to_context						|
    PERC_PARENT	|sem_size = perc     	|X				|sol_size = perc(final_parent_size, sem_size)					|
       CHILDSUM	|X			|lay_size = sum(lay_size_child) |sol_size = solve_according_to_context						|
           UNIT	|sem_size = unit	|X				|sol_size = perc(final_parent_size, perc(sem_size)) OR position dependent 	|
		+-----------------------+-------------------------------+-------------------------------------------------------------------------------+

First, note that for PIXEL, TEXT we may immediately calculate their layout sizes (in pixels). Similarily, for PERC_PARENT and UNIT, we may derive
their percentages immediately (as the viewable interval or parent size (ds_Asserted not to be CHILDSUM) are assumed to be present). After the percentages
have been calculated, they are not needed again until the solver phase, at which we simply derive the final node size by taking a percentage of the solved
parents size. One thing to consider is to merge UNIT and PERC_PARENT into a single type, and instead use a LAYOUT_UNIS_AXIS flag to indicate a preprocess
step for the percentage calculation. For further discussion regarding the flag below.

Now, these semantic sizes are not enough to derive all positions from; only CHILDSUM provides a positioning rule for its children. In almost every case, 
we wish for the position of a node to be implicitly defined by its context as in the CHILDSUM case; only at the root of some sub-hierarchy of nodes is it
reasonable to define an explicit position. Since we cannot control this using only semantic sizes, we introduce potential flags to help us. 

ALLOW_VIOLATIONS_AXIS: specifies the node to ignore any of its violations in the axis such that sol_pos = lay_pos;

FLOATING_AXIS: Specifies that the node shall have its final position immediately set to some pushed value. any violations are ignored by also setting
			ALLOW_VIOLATIONS_AXIS. This becomes our "atomic" position setter, used to initalize the root position for a sub hierarchy of nodes.

VISIBILITY_MASK_AXIS: Consider the case when we have a child that is straddling the boundary of its parent. If VISIBILITY_MASK is set, we clip the child node's
		      rectangle against its parents visiblity mask. Now the child's visibility mask becomes the visible part of the node. There are two reasons 
		      for doing this; First, we can clip the node using our mask on the gpu to get the correct draw result. Secondly, this visibility mask 
		      becomes the region which we may interact with. Finally, if the mask is to set, the node's mask simply becomes its full rectangle. 
		 
		      The question now becomes: Is the flag needed?. We want VISIBILITY_MASK off if the FLOATING flag is set; so if the answer is yes, we must
		      find a reason to have VISIBILITY_MASK=off and FLOATING=off. If FLOATING is off, we are almost surely having the node's position be 
		      implicitly defined, so for the moment, we should probably put this flag off and instead have apply visibility_masking iff FLOATING is
		      off.

UNIT_INTERVAL_AXIS: The node's size and position is dependant on its unit interval and some visible interval. The question is: Do we want this flag, or do we
		    or would it be simpler to create an additional size type?  It seems reasonable to go with the SIZE_UNIT case instead of what would become
		    a SIZE_PERC_PARENT + UNIT_INTERVAL case, as it most likely simplifies Logic throughout the core.  Most importantly, not going with the 
		    second case means we don't introduce implicit size types in our code that overwrites to current one. The workflow using case 1 becomes
		    something like:

			------ user ------							------ core ------
                                                                                                                                                         
			(ui_unit_visible_x(visible_low, visible_high))			        ui_NodeAlloc()
			{                                                                       {
				ui_Size_x_set(ui_unit_x(low1, high1))                           	layout_position = { 0 }
				ui_NodeAlloc()                                                 	layout_size = { 0 }
				ui_Size_x_set(ui_unit_x(low2, high2))                           	implicit_flags = UI_FLAGS_NONE 
				ui_NodeAlloc()                                                 	if (size_type == UNIT)
				ui_Size_x_set(ui_unit_x(low3, high3))                           	{
				ui_NodeAlloc()                                                 		layout_position= ...
			}                                                                       		layout_size = ...
                                                                                                		if (not visible)
			                                                                        			return nil
                                                                                                		implicit_flags |= UI_ALLOW_VIOLATIONS;
			                                                                        	}
			                                                                        	....
			                                                                        	node->layout_position = layout_position;	
			                                                                        	node->layout_size = layout_position;	
			                                                                        	....
			                                                                        }

The following table describes the current state of what layout positions we have mapped out, taking into account only our size types.

		 LAYOUT POSITION and ASSERTIONS
	       	+---------------------------------------------------------------------------
          PIXEL |DEFINED(parent == CHILDSUM)
	       	+---------------------------------------------------------------------------
           TEXT	|DEFINED(parent == CHILDSUM)
	       	+---------------------------------------------------------------------------
    PERC_PARENT	|ASSERT(parent != CHILDSUM)
	       	+---------------------------------------------------------------------------
       CHILDSUM	|DEFINED(parent == CHILDSUM)
	       	+---------------------------------------------------------------------------
           UNIT	|DEFINED(always) and ASSERT(parent != CHILDSUM) and ASSERT(viewable_interval)
	       	+---------------------------------------------------------------------------

If we in addition to our sizes also consider the possibility of FLOATING_AXIS, we get the updated table 

		 LAYOUT POSITION and ASSERTIONS
	       	+------------------------------------------------------------------------------------------------------+
          PIXEL | 1. DEFINED(parent == CHILDSUM) && ASSERT(!FLOATING)
	  	| 2. DEFINED(FLOATING) && ASSERT(parent != CHILDSUM) 
	       	+------------------------------------------------------------------------------------------------------+
           TEXT	| 1. DEFINED(parent == CHILDSUM) && ASSERT(!FLOATING)
	   	| 2. DEFINED(FLOATING) && ASSERT(parent != CHILDSUM)
	       	+------------------------------------------------------------------------------------------------------+
    PERC_PARENT	| 1. DEFINED(FLOATING) && ASSERT(parent != CHILDSUM)
	       	+------------------------------------------------------------------------------------------------------+
       CHILDSUM	| 1. DEFINED(parent == CHILDSUM) && ASSERT(!FLOATING)
       		| 2. DEFINED(FLOATING) && ASSERT(parent != CHILDSUM)
	       	+------------------------------------------------------------------------------------------------------+
           UNIT	| 1. DEFINED(always) and ASSERT(parent != CHILDSUM) and ASSERT(viewable_interval) and ASSERT(!FLOATING)
	       	+------------------------------------------------------------------------------------------------------+

Now we have positions defined for each size type, and the requirements needed. We suspect that the 5 size types together with the FLOATING flag
should be able to express a decent amount of widgets for the moment. One ambiguous question remains: How are we to interpret a floating position?.
Before we answer that question, we entertain the idea of having CHILDSUM so simply be a sizing rule, and not also a child position enforcing rule.

If we seperate those to ideas, we immediately conclude that every node needs an additional value, child_layout_axis. It now follows naturally that
every node can define a layout position rule for its children, or, in the case of children being of size type unit, no layout rule. We discuss the
pros and cons of this new approach. First we update the tables:

		 LAYOUT POSITION and ASSERTIONS
	       	+---------------------------------------------------------------------------
          PIXEL |DEFINED(parent.child_layout_axis != COUNT)
	       	+---------------------------------------------------------------------------
           TEXT	|DEFINED(parent.child_layout_axis != COUNT)
	       	+---------------------------------------------------------------------------
    PERC_PARENT	|DEFINED(parent.child_layout_axis != COUNT) and ASSERT(parent != CHILDSUM)
	       	+---------------------------------------------------------------------------
       CHILDSUM	|DEFINED(parent.child_layout_axis != COUNT)
	       	+---------------------------------------------------------------------------
           UNIT	|DEFINED(always) and ASSERT(parent != CHILDSUM) and ASSERT(parent.child_layout_axis == COUNT) and ASSERT(viewable_interval)
	       	+---------------------------------------------------------------------------

If we in addition to our sizes also consider the possibility of FLOATING_AXIS, we get the updated table 

		 LAYOUT POSITION and ASSERTIONS
	       	+------------------------------------------------------------------------------------------------------+
          PIXEL | 1. DEFINED(parent.child_layout_axis != COUNT)
	  	| 2. DEFINED(FLOATING) && ASSERT(parent != CHILDSUM) 
	       	+------------------------------------------------------------------------------------------------------+
           TEXT	| 1. DEFINED(parent.child_layout_axis != COUNT)
	   	| 2. DEFINED(FLOATING) && ASSERT(parent != CHILDSUM)
	       	+------------------------------------------------------------------------------------------------------+
    PERC_PARENT	| 1. DEFINED(parent.child_layout_axis != COUNT) and ASSERT(parent != CHILDSUM)
	       	+------------------------------------------------------------------------------------------------------+
       CHILDSUM	| 1. DEFINED(parent.child_layout_axis != COUNT)
       		| 2. DEFINED(FLOATING) && ASSERT(parent != CHILDSUM)
	       	+------------------------------------------------------------------------------------------------------+
           UNIT	| 1. DEFINED(always) and ASSERT(parent != CHILDSUM) and ASSERT(parent.child_layout_axis == COUNT) and ASSERT(viewable_interval) and ASSERT(!FLOATING)
	       	+------------------------------------------------------------------------------------------------------+

For a unit of PIXEL, TEXT, PERC_PARENT or CHILDSUM, this new approach does not change much; if FLOATING is set, we simply skip laying out the node, 
and move onto the next child. If the flag isn't set, the parent should have a valid axis set as its child layout axis.  Furthermore, the ds_Assertions
still hold regarding CHILDSUM, but they can of course still be removed, as this only affects visual rendering. For the UNIT case, we have an added
ds_Assertion that requires the parent's child layout axis to be invalid, since UNIT enforces its own positioning. we can make a case for this to be a
real ds_Assertion, as in what world would we want to mix UNITS and compact non-UNITS.

Thinking a little more about some of these ds_Assertions, it would be more applicable to call them "ignore", as these hypothetical worlds will probably
prop up and bite us in the ass otherwise. So the better approach may simply be to allow the combinations and just ignore layouts for node specific
flags/size types. The updated table becomes 

		 LAYOUT POSITION and ASSERTIONS
	       	+------------------------------------------------------------------------------------------------------+
          PIXEL | 1. DEFINED(parent.child_layout_axis != COUNT)
	  	| 2. DEFINED(FLOATING) && ASSERT(parent != CHILDSUM) 
	       	+------------------------------------------------------------------------------------------------------+
           TEXT	| 1. DEFINED(parent.child_layout_axis != COUNT)
	   	| 2. DEFINED(FLOATING) && ASSERT(parent != CHILDSUM)
	       	+------------------------------------------------------------------------------------------------------+
    PERC_PARENT	| 1. DEFINED(parent.child_layout_axis != COUNT) and ASSERT(parent != CHILDSUM)
	       	+------------------------------------------------------------------------------------------------------+
       CHILDSUM	| 1. DEFINED(parent.child_layout_axis != COUNT)
       		| 2. DEFINED(FLOATING) && ASSERT(parent != CHILDSUM)
	       	+------------------------------------------------------------------------------------------------------+
           UNIT	| 1. DEFINED(always) and ASSERT(parent != CHILDSUM) and IGNORE(parent.child_layout_axis) and ASSERT(viewable_interval) and ASSERT(!FLOATING)
	       	+------------------------------------------------------------------------------------------------------+

With a more clear pixture of how positions and sizes are defined, we return to the phase table of when and where we can calulate sizes and positions.

Sizes:		(semantic unit)		(pixel unit)					(pixel unit)
		SEMANTIC PHASE		LAYOUT PHASE					SOLVER_PHASE				
	       	+-----------------------+-----------------------------------------------+-------------------------------------------------------------------------------+
          PIXEL |px   			|lay_size = px 					|sol_size = solve_according_to_context						|
           TEXT	|text_px		|lay_size = text_px				|sol_size = solve_according_to_context						|
    PERC_PARENT	|sem_size = perc     	|lay_size = perc(parent->lay_size)		|sol_size = perc(parent->pixel_size, sem_size)					|
       CHILDSUM	|0.0f			|lay_size = sum(lay_size_child) 		|sol_size = solve_according_to_context						|
           UNIT	|sem_size = unit	|lay_size = unit(parent->lay_size, sem_size) 	|sol_size = unit(parent->pixel_size, sem_size) 					|
		+-----------------------+-----------------------------------------------+-------------------------------------------------------------------------------+

Positions:	
		SEMANTIC PHASE		LAYOUT PHASE					SOLVER_PHASE				
	       	+-----------------------+-----------------------------------------------+-------------------------------------------------------------------------------+
      FLOATING |px (abs)		|lay_pos = px (abs)				|sol_pos  = px (abs)								|
      UNIT     |perc_intv (rel)  	|lay_pos = px (rel?) (perc_intv)		|sol_size = px (abs) (perc_intv)						|
      AXIS     |X     			|lay_pos = px (rel?)compact_layout 		|sol_size = px (abs) solve_according_to_context					|
		+-----------------------+-----------------------------------------------+-------------------------------------------------------------------------------+

================================= RENDERING ==================================

When creating a node, we can immediately determine the draw bucket it will go using its depth, layer and texture. While we do not have to sort these keys, and instead
defer that to the renderer, we do wish to use the hash our buckets for quick lookups.

	DRAW KEY: { Depth(n): Layer(2) : Texture(m) }

		Layer: 00 - Text
		Layer: 01 - Interactable nodes
		Layer: 10 - Visual/Padding	(Used in profiler, timeline unit lines are on the same depth as the interactable task nodes)

=========================== HANDLING TEXT ===========================

Here are some use cases we must support:

	Line Text: In most cases we wish to display text on a single line, or within a single line box. In that case, we

		( Positioning )
		- align text left/right/center 	(X)
		- align text top/bottom/center 	(Y)
		- pad text from semantic position (X)
		- pad text from semantic position (Y)

		( Sizing )
		- Calculate the node layout size (X) according to the text string
		- Truncate the node layout size (X) according to requirements

		( Rendering )
		- Fade text at boundary (X) (If wanted)
		- Fade text at boundary (Y) (If wanted)

In order to determine the position and size of the text to display, (and possibly the node's size itself)
we need several values. The process looks like

(1) IMMEDIATE_SIZE_CALCULATIONS:	
	
	(node->size_type[X] == TEXT || node->size_type[T] == TEXT)
 	=> 
	{ 
		node->textLayout = textLayout_calulcations
		node->size[0] = (node->size_type[X] == TEXT)
			? text_size[0]
			: normal_size_calculation(X)
		node->size[1] = (node->size_type[Y] == TEXT)
			? text_size[1]
			: normal_size_calculation(Y)
			
	}

	(node->size_type[X] != TEXT && node->size_type[T] != TEXT)
 	=> 
	{ 
		node->textLayout = NULL;
		node->flag = FLAG_TEXT_LAYOUT_POSTPONED
		node->size = normal_size_calculation(X) 
		node->size = normal_size_calculation(Y) 
	}

(2) VIOLATION_SOLVING:
	if (TEXT_ATTACHED && (node->size_type[X] == TEXT || node->size_type[T] == TEXT) && node_violation)
		node->flag |= FLAG_TEXT_LAYOUT_POSTPONED

(3) POSITIONING
	if (TEXT_ATTACHED && FLAG_TEXT_LAYOUT_POSTPONED)
		line_width = (FLAG_TEXT_ALLOW_LINE_OVERFLOW)
			   ? F32_INFINITY
			   : f32_max(node->pixel_size[0] - 2.0f-pad[0], 0.0f);
		node->textLayout = textLayout_calulcations
	
	

=========================== INTERACTIVITY API AND NODE vs. RECURSIVE INTERACTIVITY ===========================

	--- api ---

	Suppose for a moment that that each node has a set of local interactions (inter_local) and a set of
	recursive interactions (inter_rec) inherited from some of its ancestors. As an example, consider a timeline
	with a set of rows consisting of clickable nodes. If we wish to be able to drag the timeline, then we let
	the timeline and all of its decendants store a recursive interaction DRAGGABLE_X. Since each node within
	the rows are clickable, they store CLICKABLE in its local interactions.

	local interactions can easily be set using the ui_Flags, and we can act upon the as soon as the node allocation
	is done. Recursive interactions must somehow be set in a way to differentiate them from local interactions; in
	our example, it is set at the creation of the timeline, and we do not check it (singular!) until the timeline
	is complete. Any recursive interactions happending to any affected nodes set the recursive interaction_node.
	Thus, we can view recursive interactions as a triple (root, interaction, inter_node) with usage

		create (root, interaction, inter_node) => { create intermediate nodes ... } => check inter_node 
	
	A helper could be created to help with this:

		ui_node_inter_rec_alloc_f(FLAGS, INTER_FLAGS, format, ...) => ui_NodeAlloc(FLAGS, INTER_FLAGS, id)

Text input handling
===================

Context: The ui aliases some ui_TextInput state owned by some external system. 
Whenever a text operation happens, the ui will in its command queue execute these
operation on that alias. We show issues with this approach and develop a better
solution.


Issue 1: (aliasing ui_TextInput in reallocatable address space)
----------------------------------------------------------------
Suppose we have nodes in some subsystem where each node contains a ui_TextInput
struct. Furthermore, suppose that the subsystem is growable and may reallocate the
node's address. If we only provide the ui an alias of the node's ui_TextInput when
focus is gained, the alias will be invalidated on a future reallocation.


Solution:
---------
We reset the ui's ui_TextInput alias each frame: 

	ui_TextInputPush(&subsystem_node.ui_TextInput)
	ui_NodeAlloc()		<----- ui alises ui_TextInput if node focused 
	ui_TextInputPop()


Issue 2: (Freeing aliased ui_TextInput)
----------------------------------------
Suppose we have nodes in some subsystem where each node contains a ui_TextInput
struct. When the node is being focused, the ui aliases the ui_TextInput struct
to modify the text edit state. It follows that the subsystem node's lifetime MUST
AT LEAST extent 1 ui frame into the future. The following example illustrates the
aliasing:

FRAME N: 
	--- UI_FRAME ---
	ui_TextInputPush(&subsystem_node.ui_TextInput)
	ui_NodeAlloc()		<----- ui alises ui_TextInput if node focused 
	ui_TextInputPop()
	(...)
	--- SUBSYSTEM_FRAME ---
	(...)
	subsystem_node_free(subsystem_node)	<----- the subsystem determines that it should free the
						       aliased node, invalidating the ui's ui_TextInput
						       alias.
FRAME N+1:
	--- UI_FRAME ---
	ui_process_commands() <---- text_ops executed; WILL modify invalid ui_TextInput! 


Workaround 2:
-------------
We first give some context to when and where text editting will happen.  The common 
case for text editing will be editting small utf32 strings; for example, when 
modifying u64/i64/f32 values. In those cases we only require buffers containing 32 
codepoints. Issue 2 would most certainly come up here; suppose we are editing some
level editor node's positional value, and while editing we remove the node. We are
now keeping an invalid ui_TextInput alias going into the next frame.

Since the common case is modifying / creating small strings, and since we can only
edit one text at a time, we may as well just store an internal buffer in the ui
that is used if no external ui_TextInput is provided. The only downside to
this is that when we lose focus of a node that "owns" the internal buffer, the
buffer contents is lost. 

We can still provide the ui system an external ui_TextInput, we just have to make
sure in those few cases that the address stays valid for the remainder to the last
frame it was provided. This works fine for cases like per-window command consoles
whose address stays constant for the lifetime of the window.
*/

#ifdef __cplusplus
} 
#endif

#endif
