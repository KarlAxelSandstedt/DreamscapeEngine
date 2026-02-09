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

#include <string.h>

#include "ui_local.h"
#include "ds_font.h"
#include "hash_map.h"

#define INITIAL_UNIT_COUNT	1024
#define INITIAL_HASH_COUNT	1024

//#define UI_DEBUG_FLAGS	UI_DRAW_BORDER
#define UI_DEBUG_FLAGS		UI_FLAG_NONE 

DEFINE_STACK(ui_Size);
DEFINE_STACK(ui_TextSelection);
DEFINE_STACK(utf32);

static void ui_CommandStaticAssert(void)
{
	ds_StaticAssert(UI_CMD_LAYER_BITS
			+ UI_CMD_DEPTH_BITS
			+ UI_CMD_TEXTURE_BITS
			== 32, "ui_cmd definitions should span whole 32 bits");

	//TODO Show no overlap between masks
	ds_StaticAssert((UI_CMD_LAYER_MASK & UI_CMD_DEPTH_MASK) == 0, "UI_CMD_*_MASK values should not overlap");
	ds_StaticAssert((UI_CMD_LAYER_MASK & UI_CMD_TEXTURE_MASK) == 0, "UI_CMD_*_MASK values should not overlap");
	ds_StaticAssert((UI_CMD_DEPTH_MASK & UI_CMD_TEXTURE_MASK) == 0, "UI_CMD_*_MASK values should not overlap");

	ds_StaticAssert(UI_CMD_LAYER_MASK
			+ UI_CMD_DEPTH_MASK
			+ UI_CMD_TEXTURE_MASK
			== U32_MAX, "sum of ui_cmd masks should be U32");

	ds_StaticAssert(TEXTURE_COUNT <= (UI_CMD_TEXTURE_MASK >> UI_CMD_TEXTURE_LOW_BIT), "texture mask must be able to contain all texture ids");
}

/* Set to current ui being operated on */
struct ui *g_ui = NULL;
/* ui allocator  */

u32	cmd_ui_text_op;
u32	cmd_ui_popup_build;

void ds_UiApiInit(void)
{
	CmdFunctionRegister(Utf8Inline("ui_TimelineDrag"), 4, &ui_TimelineDrag);
	CmdFunctionRegister(Utf8Inline("ui_TextInputModeEnable"), 2, &ui_TextInputModeEnable);
	CmdFunctionRegister(Utf8Inline("ui_TextInputFlush"), 1, &ui_TextInputFlush);
	CmdFunctionRegister(Utf8Inline("ui_TextInputModeDisable"), 1, &ui_TextInputModeDisable);
	cmd_ui_text_op = CmdFunctionRegister(Utf8Inline("ui_TextOp"), 3, &ui_TextOp).index;
	cmd_ui_popup_build = CmdFunctionRegister(Utf8Inline("ui_PopupBuild"), 2, &ui_PopupBuild).index;
}

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
		, const f32 text_pad_y)
{
	struct ui_Visual visual = { 0 };

	Vec4Copy(visual.background_color, background_color);
	Vec4Copy(visual.border_color, border_color);
	Vec4Copy(visual.gradient_color[0], gradient_color[0]);
	Vec4Copy(visual.gradient_color[1], gradient_color[1]);
	Vec4Copy(visual.gradient_color[2], gradient_color[2]);
	Vec4Copy(visual.gradient_color[3], gradient_color[3]);
	Vec4Copy(visual.sprite_color, sprite_color);
	visual.pad = pad;
	visual.edge_softness = edge_softness;
	visual.corner_radius = corner_radius;
	visual.border_size = border_size;
	visual.font = font;
	visual.text_alignment_x = text_alignment_x;
	visual.text_alignment_y = text_alignment_y;
	visual.text_pad_x = text_pad_x;
	visual.text_pad_y = text_pad_y;

	return visual;
}

struct ui_TextSelection ui_TextSelectionEmpty(void)
{
	struct ui_TextSelection sel = { 0 };
	return sel;
}

struct ui_TextInput  text_edit_stub = { 0 };
struct ui_TextInput *text_edit_stub_ptr(void)
{
	return &text_edit_stub;
}

struct ui *ui_Alloc(void)
{
	ds_StaticAssert(sizeof(struct ui_Size) == 16, "Expected size");

	struct memSlot mem_slot;
	struct ui *ui = ds_Alloc(&mem_slot, sizeof(struct ui), NO_HUGE_PAGES);
	ui->mem_slot = mem_slot;
	
	memset(ui, 0, sizeof(struct ui));
	ui->node_hierarchy = hi_Alloc(NULL, INITIAL_UNIT_COUNT, struct ui_Node, GROWABLE);
	ui->node_map = HashMapAlloc(NULL, U16_MAX, U16_MAX, GROWABLE);
	ui->bucket_pool = PoolAlloc(NULL, 64, struct ui_DrawBucket, GROWABLE);
	ui->bucket_list = dll_Init(struct ui_DrawBucket);
	ui->bucket_map = HashMapAlloc(NULL, 128, 128, GROWABLE);
	ui->event_pool = PoolAlloc(NULL, 32, struct dsEvent, GROWABLE);
	ui->event_list = dll_Init(struct dsEvent);
	ui->frame = 0;
	ui->root = HI_ROOT_STUB_INDEX;
	ui->node_count_prev_frame = 0;
	ui->node_count_frame = 0;
	ui->mem_frame_arr[0] = ArenaAlloc(64*1024*1024);
	ui->mem_frame_arr[1] = ArenaAlloc(64*1024*1024);
	ui->mem_frame = ui->mem_frame_arr + (ui->frame & 0x1);
	ui->stack_parent = stack_u32Alloc(NULL, 32, GROWABLE);
	ui->stack_sprite = stack_u32Alloc(NULL, 32, GROWABLE);
	ui->stack_font = stack_ptrAlloc(NULL, 8, GROWABLE);
	ui->stack_external_text_input = stack_ptrAlloc(NULL, 8, GROWABLE);
	ui->stack_flags = stack_u64Alloc(NULL, 16, GROWABLE);
	ui->stack_recursive_interaction_flags = stack_u64Alloc(NULL, 16, GROWABLE);
	ui->stack_external_text = stack_utf32Alloc(NULL, 8, GROWABLE);
	ui->stack_external_text_layout = stack_ptrAlloc(NULL, 8, GROWABLE);
	ui->stack_floating_node = stack_u32Alloc(NULL, 32, GROWABLE);
	ui->stack_floating_depth = stack_u32Alloc(NULL, 32, GROWABLE);
	ui->stack_floating[AXIS_2_X] = stack_f32Alloc(NULL, 16, GROWABLE);
	ui->stack_floating[AXIS_2_Y] = stack_f32Alloc(NULL, 16, GROWABLE);
	ui->stack_ui_Size[AXIS_2_X] = stack_ui_SizeAlloc(NULL, 16, GROWABLE);
	ui->stack_ui_Size[AXIS_2_Y] = stack_ui_SizeAlloc(NULL, 16, GROWABLE);
	ui->stack_gradient_color[BOX_CORNER_BR] = stackVec4Alloc(NULL, 16, GROWABLE);
	ui->stack_gradient_color[BOX_CORNER_TR] = stackVec4Alloc(NULL, 16, GROWABLE);
	ui->stack_gradient_color[BOX_CORNER_TL] = stackVec4Alloc(NULL, 16, GROWABLE);
	ui->stack_gradient_color[BOX_CORNER_BL] = stackVec4Alloc(NULL, 16, GROWABLE);
	ui->stack_viewable[AXIS_2_X] = stack_intvAlloc(NULL, 8, GROWABLE);
	ui->stack_viewable[AXIS_2_Y] = stack_intvAlloc(NULL, 8, GROWABLE);
	ui->stack_child_layout_axis = stack_u32Alloc(NULL, 16, GROWABLE);
	ui->stack_background_color = stackVec4Alloc(NULL, 16, GROWABLE);
	ui->stack_border_color = stackVec4Alloc(NULL, 16, GROWABLE);
	ui->stack_sprite_color = stackVec4Alloc(NULL, 16, GROWABLE);
	ui->stack_edge_softness = stack_f32Alloc(NULL, 16, GROWABLE);
	ui->stack_corner_radius = stack_f32Alloc(NULL, 16, GROWABLE);
	ui->stack_border_size = stack_f32Alloc(NULL, 16, GROWABLE);
	ui->stack_text_alignment_x = stack_u32Alloc(NULL, 8, GROWABLE);
	ui->stack_text_alignment_y = stack_u32Alloc(NULL, 8, GROWABLE);
	ui->stack_text_pad[AXIS_2_X] = stack_f32Alloc(NULL, 8, GROWABLE);
	ui->stack_text_pad[AXIS_2_Y] = stack_f32Alloc(NULL, 8, GROWABLE);
	ui->stack_fixed_depth = stack_u32Alloc(NULL, 16, GROWABLE);
	ui->stack_pad = stack_f32Alloc(NULL, 8, GROWABLE);
	ui->frame_stack_text_selection = stack_ui_TextSelectionAlloc(NULL, 128, GROWABLE);

	ui->inter.node_hovered = Utf8Empty();
	ui->inter.text_edit_mode = 0;
	ui->inter.text_edit_id = Utf8Empty();
	ui->inter.text_edit = text_edit_stub_ptr();

	/* setup root stub values */
	stack_u32Push(&ui->stack_parent, HI_ROOT_STUB_INDEX);
	struct ui_Node *stub = hi_Address(&ui->node_hierarchy, HI_ROOT_STUB_INDEX);
	stub->id = Utf8Empty();
	stub->semantic_size[AXIS_2_X] = ui_SizePixel(0.0f, 0.0f);
	stub->semantic_size[AXIS_2_Y] = ui_SizePixel(0.0f, 0.0f);
	stub->child_layout_axis = AXIS_2_X;
	stub->depth = 0;
	stub->flags = UI_FLAG_NONE;
	stub->inter = 0;
	stub->inter_recursive_flags = 0;
	stub->inter_recursive_mask = 0;
	stub->last_frame_touched = U64_MAX;

	struct ui_Node *orphan_root = hi_Address(&ui->node_hierarchy, HI_ORPHAN_STUB_INDEX);
	orphan_root->id = Utf8Empty();
	orphan_root->semantic_size[AXIS_2_X] = ui_SizePixel(0.0f, 0.0f);
	orphan_root->semantic_size[AXIS_2_Y] = ui_SizePixel(0.0f, 0.0f);
	orphan_root->child_layout_axis = AXIS_2_X;
	orphan_root->depth = 0;
	orphan_root->flags = UI_FLAG_NONE;
	orphan_root->inter = 0;
	orphan_root->inter_recursive_flags = 0;
	orphan_root->inter_recursive_mask = 0;
	orphan_root->last_frame_touched = U64_MAX;

	stack_u64Push(&ui->stack_flags, UI_FLAG_NONE);
	stack_u64Push(&ui->stack_recursive_interaction_flags, UI_FLAG_NONE);

	/* setup stub bucket */
	struct slot slot = PoolAdd(&ui->bucket_pool);
	dll_Append(&ui->bucket_list, ui->bucket_pool.buf, slot.index);
	ui->bucket_cache = slot.index;
	struct ui_DrawBucket *bucket = slot.address;
	bucket->cmd = 0;
	bucket->count = 0;

	return ui;
}

void ui_Dealloc(struct ui *ui)
{
	ArenaFree(ui->mem_frame_arr + 0);
	ArenaFree(ui->mem_frame_arr + 1);

	stack_ui_TextSelectionFree(&ui->frame_stack_text_selection);
	stack_f32Free(&ui->stack_pad);
	stack_u64Free(&ui->stack_flags);
	stack_u64Free(&ui->stack_recursive_interaction_flags);
	stack_utf32Free(&ui->stack_external_text);
	stack_ptrFree(&ui->stack_external_text_layout);
	stack_ptrFree(&ui->stack_external_text_input);
	stack_u32Free(&ui->stack_text_alignment_x);
	stack_u32Free(&ui->stack_text_alignment_y);
	stack_f32Free(ui->stack_text_pad + AXIS_2_X);
	stack_f32Free(ui->stack_text_pad + AXIS_2_Y);
	stack_f32Free(&ui->stack_edge_softness);
	stack_f32Free(&ui->stack_corner_radius);
	stack_f32Free(&ui->stack_border_size);
	stack_u32Free(&ui->stack_parent);
	stack_u32Free(&ui->stack_sprite);
	stack_ptrFree(&ui->stack_font);
	stack_f32Free(ui->stack_floating + AXIS_2_X);
	stack_f32Free(ui->stack_floating + AXIS_2_Y);
	stack_ui_SizeFree(ui->stack_ui_Size + AXIS_2_X);
	stack_ui_SizeFree(ui->stack_ui_Size + AXIS_2_Y);
	stackVec4Free(ui->stack_gradient_color + BOX_CORNER_BR);
	stackVec4Free(ui->stack_gradient_color + BOX_CORNER_TR);
	stackVec4Free(ui->stack_gradient_color + BOX_CORNER_TL);
	stackVec4Free(ui->stack_gradient_color + BOX_CORNER_BL);
	stack_intvFree(ui->stack_viewable + AXIS_2_X);
	stack_intvFree(ui->stack_viewable + AXIS_2_Y);
	stack_u32Free(&ui->stack_child_layout_axis);
	stackVec4Free(&ui->stack_background_color);
	stackVec4Free(&ui->stack_border_color);
	stackVec4Free(&ui->stack_sprite_color);
	stack_u32Free(&ui->stack_floating_node);
	stack_u32Free(&ui->stack_floating_depth);
	stack_u32Free(&ui->stack_fixed_depth);
	HashMapFree(&ui->node_map);
	PoolDealloc(&ui->event_pool);
	PoolDealloc(&ui->bucket_pool);
	HashMapFree(&ui->bucket_map);
	hi_Dealloc(&ui->node_hierarchy);
	ds_Free(&ui->mem_slot);
	if (g_ui == ui)
	{
		g_ui = NULL;
	}
}

static void ui_DrawBucketAddNode(const u32 cmd, const u32 index)
{
	struct ui_DrawBucket *bucket = PoolAddress(&g_ui->bucket_pool, g_ui->bucket_cache);
	if (bucket->cmd != cmd)
	{
		u32 bi = HashMapFirst(&g_ui->bucket_map, cmd);
		for (; bi != HASH_NULL; bi = HashMapNext(&g_ui->bucket_map, bi))
		{
			bucket = PoolAddress(&g_ui->bucket_pool, bi);
			if (bucket->cmd == cmd)
			{
				break;
			}
		}

		if (bi == HASH_NULL)
		{
			struct slot slot = PoolAdd(&g_ui->bucket_pool);
			bi = slot.index;
			HashMapAdd(&g_ui->bucket_map, cmd, bi);
			dll_Append(&g_ui->bucket_list, g_ui->bucket_pool.buf, bi);
			bucket = slot.address;
			bucket->cmd = cmd;
			bucket->count = 0;
			bucket->list = NULL;
		}
	}

	struct ui_DrawNode *tmp = bucket->list;
	bucket->list = ArenaPush(g_ui->mem_frame, sizeof(struct ui_DrawNode));
	bucket->list->next = tmp;
	bucket->list->index = index;
	bucket->count += 1;
}

void ui_Set(struct ui *ui)
{
	g_ui = ui;
}

static struct slot ui_RootF(const char *format, ...)
{
	va_list args;
	va_start(args, format);
	utf8 id = Utf8FormatVariadic(g_ui->mem_frame, format, args);
	va_end(args);

	return ui_NodeAlloc(UI_FLAG_NONE, &id);
}

static void ui_NodeDealloc(const struct hi *node_hierarchy, const u32 index, void *data)
{
	const struct ui_Node *node = hi_Address(node_hierarchy, index);
	if ((node->flags & UI_NON_HASHED) == 0)
	{
		//fprintf(stderr, "pruning hashed orphan %s\n",(char*) ((struct ui_Node *) node)->id.buf);
		HashMapRemove(&g_ui->node_map, node->hash, index);
	}
}

void ui_TextInputModeEnable(void)
{
	const utf8 id = g_queue->cmd_exec->arg[0].utf8;
	struct ui_TextInput *text_edit = g_queue->cmd_exec->arg[1].ptr;

	struct ui_Node *node = ui_NodeLookup(&g_ui->inter.text_edit_id).address;
	if (node)
	{
		node->inter &= ~UI_INTER_FOCUS;
		node->inter |= UI_INTER_FOCUS_OUT;
		g_ui->inter.text_edit->focused = 0;
	}
	else
	{
		ds_WindowTextInputModeEnable();
	}

	node = ui_NodeLookup(&id).address;
	if (node->flags & UI_TEXT_EDIT_COPY_ON_FOCUS)
	{
		const u32 buflen = sizeof(g_ui->inter.text_internal_buf) / sizeof(u32);
		u32 *buf = g_ui->inter.text_internal_buf;
		node->input.text = Utf32CopyBuffered(buf, buflen, node->input.text);
		ds_Assert(&node->input == text_edit);
	}

	g_ui->inter.text_edit_mode = 1;
	g_ui->inter.text_edit_id = Utf8Copy(g_ui->mem_frame, id);
	g_ui->inter.text_edit = text_edit;
	g_ui->inter.text_edit->focused = 1;

	text_edit->cursor = (text_edit->cursor <= text_edit->text.len) 
		? text_edit->cursor
		: text_edit->text.len;

	text_edit->mark = (text_edit->mark <= text_edit->text.len) 
		? text_edit->mark
		: text_edit->text.len;
}

void ui_TextInputModeDisable(void)
{
	const utf8 id = g_queue->cmd_exec->arg[0].utf8;
	if (Utf8Equivalence(id, g_ui->inter.text_edit_id))
	{
		ds_WindowTextInputModeDisable();
		struct ui_Node *node = ui_NodeLookup(&g_ui->inter.text_edit_id).address;
		if (node)
		{
			node->inter &= ~UI_INTER_FOCUS;
			node->inter |= UI_INTER_FOCUS_OUT;
		}

		g_ui->inter.text_edit_mode = 0;
		g_ui->inter.text_edit_id = Utf8Empty();
		g_ui->inter.text_edit->focused = 0;
		g_ui->inter.text_edit = text_edit_stub_ptr();
	}
}

void ui_TextInputFlush(void)
{
	const utf8 id = g_queue->cmd_exec->arg[0].utf8;
	if (Utf8Equivalence(g_ui->inter.text_edit_id, id))
	{
		g_ui->inter.text_edit->text.len = 0;
		g_ui->inter.text_edit->cursor = 0;
		g_ui->inter.text_edit->mark = 0;
	}
}

struct ui_TextInput ui_TextInputEmpty(void)
{
	return (struct ui_TextInput) { .focused = 0, .cursor = 0, .mark = 0, .text = Utf32Empty() };
}

struct ui_TextInput ui_TextInputBuffered(u32 buf[], const u32 len)
{
	return (struct ui_TextInput) { .focused = 0, .cursor = 0, .mark = 0, .text = Utf32Buffered(buf, len) };
}

struct ui_TextInput ui_TextInputAlloc(struct arena *mem, const u32 max_len)
{
	utf32 text = Utf32Alloc(mem, max_len);
	return (text.max_len)
		? (struct ui_TextInput) { .focused = 0, .cursor = 0, .mark = 0, .text = text }
		: ui_TextInputEmpty();
}

static void ui_ChildsumLayoutSizeAndPruneNodes(void)
{
	ArenaPushRecord(g_ui->mem_frame);

	//stack_u32 stackFree = stack_u32Alloc(g_ui->mem_frame, g_ui->node_count_prev_frame, 0);
	stack_ptr stack_childsum_x = stack_ptrAlloc(g_ui->mem_frame, g_ui->node_count_frame, 0);
	stack_ptr stack_childsum_y = stack_ptrAlloc(g_ui->mem_frame, g_ui->node_count_frame, 0);

	ArenaPushRecord(g_ui->mem_frame);
	struct hiIterator it = hi_IteratorAlloc(g_ui->mem_frame, &g_ui->node_hierarchy, g_ui->root);
	while(it.count)
	{
		//const u32 potential_next = hi_IteratorPeek(&it);
		//struct ui_Node *node = hi_Address(&g_ui->node_hierarchy, potential_next);
		//if (node->last_frame_touched != g_ui->frame)
		//{
		//	//fprintf(stderr, "FRAME UNTOUCHED:%lu\tID:%s\tKEY:%u\tINDEX:%u\n", g_ui->frame, node->id.buf, node->hash, potential_next);
		//	stack_u32Push(&stackFree, potential_next);
		//	hi_IteratorSkip(&it);
		//	continue;
		//}
		//hi_IteratorNextDf(&it);
		struct ui_Node *node = hi_Address(&g_ui->node_hierarchy, hi_IteratorNextDf(&it));

		if (node->semantic_size[AXIS_2_X].type == UI_SIZE_CHILDSUM)
		{
			stack_ptrPush(&stack_childsum_x, node);
		}

		if (node->semantic_size[AXIS_2_Y].type == UI_SIZE_CHILDSUM)
		{
			stack_ptrPush(&stack_childsum_y, node);
		}
	}
	ArenaPopRecord(g_ui->mem_frame);

	while (stack_childsum_y.next)
	{
		struct ui_Node *node = stack_ptrPop(&stack_childsum_y);
		node->layout_size[AXIS_2_Y] = 0.0f;
		struct ui_Node *child = NULL;
		for (u32 i = node->hi_first; i != HI_NULL_INDEX; i = child->hi_next)
		{
			child = hi_Address(&g_ui->node_hierarchy, i);
			node->layout_size[AXIS_2_Y] += child->layout_size[AXIS_2_Y];
		}
	}
	
	while (stack_childsum_x.next)
	{
		struct ui_Node *node = stack_ptrPop(&stack_childsum_x);
		node->layout_size[AXIS_2_X] = 0.0f;
		struct ui_Node *child = NULL;
		for (u32 i = node->hi_first; i != HI_NULL_INDEX; i = child->hi_next)
		{
			child = hi_Address(&g_ui->node_hierarchy, i);
			node->layout_size[AXIS_2_X] += child->layout_size[AXIS_2_X];
		}
	}

	ArenaPopRecord(g_ui->mem_frame);
}

static void ui_NodeSolveChildViolation(struct ui_Node *node, const enum axis_2 axis)
{
	if (!node->hi_child_count)
	{
		return; 
	}

	ArenaPushRecord(g_ui->mem_frame);
	struct ui_Node **child = ArenaPush(g_ui->mem_frame, node->hi_child_count * sizeof(struct ui_Node *));
	f32 *new_size = ArenaPush(g_ui->mem_frame, node->hi_child_count  * sizeof(f32));
	u32 *shrink = ArenaPush(g_ui->mem_frame, node->hi_child_count * sizeof(u32));
	f32 child_size_sum = 0.0f;
	u32 children_to_shrink = node->hi_child_count;
	u32 index = node->hi_first;

	u32 pad_fill_count = 0;
	u32 *pad_fill_index = ArenaPush(g_ui->mem_frame, node->hi_child_count * sizeof(u32));

	for (u32 i = 0; i < node->hi_child_count; ++i)
	{
		child[i] = hi_Address(&g_ui->node_hierarchy, index);

		new_size[i] = child[i]->layout_size[axis];
		//child_size_sum += child[i]->layout_size[axis];
		child_size_sum += (child[i]->flags & (UI_FLOATING_X << axis))
			? 0.0f
			: child[i]->layout_size[axis];

		const u32 child_is_pad_fill = !!(child[i]->flags & UI_PAD_FILL);
		if (child_is_pad_fill)
		{
			pad_fill_index[pad_fill_count++] = i;
		}

		if ((child[i]->flags & ((UI_FLOATING_X | UI_ALLOW_VIOLATION_X | UI_PERC_POSTPONED_X) << axis)) == 0)
		{
			shrink[i] =  1;
		}
		else
		{
			children_to_shrink -= 1;
			shrink[i] =  0;
		}
		index = child[i]->hi_next;
	}

	if (node->child_layout_axis != axis && (node->flags & (UI_ALLOW_VIOLATION_X << axis)) == 0)
	{
		for (u32 i = 0; i < node->hi_child_count; ++i)
		{
			const f32 perc = f32_max(child[i]->semantic_size[axis].strictness, f32_min(1.0f, child[i]->layout_size[axis] / node->layout_size[axis]));
			new_size[i] = (shrink[i])
				? child[i]->layout_size[axis] * perc
				: child[i]->layout_size[axis];
		}
	}
	else if (node->child_layout_axis == axis)
	{
		const f32 size_left = node->layout_size[axis] - child_size_sum;
		if (size_left < 0.0f)
		{
 			if ((node->flags & (UI_ALLOW_VIOLATION_X << axis)) == 0)
			{
				f32 child_perc_remain_after_shrink = node->layout_size[axis] / child_size_sum;

				while (1)
				{	
					/* sum of original sizes of children we may shrink again */
					f32 original_shrinkable_size = 0.0f;
					/* sum of new sizes of children we may NOT shrink again */
					f32 new_unshrinkable_size = 0.0f;
					u32 can_shrink_again_count = 0;
					for (u32 i = 0; i < node->hi_child_count; ++i)
					{
						if (shrink[i])
						{
							if (child[i]->semantic_size[axis].strictness < child_perc_remain_after_shrink)
							{
								new_size[i] = child[i]->layout_size[axis] * child_perc_remain_after_shrink;
								original_shrinkable_size += child[i]->layout_size[axis];
								can_shrink_again_count += 1;
							}
							else
							{
								new_size[i] = child[i]->layout_size[axis] * child[i]->semantic_size[axis].strictness;
								new_unshrinkable_size += new_size[i];
							}
						}
						else
						{
								new_unshrinkable_size += new_size[i];
						}
					}


					if (can_shrink_again_count == children_to_shrink)
					{
						break;
					}
					else if (!can_shrink_again_count || original_shrinkable_size < (node->layout_size[axis] - new_unshrinkable_size))
					{
						//force_shrink_all = 1;
						break;
					}

					children_to_shrink = can_shrink_again_count;
					child_perc_remain_after_shrink = (node->layout_size[axis] - new_unshrinkable_size) / original_shrinkable_size;
				}
			}
		}
		else
		{
			for (u32 i = 0; i < pad_fill_count; ++i)
			{
				new_size[pad_fill_index[i]] = size_left / pad_fill_count;
			}
		}
	}

	if (axis == AXIS_2_X)
	{
		//TODO clamp positions to pixels, (or something)
		for (u32 i = 0; i < node->hi_child_count; ++i)
		{
			if ((child[i]->flags & (UI_TEXT_ALLOW_OVERFLOW | UI_TEXT_ATTACHED)) == UI_TEXT_ATTACHED 
					&& (child[i]->layout_size[axis] != new_size[i]))
			{
				child[i]->flags |= UI_TEXT_LAYOUT_POSTPONED;
			}
			child[i]->layout_size[axis] = new_size[i];
		}
	}
	else
	{
		//TODO clamp positions to pixels, (or something)
		for (u32 i = 0; i < node->hi_child_count; ++i)
		{
			child[i]->layout_size[axis] = new_size[i];
		}
	}

	ArenaPopRecord(g_ui->mem_frame);
}

static void ui_SolveViolations(void)
{
	struct arena tmp = ArenaAlloc1MB();
	struct hiIterator it = hi_IteratorAlloc(&tmp, &g_ui->node_hierarchy, g_ui->root);
	while(it.count)
	{
		const u32 index = hi_IteratorNextDf(&it);
		struct ui_Node *node = hi_Address(&g_ui->node_hierarchy, index);

		ui_NodeSolveChildViolation(node, AXIS_2_X);
		ui_NodeSolveChildViolation(node, AXIS_2_Y);
	}
	ArenaFree1MB(&tmp);
}

static void ui_LayoutAbsolutePosition(void)
{
	struct arena tmp = ArenaAlloc1MB();
	struct hiIterator it = hi_IteratorAlloc(&tmp, &g_ui->node_hierarchy, g_ui->root);

	struct ui_Node *node = hi_Address(&g_ui->node_hierarchy, g_ui->root);
	node->pixel_position[0] = node->layout_position[0];
	node->pixel_position[1] = node->layout_position[1];
	node->pixel_size[0] = node->layout_size[0];
	node->pixel_size[1] = node->layout_size[1];
	node->pixel_visible[0] = intv_inline(node->pixel_position[0], node->pixel_position[0] + node->pixel_size[0]);
	node->pixel_visible[1] = intv_inline(node->pixel_position[1], node->pixel_position[1] + node->pixel_size[1]);

	while(it.count)
	{
		const u32 index = hi_IteratorNextDf(&it);
		node = hi_Address(&g_ui->node_hierarchy, index);

		//fprintf(stderr, "%s\n", (char *) node->id.buf);
		//Vec2Print("position", node->pixel_position);
		//Vec2Print("size", node->pixel_size);
		//Vec2Print("layout_position", node->layout_position);
		//Vec2Print("layout_size", node->layout_size);
		//fprintf(stderr, "visible area: [%f, %f] x [%f, %f]\n"
		//		,node->pixel_visible[AXIS_2_X].low
		//		,node->pixel_visible[AXIS_2_X].high
		//		,node->pixel_visible[AXIS_2_Y].low
		//		,node->pixel_visible[AXIS_2_Y].high);
		struct ui_Node *child = NULL;
		f32 child_layout_axis_offset = (node->child_layout_axis == AXIS_2_X) 
			? 0.0f
			: node->pixel_size[1];
		const u32 non_layout_axis = 1 - node->child_layout_axis;
		for (u32 next = node->hi_first; next != HI_NULL_INDEX; next = child->hi_next)
		{
			child = hi_Address(&g_ui->node_hierarchy, next);
			f32 new_child_layout_axis_offset = child_layout_axis_offset;

			if (child->flags & (UI_PERC_POSTPONED_X << node->child_layout_axis))
			{
				child->layout_position[node->child_layout_axis] = 0.0f;
				child->layout_size[node->child_layout_axis] = child->semantic_size[node->child_layout_axis].percentage * node->pixel_size[node->child_layout_axis];
			}
			else
			{
				if ((child->flags & (UI_FLOATING_X << node->child_layout_axis)) == 0)
				{
					new_child_layout_axis_offset = (node->child_layout_axis == AXIS_2_X)
					       ? child_layout_axis_offset + child->layout_size[AXIS_2_X]
					       : child_layout_axis_offset - child->layout_size[AXIS_2_Y];
				}
			}

			if (child->flags & (UI_PERC_POSTPONED_X << non_layout_axis))
			{
				child->layout_position[non_layout_axis] = 0.0f;
				child->layout_size[non_layout_axis] = child->semantic_size[non_layout_axis].percentage * node->pixel_size[non_layout_axis];
			}

			if (node->child_layout_axis == AXIS_2_X)
			{
				child->layout_position[AXIS_2_X] = ((child->flags & (UI_FLOATING_X | UI_PERC_POSTPONED_X)) || child->semantic_size[AXIS_2_X].type == UI_SIZE_UNIT)
					? child->layout_position[AXIS_2_X]
					: child_layout_axis_offset;

				child->layout_position[AXIS_2_Y] = (child->flags & UI_FLOATING_Y || child->semantic_size[AXIS_2_Y].type == UI_SIZE_UNIT)
				       	? child->layout_position[AXIS_2_Y]
			       		: 0.0f;
			}
			else
			{
				child->layout_position[AXIS_2_Y] = ((child->flags & (UI_FLOATING_Y | UI_PERC_POSTPONED_Y)) || child->semantic_size[AXIS_2_Y].type == UI_SIZE_UNIT)
					? child->layout_position[AXIS_2_Y]
					: child_layout_axis_offset - child->layout_size[AXIS_2_Y];

				child->layout_position[AXIS_2_X] = (child->flags & UI_FLOATING_X || child->semantic_size[AXIS_2_X].type == UI_SIZE_UNIT)
				       	? child->layout_position[AXIS_2_X]
			       		: 0.0f;
			}

			child_layout_axis_offset = new_child_layout_axis_offset;

			child->pixel_size[0] = child->layout_size[0];
			child->pixel_size[1] = child->layout_size[1];
			child->pixel_position[0] = (child->flags & UI_FIXED_X)
			       	? child->layout_position[0]
		       		: child->layout_position[0] + node->pixel_position[0];
			child->pixel_position[1] = (child->flags & UI_FIXED_Y)
			       	? child->layout_position[1]
		       		: child->layout_position[1] + node->pixel_position[1];

			child->pixel_visible[AXIS_2_X] = (child->flags & UI_FLOATING_X)
				? intv_inline(child->pixel_position[0], child->pixel_position[0] + child->pixel_size[0])
				: intv_inline(f32_max(child->pixel_position[0], node->pixel_visible[0].low),
					      f32_min(child->pixel_position[0] + child->pixel_size[0], node->pixel_visible[AXIS_2_X].high));
			child->pixel_visible[AXIS_2_Y] = (child->flags & UI_FLOATING_Y)
				? intv_inline(child->pixel_position[1], child->pixel_position[1] + child->pixel_size[1])
				: intv_inline(f32_max(child->pixel_position[1], node->pixel_visible[1].low),
					      f32_min(child->pixel_position[1] + child->pixel_size[1], node->pixel_visible[AXIS_2_Y].high));

			if (child->flags & UI_TEXT_LAYOUT_POSTPONED)
			{
				const f32 line_width = (child->flags & UI_TEXT_ALLOW_OVERFLOW)
					? F32_INFINITY
					: f32_max(0.0f, child->pixel_size[0] - 2.0f*child->text_pad[0]);
				child->layout_text = Utf32TextLayout(g_ui->mem_frame, &child->input.text, line_width, TAB_SIZE, child->font);
			}
		}
	}

	ArenaFree1MB(&tmp);
}

static void InterDebugPrint(const u64 inter)
{
	if (inter & UI_INTER_ACTIVE) { fprintf(stderr, "ACTIVE | "); }
	if (inter & UI_INTER_HOVER) { fprintf(stderr, "HOVER | "); }
	if (inter & UI_INTER_LEFT_CLICK) { fprintf(stderr, "LEFT_CLICK | "); }
	if (inter & UI_INTER_LEFT_DOUBLE_CLICK) { fprintf(stderr, "LEFT_DOUBLE_CLICK | "); }
	if (inter & UI_INTER_DRAG) { fprintf(stderr, "DRAG | "); }
	if (inter & UI_INTER_SCROLL) { fprintf(stderr, "SCROLL | "); }
	if (inter & UI_INTER_SELECT) { fprintf(stderr, "SELECT | "); }
	if (inter & UI_INTER_FOCUS) { fprintf(stderr, "FOCUS | "); }
}

static u64 ui_NodeSetInteractions(const struct ui_Node *node, const u64 inter_local_mask, const u64 inter_recursive_mask)
{
	u64 node_inter = UI_FLAG_NONE;
	u32 node_clicked = 0;
	u32 node_scrolled = 0;
	u32 node_selected = !!(node->inter & UI_INTER_SELECT);
	u32 node_hovered = !!(node->inter & UI_INTER_HOVER);
	u32 node_dragged = (!!(node->inter & UI_INTER_DRAG))*!(g_ui->inter.button_released[MOUSE_BUTTON_LEFT]);
	u32 node_focused_prev = !!(node->inter & UI_INTER_FOCUS);
	u32 node_focused = (node_focused_prev)*!g_ui->inter.key_clicked[DS_ESCAPE];

	if (node_hovered)
	{
		node_clicked = g_ui->inter.button_clicked[MOUSE_BUTTON_LEFT];
		node_dragged |= g_ui->inter.button_clicked[MOUSE_BUTTON_LEFT]*g_ui->inter.button_pressed[MOUSE_BUTTON_LEFT];
		node_scrolled = !!(g_ui->inter.scroll_up_count + g_ui->inter.scroll_down_count);
		node_selected ^= g_ui->inter.button_clicked[MOUSE_BUTTON_LEFT];
		node_focused |= g_ui->inter.button_clicked[MOUSE_BUTTON_LEFT];
	}

	const u32 node_focused_out = node_focused_prev*(!node_focused);
	const u32 node_focused_in = (!node_focused_prev)*(node_focused);

	node_inter = (UI_INTER_DRAG * node_dragged)
		     	  | (UI_INTER_HOVER * node_hovered)
		     	  | (UI_INTER_SELECT * node_selected)
		     	  | (UI_INTER_LEFT_CLICK * node_clicked)
		     	  | (UI_INTER_SCROLL * node_scrolled)
		     	  | (UI_INTER_FOCUS * node_focused)
		     	  | (UI_INTER_FOCUS_IN * node_focused_in)
		     	  | (UI_INTER_FOCUS_OUT * node_focused_out);

	node_inter |= UI_INTER_ACTIVE * !!((inter_local_mask & node_inter & UI_INTER_ACTIVATION_FLAGS));

	if (inter_recursive_mask & node_inter)
	{
		for (u32 i = g_ui->stack_parent.next-1; i; --i)
		{
			struct ui_Node *ancestor = hi_Address(&g_ui->node_hierarchy, g_ui->stack_parent.arr[i]);
			if ((ancestor->inter_recursive_mask & node_inter) == 0)
			{
				break;
			}

			//Utf8DebugPrint(ancestor->id);
			//fprintf(stderr, "\n\tSTART: ");
			//InterDebugPrint(ancestor->inter);

			/* TODO clean up this later when we have a better idea of what we want */
			u32 ancestor_selected = (!!(ancestor->inter & UI_INTER_SELECT)) ^ node_clicked;
			/* only reset selected bit if it is a recursive action */
			ancestor->inter &= ~(UI_INTER_SELECT & ancestor->inter_recursive_flags);
			/* only update selected bit if it is a recursive action */
			ancestor->inter |= (ancestor_selected*UI_INTER_SELECT) & ancestor->inter_recursive_flags;

			ancestor->inter |= node_inter & (UI_INTER_HOVER | UI_INTER_SCROLL | UI_INTER_LEFT_CLICK | UI_INTER_DRAG) & ancestor->inter_recursive_flags;

			ancestor->inter |= UI_INTER_ACTIVE * !!((ancestor->inter & UI_INTER_ACTIVATION_FLAGS));

			//fprintf(stderr, "\n\tEND: ");
			//InterDebugPrint(ancestor->inter);
			//fprintf(stderr, "\n");
		}
	}

	return inter_local_mask & node_inter;
}

void ui_FrameBegin(const vec2u32 window_size, const struct ui_Visual *base)
{
	g_ui->frame += 1;
	g_ui->mem_frame = g_ui->mem_frame_arr + (g_ui->frame & 0x1);
	ArenaFlush(g_ui->mem_frame);
	dll_Flush(&g_ui->bucket_list);
	PoolFlush(&g_ui->bucket_pool);
	HashMapFlush(&g_ui->bucket_map);
	
	/* setup stub bucket */
	struct slot slot = PoolAdd(&g_ui->bucket_pool);
	dll_Append(&g_ui->bucket_list, g_ui->bucket_pool.buf, slot.index);
	g_ui->bucket_cache = slot.index;
	struct ui_DrawBucket *bucket = slot.address;
	bucket->cmd = 0;
	bucket->count = 0;

	g_ui->frame_stack_text_selection.next = 0;

	g_ui->node_count_prev_frame = g_ui->node_count_frame;
	g_ui->node_count_frame = 0;

	g_ui->window_size[0] = window_size[0];
	g_ui->window_size[1] = window_size[1];

	ui_ExternalTextPush((utf32) { .len = 0, .max_len = 0, .buf = NULL });
	ui_ExternalTextInputPush(text_edit_stub_ptr());

	ui_FlagsPush(UI_INTER_HOVER | UI_INTER_ACTIVE);

	ui_ChildLayoutAxisPush(AXIS_2_X);

	ui_FontPush(base->font);

	ui_BorderSizePush(base->border_size);
	ui_CornerRadiusPush(base->corner_radius);

	ui_WidthPush(ui_SizePerc(1.0f));
	ui_HeightPush(ui_SizePerc(1.0f));
	ui_PaddingPush(base->pad);

	ui_TextAlignXPush(base->text_alignment_x);
	ui_TextAlignYPush(base->text_alignment_y);
	ui_TextPadPush(AXIS_2_X, base->text_pad_x);
	ui_TextPadPush(AXIS_2_Y, base->text_pad_y);

	ui_BackgroundColorPush(base->background_color);
	ui_BorderColorPush(base->border_color);
	ui_GradientColorPush(BOX_CORNER_BR, base->gradient_color[BOX_CORNER_BR]);
	ui_GradientColorPush(BOX_CORNER_TR, base->gradient_color[BOX_CORNER_TR]);
	ui_GradientColorPush(BOX_CORNER_TL, base->gradient_color[BOX_CORNER_TL]);
	ui_GradientColorPush(BOX_CORNER_BL, base->gradient_color[BOX_CORNER_BL]);
	ui_SpriteColorPush(base->sprite_color);

	Vec4Set(g_ui->text_cursor_color, 0.9f, 0.9f, 0.9f, 0.6f);
	Vec4Set(g_ui->text_selection_color, 0.7f, 0.7f, 0.9f, 0.6f);

	ui_FixedX(0.0f)
	ui_FixedY(0.0f)
	ui_Width(ui_SizePixel((f32) g_ui->window_size[0], 1.0f))
	ui_Height(ui_SizePixel((f32) g_ui->window_size[1], 1.0f))
	g_ui->root = ui_RootF("###root_%p", &g_ui->root).index;
	struct ui_Node *root = hi_Address(&g_ui->node_hierarchy, g_ui->root);
	root->pixel_visible[AXIS_2_X] = intv_inline(0.0f, (f32) window_size[0]);
	root->pixel_visible[AXIS_2_Y] = intv_inline(0.0f, (f32) window_size[1]);
	
	ui_NodePush(g_ui->root);
}

static void ui_IdentifyHoveredNode(void)
{
	//TODO Consider returning stub constant if if-statements occur; 
	struct ui_Node *node = ui_NodeLookup(&g_ui->inter.node_hovered).address;
	if (node)
	{
		node->inter &= ~UI_INTER_HOVER;
		while (node->hi_parent != HI_NULL_INDEX)
		{
			node = ui_NodeAddress(node->hi_parent);
			node->inter &= ~(UI_INTER_HOVER & node->inter_recursive_flags);
		}
	}

	const f32 x = g_ui->inter.cursor_position[0];
	const f32 y = g_ui->inter.cursor_position[1];
	i32 depth = -1;
	u32 index = HI_NULL_INDEX;
	/* find deepest hashed floating subtree which we are hovering */
	for (u32 i = 0; i < g_ui->stack_floating_node.next; ++i)
	{
		const u32 new_depth = g_ui->stack_floating_depth.arr[i];
		if (depth < (i32) new_depth)
		{
			const u32 new_index = g_ui->stack_floating_node.arr[i];
			node = hi_Address(&g_ui->node_hierarchy, new_index);
			if (node->pixel_visible[0].low <= x && x <= node->pixel_visible[0].high &&  
		    	    node->pixel_visible[1].low <= y && y <= node->pixel_visible[1].high &&
			    (node->flags & (UI_NON_HASHED | UI_SKIP_HOVER_SEARCH)) == 0)
			{
				depth = (i32) new_depth;
				index = new_index;
			}
		}
	}

	if (index == HI_NULL_INDEX)
	{
		g_ui->inter.node_hovered = Utf8Empty();
		return;
	}
	
	/* search floating subtree for deepest node we are hovering that is hashed */
	u32 deepest_non_hashed_hover_index = index;
	node = hi_Address(&g_ui->node_hierarchy, index);
	ds_Assert((node->flags & (UI_NON_HASHED | UI_SKIP_HOVER_SEARCH)) == 0);
	index = node->hi_first;
	while (index != HI_NULL_INDEX)
	{
		node = hi_Address(&g_ui->node_hierarchy, index);
		if (node->pixel_visible[0].low <= x && x <= node->pixel_visible[0].high &&  
	    	    node->pixel_visible[1].low <= y && y <= node->pixel_visible[1].high &&
		    (node->flags & UI_SKIP_HOVER_SEARCH) == 0)
		{
			if ((node->flags & UI_NON_HASHED) == 0)
			{
				deepest_non_hashed_hover_index = index;
			}

			index = node->hi_first;
			continue;
		}

		index = node->hi_next;
	}

	node = hi_Address(&g_ui->node_hierarchy, deepest_non_hashed_hover_index);
	ds_Assert((node->flags & (UI_NON_HASHED | UI_SKIP_HOVER_SEARCH)) == 0);
	node->inter |= (UI_INTER_HOVER & node->flags);
	g_ui->inter.node_hovered = node->id;

	while (node->hi_parent != HI_NULL_INDEX)
	{
		node = ui_NodeAddress(node->hi_parent);
		node->inter |= (UI_INTER_HOVER & node->inter_recursive_flags);
	}
}

static struct slot ui_TextSelectionAlloc(const struct ui_Node *node, const vec4 color, const u32 low, const u32 high)
{
	const f32 line_width = (node->flags & UI_TEXT_ALLOW_OVERFLOW)
		? F32_INFINITY
		: f32_max(0.0f, node->pixel_size[0] - 2.0f*node->text_pad[0]);
	const struct ui_TextSelection selection = 
	{
		.node = node,
		.layout = Utf32TextLayoutIncludeWhitespace(g_ui->mem_frame, &node->input.text, line_width, TAB_SIZE, node->font),
		.color = { color[0], color[1], color[2], color[3], },
		.low = low,
		.high = high,
	};

	const u32 index = g_ui->frame_stack_text_selection.next;
	stack_ui_TextSelectionPush(&g_ui->frame_stack_text_selection, selection);

	const u32 draw_key = ui_DrawCommand(node->depth, UI_CMD_LAYER_TEXT_SELECTION, AssetSpriteGetTextureId(node->sprite));
	ui_DrawBucketAddNode(draw_key, index);
	return (struct slot) { .index = index, .address = g_ui->frame_stack_text_selection.arr + index };
}

void ui_FrameEnd(void)
{	
	dll_Flush(&g_ui->event_list);
	PoolFlush(&g_ui->event_pool);

	ui_NodePop();

	ui_FlagsPop();

	ui_ChildLayoutAxisPop();

	ui_ExternalTextPop();
	ui_ExternalTextInputPop();

	ui_BorderSizePop();
	ui_CornerRadiusPop();

	ui_FontPop();

	ui_WidthPop();
	ui_HeightPop();
	ui_PaddingPop();

	ui_TextAlignXPop();
	ui_TextAlignYPop();
	ui_TextPadPop(AXIS_2_X);
	ui_TextPadPop(AXIS_2_Y);

	ui_BackgroundColorPop();
	ui_BorderColorPop();
	ui_GradientColorPop(BOX_CORNER_BR);
	ui_GradientColorPop(BOX_CORNER_TR);
	ui_GradientColorPop(BOX_CORNER_TL);
	ui_GradientColorPop(BOX_CORNER_BL);
	ui_SpriteColorPop();

	ui_ChildsumLayoutSizeAndPruneNodes();
	ui_SolveViolations();
	ui_LayoutAbsolutePosition();
	ui_IdentifyHoveredNode();

	stack_u32Flush(&g_ui->stack_floating_node);
	stack_u32Flush(&g_ui->stack_floating_depth);

	for (u32 i = 0; i < DS_KEY_COUNT; ++i)
	{
		g_ui->inter.key_clicked[i] = 0;
		g_ui->inter.key_released[i] = 0;
	}

	for (u32 i = 0; i < MOUSE_BUTTON_COUNT; ++i)
	{
		g_ui->inter.button_double_clicked[i] = 0;
		g_ui->inter.button_clicked[i] = 0;
		g_ui->inter.button_released[i] = 0;
	}
	g_ui->inter.scroll_up_count = 0;
	g_ui->inter.scroll_down_count = 0;

	g_ui->inter.cursor_delta[0] = 0;
	g_ui->inter.cursor_delta[1] = 0;

	ds_Assert(g_ui->stack_parent.next == 1);

	struct ui_Node *text_input = ui_NodeLookup(&g_ui->inter.text_edit_id).address;	
	if (text_input)
	{
		g_ui->inter.text_edit_id = text_input->id;

		if (text_input->last_frame_touched != g_ui->frame || (text_input->inter & UI_INTER_FOCUS) == 0)
		{
			CmdSubmitFormat(g_ui->mem_frame, "ui_TextInputModeDisable \"%k\"", &g_ui->inter.text_edit_id);
		}
		else
		{
			/* If ui is aliasing ui_node's ui_TextInput, we must renew the pointer since 
			 * the old address may have been reallocated.  */
			if (text_input->flags & UI_TEXT_EDIT_INTER_BUF_ON_FOCUS)
			{
				g_ui->inter.text_edit = &text_input->input;
			}
			/* draw cursor highlight */
			ui_TextSelectionAlloc(text_input, g_ui->text_cursor_color, g_ui->inter.text_edit->cursor, g_ui->inter.text_edit->cursor+1);

			if (g_ui->inter.text_edit->cursor+1 < g_ui->inter.text_edit->mark)
			{
				ui_TextSelectionAlloc(text_input, g_ui->text_selection_color, g_ui->inter.text_edit->cursor + 1, g_ui->inter.text_edit->mark);
			}
			else if (g_ui->inter.text_edit->mark < g_ui->inter.text_edit->cursor)
			{
				ui_TextSelectionAlloc(text_input, g_ui->text_selection_color, g_ui->inter.text_edit->mark, g_ui->inter.text_edit->cursor);
			}
		}
	}

	struct ui_Node *orphan = hi_Address(&g_ui->node_hierarchy, HI_ORPHAN_STUB_INDEX);
	struct ui_Node *node; 
	for (u32 index = orphan->hi_first; index != HI_NULL_INDEX;)
	{
		struct ui_Node *node = hi_Address(&g_ui->node_hierarchy, index);
		const u32 next = node->hi_next;
		hi_ApplyCustomFreeAndRemove(g_ui->mem_frame, &g_ui->node_hierarchy, index, &ui_NodeDealloc, NULL);
		index = next;
	}
	hi_AdoptNode(&g_ui->node_hierarchy, g_ui->root, HI_ORPHAN_STUB_INDEX);
}

/* Calculate sizes known at time of creation, i.e. every size expect CHILDSUM */
static void ui_NodeCalculateImmediateLayout(struct ui_Node *node, const enum axis_2 axis)
{
	switch (node->semantic_size[axis].type)
	{
		case UI_SIZE_PIXEL:
		{
			node->layout_size[axis] = node->semantic_size[axis].pixels;
		} break;

		case UI_SIZE_TEXT:
		{
			const f32 pad = 2.0f*node->text_pad[axis];
			if (node->flags & UI_TEXT_ATTACHED)
			{
				node->layout_size[axis] = (axis == AXIS_2_X)
					? pad + node->layout_text->width
					: pad + node->font->linespace*node->layout_text->line_count;
			}
			else
			{
				node->layout_size[axis] = pad;
			}
		} break;

		case UI_SIZE_PERC_PARENT:
		{
			const struct ui_Node *parent = hi_Address(&g_ui->node_hierarchy, node->hi_parent);
			if (parent->semantic_size[axis].type == UI_SIZE_CHILDSUM || (parent->flags & (UI_PERC_POSTPONED_X << axis)))
			{
				node->layout_size[axis] = 0.0f;
				node->flags |= UI_PERC_POSTPONED_X << axis;
			}
			else
			{
				node->layout_size[axis] = node->semantic_size[axis].percentage * parent->layout_size[axis];	
			}
		} break;

		case UI_SIZE_UNIT:
		{
			const struct ui_Node *parent = hi_Address(&g_ui->node_hierarchy, node->hi_parent);
			const intv visible = stack_intvTop(g_ui->stack_viewable + axis);
			const f32 pixels_per_unit = parent->pixel_size[axis] / (visible.high - visible.low);

			node->layout_size[axis] = pixels_per_unit*(node->semantic_size[axis].intv.high - node->semantic_size[axis].intv.low);
			node->layout_position[axis] = pixels_per_unit*(node->semantic_size[axis].intv.low - visible.low);

			if ((axis == AXIS_2_Y) && (node->flags & UI_UNIT_POSITIVE_DOWN))
			{
				node->layout_position[axis] = parent->pixel_size[axis] - node->layout_size[axis] - node->layout_position[axis];
			}
		} break;

		case UI_SIZE_CHILDSUM:
		{
			node->layout_position[axis] = 0.0f;
			node->layout_size[axis] = 0.0f;
		} break;

		default:
		{

		} break;
	}
}

static u32 ui_InternalPad(const u64 flags, const f32 value, const enum ui_SizeType type)
{
	const u32 parent_index = stack_u32Top(&g_ui->stack_parent);

	if (parent_index == HI_ORPHAN_STUB_INDEX)
	{
		return HI_ORPHAN_STUB_INDEX;
	}

	struct slot slot = hi_Add(&g_ui->node_hierarchy, parent_index);
	struct ui_Node *node = slot.address;
	g_ui->node_count_frame += 1;

	struct ui_Node *parent = hi_Address(&g_ui->node_hierarchy, parent_index);
	const u32 non_layout_axis = 1 - parent->child_layout_axis;

	node->id = Utf8Empty();
	node->flags = flags | stack_u64Top(&g_ui->stack_flags) | UI_DEBUG_FLAGS;
	node->last_frame_touched = g_ui->frame;
	node->semantic_size[parent->child_layout_axis] = (type == UI_SIZE_PIXEL)
		? ui_SizePixel(value, 0.0f)
		: ui_SizePerc(value);
	node->semantic_size[non_layout_axis] = ui_SizePerc(1.0f);
	node->child_layout_axis = stack_u32Top(&g_ui->stack_child_layout_axis);
	node->depth = (g_ui->stack_fixed_depth.next)
		? stack_u32Top(&g_ui->stack_fixed_depth)
		: parent->depth + 1;
	node->inter = 0;
	node->inter_recursive_flags = 0;
	node->inter_recursive_mask = 0;

	if (node->flags & UI_DRAW_SPRITE)
	{
		node->sprite = stack_u32Top(&g_ui->stack_sprite);
		stackVec4Top(node->sprite_color, &g_ui->stack_sprite_color);
	}
	else
	{
		node->sprite = SPRITE_NONE;
	}
	
	if (node->flags & UI_DRAW_FLAGS)
	{
		const u32 draw_key = (node->flags & UI_INTER_FLAGS)
			? ui_DrawCommand(node->depth, UI_CMD_LAYER_INTER, AssetSpriteGetTextureId(node->sprite))
			: ui_DrawCommand(node->depth, UI_CMD_LAYER_VISUAL, AssetSpriteGetTextureId(node->sprite));
		ui_DrawBucketAddNode(draw_key, slot.index);
	}

	node->input.text = Utf32Empty();
	node->font = NULL;
	node->layout_text = NULL;

	/* set possible immediate sizes and positions */
	ui_NodeCalculateImmediateLayout(node, AXIS_2_X);
	ui_NodeCalculateImmediateLayout(node, AXIS_2_Y);

	(node->flags & UI_DRAW_BACKGROUND)
		? stackVec4Top(node->background_color, &g_ui->stack_background_color)
		: Vec4Set(node->background_color, 0.0f, 0.0f, 0.0f, 0.0f);

	if (node->flags & UI_DRAW_BORDER)
	{
		node->border_size = stack_f32Top(&g_ui->stack_border_size);
		stackVec4Top(node->border_color, &g_ui->stack_border_color);
	}
	else
	{
		node->border_size = 0.0f;
		Vec4Set(node->border_color, 0.0f, 0.0f, 0.0f, 0.0f);
	}

	if (node->flags & UI_DRAW_GRADIENT)
	{
		stackVec4Top(node->gradient_color[BOX_CORNER_BR], g_ui->stack_gradient_color + BOX_CORNER_BR);
		stackVec4Top(node->gradient_color[BOX_CORNER_TR], g_ui->stack_gradient_color + BOX_CORNER_TR);
		stackVec4Top(node->gradient_color[BOX_CORNER_TL], g_ui->stack_gradient_color + BOX_CORNER_TL);
		stackVec4Top(node->gradient_color[BOX_CORNER_BL], g_ui->stack_gradient_color + BOX_CORNER_BL);
	}
	else
	{
		Vec4Set(node->gradient_color[BOX_CORNER_BR], 0.0f, 0.0f, 0.0f, 0.0f); 
                Vec4Set(node->gradient_color[BOX_CORNER_TR], 0.0f, 0.0f, 0.0f, 0.0f);
                Vec4Set(node->gradient_color[BOX_CORNER_TL], 0.0f, 0.0f, 0.0f, 0.0f);
                Vec4Set(node->gradient_color[BOX_CORNER_BL], 0.0f, 0.0f, 0.0f, 0.0f);
	}

	node->edge_softness = (node->flags & UI_DRAW_EDGE_SOFTNESS)
		? stack_f32Top(&g_ui->stack_edge_softness)
		: 0.0f;

	node->corner_radius = (node->flags & UI_DRAW_ROUNDED_CORNERS)
		? stack_f32Top(&g_ui->stack_corner_radius)
		: 0.0f;
	
	return slot.index;
}

u32 ui_Pad(void)
{
	return ui_InternalPad(UI_NON_HASHED | UI_PAD, stack_f32Top(&g_ui->stack_pad), UI_SIZE_PIXEL);
}

u32 ui_PadPixel(const f32 pixel)
{
	return ui_InternalPad(UI_NON_HASHED | UI_PAD, pixel, UI_SIZE_PIXEL);
}

u32 ui_PadPerc(const f32 perc)
{
	return ui_InternalPad(UI_NON_HASHED | UI_PAD, perc, UI_SIZE_PERC_PARENT);
}

u32 ui_PadFill(void)
{
	return ui_InternalPad(UI_NON_HASHED | UI_PAD | UI_PAD_FILL, 0.0f, UI_SIZE_PIXEL);
}

struct slot ui_NodeAllocNonHashed(const u64 flags)
{
	const utf8 id = Utf8Empty();
	return ui_NodeAlloc(flags | UI_NON_HASHED, &id);
}

struct ui_Node *ui_NodeAddress(const u32 node)
{
	return PoolAddress(&g_ui->node_hierarchy.pool, node);
}

struct slot ui_NodeLookup(const utf8 *id)
{
	struct slot slot = { .address = NULL, .index = U32_MAX };
	struct ui_Node *node;
	const u32 hash = Utf8Hash(*id);
	u32 index = HashMapFirst(&g_ui->node_map, hash);
	for (; index != HASH_NULL; index = HashMapNext(&g_ui->node_map, index))
	{
		node = hi_Address(&g_ui->node_hierarchy, index);
		if (Utf8Equivalence(node->id, *id))
		{
			slot.address = node;
			slot.index = index;
			break;
		}
	}

	return slot; 
}

struct ui_NodeCache ui_NodeCacheNull(void)
{
	struct ui_NodeCache cache =
	{
		.last_frame_touched = U64_MAX,
		.frame_node = NULL,
		.index = UI_NON_CACHED_INDEX,
	};

	return cache;
}

static struct ui_NodeCache ui_NodeCacheOrphanRoot(void)
{
	struct ui_NodeCache cache =
	{
		.last_frame_touched = U64_MAX,
		.frame_node = hi_Address(&g_ui->node_hierarchy, UI_NON_CACHED_INDEX),
		.index = UI_NON_CACHED_INDEX,
	};

	return cache;
}

struct ui_NodeCache ui_NodeAllocCached(const u64 flags, const utf8 id, const utf8 text, const struct ui_NodeCache cache)
{
	const u32 parent_index = stack_u32Top(&g_ui->stack_parent);
	struct ui_Node *parent = hi_Address(&g_ui->node_hierarchy, parent_index);

	/* Parent failed to alloc */
	if (parent_index == HI_ORPHAN_STUB_INDEX)
	{
		return ui_NodeCacheOrphanRoot();
	}

	u64 implied_flags = stack_u64Top(&g_ui->stack_flags);

	/* If not cached, index should be != STUB_INDEX */
	struct ui_Node *node = (cache.last_frame_touched+1 == g_ui->frame)
				? hi_Address(&g_ui->node_hierarchy, cache.index)
				: hi_Address(&g_ui->node_hierarchy, HI_ORPHAN_STUB_INDEX);

	ds_Assert(node->last_frame_touched != g_ui->frame);
	struct ui_Size size_x = stack_ui_SizeTop(g_ui->stack_ui_Size + AXIS_2_X);
	struct ui_Size size_y = stack_ui_SizeTop(g_ui->stack_ui_Size + AXIS_2_Y);

	/* Cull any unit_sized nodes that are not visible except if they are being interacted with */
	if (size_x.type == UI_SIZE_UNIT)
	{
		ds_Assert(g_ui->stack_viewable[AXIS_2_X].next);
		implied_flags |= UI_ALLOW_VIOLATION_X;

		const intv visible = stack_intvTop(g_ui->stack_viewable + AXIS_2_X);
		if ((size_x.intv.high < visible.low || size_x.intv.low > visible.high) && !(node->inter & UI_INTER_ACTIVE))
		{
			return ui_NodeCacheOrphanRoot();
		}
	}

	if (size_y.type == UI_SIZE_UNIT)
	{
		ds_Assert(g_ui->stack_viewable[AXIS_2_Y].next);
		implied_flags |= UI_ALLOW_VIOLATION_Y;
		
		const intv visible = stack_intvTop(g_ui->stack_viewable + AXIS_2_Y);
		if ((size_y.intv.high < visible.low || size_y.intv.low > visible.high) && !(node->inter & UI_INTER_ACTIVE))
		{
			return ui_NodeCacheOrphanRoot();
		}
	}

	const u64 inter_recursive_flags = (flags & UI_INTER_RECURSIVE_ROOT)
		? stack_u64Top(&g_ui->stack_recursive_interaction_flags)
		: 0;
	const u64 node_flags = flags | implied_flags | UI_DEBUG_FLAGS | inter_recursive_flags;
	const u64 inter_recursive_mask = parent->inter_recursive_mask | inter_recursive_flags;
	u64 inter = 0;

	const u32 depth = (g_ui->stack_fixed_depth.next)
		? stack_u32Top(&g_ui->stack_fixed_depth)
		: parent->depth + 1;

	u32 hash;
	struct slot slot;
	if (cache.last_frame_touched+1 != g_ui->frame)
	{
		hash = Utf8Hash(id);
		slot = hi_Add(&g_ui->node_hierarchy, stack_u32Top(&g_ui->stack_parent));
		node = slot.address;
		HashMapAdd(&g_ui->node_map, hash, slot.index);
	}
	else
	{
		hash = node->hash;
		slot.address = node;
		slot.index = cache.index;
		hi_AdoptNodeExclusive(&g_ui->node_hierarchy, slot.index, stack_u32Top(&g_ui->stack_parent));
		inter = ui_NodeSetInteractions(node, node_flags, inter_recursive_mask);
	}
	

	g_ui->node_count_frame += 1;

	node->id = id;
	node->hash = hash;
	node->flags = node_flags;
	node->inter_recursive_flags = inter_recursive_flags;
	node->inter_recursive_mask = inter_recursive_mask;
	node->inter = inter;
	node->last_frame_touched = g_ui->frame;
	node->semantic_size[AXIS_2_X] = size_x;
	node->semantic_size[AXIS_2_Y] = size_y;
	node->child_layout_axis = stack_u32Top(&g_ui->stack_child_layout_axis);
	node->depth = depth;

	if (node->flags & UI_DRAW_SPRITE)
	{
		node->sprite = stack_u32Top(&g_ui->stack_sprite);
		stackVec4Top(node->sprite_color, &g_ui->stack_sprite_color);
	}
	else
	{
		node->sprite = SPRITE_NONE;
	}
	
	if (node->flags & UI_DRAW_FLAGS)
	{
		const u32 draw_key = (node->flags & UI_INTER_FLAGS)
			? ui_DrawCommand(node->depth, UI_CMD_LAYER_INTER, AssetSpriteGetTextureId(node->sprite))
			: ui_DrawCommand(node->depth, UI_CMD_LAYER_VISUAL, AssetSpriteGetTextureId(node->sprite));
		ui_DrawBucketAddNode(draw_key, slot.index);
	}

	if (node->flags & UI_DRAW_TEXT)
	{
		const struct assetFont *asset = stack_ptrTop(&g_ui->stack_font);
		stackVec4Top(node->sprite_color, &g_ui->stack_sprite_color);
		node->flags |= UI_TEXT_ATTACHED;
		node->font = asset->font;
		node->text_align_x = stack_u32Top(&g_ui->stack_text_alignment_x);
		node->text_align_y = stack_u32Top(&g_ui->stack_text_alignment_y);
		node->text_pad[AXIS_2_X] = stack_f32Top(g_ui->stack_text_pad + AXIS_2_X);
		node->text_pad[AXIS_2_Y] = stack_f32Top(g_ui->stack_text_pad + AXIS_2_Y);

		u32 text_editing = 0;
		if ((node->flags & UI_TEXT_EDIT) && (node->inter & UI_INTER_FOCUS))
		{
			text_editing = 1;
			node->flags |= UI_TEXT_ALLOW_OVERFLOW | UI_TEXT_LAYOUT_POSTPONED;
			if (node->inter & UI_INTER_FOCUS_IN)  
			{
				if (node->flags & UI_TEXT_EDIT_INTER_BUF_ON_FOCUS)
				{
					/* we should not modify the possibly aliased text_internal_buf here; If we would, some other
					 * node could technically see our changes while still being focused... instead we just copy
					 * it if we actually become focused later on. */
					const u32 buflen = sizeof(g_ui->inter.text_internal_buf) / sizeof(u32);
					node->input = ui_TextInputBuffered(g_ui->inter.text_internal_buf, buflen);
					node->input.cursor = 0;
					node->input.mark = 0;
					if (node->flags & UI_TEXT_EDIT_COPY_ON_FOCUS)
					{
						utf32 copy = (node->flags & (UI_TEXT_EXTERNAL | UI_TEXT_EXTERNAL_LAYOUT))
							? Utf32Copy(g_ui->mem_frame, stack_utf32Top(&g_ui->stack_external_text))
							: Utf32Utf8(g_ui->mem_frame, text);

						if (copy.len)
						{
							node->input.text = copy;
							node->input.mark = 0;
							node->input.cursor = copy.len;
						}
					}
					CmdSubmitFormat(g_ui->mem_frame, "ui_TextInputModeEnable \"%k\" %p", &node->id, &node->input);
				}
				else
				{
					CmdSubmitFormat(g_ui->mem_frame, "ui_TextInputModeEnable \"%k\" %p", &node->id, stack_ptrTop(&g_ui->stack_external_text_input));
				}
			}
			else
			{
				/* renew ui_TextInput pointer to external resource; for internal
				 * buffers we must renew them on frame end! */
				if ((node->flags & UI_TEXT_EDIT_INTER_BUF_ON_FOCUS) == 0)
				{
					g_ui->inter.text_edit = stack_ptrTop(&g_ui->stack_external_text_input);
					node->input = *g_ui->inter.text_edit;
				}
			}
		}


		if (!text_editing)
		{
			if (node->flags & UI_TEXT_EXTERNAL_LAYOUT)
			{
				node->flags |= UI_TEXT_EXTERNAL | UI_TEXT_ALLOW_OVERFLOW;
				node->input.text = stack_utf32Top(&g_ui->stack_external_text); 
				node->layout_text = stack_ptrTop(&g_ui->stack_external_text_layout); 
			}
			else
			{
				node->input.text = (node->flags & UI_TEXT_EXTERNAL)
					? stack_utf32Top(&g_ui->stack_external_text)
					: Utf32Utf8(g_ui->mem_frame, text);

				/* TODO wonky, would be nice to have it in immediate calculations, but wonky there as well... */
				if (node->semantic_size[AXIS_2_X].type == UI_SIZE_TEXT)
				{
					node->semantic_size[AXIS_2_X].line_width = (node->flags & UI_TEXT_ALLOW_OVERFLOW)
						? F32_INFINITY
						: node->semantic_size[AXIS_2_X].line_width;
					node->layout_text = Utf32TextLayout(g_ui->mem_frame, &node->input.text, node->semantic_size[AXIS_2_X].line_width, TAB_SIZE, node->font);
				}
				else
				{
					node->flags |= UI_TEXT_LAYOUT_POSTPONED;
				}
			}
		}

		/* visual first (10), inter second(01), text last(00) */
		const u32 draw_key = ui_DrawCommand(node->depth, UI_CMD_LAYER_TEXT, asset->texture_id);
		ui_DrawBucketAddNode(draw_key, slot.index);
	}
	else
	{
		node->input.text = Utf32Empty();
		Vec4Set(node->sprite_color, 0.0f, 0.0f, 0.0f, 0.0f);
		node->font = NULL;
		node->layout_text = NULL;
	}

	/* set possible immediate sizes and positions */
	ui_NodeCalculateImmediateLayout(node, AXIS_2_X);
	ui_NodeCalculateImmediateLayout(node, AXIS_2_Y);

	u32 floating = 0;
	if (g_ui->stack_floating[AXIS_2_X].next)
	{
		floating = 1;
		node->layout_position[AXIS_2_X] = stack_f32Top(g_ui->stack_floating + AXIS_2_X);
		node->flags |= UI_FLOATING_X;
	}	

	if (g_ui->stack_floating[AXIS_2_Y].next)
	{
		floating = 1;
		node->layout_position[AXIS_2_Y] = stack_f32Top(g_ui->stack_floating + AXIS_2_Y);
		node->flags |= UI_FLOATING_Y;
	}

	if (floating)
	{
		stack_u32Push(&g_ui->stack_floating_node, slot.index);
		stack_u32Push(&g_ui->stack_floating_depth, node->depth);
	}

	(node->flags & UI_DRAW_BACKGROUND)
		? stackVec4Top(node->background_color, &g_ui->stack_background_color)
		: Vec4Set(node->background_color, 0.0f, 0.0f, 0.0f, 0.0f);

	if (node->flags & UI_DRAW_BORDER)
	{
		node->border_size = stack_f32Top(&g_ui->stack_border_size);
		stackVec4Top(node->border_color, &g_ui->stack_border_color);
	}
	else
	{
		node->border_size = 0.0f;
		Vec4Set(node->border_color, 0.0f, 0.0f, 0.0f, 0.0f);
	}

	if (node->flags & UI_DRAW_GRADIENT)
	{
		stackVec4Top(node->gradient_color[BOX_CORNER_BR], g_ui->stack_gradient_color + BOX_CORNER_BR);
		stackVec4Top(node->gradient_color[BOX_CORNER_TR], g_ui->stack_gradient_color + BOX_CORNER_TR);
		stackVec4Top(node->gradient_color[BOX_CORNER_TL], g_ui->stack_gradient_color + BOX_CORNER_TL);
		stackVec4Top(node->gradient_color[BOX_CORNER_BL], g_ui->stack_gradient_color + BOX_CORNER_BL);
	}
	else
	{
		Vec4Set(node->gradient_color[BOX_CORNER_BR], 0.0f, 0.0f, 0.0f, 0.0f); 
                Vec4Set(node->gradient_color[BOX_CORNER_TR], 0.0f, 0.0f, 0.0f, 0.0f);
                Vec4Set(node->gradient_color[BOX_CORNER_TL], 0.0f, 0.0f, 0.0f, 0.0f);
                Vec4Set(node->gradient_color[BOX_CORNER_BL], 0.0f, 0.0f, 0.0f, 0.0f);
	}

	node->edge_softness = (node->flags & UI_DRAW_EDGE_SOFTNESS)
		? stack_f32Top(&g_ui->stack_edge_softness)
		: 0.0f;

	node->corner_radius = (node->flags & UI_DRAW_ROUNDED_CORNERS)
		? stack_f32Top(&g_ui->stack_corner_radius)
		: 0.0f;
	
	ds_Assert(node->semantic_size[AXIS_2_Y].type != UI_SIZE_TEXT || node->semantic_size[AXIS_2_X].type == UI_SIZE_TEXT);

	const struct ui_NodeCache new_cache =
	{
		.index = slot.index,
		.frame_node = node,
		.last_frame_touched = g_ui->frame,
	};

	return new_cache;
}

struct slot ui_NodeAlloc(const u64 flags, const utf8 *formatted)
{
	const u32 parent_index = stack_u32Top(&g_ui->stack_parent);
	struct ui_Node *parent = hi_Address(&g_ui->node_hierarchy, parent_index);

	if (parent_index == HI_ORPHAN_STUB_INDEX)
	{
		return (struct slot) { .index = HI_ORPHAN_STUB_INDEX, .address = hi_Address(&g_ui->node_hierarchy, HI_ORPHAN_STUB_INDEX) };
	}

	u32 hash_count = 0;
	u32 hash_begin_index = 0;
	u32 hash_begin_offset = 0;
	u64 offset = 0;
	u32 text_len = formatted->len;
	for (u32 i = 0; i < formatted->len; ++i)
	{
		const u32 codepoint = Utf8ReadCodepoint(&offset, formatted, offset);
		if (codepoint == '#')
		{
			hash_count += 1;
			if (hash_count == 3)
			{
				hash_begin_index = i+1;
				hash_begin_offset = (u32) offset;
				text_len = i-2;
				break;
			}
			else if (hash_count == 2 && i+1 == formatted->len)
			{
				text_len = i-2;
			}
		}
		else if (hash_count == 2)
		{
			text_len = i-2;
			break;
		}
		else
		{
			hash_count = 0;
		}
	}

	const utf8 id = (utf8) { .buf = formatted->buf + hash_begin_offset, .len = formatted->len - hash_begin_index, .size = formatted->size - hash_begin_offset };
	struct slot slot = ui_NodeLookup(&id);
	struct ui_Node *node = slot.address;
	u32 hash = 0;

	const u64 inter_recursive_flags = (flags & UI_INTER_RECURSIVE_ROOT)
		? stack_u64Top(&g_ui->stack_recursive_interaction_flags)
		: 0;
	u64 node_flags = flags | stack_u64Top(&g_ui->stack_flags) | UI_DEBUG_FLAGS | inter_recursive_flags;
	const u64 inter_recursive_mask = parent->inter_recursive_mask | inter_recursive_flags;

	struct ui_Size size_x = stack_ui_SizeTop(g_ui->stack_ui_Size + AXIS_2_X);
	struct ui_Size size_y = stack_ui_SizeTop(g_ui->stack_ui_Size + AXIS_2_Y);

	if (size_x.type == UI_SIZE_UNIT)
	{
		ds_Assert(g_ui->stack_viewable[AXIS_2_X].next);
		node_flags |= UI_ALLOW_VIOLATION_X;

		const intv visible = stack_intvTop(g_ui->stack_viewable + AXIS_2_X);
		if ((size_x.intv.high < visible.low || size_x.intv.low > visible.high) && node && !(node->inter & UI_INTER_ACTIVE))
		{
			return (struct slot) { .index = HI_ORPHAN_STUB_INDEX, .address = hi_Address(&g_ui->node_hierarchy, HI_ORPHAN_STUB_INDEX) };
		}
	}

	if (size_y.type == UI_SIZE_UNIT)
	{
		ds_Assert(g_ui->stack_viewable[AXIS_2_Y].next);
		node_flags |= UI_ALLOW_VIOLATION_Y;
		
		const intv visible = stack_intvTop(g_ui->stack_viewable + AXIS_2_Y);
		if ((size_y.intv.high < visible.low || size_y.intv.low > visible.high) && node && !(node->inter & UI_INTER_ACTIVE))
		{
			return (struct slot) { .index = HI_ORPHAN_STUB_INDEX, .address = hi_Address(&g_ui->node_hierarchy, HI_ORPHAN_STUB_INDEX) };
		}
	}

	u64 inter = 0;
	if (!slot.address)
	{
		slot = hi_Add(&g_ui->node_hierarchy, parent_index);
		parent = hi_Address(&g_ui->node_hierarchy, parent_index);
		node = slot.address;
		if ((flags & UI_NON_HASHED) == 0)
		{
			hash = Utf8Hash(id);
			HashMapAdd(&g_ui->node_map, hash, slot.index);
		}
		ds_Assert((flags & UI_NON_HASHED) == UI_NON_HASHED || id.len > 0);
	}
	else
	{
		ds_Assert(node->last_frame_touched != g_ui->frame);
		hash = node->hash;
		hi_AdoptNodeExclusive(&g_ui->node_hierarchy, slot.index, stack_u32Top(&g_ui->stack_parent));
		inter = ui_NodeSetInteractions(node, node_flags, inter_recursive_mask);
	}

	g_ui->node_count_frame += 1;

	node->id = id;
	node->hash = hash;
	node->flags = node_flags;
	node->inter_recursive_flags = inter_recursive_flags;
	node->inter_recursive_mask = inter_recursive_mask;
	node->inter = inter;
	node->last_frame_touched = g_ui->frame;
	node->semantic_size[AXIS_2_X] = size_x;
	node->semantic_size[AXIS_2_Y] = size_y;
	node->child_layout_axis = stack_u32Top(&g_ui->stack_child_layout_axis);
	node->depth = (g_ui->stack_fixed_depth.next)
		? stack_u32Top(&g_ui->stack_fixed_depth)
		: parent->depth + 1;

	if (node->flags & UI_DRAW_SPRITE)
	{
		node->sprite = stack_u32Top(&g_ui->stack_sprite);
		stackVec4Top(node->sprite_color, &g_ui->stack_sprite_color);
	}
	else
	{
		node->sprite = SPRITE_NONE;
	}
	
	if (node->flags & UI_DRAW_FLAGS)
	{
		const u32 draw_key = (node->flags & UI_INTER_FLAGS)
			? ui_DrawCommand(node->depth, UI_CMD_LAYER_INTER, AssetSpriteGetTextureId(node->sprite))
			: ui_DrawCommand(node->depth, UI_CMD_LAYER_VISUAL, AssetSpriteGetTextureId(node->sprite));
		ui_DrawBucketAddNode(draw_key, slot.index);
	}

	if (node->flags & UI_DRAW_TEXT)
	{
		const struct assetFont *asset = stack_ptrTop(&g_ui->stack_font);
		stackVec4Top(node->sprite_color, &g_ui->stack_sprite_color);
		node->flags |= UI_TEXT_ATTACHED;
		node->font = asset->font;
		node->text_align_x = stack_u32Top(&g_ui->stack_text_alignment_x);
		node->text_align_y = stack_u32Top(&g_ui->stack_text_alignment_y);
		node->text_pad[AXIS_2_X] = stack_f32Top(g_ui->stack_text_pad + AXIS_2_X);
		node->text_pad[AXIS_2_Y] = stack_f32Top(g_ui->stack_text_pad + AXIS_2_Y);

		u32 text_editing = 0;
		if ((node->flags & UI_TEXT_EDIT) && (node->inter & UI_INTER_FOCUS))
		{
			text_editing = 1;
			node->flags |= UI_TEXT_ALLOW_OVERFLOW | UI_TEXT_LAYOUT_POSTPONED;
			if (node->inter & UI_INTER_FOCUS_IN)  
			{
				if (node->flags & UI_TEXT_EDIT_INTER_BUF_ON_FOCUS)
				{
					/* we should not modify the possibly aliased text_internal_buf here; If we would, some other
					 * node could technically see our changes while still being focused... instead we just copy
					 * it if we actually become focused later on. */
					const u32 buflen = sizeof(g_ui->inter.text_internal_buf) / sizeof(u32);
					node->input = ui_TextInputBuffered(g_ui->inter.text_internal_buf, buflen);
					node->input.cursor = 0;
					node->input.mark = 0;
					if (node->flags & UI_TEXT_EDIT_COPY_ON_FOCUS)
					{
						utf32 copy = (node->flags & (UI_TEXT_EXTERNAL | UI_TEXT_EXTERNAL_LAYOUT))
							? Utf32Copy(g_ui->mem_frame, stack_utf32Top(&g_ui->stack_external_text))
							: Utf32Utf8(g_ui->mem_frame, (utf8) { .buf = formatted->buf, .len = text_len, .size = formatted->size });

						if (copy.len)
						{
							node->input.text = copy;
							node->input.mark = 0;
							node->input.cursor = copy.len;
						}
					}
					CmdSubmitFormat(g_ui->mem_frame, "ui_TextInputModeEnable \"%k\" %p", &node->id, &node->input);
				}
				else
				{
					CmdSubmitFormat(g_ui->mem_frame, "ui_TextInputModeEnable \"%k\" %p", &node->id, stack_ptrTop(&g_ui->stack_external_text_input));
				}
			}
			else
			{
				/* renew ui_TextInput pointer to external resource; for internal
				 * buffers we must renew them on frame end! */
				if ((node->flags & UI_TEXT_EDIT_INTER_BUF_ON_FOCUS) == 0)
				{
					g_ui->inter.text_edit = stack_ptrTop(&g_ui->stack_external_text_input);
					node->input = *g_ui->inter.text_edit;
				}
			}
		}

		if (!text_editing)
		{
			if (node->flags & UI_TEXT_EXTERNAL_LAYOUT)
			{
				node->flags |= UI_TEXT_EXTERNAL | UI_TEXT_ALLOW_OVERFLOW;
				node->input.text = stack_utf32Top(&g_ui->stack_external_text); 
				node->layout_text = stack_ptrTop(&g_ui->stack_external_text_layout); 
			}
			else
			{
				node->input.text = (node->flags & UI_TEXT_EXTERNAL)
					? stack_utf32Top(&g_ui->stack_external_text)
					: Utf32Utf8(g_ui->mem_frame, (utf8) { .buf = formatted->buf, .len = text_len, .size = formatted->size });

				/* TODO wonky, would be nice to have it in immediate calculations, but wonky there as well... */
				if (node->semantic_size[AXIS_2_X].type == UI_SIZE_TEXT)
				{
					node->semantic_size[AXIS_2_X].line_width = (node->flags & UI_TEXT_ALLOW_OVERFLOW)
						? F32_INFINITY
						: node->semantic_size[AXIS_2_X].line_width;
					node->layout_text = Utf32TextLayout(g_ui->mem_frame, &node->input.text, node->semantic_size[AXIS_2_X].line_width, TAB_SIZE, node->font);
				}
				else
				{
					node->flags |= UI_TEXT_LAYOUT_POSTPONED;
				}
			}
		}

		/* visual first (10), inter second(01), text last(00) */
		const u32 draw_key = ui_DrawCommand(node->depth, UI_CMD_LAYER_TEXT, asset->texture_id);
		ui_DrawBucketAddNode(draw_key, slot.index);
	}
	else
	{
		node->input.text = Utf32Empty();
		Vec4Set(node->sprite_color, 0.0f, 0.0f, 0.0f, 0.0f);
		node->font = NULL;
		node->layout_text = NULL;
	}

	/* set possible immediate sizes and positions */
	ui_NodeCalculateImmediateLayout(node, AXIS_2_X);
	ui_NodeCalculateImmediateLayout(node, AXIS_2_Y);

	u32 floating = 0;
	if (g_ui->stack_floating[AXIS_2_X].next)
	{
		floating = 1;
		node->layout_position[AXIS_2_X] = stack_f32Top(g_ui->stack_floating + AXIS_2_X);
		node->flags |= UI_FLOATING_X;
	}	

	if (g_ui->stack_floating[AXIS_2_Y].next)
	{
		floating = 1;
		node->layout_position[AXIS_2_Y] = stack_f32Top(g_ui->stack_floating + AXIS_2_Y);
		node->flags |= UI_FLOATING_Y;
	}

	if (floating)
	{
		stack_u32Push(&g_ui->stack_floating_node, slot.index);
		stack_u32Push(&g_ui->stack_floating_depth, node->depth);
	}

	(node->flags & UI_DRAW_BACKGROUND)
		? stackVec4Top(node->background_color, &g_ui->stack_background_color)
		: Vec4Set(node->background_color, 0.0f, 0.0f, 0.0f, 0.0f);

	if (node->flags & UI_DRAW_BORDER)
	{
		node->border_size = stack_f32Top(&g_ui->stack_border_size);
		stackVec4Top(node->border_color, &g_ui->stack_border_color);
	}
	else
	{
		node->border_size = 0.0f;
		Vec4Set(node->border_color, 0.0f, 0.0f, 0.0f, 0.0f);
	}

	if (node->flags & UI_DRAW_GRADIENT)
	{
		stackVec4Top(node->gradient_color[BOX_CORNER_BR], g_ui->stack_gradient_color + BOX_CORNER_BR);
		stackVec4Top(node->gradient_color[BOX_CORNER_TR], g_ui->stack_gradient_color + BOX_CORNER_TR);
		stackVec4Top(node->gradient_color[BOX_CORNER_TL], g_ui->stack_gradient_color + BOX_CORNER_TL);
		stackVec4Top(node->gradient_color[BOX_CORNER_BL], g_ui->stack_gradient_color + BOX_CORNER_BL);
	}
	else
	{
		Vec4Set(node->gradient_color[BOX_CORNER_BR], 0.0f, 0.0f, 0.0f, 0.0f); 
                Vec4Set(node->gradient_color[BOX_CORNER_TR], 0.0f, 0.0f, 0.0f, 0.0f);
                Vec4Set(node->gradient_color[BOX_CORNER_TL], 0.0f, 0.0f, 0.0f, 0.0f);
                Vec4Set(node->gradient_color[BOX_CORNER_BL], 0.0f, 0.0f, 0.0f, 0.0f);
	}

	node->edge_softness = (node->flags & UI_DRAW_EDGE_SOFTNESS)
		? stack_f32Top(&g_ui->stack_edge_softness)
		: 0.0f;

	node->corner_radius = (node->flags & UI_DRAW_ROUNDED_CORNERS)
		? stack_f32Top(&g_ui->stack_corner_radius)
		: 0.0f;
	
	ds_Assert(node->semantic_size[AXIS_2_Y].type != UI_SIZE_TEXT || node->semantic_size[AXIS_2_X].type == UI_SIZE_TEXT);

	return slot;
}

struct slot ui_NodeAllocF(const u64 flags, const char *format, ...)
{
	va_list args;
	va_start(args, format);
	utf8 id = Utf8FormatVariadic(g_ui->mem_frame, format, args);
	va_end(args);

	return ui_NodeAlloc(flags, &id);
}

/********************************************************************************************************/
/*					 Push/Pop global state  					*/
/********************************************************************************************************/

void ui_NodePush(const u32 node)
{
	stack_u32Push(&g_ui->stack_parent, node);
}

void ui_NodePop(void)
{	
	stack_u32Pop(&g_ui->stack_parent);
}

struct ui_Node *ui_NodeTop(void)
{
	return PoolAddress(&g_ui->node_hierarchy.pool, stack_u32Top(&g_ui->stack_parent));
}

void ui_SizePush(const enum axis_2 axis, const struct ui_Size size)
{
	stack_ui_SizePush(g_ui->stack_ui_Size + axis, size);
}

void ui_SizeSet(const enum axis_2 axis, const struct ui_Size size)
{
	stack_ui_SizeSet(g_ui->stack_ui_Size + axis, size);
}

void ui_SizePop(const enum axis_2 axis)
{
	stack_ui_SizePop(g_ui->stack_ui_Size + axis);
}

void ui_WidthPush(const struct ui_Size size)
{
	stack_ui_SizePush(g_ui->stack_ui_Size + AXIS_2_X, size);
}

void ui_WidthSet(const struct ui_Size size)
{
	stack_ui_SizeSet(g_ui->stack_ui_Size + AXIS_2_X, size);
}

void ui_WidthPop(void)
{
	stack_ui_SizePop(g_ui->stack_ui_Size + AXIS_2_X);
}

void ui_HeightPush(const struct ui_Size size)
{
	stack_ui_SizePush(g_ui->stack_ui_Size + AXIS_2_Y, size);
}

void ui_HeightSet(const struct ui_Size size)
{
	stack_ui_SizeSet(g_ui->stack_ui_Size + AXIS_2_Y, size);
}

void ui_HeightPop(void)
{
	stack_ui_SizePop(g_ui->stack_ui_Size + AXIS_2_Y);
}

void ui_FloatingPush(const enum axis_2 axis, const f32 pixel)
{
	stack_f32Push(g_ui->stack_floating + axis, pixel);
}

void ui_FloatingSet(const enum axis_2 axis, const f32 pixel)
{
	stack_f32Set(g_ui->stack_floating + axis, pixel);
}

void ui_FloatingPop(const enum axis_2 axis)
{
	stack_f32Pop(g_ui->stack_floating + axis);
}

void ui_ChildLayoutAxisPush(const enum axis_2 axis)
{
	stack_u32Push(&g_ui->stack_child_layout_axis, axis);
}

void ui_ChildLayoutAxisSet(const enum axis_2 axis)
{
	stack_u32Set(&g_ui->stack_child_layout_axis, axis);
}

void ui_ChildLayoutAxisPop(void)
{
	stack_u32Pop(&g_ui->stack_child_layout_axis);
}

void ui_IntvViewablePush(const enum axis_2 axis, const intv inv)
{
	stack_intvPush(g_ui->stack_viewable + axis, inv);
}

void ui_IntvViewableSet(const enum axis_2 axis, const intv inv)
{
	stack_intvSet(g_ui->stack_viewable + axis, inv);
}

void ui_IntvViewablePop(const enum axis_2 axis)
{
	stack_intvPop(g_ui->stack_viewable + axis);
}

void ui_BackgroundColorPush(const vec4 color)
{
	stackVec4Push(&g_ui->stack_background_color, color);
}

void ui_BackgroundColorSet(const vec4 color)
{
	stackVec4Set(&g_ui->stack_background_color, color);
}

void ui_BackgroundColorPop(void)
{
	stackVec4Pop(&g_ui->stack_background_color);
}

void ui_BorderColorPush(const vec4 color)
{
	stackVec4Push(&g_ui->stack_border_color, color);
}

void ui_BorderColorSet(const vec4 color)
{
	stackVec4Set(&g_ui->stack_border_color, color);
}

void ui_BorderColorPop(void)
{
	stackVec4Pop(&g_ui->stack_border_color);
}

void ui_SpriteColorPush(const vec4 color)
{
	stackVec4Push(&g_ui->stack_sprite_color, color);
}

void ui_SpriteColorSet(const vec4 color)
{
	stackVec4Set(&g_ui->stack_sprite_color, color);
}

void ui_SpriteColorPop(void)
{
	stackVec4Pop(&g_ui->stack_sprite_color);
}

void ui_GradientColorPush(const enum box_corner corner, const vec4 color)
{
	stackVec4Push(g_ui->stack_gradient_color + corner, color);
}

void ui_GradientColorSet(const enum box_corner corner, const vec4 color)
{
	stackVec4Set(g_ui->stack_gradient_color + corner, color);
}

void ui_GradientColorPop(const enum box_corner corner)
{
	stackVec4Pop(g_ui->stack_gradient_color + corner);
}

void ui_FontPush(const enum fontId font)
{
	struct assetFont *asset = AssetRequestFont(g_ui->mem_frame, font);
	stack_ptrPush(&g_ui->stack_font, asset);
}

void ui_FontSet(const enum fontId font)
{
	struct assetFont *asset = AssetRequestFont(g_ui->mem_frame, font);
	stack_ptrSet(&g_ui->stack_font, asset);
}

void ui_FontPop(void)
{
	stack_ptrPop(&g_ui->stack_font);
}

void ui_SpritePush(const enum spriteId sprite)
{
	stack_u32Push(&g_ui->stack_sprite, sprite);
}

void ui_SpriteSet(const enum spriteId sprite)
{
	stack_u32Set(&g_ui->stack_sprite, sprite);
}

void ui_SpritePop(void)
{
	stack_u32Pop(&g_ui->stack_sprite);
}

void ui_EdgeSoftnessPush(const f32 softness)
{
	stack_f32Push(&g_ui->stack_edge_softness, softness);
}

void ui_EdgeSoftnessSet(const f32 softness)
{
	stack_f32Set(&g_ui->stack_edge_softness, softness);
}

void ui_EdgeSoftnessPop(void)
{
	stack_f32Pop(&g_ui->stack_edge_softness);
}

void ui_CornerRadiusPush(const f32 radius)
{
	stack_f32Push(&g_ui->stack_corner_radius, radius);
}

void ui_CornerRadiusSet(const f32 radius)
{
	stack_f32Set(&g_ui->stack_corner_radius, radius);
}

void ui_CornerRadiusPop(void)
{
	stack_f32Pop(&g_ui->stack_corner_radius);
}

void ui_BorderSizePush(const f32 pixels)
{
	stack_f32Push(&g_ui->stack_border_size, pixels);
}

void ui_BorderSizeSet(const f32 pixels)
{
	stack_f32Set(&g_ui->stack_border_size, pixels);
}

void ui_BorderSizePop(void)
{
	stack_f32Pop(&g_ui->stack_border_size);
}

void ui_TextAlignXPush(const enum alignment_x align)
{
	stack_u32Push(&g_ui->stack_text_alignment_x, align);
}

void ui_TextAlignXSet(const enum alignment_x align)
{
	stack_u32Set(&g_ui->stack_text_alignment_x, align);
}

void ui_TextAlignXPop(void)
{
	stack_u32Pop(&g_ui->stack_text_alignment_x);
}

void ui_TextAlignYPush(const enum alignment_y align)
{
	stack_u32Push(&g_ui->stack_text_alignment_y, align);
}

void ui_TextAlignYSet(const enum alignment_y align)
{
	stack_u32Set(&g_ui->stack_text_alignment_y, align);
}

void ui_TextAlignYPop(void)
{
	stack_u32Pop(&g_ui->stack_text_alignment_y);
}

void ui_TextPadPush(const enum axis_2 axis, const f32 pad)
{
	stack_f32Push(g_ui->stack_text_pad + axis, pad);
}

void ui_TextPadSet(const enum axis_2 axis, const f32 pad)
{
	stack_f32Set(g_ui->stack_text_pad + axis, pad);
}

void ui_TextPadPop(const enum axis_2 axis)
{
	stack_f32Pop(g_ui->stack_text_pad + axis);
}

void ui_FlagsPush(const u64 flags)
{
	const u64 inherited_flags = stack_u64Top(&g_ui->stack_flags);
	stack_u64Push(&g_ui->stack_flags, inherited_flags | flags);
}

void ui_FlagsSet(const u64 flags)
{
	const u64 inherited_flags = stack_u64Top(&g_ui->stack_flags);
	stack_u64Set(&g_ui->stack_flags, inherited_flags | flags);
}

void ui_FlagsPop(void)
{
	stack_u64Pop(&g_ui->stack_flags);
}

void ui_PaddingPush(const f32 pad)
{
	stack_f32Push(&g_ui->stack_pad, pad);
}

void ui_PaddingSet(const f32 pad)
{
	stack_f32Set(&g_ui->stack_pad, pad);
}

void ui_PaddingPop(void)
{
	stack_f32Pop(&g_ui->stack_pad);
}

void ui_FixedDepthPush(const u32 depth)
{
	stack_u32Push(&g_ui->stack_fixed_depth, depth);
}

void ui_FixedDepthSet(const u32 depth)
{
	stack_u32Set(&g_ui->stack_fixed_depth, depth);
}

void ui_FixedDepthPop(void)
{
	stack_u32Pop(&g_ui->stack_fixed_depth);
}

void ui_ExternalTextPush(const utf32 text)
{
	stack_utf32Push(&g_ui->stack_external_text, text);
}

void ui_ExternalTextSet(const utf32 text)
{
	stack_utf32Set(&g_ui->stack_external_text, text);
}

void ui_ExternalTextPop(void)
{
	stack_utf32Pop(&g_ui->stack_external_text);
}

void ui_ExternalTextLayoutPush(struct textLayout *layout, const utf32 text)
{
	stack_ptrPush(&g_ui->stack_external_text_layout, layout);
	stack_utf32Push(&g_ui->stack_external_text, text);
}

void ui_ExternalTextLayoutSet(struct textLayout *layout, const utf32 text)
{
	stack_ptrSet(&g_ui->stack_external_text_layout, layout);
	stack_utf32Set(&g_ui->stack_external_text, text);
}

void ui_ExternalTextLayoutPop(void)
{
	stack_ptrPop(&g_ui->stack_external_text_layout);
}

void ui_ExternalTextInputPush(struct ui_TextInput *input)
{
	stack_ptrPush(&g_ui->stack_external_text_input, input);
}

void ui_ExternalTextInputPop(void)
{
	stack_ptrPop(&g_ui->stack_external_text_input);
}

void ui_RecursiveInteractionPush(const u64 flags)
{
	stack_u64Push(&g_ui->stack_recursive_interaction_flags, flags);
}

void ui_RecursiveInteractionPop(void)
{
	stack_u64Pop(&g_ui->stack_recursive_interaction_flags);
}
