/*
==========================================================================
    Copyright (C) 2026 Axel Sandstedt 

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

#include <stdio.h>

#include "ds_test.h"
#include "ds_asset.h"
#include "ds_ui.h"

/*
The UI's layout, caching and hover, without a window: a ui of its own, frames of 800 x 600, no text (fonts
load, nothing is laid out as text).

UiLayoutTest: a 400 x 200 panel laying its children out along x: a 100 x 50 pixel child keeps both sizes, a
child of full percentage height gets the panel's, a quarter-width child gets 100, and they pack from the panel's left
edge; a stack of childsum height with two 40 x 30 children is 40 x 60 and sits right of the panel (childsum
sums both axes, so a childsum width would be 80 here: springclean #271).

UiCacheTest: a node built again next frame keeps its index; one not built is pruned (its lookup fails), the
frame's node count follows.

UiHoverTest: the cursor over the first child hovers it (node_hovered is its id, the HOVER interaction set on
it next frame); outside every node nothing is hovered.
*/

static struct ui_Visual UiTestVisual(void)
{
    const v4 gradient[BOX_CORNER_COUNT] = { V4(0.0f, 0.15f, 0.8f, 0.8f), V4(0.0f, 0.7f, 0.25f, 0.8f), V4(0.0f, 0.7f, 0.25f, 0.8f), V4(0.0f, 0.15f, 0.8f, 0.8f) };
    return ui_VisualInit(V4(0.1f, 0.1f, 0.1f, 1.0f), V4(0.5f, 0.5f, 0.5f, 1.0f), gradient, V4(0.9f, 0.9f, 0.9f, 1.0f), 8.0f, 0.0f, 3.0f, 1.0f, FONT_DEFAULT_SMALL, ALIGN_X_CENTER, ALIGN_Y_CENTER, 4.0f, 4.0f);
}

static u32 UiTestRect(const struct ui_Node *node, const f32 x, const f32 y, const f32 w, const f32 h)
{
    return node->pixel_position.x == x && node->pixel_position.y == y && node->pixel_size.x == w && node->pixel_size.y == h;
}

/* the frame's scene: the panel with three children and the stack; returns through the slots */
static void UiTestBuild(struct slot *panel, struct slot *a, struct slot *b, struct slot *c, struct slot *stack)
{
    ui_Width(ui_SizePixel(400.0f, 1.0f))
    ui_Height(ui_SizePixel(200.0f, 1.0f))
    ui_ChildLayoutAxis(AXIS_2_X)
    *panel = ui_NodeAllocF(UI_DRAW_BACKGROUND, "###panel");
    ui_Parent(panel->index)
    {
        ui_Width(ui_SizePixel(100.0f, 1.0f))
        ui_Height(ui_SizePixel(50.0f, 1.0f))
        *a = ui_NodeAllocF(UI_DRAW_BACKGROUND, "###a");

        ui_Width(ui_SizePixel(100.0f, 1.0f))
        ui_Height(ui_SizePerc(1.0f))
        *b = ui_NodeAllocF(UI_DRAW_BACKGROUND, "###b");

        ui_Width(ui_SizePerc(0.25f))
        ui_Height(ui_SizePixel(20.0f, 1.0f))
        *c = ui_NodeAllocF(UI_DRAW_BACKGROUND, "###c");
    }

    ui_Width(ui_SizePixel(40.0f, 1.0f))
    ui_Height(ui_SizeChildsum(1.0f))
    ui_ChildLayoutAxis(AXIS_2_Y)
    *stack = ui_NodeAllocF(UI_DRAW_BACKGROUND, "###stack");
    ui_Parent(stack->index)
    {
        ui_Width(ui_SizePixel(40.0f, 1.0f))
        ui_Height(ui_SizePixel(30.0f, 1.0f))
        {
            ui_NodeAllocF(UI_DRAW_BACKGROUND, "###s0");
            ui_NodeAllocF(UI_DRAW_BACKGROUND, "###s1");
        }
    }
}

static struct test_Output UiLayoutTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    struct ui *ui = ui_Alloc();
    ui_Set(ui);
    const struct ui_Visual visual = UiTestVisual();

    struct slot panel, a, b, c, stack;
    ui_FrameBegin(V2U32(800, 600), &visual);
    UiTestBuild(&panel, &a, &b, &c, &stack);
    ui_FrameEnd();

    const struct ui_Node *root = ui_NodeAddress(ui->root);
    const struct ui_Node *node_a = a.address;
    TEST_TRUE(UiTestRect(root, 0.0f, 0.0f, 800.0f, 600.0f));
    TEST_TRUE(UiTestRect(panel.address, 0.0f, 0.0f, 400.0f, 200.0f));
    if (!UiTestRect(node_a, 0.0f, 0.0f, 100.0f, 50.0f))
    {
        fprintf(stderr, "ui: the 100 x 50 child is (%g, %g) %g x %g\n", node_a->pixel_position.x, node_a->pixel_position.y, node_a->pixel_size.x, node_a->pixel_size.y);
        TEST_FAILURE;
    }
    TEST_TRUE(UiTestRect(b.address, 100.0f, 0.0f, 100.0f, 200.0f));
    TEST_TRUE(UiTestRect(c.address, 200.0f, 0.0f, 100.0f, 20.0f));
    const struct ui_Node *node_stack = stack.address;
    const struct ui_Node *s0 = ui_NodeLookup(&Utf8Inline("s0")).address;
    const struct ui_Node *s1 = ui_NodeLookup(&Utf8Inline("s1")).address;
    TEST_TRUE(s0 && s1);
    if (!UiTestRect(node_stack, 400.0f, 0.0f, 40.0f, 60.0f) || !UiTestRect(s0, 400.0f, 30.0f, 40.0f, 30.0f) || !UiTestRect(s1, 400.0f, 0.0f, 40.0f, 30.0f))
    {
        fprintf(stderr, "ui: stack (%g, %g) %g x %g, s0 (%g, %g) %g x %g, s1 (%g, %g) %g x %g\n", node_stack->pixel_position.x, node_stack->pixel_position.y, node_stack->pixel_size.x, node_stack->pixel_size.y, s0->pixel_position.x, s0->pixel_position.y, s0->pixel_size.x, s0->pixel_size.y, s1->pixel_position.x, s1->pixel_position.y, s1->pixel_size.x, s1->pixel_size.y);
        TEST_FAILURE;
    }
    TEST_TRUE(ui->node_count_frame == 8);

    ui_Dealloc(ui);
    return output;
}

static struct test_Output UiCacheTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    struct ui *ui = ui_Alloc();
    ui_Set(ui);
    const struct ui_Visual visual = UiTestVisual();

    struct slot panel, a, b, c, stack;
    ui_FrameBegin(V2U32(800, 600), &visual);
    UiTestBuild(&panel, &a, &b, &c, &stack);
    ui_FrameEnd();
    const u32 a_index = a.index;
    const u32 stack_index = stack.index;

    /* frame 2: only the panel and its first child */
    ui_FrameBegin(V2U32(800, 600), &visual);
    ui_Width(ui_SizePixel(400.0f, 1.0f))
    ui_Height(ui_SizePixel(200.0f, 1.0f))
    ui_ChildLayoutAxis(AXIS_2_X)
    panel = ui_NodeAllocF(UI_DRAW_BACKGROUND, "###panel");
    ui_Parent(panel.index)
    {
        ui_Width(ui_SizePixel(100.0f, 1.0f))
        ui_Height(ui_SizePixel(50.0f, 1.0f))
        a = ui_NodeAllocF(UI_DRAW_BACKGROUND, "###a");
    }
    ui_FrameEnd();
    TEST_TRUE(a.index == a_index && ui_NodeLookup(&Utf8Inline("a")).index == a_index);
    TEST_TRUE(ui_NodeLookup(&Utf8Inline("stack")).address == NULL && ui_NodeLookup(&Utf8Inline("b")).address == NULL);
    TEST_TRUE(ui->node_count_frame == 3 && ui->node_count_prev_frame == 8);

    /* frame 3: the stack returns in a new slot or the freed one, with a fresh layout */
    ui_FrameBegin(V2U32(800, 600), &visual);
    UiTestBuild(&panel, &a, &b, &c, &stack);
    ui_FrameEnd();
    TEST_TRUE(a.index == a_index && ui_NodeLookup(&Utf8Inline("stack")).index == stack.index);
    TEST_TRUE(UiTestRect(stack.address, 400.0f, 0.0f, 40.0f, 60.0f));
    (void) stack_index;

    ui_Dealloc(ui);
    return output;
}

static struct test_Output UiHoverTest(struct test_Environment *env)
{
    struct test_Output output = { .success = 1, .id = __func__ };
    struct ui *ui = ui_Alloc();
    ui_Set(ui);
    const struct ui_Visual visual = UiTestVisual();

    struct slot panel, a, b, c, stack;
    ui->inter.cursor_position = V2(50.0f, 25.0f);
    ui_FrameBegin(V2U32(800, 600), &visual);
    UiTestBuild(&panel, &a, &b, &c, &stack);
    ui_FrameEnd();
    TEST_TRUE(Utf8Equivalence(ui->inter.node_hovered, Utf8Inline("a")));

    /* the interaction reaches the node when it is built again */
    ui_FrameBegin(V2U32(800, 600), &visual);
    UiTestBuild(&panel, &a, &b, &c, &stack);
    ui_FrameEnd();
    const struct ui_Node *node_a = a.address;
    const struct ui_Node *node_b = b.address;
    TEST_TRUE((node_a->inter & UI_INTER_HOVER) && (node_b->inter & UI_INTER_HOVER) == 0);

    /* outside every child, the root itself is hovered */
    ui->inter.cursor_position = V2(700.0f, 500.0f);
    ui_FrameBegin(V2U32(800, 600), &visual);
    UiTestBuild(&panel, &a, &b, &c, &stack);
    ui_FrameEnd();
    TEST_TRUE(Utf8Equivalence(ui->inter.node_hovered, ui_NodeAddress(ui->root)->id));
    ui_FrameBegin(V2U32(800, 600), &visual);
    UiTestBuild(&panel, &a, &b, &c, &stack);
    ui_FrameEnd();
    node_a = a.address;
    TEST_TRUE((node_a->inter & UI_INTER_HOVER) == 0);

    ui_Dealloc(ui);
    return output;
}

static struct test_Output (*ui_tests[])(struct test_Environment *) =
{
    UiLayoutTest,
    UiCacheTest,
    UiHoverTest,
};

struct suite_Correctness m_ui_suite =
{
    .id = "ui",
    .unit_test = ui_tests,
    .unit_test_count = sizeof(ui_tests) / sizeof(ui_tests[0]),
};

struct suite_Correctness *ui_correctness_suite = &m_ui_suite;
