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

#include <stdio.h>

#include "ds_test.h"
#include "array_list.h"
#include "hierarchy_index.h"

static struct test_Output array_list_slot_size(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };
	struct array_list *list;

	list = array_list_alloc(env->mem_1, 4, 1, 0);
	TEST_EQUAL(list->data_size, 1);
	TEST_EQUAL(list->slot_size, 4);

	list = array_list_alloc(env->mem_1, 4, 2, 0);
	TEST_EQUAL(list->slot_size, 4);

	list = array_list_alloc(env->mem_1, 4, 4, 0);
	TEST_EQUAL(list->slot_size, 4);

	list = array_list_alloc(env->mem_1, 4, 8, 0);
	TEST_EQUAL(list->slot_size, 8);

	list = array_list_alloc(env->mem_1, 4, 12, 0);
	TEST_EQUAL(list->slot_size, 12);

	TEST_EQUAL(list->length, 4);
	TEST_EQUAL(list->count, 0);

	return output;
}

static struct test_Output array_list_try_overflow(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };
	struct array_list *list = array_list_alloc(env->mem_1, 4, 16, 0);
	TEST_NOT_EQUAL(list, NULL);

	struct { u64 a; u64 b; } s;
	assert(sizeof(s) == 16);

	TEST_EQUAL((void *) (list->slot + 0 * 16), array_list_reserve(list));
	TEST_EQUAL((void *) (list->slot + 1 * 16), array_list_reserve(list));
	TEST_EQUAL((void *) (list->slot + 2 * 16), array_list_reserve(list));
	TEST_EQUAL((void *) (list->slot + 3 * 16), array_list_reserve(list));
	TEST_EQUAL(NULL, array_list_reserve(list));

	return output;
}

static struct test_Output array_list_add_remove_add(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };
	struct array_list *list = array_list_alloc(env->mem_1, 4, 16, 0);

	array_list_reserve(list);
	TEST_EQUAL(list->max_count, 1);
	array_list_remove_index(list, 0);
	TEST_EQUAL((void *) (list->slot + 0 * 16), array_list_reserve(list));
	TEST_EQUAL(list->max_count, 1);

	array_list_reserve(list);
	array_list_reserve(list);
	TEST_EQUAL(list->max_count, 3);
	TEST_EQUAL(list->count, 3);

	array_list_remove_index(list, 0);
	TEST_EQUAL((void *) (list->slot + 0 * 16), array_list_reserve(list));
	TEST_EQUAL(list->max_count, 3);

	array_list_remove_index(list, 1);
	TEST_EQUAL((void *) (list->slot + 1 * 16), array_list_reserve(list));
	TEST_EQUAL(list->max_count, 3);

	array_list_remove_index(list, 2);
	TEST_EQUAL((void *) (list->slot + 2 * 16), array_list_reserve(list));
	TEST_EQUAL(list->max_count, 3);

	TEST_EQUAL((void *) (list->slot + 3 * 16), array_list_reserve(list));
	TEST_EQUAL(list->max_count, 4);

	array_list_remove(list, (list->slot + 0 * 16));
	TEST_EQUAL((void *) (list->slot + 0 * 16), array_list_reserve(list));

	array_list_remove(list, (list->slot + 1 * 16));
	TEST_EQUAL((void *) (list->slot + 1 * 16), array_list_reserve(list));

	array_list_remove(list, (list->slot + 2 * 16));
	TEST_EQUAL((void *) (list->slot + 2 * 16), array_list_reserve(list));

	array_list_remove(list, (list->slot + 3 * 16));
	TEST_EQUAL((void *) (list->slot + 3 * 16), array_list_reserve(list));

	return output;
}

struct test_struct
{
	struct hierarchy_index_node node;
	u32 val[10];	
};

static struct test_Output hierarchy_index_add_remove_single(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

	struct hierarchy_index *hi = hierarchy_index_alloc(env->mem_1, 64, sizeof(struct test_struct), 0);
	struct hierarchy_index_node *root = hierarchy_index_address(hi, HI_ROOT_STUB_INDEX);
	TEST_NOT_EQUAL(hi, NULL);
	TEST_EQUAL(root->parent, HI_NULL_INDEX);
	TEST_EQUAL(root->next, HI_NULL_INDEX);
	TEST_EQUAL(root->prev, HI_NULL_INDEX);
	TEST_EQUAL(root->first, HI_NULL_INDEX);
	TEST_EQUAL(root->last, HI_NULL_INDEX);
	
	const u32 i1 = hierarchy_index_add(hi, HI_ROOT_STUB_INDEX).index;
	struct hierarchy_index_node *n1 = hierarchy_index_address(hi, i1);
	TEST_EQUAL(root->parent, HI_NULL_INDEX);
	TEST_EQUAL(root->next, HI_NULL_INDEX);
	TEST_EQUAL(root->prev, HI_NULL_INDEX);
	TEST_EQUAL(root->first, i1);
	TEST_EQUAL(root->last, i1);

	TEST_EQUAL(n1->parent, HI_ROOT_STUB_INDEX);
	TEST_EQUAL(n1->next, HI_NULL_INDEX);
	TEST_EQUAL(n1->prev, HI_NULL_INDEX);
	TEST_EQUAL(n1->first, HI_NULL_INDEX);
	TEST_EQUAL(n1->last, HI_NULL_INDEX);

	const u32 i2 = hierarchy_index_add(hi, HI_ROOT_STUB_INDEX).index;
	struct hierarchy_index_node *n2 = hierarchy_index_address(hi, i2);

	TEST_EQUAL(root->parent, HI_NULL_INDEX);
	TEST_EQUAL(root->next, HI_NULL_INDEX);
	TEST_EQUAL(root->prev, HI_NULL_INDEX);
	TEST_EQUAL(root->first, i1);
	TEST_EQUAL(root->last, i2);

	TEST_EQUAL(n1->parent, HI_ROOT_STUB_INDEX);
	TEST_EQUAL(n1->next, i2);
	TEST_EQUAL(n1->prev, HI_NULL_INDEX);
	TEST_EQUAL(n1->first, HI_NULL_INDEX);
	TEST_EQUAL(n1->last, HI_NULL_INDEX);

	TEST_EQUAL(n2->parent, HI_ROOT_STUB_INDEX);
	TEST_EQUAL(n2->next, HI_NULL_INDEX);
	TEST_EQUAL(n2->prev, i1);
	TEST_EQUAL(n2->first, HI_NULL_INDEX);
	TEST_EQUAL(n2->last, HI_NULL_INDEX);

	const u32 i3 = hierarchy_index_add(hi, HI_ROOT_STUB_INDEX).index;
	struct hierarchy_index_node *n3 = hierarchy_index_address(hi, i3);

	TEST_EQUAL(root->parent, HI_NULL_INDEX);
	TEST_EQUAL(root->next, HI_NULL_INDEX);
	TEST_EQUAL(root->prev, HI_NULL_INDEX);
	TEST_EQUAL(root->first, i1);
	TEST_EQUAL(root->last, i3);

	TEST_EQUAL(n1->parent, HI_ROOT_STUB_INDEX);
	TEST_EQUAL(n1->next, i2);
	TEST_EQUAL(n1->prev, HI_NULL_INDEX);
	TEST_EQUAL(n1->first, HI_NULL_INDEX);
	TEST_EQUAL(n1->last, HI_NULL_INDEX);

	TEST_EQUAL(n2->parent, HI_ROOT_STUB_INDEX);
	TEST_EQUAL(n2->next, i3);
	TEST_EQUAL(n2->prev, i1);
	TEST_EQUAL(n2->first, HI_NULL_INDEX);
	TEST_EQUAL(n2->last, HI_NULL_INDEX);

	TEST_EQUAL(n3->parent, HI_ROOT_STUB_INDEX);
	TEST_EQUAL(n3->next, HI_NULL_INDEX);
	TEST_EQUAL(n3->prev, i2);
	TEST_EQUAL(n3->first, HI_NULL_INDEX);
	TEST_EQUAL(n3->last, HI_NULL_INDEX);

	hierarchy_index_remove(env->mem_1, hi, i2);

	TEST_EQUAL(root->parent, HI_NULL_INDEX);
	TEST_EQUAL(root->next, HI_NULL_INDEX);
	TEST_EQUAL(root->prev, HI_NULL_INDEX);
	TEST_EQUAL(root->first, i1);
	TEST_EQUAL(root->last, i3);

	TEST_EQUAL(n1->parent, HI_ROOT_STUB_INDEX);
	TEST_EQUAL(n1->next, i3);
	TEST_EQUAL(n1->prev, HI_NULL_INDEX);
	TEST_EQUAL(n1->first, HI_NULL_INDEX);
	TEST_EQUAL(n1->last, HI_NULL_INDEX);

	TEST_EQUAL(n3->parent, HI_ROOT_STUB_INDEX);
	TEST_EQUAL(n3->next, HI_NULL_INDEX);
	TEST_EQUAL(n3->prev, i1);
	TEST_EQUAL(n3->first, HI_NULL_INDEX);
	TEST_EQUAL(n3->last, HI_NULL_INDEX);

	hierarchy_index_remove(env->mem_1, hi, i3);

	TEST_EQUAL(root->parent, HI_NULL_INDEX);
	TEST_EQUAL(root->next, HI_NULL_INDEX);
	TEST_EQUAL(root->prev, HI_NULL_INDEX);
	TEST_EQUAL(root->first, i1);
	TEST_EQUAL(root->last, i1);

	TEST_EQUAL(n1->parent, HI_ROOT_STUB_INDEX);
	TEST_EQUAL(n1->next, HI_NULL_INDEX);
	TEST_EQUAL(n1->prev, HI_NULL_INDEX);
	TEST_EQUAL(n1->first, HI_NULL_INDEX);
	TEST_EQUAL(n1->last, HI_NULL_INDEX);

	const u32 i4 = hierarchy_index_add(hi, HI_ROOT_STUB_INDEX).index;
	struct hierarchy_index_node *n4 = hierarchy_index_address(hi, i4);

	TEST_EQUAL(root->parent, HI_NULL_INDEX);
	TEST_EQUAL(root->next, HI_NULL_INDEX);
	TEST_EQUAL(root->prev, HI_NULL_INDEX);
	TEST_EQUAL(root->first, i1);
	TEST_EQUAL(root->last, i4);

	TEST_EQUAL(n1->parent, HI_ROOT_STUB_INDEX);
	TEST_EQUAL(n1->next, i4);
	TEST_EQUAL(n1->prev, HI_NULL_INDEX);
	TEST_EQUAL(n1->first, HI_NULL_INDEX);
	TEST_EQUAL(n1->last, HI_NULL_INDEX);

	TEST_EQUAL(n4->parent, HI_ROOT_STUB_INDEX);
	TEST_EQUAL(n4->next, HI_NULL_INDEX);
	TEST_EQUAL(n4->prev, i1);
	TEST_EQUAL(n4->first, HI_NULL_INDEX);
	TEST_EQUAL(n4->last, HI_NULL_INDEX);

	hierarchy_index_remove(env->mem_1, hi, i1);

	TEST_EQUAL(root->parent, HI_NULL_INDEX);
	TEST_EQUAL(root->next, HI_NULL_INDEX);
	TEST_EQUAL(root->prev, HI_NULL_INDEX);
	TEST_EQUAL(root->first, i4);
	TEST_EQUAL(root->last, i4);

	TEST_EQUAL(n4->parent, HI_ROOT_STUB_INDEX);
	TEST_EQUAL(n4->next, HI_NULL_INDEX);
	TEST_EQUAL(n4->prev, HI_NULL_INDEX);
	TEST_EQUAL(n4->first, HI_NULL_INDEX);
	TEST_EQUAL(n4->last, HI_NULL_INDEX);

	hierarchy_index_remove(env->mem_1, hi, i4);

	TEST_EQUAL(root->parent, HI_NULL_INDEX);
	TEST_EQUAL(root->next, HI_NULL_INDEX);
	TEST_EQUAL(root->prev, HI_NULL_INDEX);
	TEST_EQUAL(root->first, HI_NULL_INDEX);
	TEST_EQUAL(root->last, HI_NULL_INDEX);

	return output;
}

static struct test_Output hierarchy_index_add_remove_sub_hierarchy(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

	struct hierarchy_index *hi = hierarchy_index_alloc(env->mem_1, 64, sizeof(struct test_struct), 0);

	const u32 i1 = hierarchy_index_add(hi, HI_ROOT_STUB_INDEX).index;

	const u32 i11 = hierarchy_index_add(hi, i1).index;
	const u32 i12 = hierarchy_index_add(hi, i1).index;
	const u32 i13 = hierarchy_index_add(hi, i1).index;
	const u32 i14 = hierarchy_index_add(hi, i1).index;
	const u32 i111 = hierarchy_index_add(hi, i11).index;
	const u32 i112 = hierarchy_index_add(hi, i11).index;
	const u32 i113 = hierarchy_index_add(hi, i11).index;
	const u32 i121 = hierarchy_index_add(hi, i12).index;
	const u32 i122 = hierarchy_index_add(hi, i12).index;
	const u32 i131 = hierarchy_index_add(hi, i13).index;

	struct hierarchy_index_node * root = hierarchy_index_address(hi, HI_ROOT_STUB_INDEX);
	struct hierarchy_index_node * n1 = hierarchy_index_address(hi, i1);
	struct hierarchy_index_node * n11 = hierarchy_index_address(hi, i11);
	struct hierarchy_index_node * n12 = hierarchy_index_address(hi, i12);
	struct hierarchy_index_node * n13 = hierarchy_index_address(hi, i13);
	struct hierarchy_index_node * n14 = hierarchy_index_address(hi, i14);
	struct hierarchy_index_node * n111 = hierarchy_index_address(hi, i111);
	struct hierarchy_index_node * n112 = hierarchy_index_address(hi, i112);
	struct hierarchy_index_node * n113 = hierarchy_index_address(hi, i113);
	struct hierarchy_index_node * n121 = hierarchy_index_address(hi, i121);
	struct hierarchy_index_node * n122 = hierarchy_index_address(hi, i122);
	struct hierarchy_index_node * n131 = hierarchy_index_address(hi, i131);

	TEST_EQUAL(root->parent , HI_NULL_INDEX);
	TEST_EQUAL(root->prev 	, HI_NULL_INDEX);
	TEST_EQUAL(root->next 	, HI_NULL_INDEX);
	TEST_EQUAL(root->first 	, i1);
	TEST_EQUAL(root->last 	, i1);

	TEST_EQUAL(n1->parent 	, HI_ROOT_STUB_INDEX);
	TEST_EQUAL(n1->prev 	, HI_NULL_INDEX);
	TEST_EQUAL(n1->next 	, HI_NULL_INDEX);
	TEST_EQUAL(n1->first 	, i11);
	TEST_EQUAL(n1->last 	, i14);

	TEST_EQUAL(n11->parent 	, i1);
	TEST_EQUAL(n11->prev 	, HI_NULL_INDEX);
	TEST_EQUAL(n11->next 	, i12);
	TEST_EQUAL(n11->first 	, i111);
	TEST_EQUAL(n11->last 	, i113);

	TEST_EQUAL(n12->parent 	, i1);
	TEST_EQUAL(n12->prev 	, i11);
	TEST_EQUAL(n12->next 	, i13);
	TEST_EQUAL(n12->first 	, i121);
	TEST_EQUAL(n12->last 	, i122);

	TEST_EQUAL(n13->parent 	, i1);
	TEST_EQUAL(n13->prev 	, i12);
	TEST_EQUAL(n13->next 	, i14);
	TEST_EQUAL(n13->first 	, i131);
	TEST_EQUAL(n13->last 	, i131);

	TEST_EQUAL(n14->parent 	, i1);
	TEST_EQUAL(n14->prev 	, i13);
	TEST_EQUAL(n14->next 	, HI_NULL_INDEX);
	TEST_EQUAL(n14->first 	, HI_NULL_INDEX);
	TEST_EQUAL(n14->last 	, HI_NULL_INDEX);

	TEST_EQUAL(n111->parent	, i11);
	TEST_EQUAL(n111->prev 	, HI_NULL_INDEX);
	TEST_EQUAL(n111->next 	, i112);
	TEST_EQUAL(n111->first 	, HI_NULL_INDEX);
	TEST_EQUAL(n111->last 	, HI_NULL_INDEX);

	TEST_EQUAL(n112->parent	, i11);
	TEST_EQUAL(n112->prev 	, i111);
	TEST_EQUAL(n112->next 	, i113);
	TEST_EQUAL(n112->first 	, HI_NULL_INDEX);
	TEST_EQUAL(n112->last 	, HI_NULL_INDEX);

	TEST_EQUAL(n113->parent	, i11);
	TEST_EQUAL(n113->prev 	, i112);
	TEST_EQUAL(n113->next 	, HI_NULL_INDEX);
	TEST_EQUAL(n113->first 	, HI_NULL_INDEX);
	TEST_EQUAL(n113->last 	, HI_NULL_INDEX);

	TEST_EQUAL(n121->parent	, i12);
	TEST_EQUAL(n121->prev 	, HI_NULL_INDEX);
	TEST_EQUAL(n121->next 	, i122);
	TEST_EQUAL(n121->first 	, HI_NULL_INDEX);
	TEST_EQUAL(n121->last 	, HI_NULL_INDEX);

	TEST_EQUAL(n122->parent	, i12);
	TEST_EQUAL(n122->prev 	, i121);
	TEST_EQUAL(n122->next 	, HI_NULL_INDEX);
	TEST_EQUAL(n122->first 	, HI_NULL_INDEX);
	TEST_EQUAL(n122->last 	, HI_NULL_INDEX);

	TEST_EQUAL(n131->parent , i13);
	TEST_EQUAL(n131->prev 	, HI_NULL_INDEX);
	TEST_EQUAL(n131->next 	, HI_NULL_INDEX);
	TEST_EQUAL(n131->first 	, HI_NULL_INDEX);
	TEST_EQUAL(n131->last 	, HI_NULL_INDEX);

	TEST_EQUAL(hi->list->count, 13);

	hierarchy_index_remove(env->mem_1, hi, i1);

	TEST_EQUAL(hi->list->count, 2);

	TEST_EQUAL(root->parent , HI_NULL_INDEX);
	TEST_EQUAL(root->prev 	, HI_NULL_INDEX);
	TEST_EQUAL(root->next 	, HI_NULL_INDEX);
	TEST_EQUAL(root->first 	, HI_NULL_INDEX);
	TEST_EQUAL(root->last 	, HI_NULL_INDEX);

	return output;
}

static struct test_Output hierarchy_index_add_remove_sub_hierarchy_recursive(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

	struct hierarchy_index *hi = hierarchy_index_alloc(env->mem_1, 64, sizeof(struct test_struct), 0);

	const u32 i1 = hierarchy_index_add(hi, HI_ROOT_STUB_INDEX).index;

	const u32 i11 = hierarchy_index_add(hi, i1).index;
	const u32 i12 = hierarchy_index_add(hi, i1).index;
	const u32 i13 = hierarchy_index_add(hi, i1).index;
	const u32 i14 = hierarchy_index_add(hi, i1).index;
	const u32 i111 = hierarchy_index_add(hi, i11).index;
	const u32 i112 = hierarchy_index_add(hi, i11).index;
	const u32 i113 = hierarchy_index_add(hi, i11).index;
	const u32 i121 = hierarchy_index_add(hi, i12).index;
	const u32 i122 = hierarchy_index_add(hi, i12).index;
	const u32 i131 = hierarchy_index_add(hi, i13).index;

	struct hierarchy_index_node * root = hierarchy_index_address(hi, HI_ROOT_STUB_INDEX);

	TEST_EQUAL(root->parent , HI_NULL_INDEX);
	TEST_EQUAL(root->prev 	, HI_NULL_INDEX);
	TEST_EQUAL(root->next 	, HI_NULL_INDEX);
	TEST_EQUAL(root->first 	, i1);
	TEST_EQUAL(root->last 	, i1);

	struct arena empty = { 0 };
	hierarchy_index_remove(&empty, hi, i1);

	TEST_EQUAL(hi->list->count, 2);

	TEST_EQUAL(root->parent , HI_NULL_INDEX);
	TEST_EQUAL(root->prev 	, HI_NULL_INDEX);
	TEST_EQUAL(root->next 	, HI_NULL_INDEX);
	TEST_EQUAL(root->first 	, HI_NULL_INDEX);
	TEST_EQUAL(root->last 	, HI_NULL_INDEX);

	return output;
}

static struct test_Output (*array_list_tests[])(struct test_Environment *) =
{
	array_list_slot_size,
	array_list_try_overflow,
	array_list_add_remove_add,	
};

static struct test_Output(*hierarchy_index_tests[])(struct test_Environment *) =
{
	hierarchy_index_add_remove_single,
	hierarchy_index_add_remove_sub_hierarchy,
	hierarchy_index_add_remove_sub_hierarchy_recursive,
};

struct test_CorrectnessRepetition_Correctness m_array_list_suite =
{
	.id = "array_list",
	.unit_test = array_list_tests,
	.unit_test_count = sizeof(array_list_tests) / sizeof(array_list_tests[0]),
};

struct test_CorrectnessRepetition_Correctness m_hierarchy_index_suite =
{
	.id = "hierarchy_index",
	.unit_test = hierarchy_index_tests,
	.unit_test_count = sizeof(hierarchy_index_tests) / sizeof(hierarchy_index_tests[0]),
};

struct test_CorrectnessRepetition_Correctness *array_list_suite = &m_array_list_suite;
struct test_CorrectnessRepetition_Correctness *hierarchy_index_suite = &m_hierarchy_index_suite;


