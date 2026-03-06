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

#include "ds_test.h"

struct block_allocator_stress_input
{
	u32 	allocations_left;
	u64		block_size;
};

struct list_node 
{
	void *next;
};

const u64 g_256B_count = 100000;
const u64 g_1MB_count = 10000;

struct ds_Struct
{
    TPOOL_NODE;
    u8  pad[52];
};

TPOOL_DECLARE(ds_Struct)
TPOOL_DEFINE(ds_Struct)

static struct ds_StructTPool *g_tpool;
static u32 first = 1;

void *ds_StructTPoolIncrementTestInit(void)
{
    if (ds_ThreadSelfIndex() == 0)
    {
        AtomicStoreRel64(&g_tpool, malloc(sizeof(struct ds_StructTPool)));
    }

    return &g_tpool;
}

void ds_StructTPoolIncrementTestReset(void *args)
{
    if (ds_ThreadSelfIndex() == 0)
    {
	    struct ds_StructTPool *pool = AtomicLoadAcq64(&g_tpool);
        if (!first)
        {
            ds_StructTPoolDealloc(pool);
        }
        first = 0;

        ds_StructTPoolAlloc(pool, g_arch_config->logical_core_count, 1);
    }
}

void ds_StructTPoolIncrementTestFree(void *args)
{
    if (ds_ThreadSelfIndex() == 0 && g_tpool)
    {
	    struct ds_StructTPool *pool = AtomicLoadAcq64(&g_tpool);
        g_tpool = NULL;
        ds_StructTPoolDealloc(pool);
        free(pool);
        first = 1;
    }
}

void ds_StructTPoolIncrementTest(void *void_pool)
{
	struct ds_StructTPool *pool = AtomicLoadAcq64(&g_tpool);
    for (u32 i = 0; i < 1024*64; ++i)
    {
        const u32 index = ds_StructTPoolIncrement(pool).index;
        struct ds_Struct *addr = ds_StructTPoolAddress(pool, index);
        memset(addr, 0xff, sizeof(*addr));
    }
}

void ds_StructTStackTest(void *void_pool)
{
	struct ds_StructTPool *pool = AtomicLoadAcq64(&g_tpool);
    const u32 count = 1000000;
    if (ds_ThreadSelfIndex() == 0)
    {
        for (u32 i = 0; i < count*pool->free_list_count; ++i)
        {
            struct slot slot = ds_StructTPoolIncrement(pool);
            ds_StructFreeListTStackPush(pool->t_free_list + 0, slot.index);
        }
    }

    for (u32 i = 0; i < count; )
    {
        struct slot slot = ds_StructFreeListTStackPop(pool->t_free_list + 0);
        if (slot.address)
        {
            i += 1;
            struct ds_Struct *addr = slot.address;
            memset(addr, 0xff, sizeof(*addr));
        }
    }
}

void *block_allocator_stress_test_256B_init(void)
{
	struct block_allocator_stress_input *args = malloc(sizeof(struct block_allocator_stress_input));
	args->allocations_left = g_256B_count;
	args->block_size = 256;
	return args;
}

void block_allocator_stress_test_256B_reset(void *args)
{
	struct block_allocator_stress_input *input = args;
	input->allocations_left = g_256B_count;
}

void block_allocator_stress_test_256B_free(void *args)
{
	free(args);
}

void *block_allocator_stress_test_1MB_init(void)
{
	struct block_allocator_stress_input *args = malloc(sizeof(struct block_allocator_stress_input));
	args->allocations_left = g_1MB_count;
	args->block_size = 1024*1024;
	return args;
}

void block_allocator_stress_test_1MB_reset(void *args)
{
	struct block_allocator_stress_input *input = args;
	input->allocations_left = g_1MB_count;
}

void block_allocator_stress_test_1MB_free(void *args)
{
	free(args);
}

void serial_block_allocator_test_256B(void *null)
{
	u32 allocations_left = g_256B_count;
	struct list_node *head = NULL;
	while (allocations_left || head)
	{
		if (allocations_left)
		{
			if (!head || RngU64Range(0, 1))
			{
				allocations_left -= 1;
				struct list_node *tmp = ThreadAlloc256B();
				tmp->next = head;
				head = tmp;
			}
			else
			{
				struct list_node *tmp = head->next;
				ThreadFree256B(head);
				head = tmp;
			}
		}
		else
		{
			struct list_node *tmp = head->next;
			ThreadFree256B(head);
			head = tmp;
		}
	}
}

void block_allocator_stress_test_256B(void *void_input)
{
	struct block_allocator_stress_input *input = void_input;
	struct list_node *head = NULL;
	while (input->allocations_left || head)
	{
		if (input->allocations_left)
		{
			if (!head || RngU64Range(0, 1))
			{
				input->allocations_left -= 1;
				struct list_node *tmp = ThreadAlloc256B();
				tmp->next = head;
				head = tmp;
			}
			else
			{
				struct list_node *tmp = head->next;
				ThreadFree256B(head);
				head = tmp;
			}
		}
		else
		{
			struct list_node *tmp = head->next;
			ThreadFree256B(head);
			head = tmp;
		}
	}
}

void block_allocator_stress_test_1MB(void *void_input)
{
	struct block_allocator_stress_input *input = void_input;

	struct list_node *head = NULL;
	while (input->allocations_left || head)
	{
		if (input->allocations_left)
		{
			if (!head || RngU64Range(0, 1))
			{
				struct list_node *tmp = ThreadAlloc1MB();
				if (!tmp)
				{
					continue;
				}
				input->allocations_left -= 1;
				tmp->next = head;
				head = tmp;
			}
			else
			{
				struct list_node *tmp = head->next;
				ThreadFree1MB(head);
				head = tmp;
			}
		}
		else
		{
			struct list_node *tmp = head->next;
			ThreadFree1MB(head);
			head = tmp;
		}
	}
}

void malloc_stress_test(void *void_input)
{
	struct block_allocator_stress_input *input = void_input;

	struct list_node *head = NULL;
	while (input->allocations_left || head)
	{
		if (input->allocations_left)
		{
			if (!head || RngU64Range(0, 1))
			{
				input->allocations_left -= 1;
				struct list_node *tmp = malloc(input->block_size);
				tmp->next = head;
				head = tmp;
			}
			else
			{
				struct list_node *tmp = head->next;
				free(head);
				head = tmp;
			}
		}
		else
		{
			struct list_node *tmp = head->next;
			free(head);
			head = tmp;
		}
	}
}

struct test_PerformanceSerial allocator_serial_test[] =
{
	{
		.id = "serial_block_allocator_256B_test",
		.size = g_256B_count * 256,
		.test = &serial_block_allocator_test_256B,
		.test_init = NULL,
		.test_reset = NULL,
		.test_free = NULL,
	},
};

struct suite_Correctness storage_allocator_correctness_suite =
{
    0
};

struct test_PerformanceParallel allocator_parallel_test[] =
{
    {
        .id = "ds_StructTStackTest",
        .size = 1,
        .test = &ds_StructTStackTest,
        .test_init = &ds_StructTPoolIncrementTestInit,
        .test_reset = &ds_StructTPoolIncrementTestReset,
        .test_free = &ds_StructTPoolIncrementTestFree,
    },

    {
        .id = "ds_StructTPoolIncrementTest",
        .size = 1,
        .test = &ds_StructTPoolIncrementTest,
        .test_init = &ds_StructTPoolIncrementTestInit,
        .test_reset = &ds_StructTPoolIncrementTestReset,
        .test_free = &ds_StructTPoolIncrementTestFree,
    },

	{
		.id = "parallel_block_allocator_256B_stress_test",
		.size = g_256B_count * 256,
		.test = &block_allocator_stress_test_256B,
		.test_init = &block_allocator_stress_test_256B_init,
		.test_reset = &block_allocator_stress_test_256B_reset,
		.test_free = &block_allocator_stress_test_256B_free,
	},

	{
		.id = "parallel_malloc_256B_stress_test",
		.size = g_256B_count * 256,
		.test = &malloc_stress_test,
		.test_init = &block_allocator_stress_test_256B_init,
		.test_reset = &block_allocator_stress_test_256B_reset,
		.test_free = &block_allocator_stress_test_256B_free,
	},

	{
		.id = "parallel_block_allocator_1MB_stress_test",
		.size = g_1MB_count * 1024*1024,
		.test = &block_allocator_stress_test_1MB,
		.test_init = &block_allocator_stress_test_1MB_init,
		.test_reset = &block_allocator_stress_test_1MB_reset,
		.test_free = &block_allocator_stress_test_1MB_free,
	},

	{
		.id = "parallel_malloc_1MB_stress_test",
		.size = g_1MB_count * 1024*1024,
		.test = &malloc_stress_test,
		.test_init = &block_allocator_stress_test_1MB_init,
		.test_reset = &block_allocator_stress_test_1MB_reset,
		.test_free = &block_allocator_stress_test_1MB_free,
	},
};

struct suite_Performance storage_performance_allocator_suite =
{
	.id = "Allocator Performance",
	.parallel_test = allocator_parallel_test,
	.parallel_test_count = sizeof(allocator_parallel_test) / sizeof(allocator_parallel_test[0]),
	.serial_test = allocator_serial_test,
	.serial_test_count = sizeof(allocator_serial_test) / sizeof(allocator_serial_test[0]),
};

struct suite_Performance *allocator_performance_suite = &storage_performance_allocator_suite;
struct suite_Correctness *allocator_correctness_suite = &storage_allocator_correctness_suite;
