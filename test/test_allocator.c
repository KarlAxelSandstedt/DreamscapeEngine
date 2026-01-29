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

#include "test_local.h"

struct block_allocator_stress_input
{
	u32 				allocations_left;
	u64				block_size;
};

struct list_node 
{
	void *next;
};

const u64 g_256B_count = 100000;
const u64 g_1MB_count = 10000;

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

struct serial_test allocator_serial_test[] =
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

struct parallel_test allocator_parallel_test[] =
{
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

struct performance_suite storage_performance_allocator_suite =
{
	.id = "Allocator Performance",
	.parallel_test = allocator_parallel_test,
	.parallel_test_count = sizeof(allocator_parallel_test) / sizeof(allocator_parallel_test[0]),
	.serial_test = allocator_serial_test,
	.serial_test_count = sizeof(allocator_serial_test) / sizeof(allocator_serial_test[0]),
};

struct performance_suite *allocator_performance_suite = &storage_performance_allocator_suite;
