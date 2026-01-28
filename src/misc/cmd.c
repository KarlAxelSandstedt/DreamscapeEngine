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

#include <stdlib.h> 

#include "cmd.h"
#include "hash_map.h"
#include "ds_vector.h"
#include "Log.h"
#include "sys_public.h"

DECLARE_STACK(cmd_function);
DEFINE_STACK(cmd_function);

static struct arena mem_persistent;
static struct hashMap *g_name_to_cmd_f_map;
struct cmd_queue *g_queue = NULL;
static stack_cmd_function g_cmd_f;
u32			  g_cmd_internal_debug_print_index;	

static void cmd_internal_debug_print(void)
{
	Utf8DebugPrint(g_queue->cmd_exec->arg[0].utf8);
	ThreadFree256B(g_queue->cmd_exec->arg[0].utf8.buf);
}

void cmd_alloc(void)
{
	g_name_to_cmd_f_map = HashMapAlloc(NULL, 128, 128, GROWABLE);
	g_cmd_f = stack_cmd_function_alloc(NULL, 128, STACK_GROWABLE);
	mem_persistent = ArenaAlloc1MB();

	const utf8 debug_print_str = Utf8Inline("debug_print");
	g_cmd_internal_debug_print_index = cmd_function_register(debug_print_str, 1, &cmd_internal_debug_print).index;
}

void cmd_free(void)
{
	HashMapFree(g_name_to_cmd_f_map);
	stack_cmd_function_free(&g_cmd_f);
	ArenaFree1MB(&mem_persistent);
}

struct cmd_queue *cmd_queue_alloc(void)
{
	struct cmd_queue *queue = malloc(sizeof(struct cmd_queue));
	queue->cmd_list = array_list_intrusive_alloc(NULL, 64, sizeof(struct cmd), ARRAY_LIST_GROWABLE);
	queue->cmd_first = U32_MAX;
	queue->cmd_last = U32_MAX;
	queue->cmd_first_next_frame = U32_MAX;
	queue->cmd_last_next_frame = U32_MAX;
	return queue;
}

void cmd_queue_free(struct cmd_queue *queue)
{
	if (queue)
	{
		array_list_intrusive_free(queue->cmd_list);
		free(queue);
	}
}

void cmd_queue_set(struct cmd_queue *queue)
{
	g_queue = queue;
}

enum cmd_token
{
	CMD_TOKEN_INVALID,
	CMD_TOKEN_STRING,
	CMD_TOKEN_I64,
	CMD_TOKEN_U64,
	CMD_TOKEN_F64,
	CMD_TOKEN_COUNT
};

static void cmd_tokenize_string(struct cmd *cmd)
{
	u64 i = 0;
	u32 token_count = 0;
	u8 *text = cmd->string.buf;
	u32 codepoints_left = cmd->string.len;
	u8 *token_start = NULL;
	u32 token_length = 0;
	utf8 token_string;

	while (codepoints_left && (text[i] == ' ' || text[i] == '\t' || text[i] == '\n'))
	{
		Utf8ReadCodepoint(&i, &cmd->string, i);
		codepoints_left -= 1;
	}

	token_start = text;
	while (codepoints_left && (text[i] != ' ' && text[i] != '\t' && text[i] != '\n'))
	{
		Utf8ReadCodepoint(&i, &cmd->string, i);
		token_length += 1;
		codepoints_left -= 1;
	}

	token_string = (utf8) { .buf = token_start, .len = token_length, .size = token_length };
	cmd->function = cmd_function_lookup(token_string).address;

	if (cmd->function == NULL)
	{
		cmd->function = g_cmd_f.arr + g_cmd_internal_debug_print_index;
		u8 *buf = ThreadAlloc256B();
		cmd->arg[0].utf8 = Utf8FormatBuffered(buf, 256, "Error in tokenizing %k: invalid command name", (char *) &cmd->string); 
		return;
	}

	struct arena tmp_arena = ArenaAlloc1MB();
	while (1)
	{
		while (codepoints_left && (text[i] == ' ' || text[i] == '\t' || text[i] == '\n'))
		{
			i += 1;
			codepoints_left -= 1;
		}

		if (!codepoints_left)
		{
			break;
		}

		if (token_count == cmd->function->args_count)
		{
			u8 *buf = ThreadAlloc256B();
			cmd->arg[0].utf8 = Utf8FormatBuffered(buf, 256, "Error in tokenizing %k: command expects %u arguments.", &cmd->string, cmd->function->args_count); 
			cmd->function = g_cmd_f.arr + g_cmd_internal_debug_print_index;
			break;
		}

		enum cmd_token token_type = CMD_TOKEN_INVALID;
		u8 *token_start = text + i;
		token_length = 0;
		if (text[i] == '"')
		{
			i += 1;
			codepoints_left -= 1;
			token_start += 1;
			while (codepoints_left && text[i] != '"')
			{
				Utf8ReadCodepoint(&i, &cmd->string, i);	
				token_length += 1;
				codepoints_left -= 1;
			}

			if (text[i] != '"')
			{
				cmd->function = g_cmd_f.arr + g_cmd_internal_debug_print_index;
				u8 *buf = ThreadAlloc256B();
				cmd->arg[0].utf8 = Utf8FormatBuffered(buf, 256, "Error in tokenizing %k: non-closed string beginning.", &cmd->string); 
				break;
			}
				
			i += 1;
			codepoints_left -= 1;
			token_type = CMD_TOKEN_STRING;
		}
		else
		{
			u32 sign = 0;
			u32 fraction = 0;

			if (text[i] == '-')
			{
				sign = 1;
				i += 1;
				codepoints_left -= 1;
				token_length += 1;
			}

			while (codepoints_left && '0' <= text[i] && text[i] <= '9')
			{
				i += 1;
				codepoints_left -= 1;
				token_length += 1;
			}

			if (codepoints_left && text[i] == '.')
			{
				fraction = 1;	
				do 
				{
					i += 1;
					codepoints_left -= 1;
					token_length += 1;
				} while (codepoints_left && '0' <= text[i] && text[i] <= '9');
			}

			if ((sign + 1 + 2*fraction <= token_length) && '0' <= text[i-1] && text[i-1] <= '9')
			{
				u32 min_length = 1 + sign + fraction*2;
				if (fraction)
				{
					token_type = CMD_TOKEN_F64;
				}
				else if (sign)
				{
					token_type = CMD_TOKEN_I64;
				}
				else
				{
					token_type = CMD_TOKEN_U64;
				}
			}
			ds_Assert(token_type != CMD_TOKEN_INVALID);
		}

		ds_Assert(!codepoints_left || text[i] == ' ' || text[i] == '\t' || text[i] == '\n');
		if (codepoints_left && (text[i] != ' ' && text[i] != '\t' && text[i] != '\n'))
		{
			token_type = CMD_TOKEN_INVALID;
		}

		token_string = (utf8) { .buf = token_start, .len = token_length, .size = token_length };
		struct parseRetval ret = { .op_result = PARSE_SUCCESS };
		switch (token_type)
		{
			case CMD_TOKEN_STRING:
			{
				cmd->arg[token_count++].utf8 = token_string;
			} break;

			case CMD_TOKEN_I64:
			{
				ret = I64Utf8(token_string);
				cmd->arg[token_count++].i64 = ret.i64;
			} break;

			case CMD_TOKEN_U64:
			{
				ret = U64Utf8(token_string);
				cmd->arg[token_count++].u64 = ret.u64;
			} break;

			case CMD_TOKEN_F64:
			{
				cmd->arg[token_count++].f64 = F64Utf8(&tmp_arena, token_string);
			} break;

			case CMD_TOKEN_INVALID:
			{
				ret.op_result = PARSE_STRING_INVALID;
			} break;
		}

		if (ret.op_result != PARSE_SUCCESS)
		{
			cmd->function = g_cmd_f.arr + g_cmd_internal_debug_print_index;
			u8 *buf = ThreadAlloc256B();
			switch (ret.op_result)
			{
				case PARSE_UNDERFLOW: 
				{ 
					cmd->arg[0].utf8 = Utf8FormatBuffered(buf, 256, "Error in tokenizing %k: signed integer underflow in argument %u", &cmd->string, token_count); 
				} break;

				case PARSE_OVERFLOW: 
				{ 
					cmd->arg[0].utf8 = Utf8FormatBuffered(buf, 256, "Error in tokenizing %k: integer overflow in argument %u", &cmd->string, token_count); 
				} break;

				case PARSE_STRING_INVALID: 
				{ 
					g_queue->cmd_exec->arg[0].utf8 = Utf8FormatBuffered(buf, 256, "Error in tokenizing %k: unexpected character in argument %k", &cmd->string, &token_string); 
				} break;
			}
			break;
		}
	}
	ArenaFree1MB(&tmp_arena);
}

void cmd_queue_execute(void)
{
	u32 next = U32_MAX;
	for (u32 i = g_queue->cmd_first; i != U32_MAX; i = next)
	{
		g_queue->cmd_exec = array_list_intrusive_address(g_queue->cmd_list, i);
		next = g_queue->cmd_exec->header.next;
		if (g_queue->cmd_exec->args_type == CMD_ARGS_TOKEN)
		{
			//Utf8DebugPrint(g_queue->cmd_exec->string);
			cmd_tokenize_string(g_queue->cmd_exec);
		}

		g_queue->cmd_exec->function->call();
		array_list_intrusive_remove_index(g_queue->cmd_list, i);
	}

	g_queue->cmd_first = g_queue->cmd_first_next_frame;
	g_queue->cmd_last = g_queue->cmd_last_next_frame;
	g_queue->cmd_first_next_frame = U32_MAX;
	g_queue->cmd_last_next_frame = U32_MAX;
}

void cmd_queue_flush(struct cmd_queue *queue)
{
	array_list_intrusive_flush(queue->cmd_list);
	queue->cmd_first = U32_MAX;
	queue->cmd_last = U32_MAX;
	queue->cmd_first_next_frame = U32_MAX;
	queue->cmd_last_next_frame = U32_MAX;
}

struct slot cmd_function_register(const utf8 name, const u32 args_count, void (*call)(void))
{
	if (CMD_REGISTER_COUNT < args_count)
	{
		return (struct slot) { .index = U32_MAX, .address = NULL };
	}

	const struct cmd_function cmd_f = { .name = name, .args_count = args_count, .call = call };
	struct slot slot = cmd_function_lookup(name);
	if (!slot.address)
	{
		slot.index = g_cmd_f.next;
		slot.address = g_cmd_f.arr + g_cmd_f.next;
		stack_cmd_function_push(&g_cmd_f, cmd_f);
	
		const u32 key = Utf8Hash(name);
		HashMapAdd(g_name_to_cmd_f_map, key, slot.index);
	}
	else
	{
		g_cmd_f.arr[slot.index] = cmd_f;
	}

	return slot;
}

struct slot cmd_function_lookup(const utf8 name)
{
	const u32 key = Utf8Hash(name);
	struct slot slot = { .index = HashMapFirst(g_name_to_cmd_f_map, key), .address = NULL };
	for (; slot.index != U32_MAX; slot.index = HashMapNext(g_name_to_cmd_f_map, slot.index))
	{
		if (Utf8Equivalence(g_cmd_f.arr[slot.index].name, name))
		{
			slot.address = g_cmd_f.arr + slot.index;
			break;
		}
	}

	return slot; 
}

void cmd_submit_f(struct arena *mem, const char *format, ...)
{
	va_list args;
	va_start(args, format);

	cmd_submit_utf8(Utf8FormatVariadic(mem, format, args));

	va_end(args);
}

void cmd_queue_submit_f(struct arena *mem, struct cmd_queue *queue, const char *format, ...)
{
	va_list args;
	va_start(args, format);

	cmd_queue_submit_utf8(queue, Utf8FormatVariadic(mem, format, args));

	va_end(args);
}

void cmd_submit_utf8(const utf8 string)
{
	cmd_queue_submit_utf8(g_queue, string);
}

void cmd_queue_submit_utf8(struct cmd_queue *queue, const utf8 string)
{
	const u32 index = array_list_intrusive_reserve_index(queue->cmd_list);
	struct cmd *cmd = array_list_intrusive_address(queue->cmd_list, index);
	cmd->args_type = CMD_ARGS_TOKEN;
	cmd->string = string;
	cmd->header.next = U32_MAX;

	if (queue->cmd_last != U32_MAX)
	{
		struct cmd *last = array_list_intrusive_address(queue->cmd_list, queue->cmd_last);
		last->header.next = index;
	}
	else
	{
		queue->cmd_first = index;
	}
	queue->cmd_last = index;
}

void cmd_submit(const u32 cmd_function)
{
	cmd_queue_submit(g_queue, cmd_function);
}

void cmd_queue_submit(struct cmd_queue *queue, const u32 cmd_function)
{
	const u32 index = array_list_intrusive_reserve_index(queue->cmd_list);
	struct cmd *cmd = array_list_intrusive_address(queue->cmd_list, index);
	cmd->args_type = CMD_ARGS_REGISTER;
	cmd->function = g_cmd_f.arr + cmd_function;
	cmd->header.next = U32_MAX;

	for (u32 i = 0; i < cmd->function->args_count; ++i)
	{
		cmd->arg[i] = queue->regs[i];
	}

	if (queue->cmd_last != U32_MAX)
	{
		struct cmd *last = array_list_intrusive_address(queue->cmd_list, queue->cmd_last);
		last->header.next = index;
	}
	else
	{
		queue->cmd_first = index;
	}
	queue->cmd_last = index;
}

void cmd_queue_submit_next_frame(struct cmd_queue *queue, const u32 cmd_function)
{
	const u32 index = array_list_intrusive_reserve_index(queue->cmd_list);
	struct cmd *cmd = array_list_intrusive_address(queue->cmd_list, index);
	cmd->args_type = CMD_ARGS_REGISTER;
	cmd->function = g_cmd_f.arr + cmd_function;
	cmd->header.next = U32_MAX;

	for (u32 i = 0; i < cmd->function->args_count; ++i)
	{
		cmd->arg[i] = queue->regs[i];
	}

	if (queue->cmd_last_next_frame != U32_MAX)
	{
		struct cmd *last = array_list_intrusive_address(queue->cmd_list, queue->cmd_last_next_frame);
		last->header.next = index;
	}
	else
	{
		queue->cmd_first_next_frame = index;
	}
	queue->cmd_last_next_frame = index;
}

void cmd_submit_next_frame(const u32 cmd_function)
{
	cmd_queue_submit_next_frame(g_queue, cmd_function);
}

void cmd_queue_submit_f_next_frame(struct arena *mem, struct cmd_queue *queue, const char *format, ...)
{
	va_list args;
	va_start(args, format);

	cmd_queue_submit_utf8_next_frame(queue, Utf8FormatVariadic(mem, format, args));

	va_end(args);
}

void cmd_submit_f_next_frame(struct arena *mem, const char *format, ...)
{
	va_list args;
	va_start(args, format);

	cmd_queue_submit_utf8_next_frame(g_queue, Utf8FormatVariadic(mem, format, args));

	va_end(args);
}

void cmd_queue_submit_utf8_next_frame(struct cmd_queue *queue, const utf8 string)
{
	const u32 index = array_list_intrusive_reserve_index(queue->cmd_list);
	struct cmd *cmd = array_list_intrusive_address(queue->cmd_list, index);
	cmd->args_type = CMD_ARGS_TOKEN;
	cmd->string = string;
	cmd->header.next = U32_MAX;

	if (queue->cmd_last_next_frame != U32_MAX)
	{
		struct cmd *last = array_list_intrusive_address(queue->cmd_list, queue->cmd_last_next_frame);
		last->header.next = index;
	}
	else
	{
		queue->cmd_first_next_frame = index;
	}
	queue->cmd_last_next_frame = index;
}

void cmd_submit_utf8_next_frame(const utf8 string)
{
	cmd_queue_submit_utf8_next_frame(g_queue, string);	
}

