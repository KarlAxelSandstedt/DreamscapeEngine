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

#include <stdlib.h> 

#include "cmd.h"
#include "ds_hash_map.h"

POOL_DEFINE(cmd);
DEFINE_CPOOL_STRUCT(cmdFunction);

static struct ds_HashMap g_name_to_cmd_f_map;
struct cmdQueue *g_queue = NULL;
static ds_CPool(cmdFunction) g_cmd_f;
u32			  g_cmd_internal_debug_print_index;	

//TODO REMOVE mallocs/free in debug printing

static void CmdDebugPrint(void)
{
	Utf8DebugPrint(g_queue->cmd_exec->arg[0].utf8);
	free(g_queue->cmd_exec->arg[0].utf8.buf);
}

void ds_CmdApiInit(void)
{
	g_name_to_cmd_f_map = ds_HashMapAlloc(NULL, 128, 128, GROWABLE);
	ds_CPoolAlloc(NULL, g_cmd_f, 128, GROWABLE);

	const utf8 debug_print_str = Utf8Inline("debug_print");
	g_cmd_internal_debug_print_index = CmdFunctionRegister(debug_print_str, 1, &CmdDebugPrint).index;
}

void ds_CmdApiShutdown(void)
{
	ds_HashMapDealloc(&g_name_to_cmd_f_map);
	ds_CPoolDealloc(g_cmd_f);
}

struct cmdQueue CmdQueueAlloc(void)
{
	struct cmdQueue queue = { 0 };
	queue.cmd_pool = cmdPoolAlloc(NULL, 64, GROWABLE);
	ds_LLFlush(queue.cmd_list);
	ds_LLFlush(queue.cmd_list_next_frame);
	return queue;
}

void CmdQueueDealloc(struct cmdQueue *queue)
{
	cmdPoolDealloc(&queue->cmd_pool);
}

void CmdQueueSet(struct cmdQueue *queue)
{
	g_queue = queue;
}

enum cmdToken
{
	CMD_TOKEN_INVALID,
	CMD_TOKEN_STRING,
	CMD_TOKEN_I64,
	CMD_TOKEN_U64,
	CMD_TOKEN_F64,
	CMD_TOKEN_COUNT
};

static void CmdTokenizeString(struct arena *tmp, struct cmd *cmd)
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
	cmd->function = CmdFunctionLookup(token_string).address;

	if (cmd->function == NULL)
	{
		cmd->function = g_cmd_f.buf + g_cmd_internal_debug_print_index;
		u8 *buf = malloc(256);
		cmd->arg[0].utf8 = Utf8FormatBuffered(buf, 256, "Error in tokenizing %k: invalid command name", (char *) &cmd->string); 
		return;
	}

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
			u8 *buf = malloc(256);
			cmd->arg[0].utf8 = Utf8FormatBuffered(buf, 256, "Error in tokenizing %k: command expects %u arguments.", &cmd->string, cmd->function->args_count); 
			cmd->function = g_cmd_f.buf + g_cmd_internal_debug_print_index;
			break;
		}

		enum cmdToken token_type = CMD_TOKEN_INVALID;
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
				cmd->function = g_cmd_f.buf + g_cmd_internal_debug_print_index;
				u8 *buf = malloc(256);
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
				cmd->arg[token_count++].f64 = F64Utf8(tmp, token_string);
			} break;

			case CMD_TOKEN_INVALID:
			{
				ret.op_result = PARSE_STRING_INVALID;
			} break;
		}

		if (ret.op_result != PARSE_SUCCESS)
		{
			cmd->function = g_cmd_f.buf + g_cmd_internal_debug_print_index;
			u8 *buf = malloc(256);
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
}

void CmdQueueExecute(void)
{
	struct arena *tmp = ArenaPushScratch();

	u32 next = U32_MAX;
	for (i32 i = g_queue->cmd_list.first; i != LL_SENTINEL; i = next)
	{
		g_queue->cmd_exec = g_queue->cmd_pool.buf + i;
		next = g_queue->cmd_exec->next;
		if (g_queue->cmd_exec->args_type == CMD_ARGS_TOKEN)
		{
			//Utf8DebugPrint(g_queue->cmd_exec->string);
			CmdTokenizeString(tmp, g_queue->cmd_exec);
		}
		g_queue->cmd_exec->function->call();
		cmdPoolRemove(&g_queue->cmd_pool, i);
	}

	g_queue->cmd_list = g_queue->cmd_list_next_frame;
	ds_LLFlush(g_queue->cmd_list_next_frame);

	ArenaPopScratch();
}

void CmdQueueFlush(struct cmdQueue *queue)
{
	cmdPoolFlush(&queue->cmd_pool);
	ds_LLFlush(g_queue->cmd_list);
	ds_LLFlush(g_queue->cmd_list_next_frame);
}

struct slot CmdFunctionRegister(const utf8 name, const u32 args_count, void (*call)(void))
{
	if (CMD_REGISTER_COUNT < args_count)
	{
		return (struct slot) { .index = U32_MAX, .address = NULL };
	}

	const struct cmdFunction cmd_f = { .name = name, .args_count = args_count, .call = call };
	struct slot slot = CmdFunctionLookup(name);
	if (!slot.address)
	{
		slot = ds_CPoolPush(g_cmd_f);
        memcpy(slot.address, &cmd_f, sizeof(cmd_f));
	
		const u32 key = Utf8Hash(name);
		ds_HashMapAdd(&g_name_to_cmd_f_map, key, slot.index);
	}
	else
	{
		g_cmd_f.buf[slot.index] = cmd_f;
	}

	return slot;
}

struct slot CmdFunctionLookup(const utf8 name)
{
	const u32 key = Utf8Hash(name);
	struct slot slot = { .index = ds_HashMapFirst(&g_name_to_cmd_f_map, key), .address = NULL };
	for (; slot.index != U32_MAX; slot.index = ds_HashMapNext(&g_name_to_cmd_f_map, slot.index))
	{
		if (Utf8Equivalence(g_cmd_f.buf[slot.index].name, name))
		{
			slot.address = g_cmd_f.buf + slot.index;
			break;
		}
	}

	return slot; 
}

void CmdSubmitFormat(struct arena *mem, const char *format, ...)
{
	va_list args;
	va_start(args, format);

	CmdSubmitUtf8(Utf8FormatVariadic(mem, format, args));

	va_end(args);
}

void CmdQueueSubmitFormat(struct arena *mem, struct cmdQueue *queue, const char *format, ...)
{
	va_list args;
	va_start(args, format);

	CmdQueueSubmitUtf8(queue, Utf8FormatVariadic(mem, format, args));

	va_end(args);
}

void CmdSubmitUtf8(const utf8 string)
{
	CmdQueueSubmitUtf8(g_queue, string);
}

void CmdQueueSubmitUtf8(struct cmdQueue *queue, const utf8 string)
{
	struct slot slot = cmdPoolAdd(&queue->cmd_pool);
	struct cmd *cmd = slot.address;
	cmd->args_type = CMD_ARGS_TOKEN;
	cmd->string = string;

	ds_LLAppend(queue->cmd_list, queue->cmd_pool.buf, slot.index, next);
}

void CmdSubmit(const u32 cmdFunction)
{
	CmdQueueSubmit(g_queue, cmdFunction);
}

void CmdQueueSubmit(struct cmdQueue *queue, const u32 cmdFunction)
{
	struct slot slot = cmdPoolAdd(&queue->cmd_pool);
	struct cmd *cmd = slot.address;
	cmd->args_type = CMD_ARGS_REGISTER;
	cmd->function = g_cmd_f.buf + cmdFunction;

	for (u32 i = 0; i < cmd->function->args_count; ++i)
	{
		cmd->arg[i] = queue->regs[i];
	}

	ds_LLAppend(queue->cmd_list, queue->cmd_pool.buf, slot.index, next);
}

void CmdQueueSubmitNextFrame(struct cmdQueue *queue, const u32 cmdFunction)
{
	struct slot slot = cmdPoolAdd(&queue->cmd_pool);
	struct cmd *cmd = slot.address;
	cmd->args_type = CMD_ARGS_REGISTER;
	cmd->function = g_cmd_f.buf + cmdFunction;

	for (u32 i = 0; i < cmd->function->args_count; ++i)
	{
		cmd->arg[i] = queue->regs[i];
	}

	ds_LLAppend(queue->cmd_list_next_frame, queue->cmd_pool.buf, slot.index, next);
}

void CmdSubmitNextFrame(const u32 cmdFunction)
{
	CmdQueueSubmitNextFrame(g_queue, cmdFunction);
}

void CmdQueueSubmitFormatNextFrame(struct arena *mem, struct cmdQueue *queue, const char *format, ...)
{
	va_list args;
	va_start(args, format);

	CmdQueueSubmitUtf8NextFrame(queue, Utf8FormatVariadic(mem, format, args));

	va_end(args);
}

void CmdSubmitFormatNextFrame(struct arena *mem, const char *format, ...)
{
	va_list args;
	va_start(args, format);

	CmdQueueSubmitUtf8NextFrame(g_queue, Utf8FormatVariadic(mem, format, args));

	va_end(args);
}

void CmdQueueSubmitUtf8NextFrame(struct cmdQueue *queue, const utf8 string)
{
	struct slot slot = cmdPoolAdd(&queue->cmd_pool);
	struct cmd *cmd = slot.address;
	cmd->args_type = CMD_ARGS_TOKEN;
	cmd->string = string;

	ds_LLAppend(queue->cmd_list_next_frame, queue->cmd_pool.buf, slot.index, next);
}

void CmdSubmitUtf8NextFrame(const utf8 string)
{
	CmdQueueSubmitUtf8NextFrame(g_queue, string);	
}

