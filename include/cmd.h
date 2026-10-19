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

#ifndef __DS_CMD_H__
#define __DS_CMD_H__

#ifdef __cplusplus
extern "C" { 
#endif

/*
   ========================================== CMD API ==========================================

   User submit commands either by utf8 command string or by the command's registered index. For the second case, we
   also have to push register values before submitting the command. CmdSubmit then reads for each of it's
   arguments the corresponding register, beginning with regs[0]. The command reads its arguments from
   g_queue->cmd_exec->arg[i] when called.

	Suppose we have the triple  { cmd_func : u32, str(func) : utf8, func() : void(*)(void) } where func() 
	normally would take in 2 arguments. For utf8, we format and submit a command executing the function by
	calling either one of the two following methods:

		CmdSubmitFormat(mem, "func %arg0 %arg1", arg0, arg1)
		CmdSubmitUtf8("func str(arg0) str(arg1)")

	The string is referenced, not copied: it (and mem) must live until the queue executes, one CmdQueueExecute
	later for the ...NextFrame variants. String arguments passed to the command point into it.
	
	The format string should, if the command in registered at compile time, determined at the same time. So
	we extend our triple to 

	{ cmd_func : u32, str(func) : utf8, str(func_format) : utf8,  func() : void(*)(void) }
		
	If we wish to sumbit a command using function's index, we manually "push" arguments by
	setting the registers to their corresponding arguments before calling CmdSubmit:

		g_queue->regs[0].arg0_type = arg0;
		g_queue->regs[1].arg1_type = arg1;
		CmdSubmit(cmd_index)
 */

#include "ds_base.h"
#include "ds_list.h"

enum cmdArgsType
{
	CMD_ARGS_TOKEN,		/* cmd is tokenized and matched against registered commands */
	CMD_ARGS_REGISTER,	/* cmd is identifiable */
	CMD_ARGS_COUNT
};

enum cmdId
{
	CMD_STATIC_COUNT
};

#define CMD_REGISTER_COUNT	16
union cmdRegister
{
	u8	u8;
	u16	u16;
	u32	u32;
	u64	u64;
	i8	i8;
	i16	i16;
	i32	i32;
	i64	i64;
	f32	f32;
	f64	f64;
	void *	ptr;
	utf8	utf8;
	utf32	utf32;
	intv	intv;
    v3      v3;
    v4      vec4;
};

typedef struct cmdFunction
{
	utf8	name;
	u32 	args_count;
	void	(*call)(void);
} cmdFunction;

struct cmd
{
	POOL_NODE;
	i32                         next;

	const struct cmdFunction *	function;
	utf8				        string;				        /* defined if args_type == TOKEN */
	union cmdRegister		    arg[CMD_REGISTER_COUNT];	/* defined if args_type == REGISTER */
	enum cmdArgsType		    args_type;	
};
DEFINE_POOL(cmd);

struct cmdQueue
{
	ds_Pool(cmd)	cmd_pool;
	struct LL	cmd_list;
	struct LL	cmd_list_next_frame;

	struct cmd *		cmd_exec;

	union cmdRegister	regs[CMD_REGISTER_COUNT];	/* defined if args_type == REGISTER */
};

extern struct cmdQueue *	g_queue;

/* init cmd infrastrucutre */
void 			ds_CmdApiInit(void);
/* free cmd infrastrucutre */
void 			ds_CmdApiShutdown(void);

struct cmdQueue 	CmdQueueAlloc(void);
void 			CmdQueueDealloc(struct cmdQueue *queue);
/* set queue to current global queue */
void			CmdQueueSet(struct cmdQueue *queue);
/*
 * Execute commands in global queue. A string command is tokenized first: the name, then arguments separated by
 * whitespace, each a quoted string, an integer (I64 with a '-', U64 without) or a decimal (F64). An unknown
 * name, a malformed argument or a wrong argument count prints an error instead of calling the command.
 * Commands submitted during execution run in the same pass (the ...NextFrame variants in the next one).
 */
void 			CmdQueueExecute(void);
/* flush any commands in queue */
void 			CmdQueueFlush(struct cmdQueue *queue);

/* Register the command function (or overwrite existing one); args_count > CMD_REGISTER_COUNT: { U32_MAX, NULL } */
struct slot		CmdFunctionRegister(const utf8 name, const u32 args_count, void (*call)(void));
/* Lookup command function; if function not found, return { .index = U32_MAX, .address = NULL } */
struct slot 		CmdFunctionLookup(const utf8 name);

/* push current global register values as command arguments and submit the command */
void			CmdSubmit(const u32 cmdFunction);
/* format a cmd string and submit the command */
void			CmdSubmitFormat(struct arena *mem, const char *format, ...);
/* submit a cmd string */
void			CmdSubmitUtf8(const utf8 string);

/*
 * As above, but sumbit command for next frame being built. This is useful when we want to continuously spawn
 * a command from itself, but only once a frame; Thus, during command execution, we are still building the
 * current frame, so any calls to these functions will construct a command for the next wave of commands,
 * as to not fall into an infinite loop.
 */
void			CmdSubmitNextFrame(const u32 cmdFunction);
void			CmdSubmitFormatNextFrame(struct arena *mem, const char *format, ...);
void			CmdSubmitUtf8NextFrame(const utf8 string);

/* push current local register values as command arguments and submit the command */
void			CmdQueueSubmit(struct cmdQueue *queue, const u32 cmdFunction);
/* format a cmd string and submit the command to the given command queue */
void 			CmdQueueSubmitFormat(struct arena *mem, struct cmdQueue *queue, const char *format, ...);
/* submit a cmd string to the given command queue */
void			CmdQueueSubmitUtf8(struct cmdQueue *queue, const utf8 string);

/* similar to above but for next frame command submission */
void			CmdQueueSubmitNextFrame(struct cmdQueue *queue, const u32 cmdFunction);
void			CmdQueueSubmitFormatNextFrame(struct arena *mem, struct cmdQueue *queue, const char *format, ...);
void			CmdQueueSubmitUtf8NextFrame(struct cmdQueue *queue, const utf8 string);

#ifdef __cplusplus
} 
#endif

#endif
