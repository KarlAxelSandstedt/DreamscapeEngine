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

#include "ds_base.h"
#include "ds_list.h"

/*
cmd.h
=====
Commands: functions void (*)(void) registered under a name with an argument count, queued on a command queue
and run in order by CmdQueueExecute. A command is submitted in one of two ways:

	as a string "name arg0 arg1 ...", tokenized when the queue executes (CmdSubmitUtf8, CmdSubmitFormat);
	by its registered index, the arguments set in the queue's registers first (CmdSubmit):

		g_queue->regs[0].u32 = arg0;
		g_queue->regs[1].utf8 = arg1;
		CmdSubmit(cmd_index);

Either way the command reads its arguments as g_queue->cmd_exec->arg[i] when it runs. The Cmd... forms act on
the global queue (CmdQueueSet), the CmdQueue... forms on the given queue. The plain forms queue the command for
the next execute (the running one, when submitted from inside a command); the ...NextFrame forms for the execute
after that.

Strings are referenced, not copied: a submitted string (and the arena CmdSubmitFormat formats on) lives until the
command has run, and the string arguments a command receives point into it. Registered names are referenced too.

Sections: types, the command table, queues, submission.
*/

/********************************** types **********************************/

/* how a queued command carries its arguments */
enum cmdArgsType
{
	CMD_ARGS_TOKEN,		/* a string, tokenized at execution; the arguments are then in arg[] */
	CMD_ARGS_REGISTER,	/* arg[] was copied from the queue's registers at submission */
	CMD_ARGS_COUNT
};

enum cmdId
{
	CMD_STATIC_COUNT
};

/* one register per argument; the command reads the member of the type it expects */
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

/* a registered command: name is referenced, args_count <= CMD_REGISTER_COUNT */
typedef struct cmdFunction
{
	utf8	name;
	u32 	args_count;
	void	(*call)(void);
} cmdFunction;

/*
 * a queued command: function is the command's index in the table (resolved at execution, so the table may grow
 * meanwhile); string until a CMD_ARGS_TOKEN command is tokenized, arg[0..args_count) once it has been or for
 * CMD_ARGS_REGISTER
 */
struct cmd
{
	POOL_NODE;
	i32                         next;

	u32				            function;
	utf8				        string;				        /* defined if args_type == TOKEN */
	union cmdRegister		    arg[CMD_REGISTER_COUNT];	/* defined if args_type == REGISTER */
	enum cmdArgsType		    args_type;	
};
DEFINE_POOL(cmd);

/*
 * a queue: the commands of the next execute (cmd_list) and of the one after (cmd_list_next_frame), the command
 * being run (cmd_exec, where a running command reads its arguments) and the registers CmdQueueSubmit copies
 */
struct cmdQueue
{
	ds_Pool(cmd)	cmd_pool;
	struct LL	cmd_list;
	struct LL	cmd_list_next_frame;

	struct cmd *		cmd_exec;

	union cmdRegister	regs[CMD_REGISTER_COUNT];	/* defined if args_type == REGISTER */
};

/* the global queue (CmdQueueSet): the Cmd... forms and running commands use it */
extern struct cmdQueue *	g_queue;

/********************************** the command table **********************************/

/* init cmd infrastrucutre: the command table (heap), with "debug_print" registered (one string argument, printed) */
void 			ds_CmdApiInit(void);
/* free cmd infrastrucutre; queues are freed separately (CmdQueueDealloc) */
void 			ds_CmdApiShutdown(void);

/*
 * Register the command function (or overwrite existing one, which keeps its index). name is referenced, not
 * copied. The slot's index is the command's id for CmdSubmit, stable for the program's lifetime;
 * { U32_MAX, NULL } when args_count > CMD_REGISTER_COUNT.
 */
struct slot		CmdFunctionRegister(const utf8 name, const u32 args_count, void (*call)(void));
/* Lookup command function; if function not found, return { .index = U32_MAX, .address = NULL } */
struct slot 		CmdFunctionLookup(const utf8 name);

/********************************** queues **********************************/

/* a queue of its own, on the heap (growable) */
struct cmdQueue 	CmdQueueAlloc(void);
/* free the queue's memory; if it is the global queue, set another before the next Cmd... call */
void 			CmdQueueDealloc(struct cmdQueue *queue);
/* set queue to current global queue */
void			CmdQueueSet(struct cmdQueue *queue);
/*
 * Execute commands in global queue, in submission order, each removed after its call. A string command is
 * tokenized first: the name, then arguments separated by whitespace, each a quoted string, an integer (I64 with
 * a '-', U64 without) or a decimal (F64). An unknown name, a malformed argument or a wrong argument count prints
 * an error instead of calling the command. Commands submitted during execution run in the same pass (the
 * ...NextFrame variants in the next one). Afterwards the next-frame list is the queue's list. Pushes one scratch
 * arena for the pass.
 */
void 			CmdQueueExecute(void);
/* flush any commands in queue, for this execute and the next; nothing is run */
void 			CmdQueueFlush(struct cmdQueue *queue);

/********************************** submission **********************************/

/*
 * push current global register values as command arguments and submit the command: regs[0..args_count) are
 * copied now, so the registers are free afterwards. cmdFunction is the index CmdFunctionRegister or
 * CmdFunctionLookup returned.
 */
void			CmdSubmit(const u32 cmdFunction);
/* format a cmd string on mem and submit the command; mem lives until the command has run */
void			CmdSubmitFormat(struct arena *mem, const char *format, ...);
/* submit a cmd string; it is referenced and lives until the command has run */
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

/* as the Cmd... forms above, on the given queue (its registers for CmdQueueSubmit) */
void			CmdQueueSubmit(struct cmdQueue *queue, const u32 cmdFunction);
void 			CmdQueueSubmitFormat(struct arena *mem, struct cmdQueue *queue, const char *format, ...);
void			CmdQueueSubmitUtf8(struct cmdQueue *queue, const utf8 string);

/* as the ...NextFrame forms above, on the given queue */
void			CmdQueueSubmitNextFrame(struct cmdQueue *queue, const u32 cmdFunction);
void			CmdQueueSubmitFormatNextFrame(struct arena *mem, struct cmdQueue *queue, const char *format, ...);
void			CmdQueueSubmitUtf8NextFrame(struct cmdQueue *queue, const utf8 string);

#ifdef __cplusplus
} 
#endif

#endif
