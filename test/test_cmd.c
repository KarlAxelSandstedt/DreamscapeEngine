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

#include "ds_test.h"
#include "cmd.h"

/*
The command queue on a queue of its own (the global queue is set back afterwards); the test commands are
registered once, on the engine's command table.

CmdParseTest: a string command with a quoted string, a negative integer, an unsigned integer and a decimal
reaches its function with those values; an unknown name, a bare word, a digit run with letters, an unclosed
quote, too few and too many arguments do not call it (they print an error), and every queued slot is freed
after the execute.

CmdChainTest: a command that submits the next link of a chain for the same frame runs the whole chain in one
execute, also when it is the last command queued; a command that re-submits itself for the next frame runs
once per execute; flushing a second queue leaves the global queue alone; debug_print through the registers
prints a string it does not own.

CmdRegisterTest: a lookup finds a registered name and not an unknown one, re-registering keeps the index and
takes the new function, more than CMD_REGISTER_COUNT arguments are refused.
*/

static u32 m_cmd_calls;
static utf8 m_cmd_str;
static i64 m_cmd_i64;
static u64 m_cmd_u64;
static f64 m_cmd_f64;
static u32 m_cmd_chain;
static u32 m_cmd_spawn_index;
static u32 m_cmd_again;
static u32 m_cmd_again_index;

static void CmdTestArgs(void)
{
	m_cmd_calls += 1;
	m_cmd_str = g_queue->cmd_exec->arg[0].utf8;
	m_cmd_i64 = g_queue->cmd_exec->arg[1].i64;
	m_cmd_u64 = g_queue->cmd_exec->arg[2].u64;
	m_cmd_f64 = g_queue->cmd_exec->arg[3].f64;
}

/* the chain: n links left, the next one submitted for this frame */
static void CmdTestSpawn(void)
{
	m_cmd_chain += 1;
	const u64 n = g_queue->cmd_exec->arg[0].u64;
	if (n > 0)
	{
		g_queue->regs[0].u64 = n - 1;
		CmdSubmit(m_cmd_spawn_index);
	}
}

static void CmdTestAgain(void)
{
	m_cmd_again += 1;
	CmdSubmitNextFrame(m_cmd_again_index);
}

static void CmdTestOther(void)
{
	m_cmd_calls += 100;
}

static void CmdTestRegister(void)
{
	if (CmdFunctionLookup(Utf8Inline("test_cmd_args")).address == NULL)
	{
		CmdFunctionRegister(Utf8Inline("test_cmd_args"), 4, &CmdTestArgs);
		m_cmd_spawn_index = CmdFunctionRegister(Utf8Inline("test_cmd_spawn"), 1, &CmdTestSpawn).index;
		m_cmd_again_index = CmdFunctionRegister(Utf8Inline("test_cmd_again"), 0, &CmdTestAgain).index;
	}
}

static struct test_Output CmdParseTest(struct test_Environment *env)
{
	struct test_Output output = { .id = "CmdParseTest", .success = 1 };
	(void) env;

	CmdTestRegister();
	struct cmdQueue *old = g_queue;
	struct cmdQueue queue = CmdQueueAlloc();
	CmdQueueSet(&queue);

	m_cmd_calls = 0;
	CmdSubmitUtf8(Utf8Inline("  test_cmd_args \"hello world\" -12 34 2.5 "));
	CmdQueueExecute();
	TEST_TRUE(m_cmd_calls == 1);
	TEST_TRUE(Utf8Equivalence(m_cmd_str, Utf8Inline("hello world")));
	TEST_TRUE(m_cmd_i64 == -12 && m_cmd_u64 == 34 && m_cmd_f64 == 2.5);
	TEST_TRUE(queue.cmd_pool.count == 0 && queue.cmd_list.count == 0);

	/* every one of these prints an error and calls nothing */
	CmdSubmitUtf8(Utf8Inline("test_cmd_unknown 1 2 3 4"));
	CmdSubmitUtf8(Utf8Inline("test_cmd_args abc 1 2 3"));
	CmdSubmitUtf8(Utf8Inline("test_cmd_args \"x\" 12abc 2 3"));
	CmdSubmitUtf8(Utf8Inline("test_cmd_args \"unclosed 1 2 3"));
	CmdSubmitUtf8(Utf8Inline("test_cmd_args \"x\" 1 2"));
	CmdSubmitUtf8(Utf8Inline("test_cmd_args \"x\" 1 2 3 4"));
	CmdSubmitUtf8(Utf8Inline("test_cmd_args"));
	CmdSubmitUtf8(Utf8Inline(""));
	CmdQueueExecute();
	TEST_TRUE(m_cmd_calls == 1);
	TEST_TRUE(queue.cmd_pool.count == 0 && queue.cmd_list.count == 0);

	/* a valid one after them still runs */
	CmdSubmitUtf8(Utf8Inline("test_cmd_args \"\" -1 0 -0.25"));
	CmdQueueExecute();
	TEST_TRUE(m_cmd_calls == 2 && m_cmd_str.len == 0 && m_cmd_i64 == -1 && m_cmd_u64 == 0 && m_cmd_f64 == -0.25);

	CmdQueueSet(old);
	CmdQueueDealloc(&queue);
	return output;
}

static struct test_Output CmdChainTest(struct test_Environment *env)
{
	struct test_Output output = { .id = "CmdChainTest", .success = 1 };
	(void) env;

	CmdTestRegister();
	struct cmdQueue *old = g_queue;
	struct cmdQueue queue = CmdQueueAlloc();
	CmdQueueSet(&queue);

	/* the spawner is the last command queued: its links still run in this execute */
	m_cmd_chain = 0;
	CmdSubmitUtf8(Utf8Inline("test_cmd_spawn 3"));
	CmdQueueExecute();
	TEST_TRUE(m_cmd_chain == 4);
	TEST_TRUE(queue.cmd_pool.count == 0 && queue.cmd_list.count == 0);

	/* and in the middle of the list */
	m_cmd_chain = 0;
	m_cmd_calls = 0;
	CmdSubmitUtf8(Utf8Inline("test_cmd_spawn 2"));
	CmdSubmitUtf8(Utf8Inline("test_cmd_args \"a\" 1 2 3"));
	CmdQueueExecute();
	TEST_TRUE(m_cmd_chain == 3 && m_cmd_calls == 1);
	TEST_TRUE(queue.cmd_pool.count == 0 && queue.cmd_list.count == 0);

	/* next frame: once per execute */
	m_cmd_again = 0;
	CmdSubmitUtf8(Utf8Inline("test_cmd_again"));
	CmdQueueExecute();
	TEST_TRUE(m_cmd_again == 1 && queue.cmd_list.count == 1 && queue.cmd_list_next_frame.count == 0);
	CmdQueueExecute();
	TEST_TRUE(m_cmd_again == 2 && queue.cmd_list.count == 1);

	/* flushing another queue leaves this one */
	struct cmdQueue other = CmdQueueAlloc();
	CmdQueueSubmitUtf8(&other, Utf8Inline("test_cmd_again"));
	CmdQueueSubmitUtf8NextFrame(&other, Utf8Inline("test_cmd_again"));
	TEST_TRUE(other.cmd_list.count == 1 && other.cmd_list_next_frame.count == 1);
	CmdQueueFlush(&other);
	TEST_TRUE(other.cmd_pool.count == 0 && other.cmd_list.count == 0 && other.cmd_list_next_frame.count == 0);
	TEST_TRUE(queue.cmd_list.count == 1 && queue.cmd_pool.count == 1);
	CmdQueueFlush(&queue);
	TEST_TRUE(queue.cmd_pool.count == 0 && queue.cmd_list.count == 0);
	CmdQueueDealloc(&other);

	/* debug_print through the registers: the string is not the queue's to free */
	const struct slot print = CmdFunctionLookup(Utf8Inline("debug_print"));
	TEST_TRUE(print.address != NULL);
	queue.regs[0].utf8 = Utf8Inline("cmd test: debug_print through the registers");
	CmdSubmit(print.index);
	CmdQueueExecute();
	TEST_TRUE(queue.cmd_pool.count == 0);

	CmdQueueSet(old);
	CmdQueueDealloc(&queue);
	return output;
}

static struct test_Output CmdRegisterTest(struct test_Environment *env)
{
	struct test_Output output = { .id = "CmdRegisterTest", .success = 1 };
	(void) env;

	CmdTestRegister();
	struct cmdQueue *old = g_queue;
	struct cmdQueue queue = CmdQueueAlloc();
	CmdQueueSet(&queue);

	const struct slot args = CmdFunctionLookup(Utf8Inline("test_cmd_args"));
	const struct cmdFunction *args_f = args.address;
	TEST_TRUE(args_f != NULL && args_f->args_count == 4 && args_f->call == &CmdTestArgs);
	const struct slot none = CmdFunctionLookup(Utf8Inline("test_cmd_none"));
	TEST_TRUE(none.address == NULL && none.index == U32_MAX);
	const struct slot refused = CmdFunctionRegister(Utf8Inline("test_cmd_refused"), CMD_REGISTER_COUNT + 1, &CmdTestOther);
	TEST_TRUE(refused.address == NULL && refused.index == U32_MAX && CmdFunctionLookup(Utf8Inline("test_cmd_refused")).address == NULL);

	/* re-registering keeps the index and takes the new function */
	const struct slot again = CmdFunctionRegister(Utf8Inline("test_cmd_args"), 4, &CmdTestOther);
	const struct cmdFunction *again_f = again.address;
	TEST_TRUE(again.index == args.index && again_f->call == &CmdTestOther);
	m_cmd_calls = 0;
	CmdSubmitUtf8(Utf8Inline("test_cmd_args \"x\" 1 2 3"));
	CmdQueueExecute();
	TEST_TRUE(m_cmd_calls == 100);
	CmdFunctionRegister(Utf8Inline("test_cmd_args"), 4, &CmdTestArgs);

	CmdQueueSet(old);
	CmdQueueDealloc(&queue);
	return output;
}

static struct test_Output (*cmd_tests[])(struct test_Environment *) =
{
	CmdParseTest,
	CmdChainTest,
	CmdRegisterTest,
};

struct suite_Correctness m_cmd_suite =
{
	.id = "cmd",
	.unit_test = cmd_tests,
	.unit_test_count = sizeof(cmd_tests) / sizeof(cmd_tests[0]),
};

struct suite_Correctness *cmd_correctness_suite = &m_cmd_suite;
