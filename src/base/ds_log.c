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

#include "ds_base.h"

#if defined(DS_LOG)

#if __DS_PLATFORM__ == __DS_WEB__
#include <emscripten/console.h>
#endif

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "ticket_factory.h"

static utf8 systems[T_COUNT];
static utf8 severities[S_COUNT];

struct Log_message
{
	u64 	time;			/* ms */
	u32 	system;
	u32 	severity;
	u32 	thread_id;
	u64	size;			/* bytes in buf (without null termination) */
	u8 	buf[LOG_MAX_MESSAGE_SIZE];	
	u32 	a_in_use_and_completed;	/* ready to be sent of to disk and reused */
};

struct Log 
{
	struct Log_message *	msg;
	struct ticketFactory 	tf;    			/* index factory */
	u32 			a_writing_to_disk;
	u32 			has_file;		/* If not, simply skip file IO */ 
	FILE *			file;
	u32			initialized;		/* set by LogInit (before any worker exists); until then messages go to stderr */
	u32			a_shut_down;		/* set by LogShutdown: later messages are dropped (thread memory may be freed) */
};

static struct Log g_log;

static void LogNamesInit(void)
{
	systems[T_SYSTEM] = Utf8Inline("System");
	systems[T_RENDERER] = Utf8Inline("Renderer");
	systems[T_PHYSICS] = Utf8Inline("Physics");
	systems[T_ASSET] = Utf8Inline("Asset");
	systems[T_UTILITY] = Utf8Inline("Utility");
	systems[T_PROFILER] = Utf8Inline("Profiler");
	systems[T_ASSERT] = Utf8Inline("Assert");
	systems[T_GAME] = Utf8Inline("Game");
	systems[T_UI] = Utf8Inline("Ui");
	systems[T_LED] = Utf8Inline("Led");

	severities[S_SUCCESS] = Utf8Inline("success");
	severities[S_NOTE] = Utf8Inline("note");
	severities[S_WARNING] = Utf8Inline("warning");
	severities[S_ERROR] = Utf8Inline("error");
	severities[S_FATAL] = Utf8Inline("fatal");
}

void LogInit(struct arena *mem, const char *filepath)
{
	LogNamesInit();

	g_log.msg = ArenaPush(mem, LOG_MAX_MESSAGES * sizeof(struct Log_message));
	TicketFactoryInit(&g_log.tf, LOG_MAX_MESSAGES);
	g_log.file = (filepath) ? fopen(filepath, "w+") : NULL;
	g_log.has_file = g_log.file != NULL;
	AtomicStoreRel32(&g_log.a_writing_to_disk, 0);
	AtomicStoreRel32(&g_log.a_shut_down, 0);
	g_log.initialized = 1;
}

/*
 * The thread winning the CAS on a_writing_to_disk writes completed slots in order until
 * one is still being formatted, returns that many tickets, and releases a_writing_to_disk.
 */
static void LogTryWriteToDisk(void)
{
	u32 desired = 0;
	/* become the single disk writer unless another thread is */
	if (AtomicCompareExchangeAcqRlx32(&g_log.a_writing_to_disk, &desired, 1))
	{
		u32 count = 0;
		/* we are the single owner of a_serve: sync with any previous disk-writer */
		u32 serving = AtomicLoadAcq32(&g_log.tf.a_serve) % LOG_MAX_MESSAGES;
		u32 completed = 1;
		while (AtomicCompareExchangeAcqRlx32(&g_log.msg[serving].a_in_use_and_completed, &completed, 0))
		{
			struct Log_message *msg = g_log.msg + serving;

			if (g_log.has_file && msg->size)
			{
				fwrite(msg->buf, 1, msg->size, g_log.file);
			}
			serving = (serving + 1) % LOG_MAX_MESSAGES;
			count += 1;
		}

		TicketFactoryReturnTickets(&g_log.tf, count);
		AtomicStoreRel32(&g_log.a_writing_to_disk, 0);
	}
}

static void internal_write_to_disk(void)
{
	/* spin until all tickets have finished and the messages have been written to disk */
	while (AtomicLoadAcq32(&g_log.tf.a_serve) != AtomicLoadAcq32(&g_log.tf.a_next))
	{
		LogTryWriteToDisk();
	}
}

void LogShutdown()
{
	LogString(T_SYSTEM, S_NOTE, "Log system initiated shutdown");
	AtomicStoreRel32(&g_log.a_shut_down, 1);
	if (!g_log.initialized)
	{
		/* FatalCleanupAndExit before LogInit */
		return;
	}

	AtomicStoreRel32(&g_log.tf.a_open, 0);
	if (g_log.has_file)
	{
		internal_write_to_disk();
		fclose(g_log.file);
	}
	TicketFactoryDestroy(&g_log.tf);
}

/*
 * Spin on TicketFactoryTryGetTicket, writing completed slots to disk while none is free
 * (LogTryWriteToDisk); format into slot ticket % LOG_MAX_MESSAGES, print it, then release-store
 * a_in_use_and_completed = 1.
 */
void LogWriteMessage(const enum system_id system, const enum severity_id severity, const char *format, ... )
{
	ProfZone;

	if (AtomicLoadAcq32(&g_log.a_shut_down))
	{
		goto end;
	}

	/* before LogInit or ds_ThreadMasterInit: no message slots, timer or thread index yet */
	if (!g_log.initialized || g_tl_self == NULL)
	{
		LogNamesInit();
		u8 early_buf[LOG_MAX_MESSAGE_SIZE];
		u64 early_req_size;
		va_list early_args;
		va_start(early_args, format);
		const utf8 early = Utf8FormatBufferedVariadic(&early_req_size, early_buf, LOG_MAX_MESSAGE_SIZE, format, early_args);
		va_end(early_args);
		fprintf(stderr, "[before log init] %.*s %.*s: %.*s\n",
			(int) systems[system].len, (const char *) systems[system].buf,
			(int) severities[severity].len, (const char *) severities[severity].buf,
			(int) Utf8SizeRequired(early), (const char *) early.buf);
		goto end;
	}

	const u32 thread_id = ds_ThreadSelfIndex();
	/* spin until a new msg slot is up for grabs for us to publish */
	u32 ticket;
	u32 ret;
	while (!(ret = TicketFactoryTryGetTicket(&ticket, &g_log.tf)))
	{
		/* If we fail to get a ticket after the shutdown process has started, 
		 * we should probably just abort to keep it simple for now. We should
		 * instead ensure that all threads have terminated before we even call
		 * LogShutdown();
		 */
		LogTryWriteToDisk();
	}

	if (ret == TICKET_FACTORY_CLOSED) { goto end; }

	struct Log_message *msg = g_log.msg + (ticket % LOG_MAX_MESSAGES);

	/* format message */
	const u64 ms = ds_TimeMs();
	msg->time = ms; 
	msg->system = system;
	msg->severity = severity;
	msg->thread_id = thread_id;

	/* prefix, then text, straight into the slot; 2 bytes stay free for '\n' and '\0' */
	const u64 capacity = LOG_MAX_MESSAGE_SIZE - 2;
	u64 size = Utf8SizeRequired(Utf8FormatBuffered(msg->buf, capacity, "[%lu.%lu%lu%lus] %k %k - Thread %u: ",
		ms / 1000,
		(ms / 100) % 10,
		(ms / 10) % 10,
		ms % 10,
		systems + system,
		severities + severity,
		thread_id));

	u64 text_size;
	va_list args;
	va_start(args, format);
	Utf8FormatBufferedVariadic(&text_size, msg->buf + size, capacity - size, format, args);
	va_end(args);
	size += text_size;

#if __DS_PLATFORM__ != __DS_WEB__
	msg->buf[size++] = '\n';
#endif
	msg->buf[size] = '\0';
	msg->size = size;

#if __DS_PLATFORM__ == __DS_WEB__
	emscripten_out((char *) msg->buf);
#else
	fprintf(stdout, "%s", msg->buf);
#endif
	/* sync-point, msg ready for writing */
	AtomicStoreRel32(&msg->a_in_use_and_completed, 1);
end:
	ProfZoneEnd;
}

#else

void LogInit(struct arena *mem, const char *filepath) {}
void LogShutdown(void) {}
void LogWriteMessage(const enum system_id system, const enum severity_id severity, const char *format, ... ) {}

#endif
