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

#include <stddef.h>
#include <string.h>

#include "ds_init.h"
#include "ds_base.h"
#include "ds_platform.h"
#include "ds_graphics.h"
#include "ds_asset.h"
#include "cmd.h"
#include "ds_ui.h"
#include "ds_random.h"

static struct ds_RuntimeConfig g_config_storage;
const struct ds_RuntimeConfig *g_config = NULL;

utf8 g_argv[ARGV_COUNT];

static struct arena g_init_persistent;
static u32 g_init_called = 0;

static struct ds_RuntimeConfig ds_ConfigDefault(void)
{
    struct ds_RuntimeConfig config = { 0 };
    config.persistent_size = 256*1024*1024;
    config.thread_count = 0;
    config.thread_scratchsize = 1*1024*1024;
    config.thread_scratch_count = 5;
    config.headless = 0;
    config.dynamics_worker_framesize = 4*1024*1024;
    config.server_framesize = 32*1024*1024;
    return config;
}

/* Parse problems; the offending line is ignored. */
enum ds_ConfigIssue
{
    DS_CONFIG_ISSUE_NO_COLON,
    DS_CONFIG_ISSUE_UNKNOWN_KEY,
    DS_CONFIG_ISSUE_BAD_VALUE,
    DS_CONFIG_ISSUE_DUPLICATE_KEY,
    DS_CONFIG_ISSUE_COUNT
};

static const char *g_config_issue_string[DS_CONFIG_ISSUE_COUNT] =
{
    "line without ':'",
    "unknown key",
    "bad value",
    "duplicate key (the last one wins)",
};

/* One entry per ds_RuntimeConfig field, in struct order: this table is the file format. */
#define DS_CONFIG_FIELD(field, type, count, min, max, comment) \
    { #field, sizeof(#field) - 1, offsetof(struct ds_RuntimeConfig, field), type, count, min, max, comment }

enum ds_ConfigFieldType
{
    DS_CONFIG_FIELD_U32,
    DS_CONFIG_FIELD_U64,
};

static const struct
{
    const char *            key;
    u32                     key_len;
    u32                     offset;
    enum ds_ConfigFieldType type;
    u32                     count;      /* values on the line */
    u64                     min;
    u64                     max;
    const char *            comment;
} g_config_field[] =
{
    DS_CONFIG_FIELD(rng_seed,                 DS_CONFIG_FIELD_U64, 4, 0, U64_MAX, "Xoshiro256Init seed, 4 words; all 0: system entropy"),
    DS_CONFIG_FIELD(persistent_size,      DS_CONFIG_FIELD_U64, 1, 1, U64_MAX, "engine persistent arena, bytes"),
    DS_CONFIG_FIELD(thread_count,         DS_CONFIG_FIELD_U32, 1, 0, U32_MAX, "workers including the main thread; 0: logical cores - 2"),
    DS_CONFIG_FIELD(thread_scratchsize,   DS_CONFIG_FIELD_U64, 1, 1, U64_MAX, "per-thread scratch arena, bytes"),
    DS_CONFIG_FIELD(thread_scratch_count, DS_CONFIG_FIELD_U32, 1, 1, U32_MAX, "per-thread scratch arenas"),
    DS_CONFIG_FIELD(headless,             DS_CONFIG_FIELD_U32, 1, 0, 1,       "1: no graphics (SDL, windows, UI, GL)"),
    DS_CONFIG_FIELD(dynamics_worker_framesize, DS_CONFIG_FIELD_U64, 1, 1, U64_MAX, "per dynamics worker frame arena, bytes"),
    DS_CONFIG_FIELD(server_framesize,     DS_CONFIG_FIELD_U64, 1, 1, U64_MAX, "server frame arena (arrivals and the ticks' scratch), bytes"),
};

#define DS_CONFIG_FIELD_COUNT   (sizeof(g_config_field) / sizeof(g_config_field[0]))
#define DS_CONFIG_REPORT_MAX    16
#define DS_CONFIG_VALUE_MAX     4
#define DS_CONFIG_DIGITS_MAX    64
#define DS_CONFIG_TEXT_MAX      4096

/* The first DS_CONFIG_REPORT_MAX problems (1-based lines); count is the total. */
struct ds_ConfigReport
{
    u32                 line[DS_CONFIG_REPORT_MAX];
    enum ds_ConfigIssue issue[DS_CONFIG_REPORT_MAX];
    u32                 count;
};

static u32 ds_ConfigSpaceCheck(const u8 c)
{
    return c == ' ' || c == '\t' || c == '\r';
}

static void ds_ConfigReportAdd(struct ds_ConfigReport *report, const u32 line, const enum ds_ConfigIssue issue)
{
    if (report->count < DS_CONFIG_REPORT_MAX)
    {
        report->line[report->count] = line;
        report->issue[report->count] = issue;
    }
    report->count += 1;
}

/* Exactly count decimal values in v[0..n), separated by spaces/tabs, each in [min, max]. */
static u32 ds_ConfigValuesTryParse(u64 value[], const u8 *v, const u64 n, const u32 count, const u64 min, const u64 max)
{
    u64 i = 0;
    for (u32 k = 0; k < count; ++k)
    {
        while (i < n && ds_ConfigSpaceCheck(v[i])) { i += 1; }
        const u64 begin = i;
        while (i < n && !ds_ConfigSpaceCheck(v[i])) { i += 1; }
        if (i == begin || DS_CONFIG_DIGITS_MAX < i - begin)
        {
            return 0;
        }

        /* U64Utf8 reads len bytes; non-digits fail */
        const utf8 token = { .buf = (u8 *) v + begin, .size = (u32) (i - begin), .len = (u32) (i - begin) };
        const struct parseRetval parsed = U64Utf8(token);
        if (parsed.op_result != PARSE_SUCCESS || parsed.u64 < min || max < parsed.u64)
        {
            return 0;
        }
        value[k] = parsed.u64;
    }

    while (i < n && ds_ConfigSpaceCheck(v[i])) { i += 1; }
    return i == n;
}

/* Apply the keys in text[0..size) onto config; report is reset first. */
static void ds_ConfigParse(struct ds_RuntimeConfig *config, struct ds_ConfigReport *report, const u8 *text, const u64 size)
{
    report->count = 0;
    u32 seen[DS_CONFIG_FIELD_COUNT] = { 0 };

    u32 line = 0;
    /* a UTF-8 byte order mark (some Windows editors) is not part of the first key */
    u64 i = (size >= 3 && text[0] == 0xEF && text[1] == 0xBB && text[2] == 0xBF) ? 3 : 0;
    while (i < size)
    {
        line += 1;
        u64 begin = i;
        while (i < size && text[i] != '\n') { i += 1; }
        u64 end = i;
        i += (i < size);

        for (u64 j = begin; j < end; ++j)
        {
            if (text[j] == '#')
            {
                end = j;
                break;
            }
        }
        while (begin < end && ds_ConfigSpaceCheck(text[begin])) { begin += 1; }
        while (begin < end && ds_ConfigSpaceCheck(text[end - 1])) { end -= 1; }
        if (begin == end)
        {
            continue;
        }

        u64 colon = begin;
        while (colon < end && text[colon] != ':') { colon += 1; }
        if (colon == end)
        {
            ds_ConfigReportAdd(report, line, DS_CONFIG_ISSUE_NO_COLON);
            continue;
        }

        u64 key_end = colon;
        while (begin < key_end && ds_ConfigSpaceCheck(text[key_end - 1])) { key_end -= 1; }

        u32 field = 0;
        for (; field < DS_CONFIG_FIELD_COUNT; ++field)
        {
            u32 equal = (key_end - begin == g_config_field[field].key_len);
            for (u32 c = 0; equal && c < g_config_field[field].key_len; ++c)
            {
                equal = (text[begin + c] == (u8) g_config_field[field].key[c]);
            }
            if (equal)
            {
                break;
            }
        }

        if (field == DS_CONFIG_FIELD_COUNT)
        {
            ds_ConfigReportAdd(report, line, DS_CONFIG_ISSUE_UNKNOWN_KEY);
            continue;
        }

        u64 value[DS_CONFIG_VALUE_MAX];
        if (!ds_ConfigValuesTryParse(value, text + colon + 1, end - colon - 1, g_config_field[field].count, g_config_field[field].min, g_config_field[field].max))
        {
            ds_ConfigReportAdd(report, line, DS_CONFIG_ISSUE_BAD_VALUE);
            continue;
        }

        /* only a valid value overriding another valid one is a duplicate */
        if (seen[field])
        {
            ds_ConfigReportAdd(report, line, DS_CONFIG_ISSUE_DUPLICATE_KEY);
        }
        seen[field] = 1;

        u8 *dst = (u8 *) config + g_config_field[field].offset;
        for (u32 k = 0; k < g_config_field[field].count; ++k)
        {
            if (g_config_field[field].type == DS_CONFIG_FIELD_U64)
            {
                memcpy(dst + k*sizeof(u64), value + k, sizeof(u64));
            }
            else
            {
                const u32 value32 = (u32) value[k];
                memcpy(dst + k*sizeof(u32), &value32, sizeof(u32));
            }
        }
    }
}

static utf8 ds_ConfigFormat(struct arena *mem, const struct ds_RuntimeConfig *config)
{
    u8 *buf = ArenaPush(mem, DS_CONFIG_TEXT_MAX);
    if (!buf)
    {
        return Utf8Empty();
    }

    /* pieces are appended at bytes; len counts their codepoints (comments may be non-ASCII) */
    utf8 piece = Utf8FormatBuffered(buf, DS_CONFIG_TEXT_MAX, "# DreamscapeEngine config: one \"key: value\" per line, '#' starts a comment\n");
    u32 bytes = piece.size;
    u32 len = piece.len;
    for (u32 field = 0; field < DS_CONFIG_FIELD_COUNT; ++field)
    {
        piece = Utf8FormatBuffered(buf + bytes, DS_CONFIG_TEXT_MAX - bytes, "\n# %s\n%s:", g_config_field[field].comment, g_config_field[field].key);
        bytes += piece.size;
        len += piece.len;
        const u8 *src = (const u8 *) config + g_config_field[field].offset;
        for (u32 k = 0; k < g_config_field[field].count; ++k)
        {
            u64 value;
            if (g_config_field[field].type == DS_CONFIG_FIELD_U64)
            {
                memcpy(&value, src + k*sizeof(u64), sizeof(u64));
            }
            else
            {
                u32 value32;
                memcpy(&value32, src + k*sizeof(u32), sizeof(u32));
                value = value32;
            }
            piece = Utf8FormatBuffered(buf + bytes, DS_CONFIG_TEXT_MAX - bytes, " %lu", value);
            bytes += piece.size;
            len += piece.len;
        }
        piece = Utf8FormatBuffered(buf + bytes, DS_CONFIG_TEXT_MAX - bytes, "\n");
        bytes += piece.size;
        len += piece.len;
    }

    return (utf8) { .buf = buf, .size = bytes, .len = len };
}

static void ds_ConfigReportLog(const struct ds_ConfigReport *report, const char *path)
{
    const u32 stored = (report->count < DS_CONFIG_REPORT_MAX) ? report->count : DS_CONFIG_REPORT_MAX;
    for (u32 i = 0; i < stored; ++i)
    {
        Log(T_SYSTEM, S_WARNING, "config %s:%u: %s", path, report->line[i], g_config_issue_string[report->issue[i]]);
    }
    if (stored < report->count)
    {
        Log(T_SYSTEM, S_WARNING, "config %s: %u more problems", path, report->count - stored);
    }
}

/* ds_ConfigTryRead without logging: report is unchanged if the file can't be read. */
static u32 ds_ConfigTryReadReport(struct arena *tmp, struct ds_RuntimeConfig *config, struct ds_ConfigReport *report, const char *path)
{
    ArenaPushRecord(tmp);
    const struct dsBuffer file = FileDumpAtCwd(tmp, path);
    const u32 read = (file.data != NULL);
    if (read)
    {
        ds_ConfigParse(config, report, file.data, file.size);
    }
    ArenaPopRecord(tmp);
    return read;
}

u32 ds_ConfigTryRead(struct arena *tmp, struct ds_RuntimeConfig *config, const char *path)
{
    struct ds_ConfigReport report = { .count = 0 };
    const u32 read = ds_ConfigTryReadReport(tmp, config, &report, path);
    ds_ConfigReportLog(&report, path);
    return read;
}

u32 ds_ConfigTryWrite(struct arena *tmp, const struct ds_RuntimeConfig *config, const char *path)
{
    ArenaPushRecord(tmp);
    u32 written = 0;
    const utf8 text = ds_ConfigFormat(tmp, config);
    struct file file = FileNull();
    if (text.len && FileTryCreateAtCwd(tmp, &file, path, 1) == FS_SUCCESS)
    {
        written = (FileWriteAppend(&file, text.buf, text.size) == text.size);
        FileClose(&file);
    }
    ArenaPopRecord(tmp);
    return written;
}

static const char *g_argv_name[ARGV_COUNT] = { "log", "config", "simulation", "record", "replay", "hash", "headless", "origin", "help" };
static const u32 g_argv_flag[ARGV_COUNT] = { 0, 0, 0, 0, 0, 0, 1, 0, 1 };

static const char *g_argv_usage =
    "usage: ./DreamscapeTest [key=value ...] [flags]\n"
    "\n"
    "  config=path       the runtime config file (none: engine defaults)\n"
    "  log=path          the log file, created (none: no log file)\n"
    "  record=path       record every external command of the simulation to path\n"
    "  replay=path       replay a record to the same final state (no commands are accepted until it ends)\n"
    "  hash=path         with record: write a state hash per simulation system tick to path\n"
    "                    with replay: compare every tick's hash to path (a difference is fatal)\n"
    "  simulation=path   (reserved) an initial simulation state; not supported yet\n"
    "  origin=x,y,z      determinism test, with record: build the scene around (x, y, z)\n"
    "  headless          no graphics\n"
    "  help              print this and exit\n"
    "\n"
    "combinations: record | replay | record hash | replay hash. hash alone, replay with record and simulation are\n"
    "refused. Any unknown or repeated key, a key without a value or a flag with one, exits with this text.\n";

/* before the log: Log writes to stderr */
static void ds_ArgvFatal(const char *problem, const char *argument)
{
    fprintf(stderr, "%s\n", g_argv_usage);
    Log(T_SYSTEM, S_FATAL, "command line: %s: %s", problem, argument);
    FatalCleanupAndExit();
}

/* a view of the null-terminated bytes at cstr */
static utf8 ds_ArgvValue(char *cstr)
{
    utf8 value = { .buf = (u8 *) cstr, .size = (u32) strlen(cstr), .len = 0 };
    for (u32 offset = 0; offset < value.size; value.len += 1)
    {
        Utf8ReadCodepoint(&offset, &value, offset);
    }
    return value;
}

void ds_ArgvParse(const int argc, char *argv[])
{
    for (u32 k = 0; k < ARGV_COUNT; ++k)
    {
        g_argv[k] = Utf8Empty();
    }

    for (int i = 1; i < argc; ++i)
    {
        char *argument = argv[i];
        const u32 size = (u32) strlen(argument);
        u32 key_size = 0;
        while (key_size < size && argument[key_size] != '=')
        {
            key_size += 1;
        }

        if (key_size == 0 || key_size + 1 == size)
        {
            ds_ArgvFatal("empty key or value", argument);
        }

        u32 key = ARGV_COUNT;
        for (u32 k = 0; k < ARGV_COUNT; ++k)
        {
            if (key_size == strlen(g_argv_name[k]) && memcmp(argument, g_argv_name[k], key_size) == 0)
            {
                key = k;
                break;
            }
        }

        if (key == ARGV_COUNT)
        {
            ds_ArgvFatal("unknown key", argument);
        }

        if (g_argv_flag[key] != (key_size == size))
        {
            ds_ArgvFatal((g_argv_flag[key]) ? "a flag takes no value" : "expected key=value", argument);
        }

        if (g_argv[key].size)
        {
            ds_ArgvFatal("repeated key", argument);
        }

        g_argv[key] = (g_argv_flag[key]) ? Utf8Inline("1") : ds_ArgvValue(argument + key_size + 1);
    }

    if (g_argv[ARGV_HELP].size)
    {
        fprintf(stdout, "%s", g_argv_usage);
        exit(0);
    }

    if (g_argv[ARGV_SIMULATION].size)
    {
        ds_ArgvFatal("not supported yet", "simulation");
    }

    if (g_argv[ARGV_HASH].size && !g_argv[ARGV_RECORD].size && !g_argv[ARGV_REPLAY].size)
    {
        ds_ArgvFatal("hash needs record or replay", "hash");
    }

    if (g_argv[ARGV_REPLAY].size && g_argv[ARGV_RECORD].size)
    {
        ds_ArgvFatal("replay with record is not supported yet", "record");
    }
}

void ds_Init(const int argc, char *argv[])
{
    ds_AssertString(!g_init_called, "ds_Init runs once per process");
    g_init_called = 1;

    ds_ArgvParse(argc, argv);
    const char *config_path = (g_argv[ARGV_CONFIG].size) ? (const char *) g_argv[ARGV_CONFIG].buf : NULL;
    const char *log_path = (g_argv[ARGV_LOG].size) ? (const char *) g_argv[ARGV_LOG].buf : NULL;

    /* first: config parsing and arenas already run lzcnt/tzcnt (a missing LZCNT runs as bsr: wrong, no fault) */
    if (!ds_ArchBaselineCheck())
    {
        LogString(T_SYSTEM, S_FATAL, "ds_Init: the CPU lacks the baseline (SSE2, POPCNT, LZCNT, BMI1)");
        FatalCleanupAndExit();
    }

    ds_MemApiInit();

    /* before the persistent arena (its size is in the config) and the log: report logged below */
    struct ds_RuntimeConfig *config = &g_config_storage;
    *config = ds_ConfigDefault();
    struct ds_ConfigReport report = { .count = 0 };
    u32 config_read = 1;
    if (config_path)
    {
        struct arena tmp = ArenaAlloc(NULL, 1024*1024);
        config_read = (tmp.stack_ptr != NULL) && ds_ConfigTryReadReport(&tmp, config, &report, config_path);
        if (tmp.stack_ptr)
        {
            ArenaFree(&tmp);
        }
    }

    if (g_argv[ARGV_HEADLESS].size)
    {
        config->headless = 1;
    }

    /* no log yet: Log writes to stderr */
    g_init_persistent = ArenaAlloc(NULL, config->persistent_size);
    if (!g_init_persistent.stack_ptr)
    {
        Log(T_SYSTEM, S_FATAL, "ds_Init: failed to allocate the %lu B persistent arena", config->persistent_size);
        FatalCleanupAndExit();
    }

    LogInit(&g_init_persistent, log_path);
    ds_ThreadMasterInit(&g_init_persistent, config->thread_scratchsize, config->thread_scratch_count);

    /* before ds_PlatformApiInit: workers derive their RNG streams from it when they start */
    if (config->rng_seed[0] == 0 && config->rng_seed[1] == 0 && config->rng_seed[2] == 0 && config->rng_seed[3] == 0)
    {
        RngSystem(config->rng_seed, sizeof(config->rng_seed));
    }
    Xoshiro256Init(config->rng_seed);

    ds_TimeApiInit(&g_init_persistent);
#ifdef DS_PERFORMANCE_MEASUREMENT_API
    ds_MeasureApiInit();
#endif
    /* logged only now: the log's timestamps need ds_TimeApiInit */
    for (int i = 0; i < argc; ++i)
    {
        Log(T_SYSTEM, S_NOTE, "ds_Init: argv[%u] %s", (u32) i, argv[i]);
    }

    if (!config_read)
    {
        Log(T_SYSTEM, S_WARNING, "ds_Init: failed to read config %s, using the defaults", config_path);
    }
    ds_ConfigReportLog(&report, config_path);
    if (!ds_ArchConfigInit(&g_init_persistent))
    {
        LogString(T_SYSTEM, S_FATAL, "ds_Init: the CPU does not meet the engine's requirements");
        FatalCleanupAndExit();
    }

    /* web: logical_core_count is navigator.hardwareConcurrency, the pthread pool size */
    const u32 core_count = g_arch_config->logical_core_count;
    if (config->thread_count == 0)
    {
        config->thread_count = (core_count > 2) ? core_count - 2 : 1;
    }
    config->thread_count = (config->thread_count > core_count) ? core_count : config->thread_count;
    config->thread_count = (config->thread_count > DS_THREAD_COUNT_MAX) ? DS_THREAD_COUNT_MAX : config->thread_count;
    g_config = config;

    Log(T_SYSTEM, S_NOTE, "ds_Init: rng_seed (%lu, %lu, %lu, %lu), %u workers, persistent %lu B, scratch %u x %lu B, headless %u, dynamics worker frame %lu B, server frame %lu B",
        config->rng_seed[0], config->rng_seed[1], config->rng_seed[2], config->rng_seed[3], config->thread_count,
        config->persistent_size, config->thread_scratch_count, config->thread_scratchsize,
        config->headless, config->dynamics_worker_framesize, config->server_framesize);

    ds_PlatformApiInit(&g_init_persistent, config->thread_scratchsize, config->thread_scratch_count, config->thread_count);
    ds_CmdApiInit();
    ds_UiApiInit();
    if (!config->headless)
    {
        ds_GraphicsApiInit();
    }
    AssetInit(&g_init_persistent);
}

void ds_Shutdown(void)
{
    ds_AssertString(g_config != NULL, "ds_Shutdown without ds_Init");
    ds_AssertString(ds_ThreadSelfIndex() == 0, "ds_Shutdown must run on the thread that called ds_Init");

    AssetShutdown();
    if (!g_config->headless)
    {
        ds_GraphicsApiShutdown();
    }
    ds_UiApiShutdown();
    ds_CmdApiShutdown();
    ds_PlatformApiShutdown();
    g_config = NULL;
    LogShutdown();
    ArenaFree(&g_init_persistent);
    ds_MemApiShutdown();
}
