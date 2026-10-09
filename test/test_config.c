#include "ds_test.h"
#include "ds_init.h"
#include "ds_platform.h"

/*
Config tests
============
Through the public API only: ds_ConfigTryWrite/ds_ConfigTryRead round trips, and hand-written files
(partial, whitespace and comments, bad lines) read back. Bad lines are logged as warnings; the tests
check that they change nothing. Files go to the working directory (test_config.txt, left behind: the
platform API has no delete).
*/

static struct ds_Config ConfigTestBase(void)
{
    struct ds_Config config = { 0 };
    config.seed[0] = 1;
    config.seed[1] = 2;
    config.seed[2] = 3;
    config.seed[3] = 4;
    config.persistent_size = 1000;
    config.thread_count = 3;
    config.thread_framesize = 2000;
    config.thread_scratchsize = 3000;
    config.thread_scratch_count = 4;
    config.headless = 0;
    return config;
}

/* field by field: the struct has padding */
static u32 ConfigTestEqual(const struct ds_Config *a, const struct ds_Config *b)
{
    return a->seed[0] == b->seed[0] && a->seed[1] == b->seed[1] && a->seed[2] == b->seed[2] && a->seed[3] == b->seed[3]
        && a->persistent_size == b->persistent_size
        && a->thread_count == b->thread_count
        && a->thread_framesize == b->thread_framesize
        && a->thread_scratchsize == b->thread_scratchsize
        && a->thread_scratch_count == b->thread_scratch_count
        && a->headless == b->headless;
}

/* Write text as test_config.txt and read it onto config. */
static u32 ConfigTestReadText(struct arena *tmp, struct ds_Config *config, const char *text)
{
    u64 size = 0;
    while (text[size]) { size += 1; }

    ArenaPushRecord(tmp);
    struct file file = FileNull();
    u32 written = 0;
    if (FileTryCreateAtCwd(tmp, &file, "test_config.txt", 1) == FS_SUCCESS)
    {
        written = (FileWriteAppend(&file, (const u8 *) text, size) == size);
        FileClose(&file);
    }
    ArenaPopRecord(tmp);

    return written && ds_ConfigTryRead(tmp, config, "test_config.txt");
}

/* Write -> read reproduces random configs exactly, extreme values included. */
struct test_Output ConfigRoundTripTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    for (u32 i = 0; i < 200; ++i)
    {
        struct ds_Config config = ConfigTestBase();
        for (u32 k = 0; k < 4; ++k)
        {
            config.seed[k] = (i == 0) ? U64_MAX : RngU64();
        }
        config.persistent_size = (i == 1) ? U64_MAX : 1 + RngU64Range(0, U64_MAX - 1);
        config.thread_count = (u32) RngU64Range(0, U32_MAX);
        config.thread_framesize = 1 + RngU64Range(0, 1u << 30);
        config.thread_scratchsize = 1 + RngU64Range(0, 1u << 30);
        config.thread_scratch_count = (u32) RngU64Range(1, U32_MAX);
        config.headless = (u32) RngU64Range(0, 1);

        struct ds_Config read = { 0 };
        TEST_EQUAL(1, ds_ConfigTryWrite(env->mem_1, &config, "test_config.txt"));
        TEST_EQUAL(1, ds_ConfigTryRead(env->mem_1, &read, "test_config.txt"));
        TEST_EQUAL(1, ConfigTestEqual(&config, &read));
    }

	return output;
}

/* Missing keys keep their values; comments, blank lines, spaces, tabs, CRLF and a missing final newline. */
struct test_Output ConfigPartialTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    struct ds_Config config = ConfigTestBase();
    TEST_EQUAL(1, ConfigTestReadText(env->mem_1, &config,
        "# comment line\r\n"
        "\r\n"
        "   \t\n"
        "  thread_count \t:   7   # trailing comment\r\n"
        "seed:5 6\t7  8\n"
        "headless: 1"));

    struct ds_Config expected = ConfigTestBase();
    expected.thread_count = 7;
    expected.seed[0] = 5;
    expected.seed[1] = 6;
    expected.seed[2] = 7;
    expected.seed[3] = 8;
    expected.headless = 1;
    TEST_EQUAL(1, ConfigTestEqual(&expected, &config));

	return output;
}

/* Bad lines (logged as warnings) change nothing; of two valid duplicates the last wins. */
struct test_Output ConfigBadLineTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    struct ds_Config config = ConfigTestBase();
    TEST_EQUAL(1, ConfigTestReadText(env->mem_1, &config,
        "thread_count 5\n"                              /* no colon                 */
        "Thread_count: 5\n"                             /* unknown key (case)       */
        "worker_count: 5\n"                             /* unknown key              */
        "thread_count:\n"                               /* empty value              */
        "thread_count: abc\n"                           /* not a number             */
        "thread_count: -1\n"                            /* negative                 */
        "thread_count: 4294967296\n"                    /* > U32_MAX                */
        "persistent_size: 18446744073709551616\n"       /* > U64_MAX (overflow)     */
        "persistent_size: 0\n"                          /* below min                */
        "headless: 2\n"                                 /* above max                */
        "seed: 1 2 3\n"                                 /* too few values           */
        "seed: 1 2 3 4 5\n"                             /* too many values          */
        "thread_count: 5 x\n"                           /* trailing garbage         */
        "thread_count: 1.5\n"                           /* not an integer           */
        "thread_count: \xc3\xa5\n"                      /* non-ASCII                */
        "thread_scratch_count: 9\n"                     /* valid                    */
        "thread_scratch_count: 10\n"));                 /* duplicate: last wins     */

    struct ds_Config expected = ConfigTestBase();
    expected.thread_scratch_count = 10;
    TEST_EQUAL(1, ConfigTestEqual(&expected, &config));

	return output;
}

/* Overwriting with a shorter file leaves no tail; a missing file leaves config unchanged. */
struct test_Output ConfigFileTest(struct test_Environment *env)
{
	struct test_Output output = { .success = 1, .id = __func__ };

    struct ds_Config long_config = ConfigTestBase();
    long_config.seed[0] = U64_MAX;
    long_config.persistent_size = U64_MAX;
    struct ds_Config config = ConfigTestBase();
    TEST_EQUAL(1, ds_ConfigTryWrite(env->mem_1, &long_config, "test_config.txt"));
    TEST_EQUAL(1, ds_ConfigTryWrite(env->mem_1, &config, "test_config.txt"));

    struct ds_Config read = { 0 };
    TEST_EQUAL(1, ds_ConfigTryRead(env->mem_1, &read, "test_config.txt"));
    TEST_EQUAL(1, ConfigTestEqual(&config, &read));

    TEST_EQUAL(0, ds_ConfigTryRead(env->mem_1, &read, "test_config_missing.txt"));
    TEST_EQUAL(1, ConfigTestEqual(&config, &read));

	return output;
}

static struct test_Output (*config_tests[])(struct test_Environment *) =
{
    ConfigRoundTripTest,
    ConfigPartialTest,
    ConfigBadLineTest,
    ConfigFileTest,
};

struct suite_Correctness m_config_suite =
{
	.id = "config",
	.unit_test = config_tests,
	.unit_test_count = sizeof(config_tests) / sizeof(config_tests[0]),
};

struct suite_Correctness *config_correctness_suite = &m_config_suite;
