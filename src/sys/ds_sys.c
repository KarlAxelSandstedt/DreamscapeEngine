#include "sys_filesystem.c"
#include "sys_event.c"
#include "sys_init.c"
#include "sys_input.c"

#if __DS_PLATFORM__ == __DS_LINUX__ || __DS_PLATFORM__ == __DS_WEB__

#include "linux/linux_filesystem.c"
#include "linux/linux_random.c"

#elif __DS_PLATFORM__ == __DS_WIN64__

#include "windows/win_filesystem.c"
#include "windows/win_random.c"

#else
#error
#endif
