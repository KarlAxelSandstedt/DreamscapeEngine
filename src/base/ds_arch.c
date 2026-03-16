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

struct dsArchConfig config = { 0 };
const struct dsArchConfig *g_arch_config = &config;

#if __DS_PLATFORM__ == __DS_LINUX__ || __DS_PLATFORM__ == __DS_WEB__

#if __DS_PLATFORM__ == __DS_WEB__

#define  __get_cpuid(f,a,b,c,d)		0
#define  __get_cpuid_count(f,s,a,b,c,d)	0

#else

#include <cpuid.h>

#endif

#include <unistd.h>
#include <sys/sysinfo.h>

void ds_Cpuid(u32 *eax, u32 *ebx, u32 *ecx, u32 *edx, const u32 function)
{
	const int supported = __get_cpuid(function, eax, ebx, ecx, edx);
	if (!supported)
	{
		*eax = 0;
		*ebx = 0;
		*ecx = 0;
		*edx = 0;
	}
}

void ds_CpuidEx(u32 *eax, u32 *ebx, u32 *ecx, u32 *edx, const u32 function, const u32 subfunction)
{
	const int supported = __get_cpuid_count(function, subfunction, eax, ebx, ecx, edx);
	if (!supported)
	{
		*eax = 0;
		*ebx = 0;
		*ecx = 0;
		*edx = 0;
	}
}

u32 ds_LogicalCoreCount(void)
{
	errno = 0;
	long count = sysconf(_SC_NPROCESSORS_ONLN);
	if (errno != 0)
	{
		LogSystemError(S_ERROR);	
		LogString(T_SYSTEM, S_WARNING, "Failed to retrieve number of Logical cores, defaulting to 2");
		count = 2;
	}

	return (u32) count;
}

u64 ds_Pagesize(void)
{
	return (u64) getpagesize();
}

pid ds_Pid(void)
{
	return getpid();
}

#elif __DS_PLATFORM__ == __DS_WIN64__

#include <intrin.h>

void ds_Cpuid(u32 *eax, u32 *ebx, u32 *ecx, u32 *edx, const u32 function)
{
	int regs[4];		
	__cpuid(regs, function);
	*eax = (u32) regs[0];	
	*ebx = (u32) regs[1];	
	*ecx = (u32) regs[2];	
	*edx = (u32) regs[3];	
}

void ds_CpuidEx(u32 *eax, u32 *ebx, u32 *ecx, u32 *edx, const u32 function, const u32 subfunction)
{
	int regs[4];				
	__cpuidex(regs, function, subfunction);
	*eax = (u32) regs[0];			
	*ebx = (u32) regs[1];			
	*ecx = (u32) regs[2];			
	*edx = (u32) regs[3];
}

u32 ds_LogicalCoreCount(void)
{
	SYSTEM_INFO info;
	GetSystemInfo(&info);
	return (u32) info.dwNumberOfProcessors;
}

u64 ds_Pagesize(void)
{
	SYSTEM_INFO info;
	GetSystemInfo(&info);
	return info.dwPageSize;
}

pid ds_Pid(void)
{
	return GetCurrentProcessId();
}

#endif

/**
 * Intel references:
 * 	- Intel® Processor Identification and the CPUID Instruction
 *	- Intel® 64 and IA-32 Architectures Software Developer’s Manual Volume 2 
 *		(2A, 2B, 2C, & 2D): Instruction Set Reference, A-Z
 *		Section: CPUID - CPU Identification
 *		Page: 317
 *
 * AMD references:
 * 	- AMD64 Architecture Programmer’s Manual, Volume 3 Appendix D,E
 */

static void dsArchConfig_log(void)
{
	const char *yes = "Y";
	const char *no = "N";

	Log(T_SYSTEM, S_NOTE, "cpu signature - %k", &config.vendor_string);
	Log(T_SYSTEM, S_NOTE, "cpu - %k", &config.processor_string);
	Log(T_SYSTEM, S_NOTE, "logical core count - %u", config.logical_core_count);
	Log(T_SYSTEM, S_NOTE, "cacheline size - %luB", config.cacheline);

	Log(T_SYSTEM, S_NOTE, "sse : Supported(%s)", (config.sse) ? yes : no);
	Log(T_SYSTEM, S_NOTE, "sse2 : Supported(%s)", (config.sse2) ? yes : no);
	Log(T_SYSTEM, S_NOTE, "sse3 : Supported(%s)", (config.sse3) ? yes : no);
	Log(T_SYSTEM, S_NOTE, "ssse3 : Supported(%s)", (config.ssse3) ? yes : no);
	Log(T_SYSTEM, S_NOTE, "sse4.1 : Supported(%s)", (config.sse4_1) ? yes : no);
	Log(T_SYSTEM, S_NOTE, "sse4.2 : Supported(%s)", (config.sse4_2) ? yes : no);
	Log(T_SYSTEM, S_NOTE, "avx : Supported(%s)", (config.avx) ? yes : no);
	Log(T_SYSTEM, S_NOTE, "avx2 : Supported(%s)", (config.avx2) ? yes : no);
	Log(T_SYSTEM, S_NOTE, "bmi1 : Supported(%s)", (config.bmi1) ? yes : no);
	Log(T_SYSTEM, S_NOTE, "Rdtsc : Supported(%s)", (config.Rdtsc) ? yes : no);
	Log(T_SYSTEM, S_NOTE, "Rdtscp : Supported(%s)", (config.Rdtscp) ? yes : no);
	Log(T_SYSTEM, S_NOTE, "tsc_invariant : Supported(%s)", (config.tsc_invariant) ? yes : no);
}

static void internal_amd_determine_cache_attributes(void)
{
	u32 eax, ebx, ecx, edx;
	const u32 cache_func = (1u << 31) + 5;
	ds_Cpuid(&eax, &ebx, &ecx, &edx, cache_func);

	config.cacheline = ecx & 0xff;

	if (config.cacheline == 0)
	{
		config.cacheline = 64;
		Log(T_SYSTEM, S_WARNING, "Failed to find cacheline size; defaulting to %luB", config.cacheline);
	}
}


static u32 internal_get_amd_arch_config(struct arena *mem)
{
	config.type = ARCH_AMD64;
	config.processor_string = Utf8Empty();

	u32 eax, ebx, edx, ecx;
	ds_Cpuid(&eax, &ebx, &ecx, &edx, 0);
	const u32 largest_standard_function_number = eax;

	ds_CpuidEx(&eax, &ebx, &ecx, &edx, (1u << 31), 0);
	const u32 largest_extended_function_number = eax;

	const u32 sse_avx_func = 1;
	const u32 bmi_avx2_func = 7;
	const u32 Rdtscp_func = (1u << 31) + 1;
	const u32 tsc_invariant_func = (1u << 31) + 7;

	const u32 cache_func = (1u << 31) + 5;
	if (largest_extended_function_number < cache_func)
	{
		return 0;
	}
	internal_amd_determine_cache_attributes();

	if (largest_standard_function_number < bmi_avx2_func)
	{ 
		return 0; 
	}

	ds_Cpuid(&eax, &ebx, &ecx, &edx, sse_avx_func);
	config.Rdtsc = (edx >> 4) & 0x1;
	config.sse = (edx >> 25) & 0x1;
	config.sse2 = (edx >> 26) & 0x1;
	config.sse3 = ecx & 0x1;
	config.ssse3 = (ecx >> 9) & 0x1;
	config.sse4_1 = (ecx >> 19) & 0x1;
	config.sse4_2 = (ecx >> 20) & 0x1;
	config.avx = (ecx >> 28) & 0x1;

	ds_CpuidEx(&eax, &ebx, &ecx, &edx, bmi_avx2_func, 0);
	config.bmi1 = (ebx >> 3) & 0x1;
	config.avx2 = (ebx >> 5) & 0x1;

	config.Rdtscp = 0;
	if (Rdtscp_func < largest_extended_function_number)
	{
		ds_Cpuid(&eax, &ebx, &ecx, &edx, Rdtscp_func);
		config.Rdtscp = (edx >> 27) & 0x1;
	}

	config.tsc_invariant = 0;
	if (tsc_invariant_func < largest_extended_function_number)
	{
		ds_Cpuid(&eax, &ebx, &ecx, &edx, tsc_invariant_func);
		config.tsc_invariant = (edx >> 8) & 0x1;
	}

	/* processor string */
	u8 buf[49];
	for (u32 i = 2; i <= 4; ++i)
	{
		ds_Cpuid(&eax, &ebx, &ecx, &edx, (1u << 31) + i);
		buf[(i-2) * 16 + 3 ] = (u8) ((eax >> 24) & 0xff); 
		buf[(i-2) * 16 + 2 ] = (u8) ((eax >> 16) & 0xff);
		buf[(i-2) * 16 + 1 ] = (u8) ((eax >>  8) & 0xff);
		buf[(i-2) * 16 + 0 ] = (u8) ((eax      ) & 0xff);
		buf[(i-2) * 16 + 7 ] = (u8) ((ebx >> 24) & 0xff);
		buf[(i-2) * 16 + 6 ] = (u8) ((ebx >> 16) & 0xff);
		buf[(i-2) * 16 + 5 ] = (u8) ((ebx >>  8) & 0xff);
		buf[(i-2) * 16 + 4 ] = (u8) ((ebx      ) & 0xff);
		buf[(i-2) * 16 + 11] = (u8) ((ecx >> 24) & 0xff);
		buf[(i-2) * 16 + 10] = (u8) ((ecx >> 16) & 0xff);
		buf[(i-2) * 16 +  9] = (u8) ((ecx >>  8) & 0xff);
		buf[(i-2) * 16 +  8] = (u8) ((ecx      ) & 0xff);
		buf[(i-2) * 16 + 15] = (u8) ((edx >> 24) & 0xff);
		buf[(i-2) * 16 + 14] = (u8) ((edx >> 16) & 0xff);
		buf[(i-2) * 16 + 13] = (u8) ((edx >>  8) & 0xff);
		buf[(i-2) * 16 + 12] = (u8) ((edx      ) & 0xff);
	}
	buf[48] = '\0';
	config.processor_string = Utf8Cstr(mem, (char *) buf);


	dsArchConfig_log();

	return 1;
}

static u32 internal_intel_determine_cache_attribute_byte(const u8 byte)
{
	u32 no_report = 0;
	//TODO 2A 3-247, fill in everything 
	switch (byte)
	{
		case 0x0A:
		{
			config.cacheline = 32;
		} break;

		case 0x0C:
		{
			config.cacheline = 32;
		} break;

		case 0x0D:
		{
			config.cacheline = 64;
		} break;

		case 0x0E:
		{
			config.cacheline = 64;
		} break;

		case 0x2C:
		{
			config.cacheline = 64;
		} break;

		case 0x60:
		{
			config.cacheline = 64;
		} break;

		case 0x66:
		{
			config.cacheline = 64;
		} break;

		case 0x67:
		{
			config.cacheline = 64;
		} break;

		case 0x68:
		{
			config.cacheline = 64;
		} break;

		/* CPUID leaf 2 does not report cacheline behaviour */
		case 0xFF:
		{
			no_report = 1;
		} break;
	}

	return no_report;
}
	
static void internal_intel_determine_cache_attributes(const u32 largest_standard_function_number)
{
	u32 eax, ebx, ecx, edx, no_report = 0;
	const u32 cacheline_tlb_func = 0x2;
	ds_Cpuid(&eax, &ebx, &ecx, &edx, cacheline_tlb_func);
	ds_Assert((eax & 0xff) == 0x1);

	if ((eax & (1u << 31)) == 0)
	{
		no_report = internal_intel_determine_cache_attribute_byte((eax >> 8 ) & 0xff);
		no_report = internal_intel_determine_cache_attribute_byte((eax >> 16) & 0xff);
		no_report = internal_intel_determine_cache_attribute_byte((eax >> 24) & 0xff);
	}

	if ((ebx & (1u << 31)) == 0)
	{
		no_report = internal_intel_determine_cache_attribute_byte((ebx      ) & 0xff);
		no_report = internal_intel_determine_cache_attribute_byte((ebx >> 8 ) & 0xff);
		no_report = internal_intel_determine_cache_attribute_byte((ebx >> 16) & 0xff);
		no_report = internal_intel_determine_cache_attribute_byte((ebx >> 24) & 0xff);
	}

	if ((ecx & (1u << 31)) == 0)
	{
		no_report = internal_intel_determine_cache_attribute_byte((ecx      ) & 0xff);
		no_report = internal_intel_determine_cache_attribute_byte((ecx >> 8 ) & 0xff);
		no_report = internal_intel_determine_cache_attribute_byte((ecx >> 16) & 0xff);
		no_report = internal_intel_determine_cache_attribute_byte((ecx >> 24) & 0xff);
	}

	if ((edx & (1u << 31)) == 0)
	{
		no_report = internal_intel_determine_cache_attribute_byte((edx      ) & 0xff);
		no_report = internal_intel_determine_cache_attribute_byte((edx >> 8 ) & 0xff);
		no_report = internal_intel_determine_cache_attribute_byte((edx >> 16) & 0xff);
		no_report = internal_intel_determine_cache_attribute_byte((edx >> 24) & 0xff);
	}

	if (config.cacheline == 0 || no_report)
	{
		// TODO check we can call this [CPUID leaves above 2 and below 80000000H are visible only when IA32_MISC_ENABLE[bit 22] has its default value of 0]
		if (largest_standard_function_number >= 0x04)
		{
			for (u32 i = 0; i < U32_MAX; ++i)
			{
				ds_CpuidEx(&eax, &ebx, &ecx, &edx, 0x04, i);			
				const u32 type = (eax & 0x1f);
				if (type == 0) { break; }
				else if (type == 1)
				{
					ds_AssertString((eax & 0xe0) == 1, "Expects first indexable data cache to be level 1");
					config.cacheline = ebx & 0xfff;
					break;
				}
			}	
		}
		
		if (config.cacheline == 0)
		{
			config.cacheline = 64;
			Log(T_SYSTEM, S_WARNING, 0, "Failed to find cacheline size; defaulting to %luB", config.cacheline);
		}
	}
}

static u32 internal_get_intel_arch_config(struct arena *mem)
{
	config.type = ARCH_INTEL64;
	config.processor_string = Utf8Empty();

	u32 eax, ebx, edx, ecx;
	ds_Cpuid(&eax, &ebx, &ecx, &edx, 0);
	const u32 largest_standard_function_number = eax;

	ds_CpuidEx(&eax, &ebx, &ecx, &edx, (1u << 31), 0);
	const u32 largest_extended_function_number = eax;

	const u32 cacheline_tlb_func = 0x2;
	if (largest_standard_function_number < cacheline_tlb_func)
	{
		return 0;
	}
	internal_intel_determine_cache_attributes(largest_standard_function_number);	

	const u32 sse_avx_func = 1;
	const u32 bmi_avx2_func = 7;
	const u32 Rdtscp_func = (1u << 31) + 1;
	const u32 tsc_invariant_func = (1u << 31) + 7;

	if (largest_standard_function_number < bmi_avx2_func)
	{ 
		return 0; 
	}

	ds_Cpuid(&eax, &ebx, &ecx, &edx, sse_avx_func);
	config.Rdtsc = (edx >> 4) & 0x1;
	config.sse = (edx >> 25) & 0x1;
	config.sse2 = (edx >> 26) & 0x1;
	config.sse3 = ecx & 0x1;
	config.ssse3 = (ecx >> 9) & 0x1;
	config.sse4_1 = (ecx >> 19) & 0x1;
	config.sse4_2 = (ecx >> 20) & 0x1;
	config.avx = (ecx >> 28) & 0x1;

	ds_CpuidEx(&eax, &ebx, &ecx, &edx, bmi_avx2_func, 0);
	config.bmi1 = (ebx >> 3) & 0x1;
	config.avx2 = (ebx >> 5) & 0x1;

	config.Rdtscp = 0;
	if (Rdtscp_func < largest_extended_function_number)
	{
		ds_Cpuid(&eax, &ebx, &ecx, &edx, Rdtscp_func);
		config.Rdtscp = (edx >> 27) & 0x1;
	}

	config.tsc_invariant = 0;
	if (tsc_invariant_func < largest_extended_function_number)
	{
		ds_Cpuid(&eax, &ebx, &ecx, &edx, tsc_invariant_func);
		config.tsc_invariant = (edx >> 8) & 0x1;
	}

	/* processor string */
	u8 buf[49];
	for (u32 i = 2; i <= 4; ++i)
	{
		ds_Cpuid(&eax, &ebx, &ecx, &edx, (1u << 31) + i);
		buf[(i-2) * 16 + 3 ] = (u8) ((eax >> 24) & 0xff); 
		buf[(i-2) * 16 + 2 ] = (u8) ((eax >> 16) & 0xff);
		buf[(i-2) * 16 + 1 ] = (u8) ((eax >>  8) & 0xff);
		buf[(i-2) * 16 + 0 ] = (u8) ((eax      ) & 0xff);
		buf[(i-2) * 16 + 7 ] = (u8) ((ebx >> 24) & 0xff);
		buf[(i-2) * 16 + 6 ] = (u8) ((ebx >> 16) & 0xff);
		buf[(i-2) * 16 + 5 ] = (u8) ((ebx >>  8) & 0xff);
		buf[(i-2) * 16 + 4 ] = (u8) ((ebx      ) & 0xff);
		buf[(i-2) * 16 + 11] = (u8) ((ecx >> 24) & 0xff);
		buf[(i-2) * 16 + 10] = (u8) ((ecx >> 16) & 0xff);
		buf[(i-2) * 16 +  9] = (u8) ((ecx >>  8) & 0xff);
		buf[(i-2) * 16 +  8] = (u8) ((ecx      ) & 0xff);
		buf[(i-2) * 16 + 15] = (u8) ((edx >> 24) & 0xff);
		buf[(i-2) * 16 + 14] = (u8) ((edx >> 16) & 0xff);
		buf[(i-2) * 16 + 13] = (u8) ((edx >>  8) & 0xff);
		buf[(i-2) * 16 + 12] = (u8) ((edx      ) & 0xff);
	}
	buf[48] = '\0';
	config.processor_string = Utf8Cstr(mem, (char *) buf);

	dsArchConfig_log();

	return 1;
}

u32 ds_ArchConfigInit(struct arena *mem)
{
	config.logical_core_count = ds_LogicalCoreCount();
	config.pagesize = ds_Pagesize(); 
	config.pid = ds_Pid();
	config.cacheline = 64; //TODO

	u32 requirements_fullfilled = 1;
#if __DS_PLATFORM__ == __DS_WEB__
	config.vendor_string = Utf8Inline("Web Browser (TODO)");
	config.processor_string = Utf8Inline("Web CPU (TODO)");
	//TODO check SIMD support 
	dsArchConfig_log();
#else
	const utf8 amd =    Utf8Inline("AuthenticAMD");
	const utf8 intel =  Utf8Inline("GenuineIntel");

	u32 eax, ebx, ecx, edx;
	ds_Cpuid(&eax, &ebx, &ecx, &edx, 0);

	u8 buf[13] =
	{
		(u8) (ebx >>  0),
		(u8) (ebx >>  8),
		(u8) (ebx >> 16),
		(u8) (ebx >> 24),
		(u8) (edx >>  0),
		(u8) (edx >>  8),
		(u8) (edx >> 16),
		(u8) (edx >> 24),
		(u8) (ecx >>  0),
		(u8) (ecx >>  8),
		(u8) (ecx >> 16),
		(u8) (ecx >> 24),
		'\0',
	};

	config.vendor_string = Utf8Cstr(mem, (char *) buf);
	if (Utf8Equivalence(config.vendor_string, amd))
	{
		requirements_fullfilled = internal_get_amd_arch_config(mem);
	}
	else if (Utf8Equivalence(config.vendor_string, intel))
	{
		requirements_fullfilled = internal_get_intel_arch_config(mem);
	}
	else
	{
		Log(T_SYSTEM, S_NOTE, 0, "cpu signature - %k [supported (N)]", &config.vendor_string);
		requirements_fullfilled = 0;
	}
#endif

	return requirements_fullfilled;
}
