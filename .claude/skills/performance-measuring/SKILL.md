---
name: performance-measuring
description: Measure the cost of engine code with the Perf block timers
  (PerfWallClock, PerfCycles, PerfBegin/PerfEnd in ds_measure.h) and compare variants or a
  speed-of-light bound. Use when asked to time, benchmark, profile or compare
  the speed of code, or when a change claims to be faster. In progress.
---

# Performance measuring (in progress)

The API is ds_measure.h (PerfWallClock, PerfCycles, and PerfBegin/PerfEnd
reading both in one pair with its own overhead constants), behind the CMake option
DS_PERFORMANCE_MEASUREMENT_API (configure with -DDS_PERFORMANCE_MEASUREMENT_API=ON;
default OFF). Design and the measurements behind these rules:
agent_notes/perf_timing_design.txt. Read the header's section before
measuring; it is the contract.

## Build
- Measured code is always compiled as release code, fully optimized:
  -DDS_OPTIMIZE=ON (-O3 on GCC/Clang, /O2 on MSVC, the flags the shipped
  build uses) with -DDS_DEBUG=OFF -DDS_PROFILE=OFF
  -DDS_PERFORMANCE_MEASUREMENT_API=ON. A DS_DEBUG number is not a
  measurement: it keeps every ds_Assert, validates the physics every tick and
  is compiled without the optimizations that decide the cost of a block
  (inlining, register allocation, vectorization). DS_PROFILE turns ProfZone
  into Tracy calls inside the measured code.
- Check the binary is what you think: the startup log's flags, and a changed
  function's assembly (objdump) if a result looks implausible.
- Never change the determinism flags (-ffp-contract=off, no -march, no
  fast-math). Same compiler, flags and seed for every variant.
- Build hygiene as usual: kill own leftovers, nice -n 10, -j2, timeout -k,
  announce durations over a minute.

## Per platform
- Linux: PerfWallClock (TSC) and PerfCycles (rdpmc, per thread). PerfCycles
  needs kernel.perf_event_paranoid <= 2 and a PMU (not in most VMs); the
  startup log says "PerfCycles empty pair: N cycles" or "PerfCycles
  unavailable". Nothing else may sample the PMU during a run (perf, Tracy
  with hardware counters): the counter index is cached per thread.
- Windows: PerfWallClock only. TODO: check on the Windows machine that MSVC
  keeps __rdtsc between the two atomic_signal_fence barriers (disassemble a
  release build); PerfCycles would need a driver.
- Web: PerfWallClock is the browser clock in ns, clamped to 5-100 us. Only
  loops over many inputs can be timed; PerfCycles unavailable.
- The laptop (i5-7200U): empty pairs 35 ticks (wall), 78 cycles (cycles),
  109 ticks / 78 cycles (PerfBegin/End: its tick window encloses the two
  counter reads); TSC 2.5 GHz, core 0.4-3.1 GHz. Record the machine, compiler and the two overheads with every
  result.

## Which API
Read both in every measurement with PerfBegin/PerfEnd (the second read costs
nothing; subtract g_perf_overhead_ticks / g_perf_overhead_cycles from the
minima); report one as the comparison figure and print the other beside it. Their ratio times
TscFrequency() is the core clock during the block, which tells whether a
difference between runs was the code or the frequency.
- PerfCycles when the block is bound by the core: arithmetic, branches, loads
  from L1/L2. Its count is a property of the code at any clock and immune to
  interrupts. This is the figure for comparing two implementations.
- PerfWallClock when the block waits on something not counted in core
  cycles: DRAM (latency and bandwidth are fixed in ns, so cycle counts of
  memory-bound code drift with the clock), kernel time (page faults,
  syscalls), other threads (a parallel phase's cost is its wall time), and
  for budgets (a frame is 16.6 ms). Also the only figure on Windows and web,
  so pass criteria must make sense from ticks alone.
- Cross-thread stamps: TSC only (Rdtscp + g_tsc_skew); cycle counts of two
  threads cannot be subtracted.

## Rules for a measurement
1. Load the block's inputs and store its results inside the zone. The
   barriers order memory operations, not register work: GCC hoisted a whole
   dependent chain out of a zone whose input came from a register.
2. The block itself should cost >= 100 cycles; the pair adds tens of counts
   that vary by a few counts per sample. Smaller blocks: N iterations inside
   one pair, divided by N (TODO: the repetition tester's job).
3. One sample is noise. Take the minimum of many (thousands for ns-scale
   blocks, 3-8 repetitions for a phase) and subtract g_perf_*_overhead from
   the minimum, never from single samples. Report the median too for phases.
4. Compare variants in one run, interleaved (A B A B ...), as ratios. Absolute
   counts drift between runs (wall ticks 25 % with the core clock on the
   laptop). Report the spread of same-variant runs before any verdict; a
   difference inside the spread is "no measurable difference".
5. Print a checksum of the outputs next to the time; variants must do the
   same work, or it is a bug, not a speedup.
6. Realistic inputs with a fixed seed (config `seed: a b c d`), from the
   engine's own generators; not adversarial geometry.
7. Plain Rdtsc() pairs are for blocks above ~10k ticks only: unfenced stamps
   hide a 100-cycle block entirely (measured 6 of 88 ticks).

## Examples
Core-bound block, cycles as the figure (the pattern of PerfBlockTest,
test_base_correctness.c):

    u64 cycles = U64_MAX, ticks = U64_MAX;
    for (u32 i = 0; i < 4096; ++i)
    {
        g_input = RngU64() | 1;                 /* input in memory */
        const struct PerfStamp begin = PerfBegin();
        u64 x = g_input;                        /* loaded inside */
        for (u32 k = 0; k < 1000; ++k) { x = x * x + 0x9E3779B97F4A7C15ull; }
        g_output = x;                           /* stored inside */
        const struct PerfStamp d = PerfEnd(begin);
        ticks = (d.ticks < ticks) ? d.ticks : ticks;
        cycles = (d.cycles < cycles) ? d.cycles : cycles;
    }
    cycles -= g_perf_overhead_cycles;           /* from the minimum only */
    ticks -= g_perf_overhead_ticks;
    /* laptop: 4002 cycles (4 per step: imul 3 + add 1), 3501 ticks -> core at 2.9 GHz */

Memory-bound loop, wall time as the figure (a buffer larger than L2):

    const u64 t = PerfWallClockBegin();
    const u64 sum = SumBytes(buffer, size);     /* reads size bytes from DRAM */
    g_output = sum;
    const u64 ticks = PerfWallClockEnd(t);
    /* bytes per ns = size / (ticks / TscFrequency() * 1e9); compare with the
       measured DRAM bandwidth, not with a cycle count */

## Speed of light (TODO, with the serialize comparison as the first case)
A bound the code cannot beat, from the data it must touch:
- bytes (or cache lines: a 4-byte read costs the 64-byte line) over the
  bandwidth of the level the data lives in: L1 ~2 loads of 32 B per cycle,
  L2 ~32 B/cycle, DRAM in GB/s measured once per machine (TODO: measure and
  record the laptop's);
- instructions over the retire width (~4 per cycle); instruction counts from
  objdump of the release object (TODO: the rdpmc instruction counter as a
  second channel, same perf_event_open with config INSTRUCTIONS).
The larger of the two is the floor; report measured / floor.
TODO: decide the volume classes (L1/L2/L3/DRAM) and how the harness picks
the bound per buffer size.

## Repetition tester (TODO)
test_repetition.c (rt_) still stamps with plain Rdtsc; to be rebuilt on the
pair: min / median / count, subtract the overheads, stop when no new minimum
for N seconds, N iterations per pair for small blocks, serial and parallel
variants (agent_notes/AGENT_TODO.txt 2026-10-16 testing library).

## Report
Question and block; variants and seed; a table per variant and thread count
with min cycles, min and median ticks, checksum; the same-variant spread and
the verdict relative to it; machine, compiler, overheads. Numbers go to the
relevant agent_notes file.

## Undecided (TODO)
- rdtscp as the end stamp (same cost, gives the core id for a same-core
  debug check) vs lfence; rdtsc.
- Whether a Windows driver for PerfCycles is ever worth it.
- Where kept benchmarks live (test/ performance suites) vs scratch exports.
