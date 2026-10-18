---
name: performance-measuring
description: Measure the cost of engine code with the Perf block timers
  (ds_measure.h) and compare variants or a speed-of-light bound. Use when
  asked to time, benchmark, profile or compare the speed of code, or when a
  change claims to be faster. In progress.
---

# Performance measuring (in progress)

API and its contract: ds_measure.h (read its section first). Why the rules
below hold: agent_notes/perf_timing_design.txt. Machine facts:
agent_notes/perf_machines.txt.

## Build
- Release only: -DDS_OPTIMIZE=ON -DDS_DEBUG=OFF -DDS_PROFILE=OFF
  -DDS_PERFORMANCE_MEASUREMENT_API=ON. Never touch the determinism flags.
- Same compiler, flags and seed for every variant; build hygiene as usual.

## Hardware
- Before measuring, find the machine's entry in agent_notes/perf_machines.txt
  (cpu, clocks, caches, counter availability, empty-pair costs). No entry, or
  empty-pair costs that differ from the startup log: write or update it
  first. Hardware facts never go into this skill.
- Cycle counts of cache-resident blocks transfer between machines of one
  core family; wall ticks do not.

## Which figure
- Measure with PerfBegin/PerfEnd (both counters); report one, print the
  other beside it. cycles / ticks * TscFrequency() is the core clock during
  the block.
- Cycles: core-bound blocks (arithmetic, L1/L2 data), the figure for
  comparing implementations.
- Wall ticks: memory-bound blocks (DRAM is fixed in ns), kernel time, phases
  across threads, budgets; the only figure on Windows and web.

## Measurement rules
1. Inputs loaded and results stored inside the zone (the barriers order
   memory, not registers).
2. Block >= 100 cycles; smaller: N iterations per pair, divide by N.
3. Minimum of many samples; subtract g_perf_overhead_* from the minimum only.
   Median too for phases.
4. Variants interleaved in one run, compared as ratios; report the
   same-variant spread first. Inside the spread: no measurable difference.
5. Checksum of the outputs next to the time; variants must match.
6. Realistic inputs, fixed seed (config `seed: a b c d`).
7. Plain Rdtsc() only for blocks above ~10k ticks.

Worked example: PerfBlockTest, test/test_base_correctness.c. Repetition
testing: RT in ds_measure.h (RT_SELECT_TICKS for phases that wait on other
threads); driver: run_performance_suite, test/test_main.c.

## Speed of light (TODO)
Floor = max(bytes or cache lines / bandwidth of their cache level,
instructions / retire width), bandwidths from the machine's entry; report
measured / floor. First case: the ds_serialize partial read/write.

## Report
Question, block, variants, seed; a table per variant and thread count: min
cycles, min and median ticks, checksum; spread and verdict; machine entry,
compiler, the run's empty-pair costs. Numbers go to agent_notes.
