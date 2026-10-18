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
- The startup log says "PerfCycles empty pair: N cycles" or "PerfCycles
  unavailable" (Linux: perf_event_paranoid > 2 or a VM; not yours to fix: say
  so, the figures are ticks).
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
3. The minimum is the figure: every measurement runs under RT (below), which
   keeps running while the minimum keeps falling and subtracts the pair's
   cost from it. Never a hand-written loop or a single pair.
4. Variants interleaved in one run, compared as ratios; report the
   same-variant spread first. Inside the spread: no measurable difference.
5. Checksum of the outputs next to the time; variants must match.
6. Realistic inputs, fixed seed (config `seed: a b c d`).
7. Plain Rdtsc() only for blocks above ~10k ticks.
8. Code layout swings a loop by double digits on any core with a uop cache
   or loop buffer: a variant that moves code can win or lose by alignment
   alone. Confirm a difference survives equal layout (this machine's way is
   in its entry) before believing it.
9. Optimize what is portable: instructions per value, dependency chain
   length, data-dependent branches, bytes and lines touched. A cycle gain
   on one core without a gain in those is layout or microarchitecture:
   measure it, note it, never design for it. The engine is tuned for no
   single core.

## Repetition testing (RT, ds_measure.h)
- Reset and input setup before RTBeginTime; several regions per repetition
  when the block has setup it must exclude.
- bytes per repetition = the data the block touches (cycles/B and GB/s mean
  something); 1 only when no byte figure applies.
- The minimum is by wall ticks; its cycles are the block's cycles. For a job
  phase the cycle line is the master's own: ignore it. 10 s without a new
  minimum.
- Report the min line and the repetition count; max and avg show noise only.
- Driver: run_performance_suite, test/test_main.c. Single-pair example:
  PerfBlockTest, test/test_base_correctness.c.

## Speed of light
Floors are counted from the code, never from a timing. The largest binds.
1. Work per repetition: elements, bytes loaded and stored, distinct cache
   lines, the dependent chain per element, minimal instructions per element.
2. Memory floor. The working set's size picks the level (L1/L2/L3/DRAM; RT
   runs warm, say so when real use is cold). Sequential or strided: bytes /
   that level's read or write bandwidth (a store reads its line first).
   Sparse: lines, not bytes; addresses known ahead: lines x 64 / random-line
   bandwidth; each address from the previous load: lines x latency. Beyond
   the TLB's reach add a page walk per line.
3. Issue floor: minimal instructions / retire width. Beside it the code's
   IPC (actual instructions, from the disassembly x iterations, / measured
   cycles): low IPC is stalls, high IPC with high cycles is too many
   instructions.
4. Chain floor: elements x the dependent latency per element (store-forward,
   load, mul, div). Structural: 1 store, 2 loads, 1 taken branch per cycle.
5. Hardware floor ignores the algorithm; algorithm floor keeps its chain
   and branches. Report both: their gap is the algorithm's cost.
6. Numbers from the machine entry: per level sequential read, sequential
   write, random lines and chase, measured under RT once per machine;
   retire width, latencies and mispredict cost from the vendor tables.
7. Inputs as in real use (predictable widths, orders), or say the floor
   includes mispredicts.
Verdict: within 2x of the floor done; 2-10x micro-optimize; above 10x the
algorithm or the layout.

## Report
Question, block, variants, seed; work; a table per variant and thread count:
min cycles, min ticks, per element, per byte, IPC, repetitions, checksum;
floors with the binding one marked, measured / floor, one line on where the
cycles go with its evidence; spread and verdict; machine entry, compiler,
the run's empty-pair costs. Numbers go to agent_notes.
