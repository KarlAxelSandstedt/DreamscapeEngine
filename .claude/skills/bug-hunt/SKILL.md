---
name: bug-hunt
description: Systematically search a code zone (file, function group or
  subsystem) for bugs and report them one at a time. Use when asked to hunt,
  audit or review existing engine code for bugs.
---

# Bug hunt

## Scope
Work on one zone at a time (a file or a group of related functions). Ask for
the zone if the request doesn't name one.

## Search
Read the zone fully, including the callers of each function. Look first for
bug classes found before in this engine:
- Output parameters or uninitialized locals used as inputs.
- Copy-paste errors: wrong index, edge, table (sub vs add), or swapped args.
- Boundary parameters: exactly 0 or 1, coplanar/parallel/degenerate input
  (0/0 -> NaN; note F32Clamp passes NaN through).
- `=` vs `==`; loops that advance the iterator before removing the element.
- Lifetime: pool slots never removed, addresses kept across pool growth,
  scratch memory used after ArenaPopScratch.
- Memory orderings that don't match the macro name (ds_atomic.h).
- Varargs/format mismatches in Log calls (no compiler format checking).

## Memory corruption: start with ASan
When memory corruption is a plausible cause, run an ASan build first
(-DDS_ASAN=ON -DDS_DEBUG=ON, as in address_sanitize.sh, but in build_agent/).
The engine's allocators poison memory that is allocated but not in use (e.g.
pool slots on the free list, popped arena memory), so ASan also catches
use-after-free within arenas and pools, not only of heap blocks.

## Verify
Before reporting, confirm with numbers when possible: a scratch harness
(outside the repo, built in build_agent/) comparing the function against a
brute-force reference over random input. Report the failure rate and worst
error. Bugs found only by reading are reported as "by reading".

## Report
One bug at a time:
1. Location (file:line, function) and a one-sentence statement.
2. Why it's wrong, with the math if needed.
3. Impact: which callers/features are affected; confirmed or by reading.
4. Suggested fix (don't apply it).
Then stop: the owner decides who fixes it. Keep a numbered list of found bugs
in agent_notes/bug_hunt.txt (zone, status) so later sessions can continue.
