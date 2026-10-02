DreamscapeEngine
================

A C game engine with deterministic physics, an editor (led) and an OpenGL
renderer.

Platform and Builds
===================

Architectures: 64-bit x86
Synchronization: use ds_atomic.h with explicit orderings; never rely on x86's
strong memory ordering (other architectures may be added).
Platforms: Linux, Windows, and Web through Emscripten.
Compilers: GCC, MSVC, Clang
Graphics: OpenGL 3.x/WebGL 2.x
External libraries: SDL3, dtoa, freetype, stb_image, tracy, xxHash.

Useful builds:
    determinism.sh - fresh reference + replay at several thread counts (tests
        one build). An old .bin is only valid for the version that recorded it.
    run.(sh/bat) - optimized build
    debug.(sh/bat) - debugging build
    address_sanitize.sh - memory corruption detection + poisoning with ASan
    thread_sanitize.sh - thread race detection with TSan
    undefined_behaviour_sanitize.sh - undefined behaviour detection with UBSan

All scripts share build/. For your own builds, use a separate directory
(cmake -B build_agent ...) unless told otherwise. Configure your builds with
-DDS_PROFILE=OFF (Tracy); you cannot read its output. Sanitizer builds must be
optimized (-DDS_OPTIMIZE=ON). DS_DEBUG=ON validates the whole physics pipeline
every tick (slow).

ds_Assert and Breakpoint(condition) are diligently used when debugging. Note
that -DDS_DEBUG=OFF compiles them out, so they must not contain side effects or
error handling. The same holds for Log (compiled out with -DDS_LOG=OFF).

Every ProfZone/ProfZoneNamed needs a ProfZoneEnd on every return path (use
goto end). Your builds have profiling off, so you will not notice a mismatch.

Workflow
========

- Never commit or stage; the owner does.
- Bugs found outside the current task: report them with a concise summary
  (math if needed); the owner decides who fixes them.
- agent_notes/ (git-ignored) holds design docs; read the relevant one before
  changing that system.

Determinism
===========

1. Deterministic physics is required: given the same input state, the physics
must produce the same output. The goal is cross-platform determinism (in the
spirit of Erin Catto's box3d): builds from all supported compilers and platforms
produce bit-identical results. Hence:
- no output dependence on thread scheduling, randomness or wall-clock time;
- no code whose float results depend on compiler choices (FMA contraction,
  fast-math, intrinsics with differing precision).

API
===

1. New functions, files, abstractions, APIs, and refactors of nearby code within
the engine are proposed, never implemented, unless the task explicitly asks for
it. Reason: the owner must understand the entire engine, so it is written mostly
by a human.

2. Strongly prefer the engine API in include/ over external libraries. The C
standard library is banned (incl. math.h). The only exceptions are the printf
family, memset and memcpy. Only the platform layer (src/sys/, src/base/ and
their headers, e.g. ds_platform.h, ds_thread.h, ds_float.h) may call it or OS
APIs directly. Freestanding headers (stddef.h, stdint.h, float.h, ...) are fine.
If no engine API exists, make a proposal. Reason: keeping dependencies low.

3. If engine API documentation you relied upon is wrong or sparse, provide
correct and concise proposals.

4. Parameter order: allocator first, then written parameters (outputs, and
e.g. a stream whose position advances), then read-only inputs:
func(mem, out1, ..., outN, in1, ..., inM).

Memory and Lifetime Handling (ds_allocator.h)
=============================================

1. General-purpose allocation (ds_Alloc, ds_SmallAlloc) is only allowed in rules 2
and 4. Reason: avoid complicated lifetimes as far as possible.

2. Persistent data (program or subsystem lifetime) goes on a persistent arena if
fixed-size, or in a ds_Alloc'd block if it grows.

3. Unknown-lifetime objects go in a typed, macro-generated pool:
   - POOL_DECLARE(T)/POOL_DEFINE(T) -> TPool, TPoolAdd/Remove/...: stable
     indices; use them as handles, never addresses (addresses move when a
     growable pool grows). Use when others hold references.
   - ds_CPool(T), ds_CPoolPush/Pop/...: compact, removal moves the last
     element into the hole. Use when nothing refers to elements by index.

4. Unknown-lifetime data that fits no pool (unusual form, lifetime of a large
and unknown number of frames) uses ds_Alloc/ds_SmallAlloc. Example: sleeping
island data in the physics engine; arrays of varying lengths and types that
live while the island sleeps.

5. Subsystem frame data should be stored on the system's single/double-buffered
frame arena(s). Such arenas may be per-system and/or per-thread.

6. Temporary, thread-local data (function call lifetimes, ...) should be
allocated on scratch arenas. You push and pop scratch arenas in a LIFO fashion
using ArenaPushScratch and ArenaPopScratch.

7. Functions taking (struct arena *mem, ..., growable): mem is optional.
Arena memory can't grow, so the valid combinations are (arena, NOT_GROWABLE)
and (NULL = heap, GROWABLE or NOT_GROWABLE).

Error Handling
==============

Severities: enum severity_id (ds_types.h). The Log API is thread-safe.

1. S_FATAL: unrecoverable errors, e.g. memory allocation failure. Log, then
call FatalCleanupAndExit().

2. S_ERROR: recoverable errors; continue with a fallback. Examples: a missing
asset is logged and replaced by a stub asset; the physics engine exceeds its
frame time budget (1/60 s).

3. S_WARNING: valid but noteworthy states. Example: the editor rejects a node
whose id already exists.

4. ds_Realloc logs and exits on failure itself; ds_Alloc returns NULL. Arena
pushes return NULL (also for size 0) and full non-growable pools {NULL, U32_MAX};
most arena pushes are assumed never to fail, so only check where overflow is
likely.

Naming
======

Conventions, not rules: follow them in new code; don't rename old code.

- ds_Type, ds_TypeFunction: engine types and the public/physics API.
- prefix_Function: subsystem API. r_ renderer, ui_ UI, led_ editor,
  c_ collision, gl_ OpenGL, gjk_/sat_ collision algorithms, ss_ serial
  stream, hi_ hierarchy index.
- Small common types: short lowercase type + uppercase function family
  (v3 -> V3Add, dll -> DLLAppend).
- ds_Try...: may fail, returns a failure state. ...Check: returns a boolean.

Comments
========

Short; state only what a reader would otherwise get wrong, never the obvious.
Multi-line comments: /* on its own line, then the text, then */ on its own line.
