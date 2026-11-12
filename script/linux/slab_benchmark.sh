#!/bin/sh
# Slab benchmark: builds the performance test binary into build_slab_bench and prints the Slab Performance
# suite's minima (the mesh-array replay: the null floor, ds_TSlab, the old per-thread-list slab) for each
# worker count. Usage: script/linux/slab_benchmark.sh [workers ...]   (default: 1 2 4 ... up to nproc)
cd "$(dirname "$0")/../.." || exit 1

if (command -v ninja > /dev/null 2>&1); then
	CMAKE_GENERATOR="Ninja"
else
	CMAKE_GENERATOR="Unix Makefiles"
fi

BUILD=build_slab_bench
cmake -B "$BUILD" -G "$CMAKE_GENERATOR" -DDS_OPTIMIZE=ON -DDS_DEBUG=OFF -DDS_PROFILE=OFF -DDS_LOG=ON \
	-DDS_TEST_PERFORMANCE=ON > /dev/null || exit 1
cmake --build "$BUILD" -j"$(nproc)" --target DreamscapeTest || exit 1

if [ $# -gt 0 ]; then
	WORKERS="$*"
else
	WORKERS=""
	n=1
	while [ "$n" -le "$(nproc)" ]; do
		WORKERS="$WORKERS $n"
		n=$((n * 2))
	done
fi

echo "cpu: $(grep -m1 'model name' /proc/cpuinfo | cut -d: -f2 | sed 's/^ //'), $(nproc) logical cores"
echo "compiler: $(grep CMAKE_C_COMPILER: "$BUILD/CMakeCache.txt" | cut -d= -f2) $(cc --version 2>/dev/null | head -1)"
echo "minimum over repetitions; ns per event per worker, 'over null' = minus the no-allocator floor"
echo

for n in $WORKERS; do
	cfg="slab_bench_$n.cfg"
	out="slab_bench_$n.txt"
	sed "s/^thread_count: .*/thread_count: $n/" config/default.cfg > "$BUILD/$cfg"
	(cd "$BUILD" && SDL_VIDEO_DRIVER=offscreen timeout -k 5 1800 ./DreamscapeTest "config=$cfg" > "$out" 2>&1) &
	pid=$!
	while kill -0 $pid 2> /dev/null; do
		sleep 2
		if grep -q "Running peformance suite Serialize" "$BUILD/$out"; then
			pkill -P $pid 2> /dev/null
			kill $pid 2> /dev/null
			break
		fi
	done
	wait $pid 2> /dev/null

	echo "== $n workers"
	awk -v n="$n" '
		/workers,/ { match($0, /^\t([^:]*):/, v); name = v[1]; match($0, / ([0-9]+) events/, e); ev = e[1];
		             match($0, /repetition ([0-9.]+) allocs/, a); al = a[1] }
		/^min:/ && ev > 0 { match($0, /\[([0-9.]+)ms\]/, m); us = m[1]*1000; ns = us*1000*n/ev;
		             if (name ~ /^null/) null = ns;
		             printf "%-46s %8.2f us  %6.1f ns/event  %6.1f over null  allocator calls/rep %7.1f\n", name, us, ns, ns - null, al;
		             ev = 0 }' "$BUILD/$out"
	echo
done
