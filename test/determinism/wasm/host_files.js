/*
 * --pre-js for the web determinism test. The browser's files (MEMFS) do not outlive the page,
 * so each program argument that names a file in the served directory (the build directory)
 * is copied into the page's working directory before main. Arguments that already exist
 * (embedded ../config) or fail to load are left alone. main.c sends its output files back
 * with emrun_file_dump (emrun --dump-out-directory).
 */
Module['preRun'] = [].concat(Module['preRun'] || [], () =>
{
    for (const arg of Module['arguments'] || [])
    {
        if (arg.startsWith('--') || FS.analyzePath(arg).exists)
        {
            continue;
        }

        const dependency = 'host file ' + arg;
        addRunDependency(dependency);
        fetch(arg)
            .then((response) => response.ok ? response.arrayBuffer() : null)
            .then((buffer) => { if (buffer) { FS.writeFile(arg, new Uint8Array(buffer)); } })
            .catch(() => {})
            .finally(() => removeRunDependency(dependency));
    }
});
