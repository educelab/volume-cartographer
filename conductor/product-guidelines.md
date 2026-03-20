# Product Guidelines — Volume Cartographer

## Voice and Tone

**Concise and direct.** Documentation, CLI help text, and UI messages should be terse and precise. Assume a technically literate audience. Avoid filler language, marketing phrasing, or over-explanation.

## Design Principles

1. **Modularity** — Algorithms are swappable and the pipeline is composable. Segmentation, meshing, flattening, and texturing are independent library layers. New algorithms should slot in via abstract base classes without touching the rest of the pipeline.

2. **Developer/researcher experience** — The library API should be clean and intuitive for C++ developers and researchers. CLI tools should be scriptable and composable. Python bindings should expose the full pipeline for scripting workflows.

3. **Performance first** — CT volumes are large (gigabytes to terabytes). Memory efficiency (LRU caching, memory-mapped I/O), parallel processing, and algorithmic complexity are first-class concerns.

## Additional Standards

- Public APIs must be documented in headers.
- Breaking changes to the `VC::core` public API require explicit version bumps.
- CLI tools must support `--help` and produce machine-parseable output where appropriate.
