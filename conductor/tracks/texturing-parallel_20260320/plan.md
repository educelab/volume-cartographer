# Implementation Plan: Parallel PPM Generation & Texturing

**Track ID:** texturing-parallel_20260320
**Spec:** [spec.md](./spec.md)
**Created:** 2026-03-20
**Status:** [ ] Not Started

## Overview

Add `#pragma omp parallel for` to the pixel loops in `PPMGenerator` and all four texturing algorithms. Fix progress reporting to be thread-safe. Add a `setThreadCount()` API. Verify output correctness and measure speedup.

## Phase 1: Baseline Benchmarks & Thread-Safety Audit

Establish correctness baselines before any changes. Identify all shared mutable state in the hot loops.

### Tasks

- [ ] Task 1.1: Write a deterministic correctness test for each algorithm: run single-threaded on a small synthetic PPM + volume, save reference output as golden files, assert byte-for-byte match on re-run
- [ ] Task 1.2: Write a timing benchmark binary (`vc_texturing_ParallelBenchmark`) that times PPMGenerator and IntersectionTexture on a real (or large synthetic) dataset; record single-thread baseline
- [ ] Task 1.3: Audit each algorithm's hot loop for shared mutable state: `NeighborhoodGenerator` instances, any static locals, OpenCV Mat writes (confirm unique-pixel access)
- [ ] Task 1.4: Confirm OpenMP is enabled in the build (`find_package(OpenMP)` already in cmake; verify `vc_texturing` links `OpenMP::OpenMP_CXX`)

### Verification

- [ ] Golden-file correctness tests pass single-threaded
- [ ] Baseline timing numbers recorded
- [ ] Shared-state audit documented (inline comments in source marking thread-safe vs. not)

## Phase 2: Parallelize PPMGenerator

### Tasks

- [ ] Task 2.1: Add `void setThreadCount(int n)` to `PPMGenerator`; store as `numThreads_` (default `omp_get_max_threads()`)
- [ ] Task 2.2: Replace the sequential `range2D` loop with `#pragma omp parallel for collapse(2) schedule(dynamic, 64) num_threads(numThreads_)` — the BVH traverser must be instantiated per-thread or confirmed thread-safe; if not thread-safe, use `firstprivate`
- [ ] Task 2.3: Fix progress reporting: move `progressUpdated` emission to only fire from `omp_get_thread_num() == 0` (or throttle to every 4096 pixels)
- [ ] Task 2.4: Run correctness test: parallel output must be bit-for-bit identical to golden reference
- [ ] Task 2.5: Run benchmark; record speedup at 2, 4, 8 threads

### Verification

- [ ] `vc_texturing_PPMGeneratorTest` passes
- [ ] Golden-file correctness test passes with OpenMP enabled
- [ ] Speedup ≥ 3× at 4 threads on benchmark dataset

## Phase 3: Parallelize Texturing Algorithms

### Tasks

- [ ] Task 3.1: **IntersectionTexture** — add `#pragma omp parallel for schedule(dynamic, 256) num_threads(numThreads_)` over the mappings loop; `Volume::interpolateAt` is read-only and thread-safe post-`vol-cache-parallel_20260320`
- [ ] Task 3.2: **CompositeTexture** — parallelize the mappings loop; `NeighborhoodGenerator` has internal state — instantiate one per thread using `#pragma omp threadprivate` or a per-thread vector initialized in a `#pragma omp parallel` block
- [ ] Task 3.3: **LayerTexture** — parallelize over pixels; note that mappings are sorted by Z before the loop — preserve this sort; each thread writes to its own pixel index across all `output_images[layer]` mats (safe)
- [ ] Task 3.4: **IntegralTexture** — parallelize; same `NeighborhoodGenerator` threading approach as CompositeTexture; ensure the normalization pass after the loop (if any) is single-threaded
- [ ] Task 3.5: Add `setThreadCount(int)` to `TexturingAlgorithm` base class (or as a mixin); propagate to all four implementations
- [ ] Task 3.6: Run correctness tests for all four algorithms against golden files

### Verification

- [ ] All four texturing algorithm correctness tests pass (parallel output matches golden)
- [ ] `vc_texturing_*Test` suite passes
- [ ] No `-fsanitize=thread` errors (run at least IntersectionTexture under TSan)

## Phase 4: Progress Reporting, CLI Integration & Final Benchmarks

### Tasks

- [ ] Task 4.1: Audit all `progressUpdated` call sites in parallel regions across all algorithms — ensure they are guarded (thread 0 only, or use `#pragma omp critical` for the signal emit)
- [ ] Task 4.2: Expose `--threads N` CLI flag in `vc_render` and `vc_generate_ppm` apps; wire through to algorithm `setThreadCount()`
- [ ] Task 4.3: Re-run full benchmark at 1, 2, 4, 8, 16 threads; document results in PR description
- [ ] Task 4.4: Full test suite: `ctest -V --test-dir build/`
- [ ] Task 4.5: `git clang-format develop` pass

### Verification

- [ ] All acceptance criteria in spec.md met
- [ ] `--threads` flag works end-to-end in CLI tools
- [ ] Benchmarks show ≥4× speedup on 8+ cores
- [ ] Full test suite passes
- [ ] Ready for review

## Final Verification

- [ ] All 5 components (PPMGenerator + 4 texturing algorithms) produce correct parallel output
- [ ] Thread count API documented in headers
- [ ] No data races (TSan clean on IntersectionTexture + PPMGenerator)
- [ ] CLI tools expose `--threads`
- [ ] Ready for review

---

_Generated by Conductor. Tasks will be marked [~] in progress and [x] complete._
