# Flattening

[TOC]

## Flattening algorithms

Volume Cartographer provides several mesh parameterization (flattening)
algorithms through the volcart::texturing::AngleBasedFlattening class. All
methods are implemented by the [OpenABF](https://github.com/educelab/OpenABF)
library.

The flattening pipeline has two independent stages:

1. **Angle optimization (optional):** ABF++ \cite sheffer2005abf computes
   optimal interior angles that minimize distortion. This is recommended for
   most use cases.
2. **Parameterization:** Maps the 3D mesh to 2D using either AngleBasedLSCM or
   HierarchicalLSCM \cite ray2003hierarchical.

### Available methods

| Method | CLI name | Description |
|--------|----------|-------------|
| ABF + LSCM (SparseLU) | `ABF` | **Default.** Fastest option. Uses a direct solver for the LSCM system. May run out of memory on very large meshes. |
| ABF + LSCM (CG) | `ABF` with `--solver CG` | Uses an iterative conjugate gradient solver. Lower memory than LU but roughly 2x slower than HLSCM. |
| ABF + HLSCM | `ABF-HLSCM` | Uses a cascadic multigrid LSCM solver. Slower than LU but significantly lower memory usage. **Preferred over LSCM (CG) when memory is a concern.** |
| LSCM (SparseLU) | `LSCM` | LSCM without ABF++ pre-optimization. |
| LSCM (CG) | `LSCM` with `--solver CG` | LSCM (CG) without ABF++ pre-optimization. |
| HLSCM | `HLSCM` | HierarchicalLSCM without ABF++ pre-optimization. |

@note The `--solver` option only applies to the standard LSCM path.
HierarchicalLSCM always uses ConjugateGradient internally to enable
warm-starting across hierarchy levels.

### Choosing a method

For most use cases, the default **ABF + LSCM (SparseLU)** is the best choice.
It produces high-quality results and is the fastest option. However, the direct
solver requires memory proportional to the mesh size, which can be a problem for
very large meshes.

When memory is limited, **ABF + HLSCM** is the recommended alternative. It uses
a cascadic multigrid approach that solves the LSCM system on a coarsened mesh
hierarchy, warm-starting each level from the previous solution. This
dramatically reduces peak memory usage compared to SparseLU while being faster
than the standard CG solver.

**ABF + LSCM (CG)** is available as a fallback but is generally slower than
HLSCM for comparable memory savings. Prefer HLSCM over LSCM (CG) unless you
have a specific reason to use the standard iterative solver.

Omitting ABF++ (i.e. using LSCM, HLSCM, or LSCM-CG alone) is faster but
produces lower quality results. This may be acceptable for previews or when
the input mesh is already well-conditioned.

@note When compiled with OpenMP support, the ConjugateGradient solver (used by
both LSCM-CG and HLSCM) is multithreaded. Thread count can be controlled
through the `--threads` option in `vc_flatten_mesh` and `vc_render` when the
project is built with OpenMP support. See
[Eigen and multi-threading](https://libeigen.gitlab.io/eigen/docs-nightly/TopicMultiThreading.html)
for more details.

### CLI usage

Flattening is available through `vc_flatten_mesh` and `vc_render`:

```{.unparsed}
# Default: ABF + LSCM with SparseLU
$ vc_flatten_mesh -i input.obj -o output.obj

# ABF + HLSCM (lower memory)
$ vc_flatten_mesh -i input.obj -o output.obj --method ABF-HLSCM

# HLSCM only (no ABF pre-optimization)
$ vc_flatten_mesh -i input.obj -o output.obj --method HLSCM

# ABF + LSCM with ConjugateGradient
$ vc_flatten_mesh -i input.obj -o output.obj --solver CG
```

In `vc_render`, the algorithm is selected with `--uv-algorithm`:

```{.unparsed}
# Default: ABF + LSCM (algorithm 0)
$ vc_render ... --uv-algorithm 0

# ABF + HLSCM (algorithm 1)
$ vc_render ... --uv-algorithm 1

# LSCM only (algorithm 2)
$ vc_render ... --uv-algorithm 2

# HLSCM only (algorithm 3)
$ vc_render ... --uv-algorithm 3
```

### C++ usage

```{.cpp}
#include <vc/texturing/AngleBasedFlattening.hpp>

using namespace volcart::texturing;

AngleBasedFlattening abf(mesh);

// Default: ABF + LSCM with SparseLU
auto result = abf.compute();

// ABF + HLSCM
abf.setUseHLSCM(true);
result = abf.compute();

// HLSCM only (no ABF)
abf.setUseABF(false);
abf.setUseHLSCM(true);
result = abf.compute();

// ABF + LSCM with ConjugateGradient
abf.setUseABF(true);
abf.setUseHLSCM(false);
abf.setSolver(AngleBasedFlattening::Solver::ConjugateGradient);
result = abf.compute();
```

## Measuring flattening error

A common way to measure flattening error is to observe the L2 and LInf stretch 
metrics introduced by Sander et al. \cite sander2001texture . Volume 
Cartographer provides a method for calculating these metrics in the Texturing
module. Once calculated, these metrics can also be plotted to an image:

```{.cpp}
#include <iostream>

#include <vc/texturing/FlatteningError.hpp> // LStretch
#include <vc/texturing/PPMGenerator.hpp> // GenerateCellMap

using namespace volcart;
using namespace volcart::texturing;

// Calculate error metrics
auto metrics = LStretch(mesh3D, mesh2D);

// Report the inverted global metrics (see note below)
metrics = InvertLStretchMetrics(metrics);
std::cout << "Global L2 stretch: " << metrics.l2 << "\n";
std::cout << "Global LInf stretch: " << metrics.lInf << "\n";

// Plot the per-face error
auto cellMap = GenerateCellMap(mesh2D, uvMap, height, width);
auto plots = PlotLStretchError(metrics, cellMap, ColorMap::Plasma);
```

Flattening error can also be plotted when running `vc_render`:

```{.unparsed}
$ vc_render ... --uv-plot-error error_plots.png
```

This will generate two images, `error_plots_l2.png` and `error_plots_lInf.png`, 
plotting the per-face L2 and LInf metrics respectively.

@note The L2 and LInf stretch metrics represent the stretch **from the 2D domain
to the 3D domain**. Thus, an LInf value of 0.5 indicates a flattening where
a 3D unit vector is **half** the length of its corresponding 2D unit vector. 
This is the inverse of how stretch is generally considered in most of Volume 
Cartographer, where we are interested in the stretch introduced relative to the 
3D domain. The VC applications dealing with these metrics will often invert the
returned values to make reporting more intuitive.