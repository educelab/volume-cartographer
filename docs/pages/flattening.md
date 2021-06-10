# Flattening

[TOC]

## Flattening algorithms
Coming soon.

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