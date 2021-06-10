# Color Mapping

[TOC]

## Color maps and LUTs
A lookup table (LUT) is a data structure used to remap raw data values to a new 
range of data values. Often this is used to false color an image by remapping 
raw pixel values through a LUT to a new range of RGB color values. This process 
is also known as color mapping.

Volume Cartographer provides support for linearly remapping images to a number 
of built-in color maps using the volcart::ApplyLUT functions. The basic process
is as follows:

```{.cpp}
#include <vc/core/util/ApplyLUT.hpp>
#include <vc/core/util/ColorMaps.hpp>

using namespace volcart;

// Get the LUT by enum or by string name
auto lut = GetColorMapLUT(ColorMap::Viridis);
lut = GetColorMapLUT("viridis");

// Automatically remap to the data min/max
auto mapped = ApplyLUT(image, lut);
```

Available color maps include:

Color map  | Output range
---------- | -----------
Magma      | ![](magma.png)
Inferno    | ![](inferno.png)
Plasma     | ![](plasma.png)
Viridis    | ![](viridis.png)
BWR        | ![](bwr.png)

See volcart::ColorMap for a full list of available color maps.

## Applying LUTs to any image

We provide the `vc_color_map` utility for remapping arbitrary images to one of 
our built-in color maps:

```{.shell}
vc_color_map --input foo.png --output bar.png --color-map viridis
```

Run `vc_color_map --help` for more options and information.