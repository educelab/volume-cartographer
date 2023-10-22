[![Volume Cartographer](docs/images/banner.svg)](https://github.com/educelab/volume-cartographer)

**Volume Cartographer** is a toolkit and set of cross-platform C++ libraries for
virtually unwrapping volumetric datasets. It was designed to recover text from
CT scans of ancient, badly damaged manuscripts, but can be applied in many
volumetric analysis applications.

## Changes
This fork contains the following changes compared to upstream currently:

- Fixed some crashes I came across
- Added volume name into dropdown
- Ensured that columns in segment table auto adjust their width (so that segment ID is fully visible)
- Adjusted mouse wheel usage:
  * Mouse wheel + Ctrl = Zoom in/out (as in most image apps); was previously Mouse wheel + Shift in VC
  * Mouse wheel + Shift = Next/previous slice (test-wise for big mouse wheel movements 10 slices; might need to be made user-configurable for different mouse models)
- Use spin boxes for slice numbers (they allow proper min/max value handling and mouse wheel can be used to change the numeric value)
- Highlight the toggle button of the currently active VC tool mode
- Added new shortcuts (see point 1 on feature wishlist) so that now via number keys 1, 2, 5, 10 and 100 steps are available
- Added a scroll area for the segmentation algorithm parameters, since at least on my screen size, there often was not enough space and all the labels/inputs got visually squished toghether.
