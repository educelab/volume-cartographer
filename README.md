[![Volume Cartographer](docs/images/banner.svg)](https://github.com/educelab/volume-cartographer)

**Volume Cartographer** is a toolkit and set of cross-platform C++ libraries for
virtually unwrapping volumetric datasets. It was designed to recover text from
CT scans of ancient, badly damaged manuscripts, but can be applied in many
volumetric analysis applications.

## Changes
This fork contains the following changes compared to upstream currently:

- Implemented both points from #1 of the segmentation wish list ([Goole Doc](https://docs.google.com/document/d/1YFILhWVHyijU_Yky3lKPvGAjmYm2QnRTYzMM7VqcogA)):
  * Added new keyboard shortcuts so that now via number keys slice navigation in 1, 2, 5, 10 and 100 steps is available
  * Added option to scan through slices while in the Segmentation Tool without losing any made curve changes
- Implemented image dragging/panning via right mouse button
- Fixed a bug in the OFS segmentation implementation that would (during the backwards interpolation part), output invalid `vcps` point sets, as the first two segment rows both list slice number 0 (should be of course 0 for the first and 1 for the second slice/row)
- Changed curve changes / point snapping to only use the left mouse button (without any additional modifiers)
- Adjusted mouse wheel usage:
  * Mouse Wheel + Ctrl = Zoom in/out (as in most image apps); was previously Mouse Wheel + Shift in VC
  * Mouse Wheel + Shift = Next/previous slice (scan range can be set via Q/E keys with visual feedback)
  * Mouse Wheel + W Key Hold = Adjust impact range
  * Mouse Wheel + R Key Hold = Follow Highlighted Curve (same as Mouse Forward/Backward buttons)
- Added Ctrl + G to easily jump to a given slice (opens an input popup to type in the slice index) => no more need to manually click into the spinner below the viewer
- Added shortcut F to jump back to the slice the segmentation tool was started on
- Added settings dialog (using an INI file `VC.ini`) to make some features configurable
- Added user configurable impact ranges (default remains 1-20)
- Added a "Recent volpkg" list and menu option to easily reopen one of the last 10 volume packages
- Added option to auto open the last volpkg upon app start
- Added the feature to remove a segment/path from the volpkg
- Added option to center image slice on mouse cursor during mouse wheel zooming (configurable via settings)
- Use spin boxes for slice numbers (they allow proper min/max value handling and mouse wheel can be used to change the numeric value)
- Added volume name into volume dropdown
- Ensured that columns in segment table auto adjust their width (so that segment ID is fully visible)
- Highlight the toggle button of the currently active VC tool mode
- Added a scroll area for the segmentation algorithm parameters, since at least on my screen size, there often was not enough space and all the labels/inputs got visually squished toghether.
- Fixed some crashes I came across


