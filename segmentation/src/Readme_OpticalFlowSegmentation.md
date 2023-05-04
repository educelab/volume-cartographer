# README: Optical Flow Segmentation Algorithm for Volume Cartographer (VC)

This algorithm is designed for segmenting a 3D scanned scroll papyrus sheet using optical flow. It propagates a chain of points forward through a volume from a starting z-index to an ending z-index. Each point is assumed to start within a page layer. The ending index is inclusive. Please note that this algorithm is not deterministic and yields slightly different results on each run.

## Parameters

Below is a list of parameters that can be adjusted for the Optical Flow Segmentation Algorithm:

1. **Optical Flow Displacement Threshold** (`optical_flow_displacement_threshold`): This parameter sets the maximum single pixel optical flow displacement before interpolating a pixel region. Range minimum: 0. Higher values allow more displacement before interpolation, while lower values trigger interpolation more frequently.

2. **Optical Flow Dark Pixel Threshold** (`optical_flow_pixel_threshold`): This parameter sets the threshold for what pixel brightness is considered while calculating optical flow. Darker pixels' optical flow is interpolated from brighter ones in the area. Range: 0-255. Higher values disregard more dark pixels during computation, while lower values include more dark pixels.

3. **Smoothen Curve at Dark Points** (`outside_threshold`): This parameter sets the threshold of what pixel brightness is considered too deep inside a sheet (higher than the threshold) and then tries to smoothen those points back towards the edge of the sheet. Range: 0-255.

4. **Smoothen Curve at Bright Points** (`smoothen_by_brightness`):  This parameter sets the threshold for what pixel brightness is considered as beeing outside the sheet. Pixels considered outside the sheet are  smoothened in an attempt to get them tracking the sheet again. Range: 0-256. Smoothen curve at pixels above this threshold..

5. **Purge Cache** (`purge_cache`): This flag enables or disables cache purging. Set to `true` to enable cache purging, and `false` to disable.

6. **Maximum Cache Size** (`nr_cache_slices`): This parameter sets the number of slices that should be cached. Range: positive integers. Higher values result in more slices being cached, while lower values reduce the number of cached slices. Setting depends on your available RAM. -1 = disabled.

