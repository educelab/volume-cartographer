utils
=====

Histogram
---------

Usage :  Histogram  inputImageFileName  numberOfHistogramBins

This program takes an image and number of histogram bins and outputs the frequency for each histogram bins. Used to figure out how many bins we need in registration in order to properly capture all relevant color values in the images.

auto_align
----------

Given a static and moving image, generates a landmark file with shared points in between the images. This seems to work well but could use improvement. The number of points should be an option, it should be faster on large images, and the points should be evenly distributed across the images (they are often clustered which is great for registration in the area but the rest of the image might not register so well).

batch_diff
----------

Operates on batch .csv file created by build_csv. For each registration that has already been performed, creates a diff image comparing registered and static images. Used to quickly review batch registrations to identify those that did not work so well (their diff images are visibly misaligned).

batch_pipeline
--------------

Given the .csv of registrations to be performed, runs them in parallel.

build_csv
---------

Generate a batch .csv file for registration. Each row is a set of commands to be passed to one instance of LandmarkRegistration. Created specifically for Chad.

plot
----

Reads the output from LandmarkRegistration through stdin and produces a plot of the metric for each iteration. Another way to visualize registration process and how playing with parameters affects the resulting registration quality.

visualize_landmarks
-------------------

Usage :  visualize_landmarks landmarksFile staticImage movingImage

Adds numbered crosshairs to the static and moving images according to the provided landmarks file, places the marked static and moving images next to each other in a composite image, and saves this as montaged.jpg. Used to quickly visualize landmarks on a pair of images to ensure they align.