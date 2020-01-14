import os
import sys
import h5py
from PIL import Image
import scipy.ndimage.filters as filter
import numpy as np
import timeit

"""
This script converts a 3D dataset (nxs or hdf) to a series of tiff images.
Users can select the bit-depth of the tiff.
Users can select the slice direction.
Please use Unix convention for path  (/ instead of \ on window)
"""

input_hdf = "I:/down_sample_5_5_5.hdf"
output_dir = "I:/tif_8bit/"

# Select the output-bit of the tiff image
convert_bit = 16  # Select 8, 16 or 32
# Select the slice direction

# Select 0,1,or 2. Note that slice through axis = 2 is extremely slow
slice_axis = 0
# Select range of extracting slice
start = 0
stop = -1   # Use -1 for extracting all slices
step = 1

# Check the input parameters
if not((convert_bit == 16) or (convert_bit == 8) or (convert_bit == 32)):
    print("Please use one of 3 options: 8, 16, 32")
    sys.exit(0)

if not((slice_axis == 0) or (slice_axis == 1) or (slice_axis == 2)):
    print("Please use one of 3 options: 0, 1, 2")
    sys.exit(0)

# Create output directory
try:
    os.makedirs(output_dir)
except OSError:
    if not os.path.isdir(output_dir):
        print('Can not create the directory, please check the permission!!!')
        raise

# Open hdf file
try:
    ifile = h5py.File(input_hdf, 'r')
except:
    print("Cannot open hdf file {} !!!").format(input_hdf)
    sys.exit(0)
data3D = ifile['entry/data']
(depth, height, width) = data3D.shape

time_start = timeit.default_timer()
if (convert_bit != 32):
    # Find global min and global max for rescaling data to 16-bit or 8-bit
    print("Find global_min and global_max for rescaling data to the lower bit")
    listuse = range(0, depth, 100)
    numsuse = len(listuse)
    listmin = np.zeros(numsuse, dtype=np.float32)
    listmax = np.zeros(numsuse, dtype=np.float32)
    for i, j in enumerate(listuse):
        mat1 = data3D[j, :, :]
        mat1 = filter.gaussian_filter(mat1, (3, 3))
        listmin[i] = np.amin(mat1)
        listmax[i] = np.amax(mat1)

    globalmin = np.min(listmin)  # You may want to change this number manually
    # to be consistent between dataset
    globalmax = np.max(listmax)  # You may want to change this number manually
    # to be consistent between dataset
    print("Global min {0} ; Global max {1}").format(globalmin, globalmax)
    if slice_axis == 0:
        if stop == -1:
            stop = depth - 1
        else:
            stop = np.clip(stop, 0, depth - 1)
        # Extract to tif
        for i in range(start, stop + 1, step):
            mat1 = data3D[i, :, :]
            mat1 = np.clip(mat1, globalmin, globalmax)
            mat1 = (mat1 - globalmin) / (globalmax - globalmin)
            if convert_bit == 16:
                mat1 = np.clip(np.uint16(mat1 * 65535), 0, 65535)
            else:
                mat1 = np.clip(np.uint8(mat1 * 255), 0, 255)
            im = Image.fromarray(mat1)
            fileorder = "00000" + str(i)
            filename = output_dir + "/slice_" + fileorder[-5:] + ".tif"
            im.save(filename)
    if slice_axis == 1:
        if stop == -1:
            stop = height - 1
        else:
            stop = np.clip(stop, 0, height - 1)
        # Extract to tif
        for i in range(start, stop + 1, step):
            mat1 = data3D[:, i, :]
            mat1 = np.clip(mat1, globalmin, globalmax)
            mat1 = (mat1 - globalmin) / (globalmax - globalmin)
            if convert_bit == 16:
                mat1 = np.clip(np.uint16(mat1 * 65535), 0, 65535)
            else:
                mat1 = np.clip(np.uint8(mat1 * 255), 0, 255)
            im = Image.fromarray(mat1)
            fileorder = "00000" + str(i)
            filename = output_dir + "/slice_" + fileorder[-5:] + ".tif"
            im.save(filename)
    if slice_axis == 2:
        if stop == -1:
            stop = width - 1
        else:
            stop = np.clip(stop, 0, width - 1)
        # Extract to tif
        for i in range(start, stop + 1, step):
            mat1 = data3D[:, :, i]
            mat1 = np.clip(mat1, globalmin, globalmax)
            mat1 = (mat1 - globalmin) / (globalmax - globalmin)
            if convert_bit == 16:
                mat1 = np.clip(np.uint16(mat1 * 65535), 0, 65535)
            else:
                mat1 = np.clip(np.uint8(mat1 * 255), 0, 255)
            im = Image.fromarray(mat1)
            fileorder = "00000" + str(i)
            filename = output_dir + "/slice_" + fileorder[-5:] + ".tif"
            im.save(filename)
else:
    if slice_axis == 0:
        if stop == -1:
            stop = depth - 1
        else:
            stop = np.clip(stop, 0, depth - 1)
        # Extract to tif
        for i in range(start, stop + 1, step):
            mat1 = data3D[i, :, :]
            im = Image.fromarray(mat1)
            fileorder = "00000" + str(i)
            filename = output_dir + "/slice_" + fileorder[-5:] + ".tif"
            im.save(filename)
    if slice_axis == 1:
        if stop == -1:
            stop = height - 1
        else:
            stop = np.clip(stop, 0, height - 1)
        # Extract to tif
        for i in range(start, stop + 1, step):
            mat1 = data3D[:, i, :]
            im = Image.fromarray(mat1)
            fileorder = "00000" + str(i)
            filename = output_dir + "/slice_" + fileorder[-5:] + ".tif"
            im.save(filename)
    if slice_axis == 2:
        if stop == -1:
            stop = width - 1
        else:
            stop = np.clip(stop, 0, width - 1)
        # Extract to tif
        for i in range(start, stop + 1, step):
            mat1 = data3D[:, :, i]
            im = Image.fromarray(mat1)
            fileorder = "00000" + str(i)
            filename = output_dir + "/slice_" + fileorder[-5:] + ".tif"
            im.save(filename)
time_stop = timeit.default_timer()
print("Time cost {}").format(time_stop - time_start)
