import argparse
import os
import sys
import timeit

import h5py
import numpy as np
from PIL import Image
import scipy.ndimage.filters as filter
from tqdm import tqdm

"""
This script converts a 3D dataset (nxs or hdf) to a series of tiff images.
Users can select the bit-depth of the tiff.
Users can select the slice direction.
Please use Unix convention for path  (/ instead of \ on window)
"""

AXIS_OPTS= { 'X' : 0, 'Y': 1, 'Z': 2 }

parser = argparse.ArgumentParser(description='Extract TIFF from NXS/HDF')
parser.add_argument('--input-file', '-i', required=True, help='Input NXS/HDF file')
parser.add_argument('--output-dir', '-o', required=True, help='Output directory')
parser.add_argument('--depth', '-d', type=int, choices=[8,16,32], default=16, help='Pixel bit depth')
parser.add_argument('--axis', '-a', choices=AXIS_OPTS.keys(), default='X', help='Slice axis')
parser.add_argument('--start', type=int, default=0, help='Starting position')
parser.add_argument('--end', type=int, default=-1, help='End position. Use -1 for all slices following start')
parser.add_argument('--step', type=int, default=1, help='Step distance between extracted slices')
parser.add_argument('--dataset', default='entry/data/data', help='HDF dataset path')
parser.add_argument('--min-x', type=int, default=None, help='Crop the resulting slice images to have a minimum x value. Only used if slice axis is X.')
parser.add_argument('--max-x', type=int, default=None, help='Crop the resulting slice images to have a maximum x value. Only used if slice axis is X.')
parser.add_argument('--min-y', type=int, default=None, help='Crop the resulting slice images to have a minimum y value. Only used if slice axis is X.')
parser.add_argument('--max-y', type=int, default=None, help='Crop the resulting slice images to have a maximum y value. Only used if slice axis is X.')
args = parser.parse_args()

input_hdf = args.input_file
output_dir = args.output_dir

# Select the output-bit of the tiff image
convert_bit = args.depth
# Select the slice direction
# Select 0,1,or 2. Note that slice through axis = 2 is extremely slow
slice_axis = AXIS_OPTS[args.axis]
# Select range of extracting slice
start = args.start
stop = args.end # Use -1 for extracting all slices
step = args.step

min_x, max_x = args.min_x, args.max_x
min_y, max_y = args.min_y, args.max_y

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
    sys.exit(-1)
data3D = ifile[args.dataset]
if not isinstance(data3D, h5py.Dataset):
    print(f'Error: Path object \'{data3D.__class__.__name__}\' is not of type \'Dataset\'')
    sys.exit(-1)
(depth, height, width) = data3D.shape

time_start = timeit.default_timer()
if (convert_bit != 32):
    # Find global min and global max for rescaling data to 16-bit or 8-bit
    print("Find global_min and global_max for rescaling data to the lower bit")
    listuse = range(0, depth, 100)
    numsuse = len(listuse)
    listmin = np.zeros(numsuse, dtype=np.float32)
    listmax = np.zeros(numsuse, dtype=np.float32)
    for i, j in enumerate(tqdm(listuse, desc='Global min/max')):
        mat1 = data3D[j, :, :]
        mat1 = filter.gaussian_filter(mat1, (3, 3))
        listmin[i] = np.amin(mat1)
        listmax[i] = np.amax(mat1)

    globalmin = np.min(listmin)  # You may want to change this number manually
    # to be consistent between dataset
    globalmax = np.max(listmax)  # You may want to change this number manually
    # to be consistent between dataset
    print("Global min {0} ; Global max {1}".format(globalmin, globalmax))
    if slice_axis == 0:
        if stop == -1:
            stop = depth - 1
        else:
            stop = np.clip(stop, 0, depth - 1)
        # Extract to tif
        for i in tqdm(range(start, stop + 1, step), desc='Extract TIFFs'):
            mat1 = data3D[i, min_y:max_y, min_x:max_x]
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
        for i in tqdm(range(start, stop + 1, step), desc='Extract TIFFs'):
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
        for i in tqdm(range(start, stop + 1, step), desc='Extract TIFFs'):
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
        for i in tqdm(range(start, stop + 1, step), desc='Extract TIFFs'):
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
        for i in tqdm(range(start, stop + 1, step), desc='Extract TIFFs'):
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
        for i in tqdm(range(start, stop + 1, step), desc='Extract TIFFs'):
            mat1 = data3D[:, :, i]
            im = Image.fromarray(mat1)
            fileorder = "00000" + str(i)
            filename = output_dir + "/slice_" + fileorder[-5:] + ".tif"
            im.save(filename)
time_stop = timeit.default_timer()
print(f'Time cost {time_stop - time_start}')
