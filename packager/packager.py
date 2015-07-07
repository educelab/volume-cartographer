#!/usr/bin/env python

# this program takes in "unformatted" scroll data
# and turns it into a .volpkg
# a .volpkg has these properties
#  - 1 config file (config.json)
#    - number of slices
#    - dimensions
#    - 
#  - 1 folder containing scroll files in .tiff format
#    - the scroll naming convention will be the slices number
#  - 1 folder containing the point cloud file

import sys
import os
import subprocess
import shutil
import re
import json
import time

sys.stdout.write('vc_packager\n\n')

# check for help
if '-h' in sys.argv or '--help' in sys.argv:
    print \
"""[USAGE] vc_packager [-sp | --slices-path] <path to slices> [-of | --output-file] <name of output file>

[OPTIONS] 
[-sp | --slice-path] <path to slices>
    this option allows you to specify a custom path to slices
    the default is the current directory the program was run in

[-of | --output-file] <name of output file>
    this option allows you to specify the location of the output file
    the default is output.volumepkg and will be located in the 
    current working directory
"""
    sys.exit()

# get the scroll location from the cli arguements
# or by default from the current file
# if there arent any .tiff files then throw an error
if "-sp" in sys.argv:
    mypath = sys.argv[sys.argv.index("-sp") + 1]
elif "--slices-path" in sys.argv:
    mypath = sys.argv[sys.argv.index("--slices-path") + 1]
else:
    mypath = os.getcwd()

slices = sorted(
    filter(lambda f: f[-4::] == '.tif', 
        os.listdir(mypath))
    )

if len(slices) < 1:
    sys.exit("[Input Error] No Slices")

# parse arguements to see if we need to give the output file a certain name
# default is just output.volumepkg
if "-of" in sys.argv:
    outpath = sys.argv[sys.argv.index("-of") + 1]
elif "--output-file" in sys.argv:
    outpath = sys.argv[sys.argv.index("--output-file") + 1]
else:
    outpath = os.getcwd() + "/volume"

# get scan information
voxelSize = float(raw_input("Enter the voxel size in microns (um): "))
materialThickness = float(raw_input("Enter the estimated material thickness in microns (um): "))

# analyze the volume
for i in range(len(slices)):
    sys.stdout.write('\r')
    sys.stdout.write('Analyzing slice ' + str(i + 1) + '/' + str(len(slices)))
    # get width/height
    info = subprocess.check_output('vc_analyze "' + mypath+slices[i] + '" all', shell=True)
    thisWidth, thisHeight, thisDepth, thisMin, thisMax = info.split()
    
    # cast to the proper type
    thisWidth = int(thisWidth)
    thisHeight = int(thisHeight)
    thisDepth = int(thisDepth)
    thisMin = float(thisMin)
    thisMax = float(thisMax)

    # set intial values from first slice 
    if i == 0:
        width = thisWidth
        height = thisHeight
        depth = thisDepth
        volMin = thisMin
        volMax = thisMax
    else:
        # width, height, and depth must match slice 0
        if (thisWidth != width or
            thisHeight != height or
            thisDepth != depth):
                sys.exit('\nError: "' + mypath+slices[i] + '" does not match size/depth of volume!\n')
        # update volume min and max intensities if they're lower/higher respectively
        if thisMin < volMin:
            volMin = thisMin
        if thisMax > volMax:
            volMax = thisMax
    sys.stdout.flush()
sys.stdout.write('\n')

# scale min and max to 16-bit if 8-bit unsigned
if depth == 0:
    volMin = volMin*65535.00/255.00
    volMax = volMax*65535.00/255.00

# create the config options and save it to the config file
config_dict = {
    # volumepkg version
    "version": 2.0,
    # name
    "volumepkg name": outpath.split('/')[-1],

    # number of slices
    "number of slices": len(slices),
    "slice location": "slices/",

    # size and intensity info for volume
    "width": width,
    "height": height,
    "min": volMin,
    "max": volMax,
    "voxelsize": voxelSize,

    # material information
    "materialthickness": materialThickness
}

# create the folder structure
if os.path.isdir("tmp"):
    shutil.rmtree("tmp")
os.mkdir("tmp")
os.mkdir("tmp/slices")
os.mkdir("tmp/paths")
os.mkdir("tmp/surface_normals")

f = open('tmp/config.json','w')
f.write(json.dumps(config_dict))
f.close()

#copy the slices to the slices folder and reformat the names
for i in range(len(slices)):
    sys.stdout.write('\r')
    if depth != 2:
        sys.stdout.write('Conforming slice ' + str(i + 1) + '/' + str(len(slices)))
        os.system('vc_conform "' + mypath + slices[i] + '" tmp/slices/' + \
        '0'*(len(str(config_dict["number of slices"]))-len(str(i))) + \
        str(i) + '.tif')
    else:
        sys.stdout.write('Copying slice ' + str(i + 1) + '/' + str(len(slices)))
        os.system('cp "' + mypath + slices[i] + '" tmp/slices/' + \
        '0'*(len(str(config_dict["number of slices"]))-len(str(i))) + \
        str(i) + '.tif')
    sys.stdout.flush()
sys.stdout.write('\n\n')

#create the volumepkg file
if outpath.endswith(".volpkg") != True:
    outpath = outpath + ".volpkg"

# compress the volumepkg
# os.system('tar -zcvf ' + outpath + ' tmp/*')

# move the volumepkg as a folder
os.system('mv tmp "' + str(outpath) + '"')

#delete the tmp directory if it still exists
if os.path.isdir("tmp"):
    shutil.rmtree("tmp")
