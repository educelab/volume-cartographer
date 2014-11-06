#!/usr/bin/env python

# this program takes in "unformatted" scroll data
# and turns it into a .volumepkg
# a .volumepkg has these properties
#  - 1 config file (config.json)
#    - number of slices
#    - dimensions
#    - 
#  - 1 folder containing scroll files in .tiff format
#    - the scroll naming convention will be the slices number
#  - 1 folder containing the gradient .ymls by slice number
#  - 1 folder containing the point cloud file

# first we should get a list of all of our slices
# we can start the index at 0


import sys
import os
import shutil
import re
import json
import time

sys.stdout.write('vc_packager\n')

# check for help
if '-h' in sys.argv or '--help' in sys.argv:
    print \
"""[NOTE] you are running the formatter right now :p
this program takes in as input the location of slices
in .tif format with the index as the last integers
in the filename (i.e '10.tif' or 'WOWTHISISSCROLL505.tif')

[USAGE] vc_packager [-sp | --slices-path] <path to slices> [-of | --output-file] <name of output file>
OR
[USAGE] python packager.py  [-sp | --slices-path] <path to slices> [-of | --output-file] <name of output file>

[NOTE] I recommend putting
alias vc_packager='python <absolute path to this program>'
into your .bashrc/.bash_profile

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

# create the folder structure
if os.path.isdir("tmp"):
    shutil.rmtree("tmp")
os.mkdir("tmp")
os.mkdir("tmp/slices")
os.mkdir("tmp/gradients")
os.mkdir("tmp/paths")
os.mkdir("tmp/surface_normals")

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
    filter(lambda f: f[-4::] == '.tif' and len(re.findall('\d+', f)) > 0, 
        os.listdir(mypath)),
    key=lambda f: int(re.findall('\d+', f)[-1]))

if len(slices) < 1:
    print "[Input Error] No Slices"
    sys.exit()

# parse arguements to see if we need to give the output file a certain name
# default is just output.volumepkg
if "-of" in sys.argv:
    outpath = sys.argv[sys.argv.index("-of") + 1]
    if outpath.endswith(".volpkg") != True:
        outpath = outpath + ".volpkg"
elif "--output-file" in sys.argv:
    outpath = sys.argv[sys.argv.index("--output-file") + 1]
    if outpath.endswith(".volpkg") != True:
        outpath = outpath + ".volpkg"
else:
    outpath = os.getcwd() + "/volume.volpkg"

# create the config options and save it to the config file
config_dict = {
    # number of slices
    "number of slices": len(slices),
    "slice location": "slices/",
    # size of the images.
    # currently no easy way to do this for .tiffs without
    # third party libs
    "dimensions": None,
}

f = open('tmp/config.json','w')
f.write(json.dumps(config_dict))
f.close()

#copy the slices to the slices folder and reformat the names
for i in range(len(slices)):
    sys.stdout.write('\r')
    sys.stdout.write('Copying slice ' + str(i + 1) + '/' + str(len(slices)))
    os.system('cp ' + mypath + slices[i] + ' tmp/slices/' + \
    '0'*(len(str(config_dict["number of slices"]))-len(str(i))) + \
    str(i) + '.tif')
    sys.stdout.flush()
sys.stdout.write('\n\n')

# compress the volumepkg
# os.system('tar -zcvf ' + outpath + ' tmp/*')

# move the volumepkg as a folder
os.system('mv tmp ' + str(outpath))

#delete the tmp directory if it still exists
if os.path.isdir("tmp"):
    shutil.rmtree("tmp")