#!/usr/bin/env python3

import argparse
import os


def main():
    # Arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--input-directory', '-i')
    parser.add_argument('--new-name', '-n')
    args = parser.parse_args()

    input_directory = args.input_directory
    new_name = args.new_name

    # Remove any trailing underscores in the provided new name
    while new_name.endswith('_'):
        new_name = new_name[0:-1]
    # Add underscore to name to get prefix
    new_prefix = new_name + '_'

    if not os.path.isdir(input_directory):
        print('Provided input directory is not a directory: {}'.format(input_directory))
        return

    original_dir_name = os.path.basename(os.path.normpath(input_directory))
    print('Original directory name:\t{}'.format(original_dir_name))

    # Detect original prefix
    files_in_dir = [filename for filename in os.listdir(input_directory)
                    if os.path.isfile(os.path.join(input_directory, filename))]
    tifs = [filename for filename in files_in_dir
            if os.path.splitext(filename)[1] == '.tif']
    tifs = sorted(tifs)
    if len(tifs) == 0:
        print('No .tifs detected in input directory: {}'.format(input_directory))
        return
    prefix_with_numbers = os.path.splitext(tifs[0])[0]
    if not prefix_with_numbers.split('_')[-1].isdigit():
        print('Detected filenames of form {} do not have a number as the final underscore separated value, as expected'
              .format(prefix_with_numbers))
        return
    original_prefix = '_'.join(prefix_with_numbers.split('_')[0:-1]) + '_'
    print('Original prefix:\t\t{}'.format(original_prefix))

    # Detect reconstruction directory if present
    dirs_in_dir = [dirname for dirname in os.listdir(input_directory)
                   if os.path.isdir(os.path.join(input_directory, dirname))]
    if len(dirs_in_dir) == 0:
        original_rec_dir = None
        original_rec_prefix = None
    elif len(dirs_in_dir) > 1:
        print('Expecting zero or one reconstruction directories, multiple subdirectories found: {}'.format(dirs_in_dir))
    else:
        original_rec_dir = dirs_in_dir[0]
        
    # Detect _rec prefix

    # Change directory and prefix names in logs
    # Rename files in rec directory
    # Rename rec directory
    # Rename files in original dir
    # Rename original dir


if __name__ == '__main__':
    main()
