#!/usr/bin/env python3

import argparse
import os
import sys


# https://stackoverflow.com/questions/3041986/apt-command-line-interface-like-yes-no-input
def query_yes_no(question, default="yes"):
    """Ask a yes/no question via raw_input() and return their answer.

    "question" is a string that is presented to the user.
    "default" is the presumed answer if the user just hits <Enter>.
        It must be "yes" (the default), "no" or None (meaning
        an answer is required of the user).

    The "answer" return value is True for "yes" or False for "no".
    """
    valid = {"yes": True, "y": True, "ye": True,
             "no": False, "n": False}
    if default is None:
        prompt = " [y/n] "
    elif default == "yes":
        prompt = " [Y/n] "
    elif default == "no":
        prompt = " [y/N] "
    else:
        raise ValueError("invalid default answer: '%s'" % default)

    while True:
        sys.stdout.write(question + prompt)
        choice = input().lower()
        if default is not None and choice == '':
            return valid[default]
        elif choice in valid:
            return valid[choice]
        else:
            sys.stdout.write("Please respond with 'yes' or 'no' "
                             "(or 'y' or 'n').\n")


def replace_string_in_file(filename, old_str, new_str):
    with open(filename, 'r') as f:
        file_contents = f.read()

    file_contents = file_contents.replace(old_str, new_str)

    with open(filename, 'w') as f:
        f.write(file_contents)


# Requires filename with full path provided
# Will not make changes to the directory names in the full path
def replace_string_in_filename(filename, old_str, new_str):
    base_path, old_filename = os.path.split(filename)
    new_filename = old_filename.replace(old_str, new_str)
    new_file_path = os.path.join(base_path, new_filename)
    os.rename(filename, new_file_path)
    return new_file_path


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

    if not os.path.isdir(input_directory):
        print('Provided input directory is not a directory: {}'.format(input_directory))
        return

    original_dir_name = os.path.basename(os.path.normpath(input_directory))

    # Detect original prefix
    files_in_dir = [filename for filename in os.listdir(input_directory)
                    if os.path.isfile(os.path.join(input_directory, filename))]
    tifs = [filename for filename in files_in_dir
            if os.path.splitext(filename)[1] == '.tif']
    tifs = sorted(tifs)
    if len(tifs) == 0:
        print('No .tifs detected in input directory: {}'.format(input_directory))
        return
    # Remove digits from end of filename to get prefix
    original_prefix = os.path.splitext(tifs[0])[0]
    while original_prefix[-1].isdigit() or original_prefix[-1] == '~':
        original_prefix = original_prefix[0:-1]

    # Detect reconstruction directory if present
    dirs_in_dir = [dirname for dirname in os.listdir(input_directory)
                   if os.path.isdir(os.path.join(input_directory, dirname))]
    if len(dirs_in_dir) == 0:
        original_rec_dir = None
    elif len(dirs_in_dir) > 1:
        print('Expecting zero or one reconstruction directories, multiple subdirectories found: {}'.format(dirs_in_dir))
        return
    else:
        original_rec_dir = dirs_in_dir[0]

    # Detect reconstruction prefix if present
    original_rec_prefix = None
    if original_rec_dir is not None:
        files_in_rec_dir = [filename for filename in os.listdir(os.path.join(input_directory, original_rec_dir))
                            if os.path.isfile(os.path.join(input_directory, original_rec_dir, filename))]
        logfiles_in_rec_dir = [filename for filename in files_in_rec_dir
                               if os.path.splitext(filename)[1] == '.log']
        if len(logfiles_in_rec_dir) == 0:
            print('No .log file found in reconstruction dir: {}, will not touch this directory'
                  .format(original_rec_dir))
            original_rec_dir = None
        elif len(logfiles_in_rec_dir) > 1:
            print('Multiple .log files found in reconstruction dir: {}, will not touch this directory'
                  .format(original_rec_dir))
            original_rec_dir = None
        else:
            original_rec_prefix = os.path.splitext(logfiles_in_rec_dir[0])[0]

    # Print summary, verify with user
    print('Name changes to be made:')
    print('    Directory name:\t\t{}\t->\t{}'.format(original_dir_name, new_name))
    print('    Prefix:\t\t\t{}\t->\t{}'.format(original_prefix, new_name + '_'))
    if original_rec_dir is not None:
        print('    Reconstruction dir:\t\t{}\t->\t{}'.format(original_rec_dir, new_name + '_rec'))
        print('    Reconstruction prefix:\t{}\t->\t{}'.format(original_rec_prefix, new_name + '_rec_'))
    else:
        print('    Reconstruction dir:\t\tSkipping')
        print('    Reconstruction prefix:\tSkipping')
    if not query_yes_no('Proceed?'):
        return

    # Patch directory and prefix names in logs (including reconstruction directory)
    print('Patching logfiles...')
    logfiles_full_names = [os.path.join(input_directory, filename)
                           for filename in os.listdir(input_directory)
                           if os.path.isfile(os.path.join(input_directory, filename))
                           and os.path.splitext(filename)[1] == '.log']
    logfiles_full_names += [os.path.join(input_directory, original_rec_dir, filename)
                            for filename in os.listdir(os.path.join(input_directory, original_rec_dir))
                            if os.path.isfile(os.path.join(input_directory, original_rec_dir, filename))
                            and os.path.splitext(filename)[1] == '.log']
    for logfile in logfiles_full_names:
        try:
            replace_string_in_file(logfile, original_rec_prefix, new_name + '_rec_')
            replace_string_in_file(logfile, original_rec_dir, new_name + '_rec')
            replace_string_in_file(logfile, original_prefix, new_name + '_')
            replace_string_in_file(logfile, original_dir_name, new_name)
            print('    Patched logfile: {}'.format(logfile))
        except UnicodeDecodeError:
            print('    Skipping non-text logfile {}'.format(logfile))
            continue

    # Rename files in reconstruction directory
    if original_rec_dir is not None:
        print('Renaming files in reconstruction directory...')
        filenames = [os.path.join(input_directory, original_rec_dir, filename)
                     for filename in os.listdir(os.path.join(input_directory, original_rec_dir))
                     if os.path.isfile(os.path.join(input_directory, original_rec_dir, filename))]
        for filename in filenames:
            fixed_filename = replace_string_in_filename(filename, original_rec_prefix, new_name + '_rec_')
            replace_string_in_filename(fixed_filename, original_prefix, new_name + '_')

    # Rename reconstruction directory
    if original_rec_dir is not None:
        print('Renaming reconstruction directory...')
        replace_string_in_filename(os.path.join(input_directory, original_rec_dir), original_rec_dir, new_name + '_rec')

    # Rename files in original dir
    print('Renaming files in input directory...')
    filenames = [os.path.join(input_directory, filename)
                 for filename in os.listdir(input_directory)
                 if os.path.isfile(os.path.join(input_directory, filename))]
    for filename in filenames:
        replace_string_in_filename(filename, original_prefix, new_name + '_')

    # Rename original dir
    print('Renaming input directory...')
    replace_string_in_filename(os.path.normpath(input_directory), original_dir_name, new_name)


if __name__ == '__main__':
    main()
