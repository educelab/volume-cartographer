#!/usr/bin/env python3

import argparse
import difflib
import logging
import re
import subprocess
import sys

CLANG_FORMAT_VERSION = '3.9.0'
CLANG_FORMAT_VERSION_SHORT = '3.9'


def callo(cmd):
    if isinstance(cmd, list):
        cmd = ' '.join(cmd)
    return subprocess.getoutput(cmd).strip()


# Only return rc, not output
def call(cmd):
    if isinstance(cmd, str):
        cmd = cmd.split()
    return subprocess.call(cmd)


# Get all changed files (even non-source ones)
def changed_files():
    files = callo('git diff --name-only @~..@').splitlines()
    if len(files) == 0:
        logging.info('No changed files')
        sys.exit(0)

    return files


# Find clang-format either in passed-in path or system path
def find_clang_format(argpath):
    if argpath:
        return argpath

    # Check system $PATH
    try:
        path = callo('which clang-format')
    except subprocess.CalledProcessError:
        msg = '''
            Could not find suitable clang-format in path. Rerun script with
            '-c/--clang-format-path' pointing to a valid clang-format
        '''
        logger.error(msg)
        sys.exit(1)

    # Validate clang-format version
    return path


def cf_version(path):
    return callo(' '.join([path, '--version']))


def cf_version_is_correct(path):
    return CLANG_FORMAT_VERSION in cf_version(path)


# Lint a given file
def lint_file(cf_path, file_path):

    # Read original text
    with open(file_path, 'r') as original_file:
        original_text = original_file.read()

    # clang-format file
    cmd = ' '.join([cf_path, '--style=file', file_path])
    formatted_text = callo(cmd)

    if formatted_text != original_text:
        original_lines = original_text.splitlines()
        formatted_lines = formatted_text.splitlines()
        result = difflib.unified_diff(original_lines, formatted_lines)

        print('Found diff for file: ' + file_path)
        print('To fix, run "{} --style=file -i {}"'.format(cf_path, file_path))
        for line in result:
            print(line.strip())
        sys.exit(1)


if __name__ == '__main__':
    # Set up some logging
    logging.basicConfig(
        stream=sys.stdout,
        level=logging.DEBUG,
        format='%(asctime)s %(name)-12s %(levelname)-8s %(message)s',
        datefmt='%m-%d %H:%M'
    )

    parser = argparse.ArgumentParser('clang-format')
    parser.add_argument(
        '-c',
        '--clang-format-path',
        help='path to clang-format',
        metavar='PATH',
        dest='path',
        type=str
    )
    args = parser.parse_args()

    # Find clang-format, validate version
    cf = find_clang_format(args.path)
    if not cf_version_is_correct(cf):
        logging.error(
            'Incorrect version of clang-format: got {} but {} is required'.
            format(cf_version(cf), CLANG_FORMAT_VERSION)
        )
        sys.exit(1)

    # Get list of files to check
    files = changed_files()

    # Then filter by extension
    ext_re = re.compile(r'\.(h|hpp|c|cpp)$')
    files = list(filter(lambda f: re.search(ext_re, f), files))

    # Validate each with clang-format
    for f in files:
        lint_file(cf, f)
