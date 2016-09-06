#!/usr/bin/env python3

import argparse
import difflib
import logging
import os
import platform
import re
import shutil
import string
import subprocess
import sys
import tarfile
import tempfile
import urllib.request

# The version we care about
CLANG_FORMAT_VERSION = '3.9.0'
CLANG_FORMAT_VERSION_SHORT = '3.9'

# Name of clang-format as a binary
CLANG_FORMAT_PROGNAME = 'clang-format'

# URL location of the 'cached' copy of clang-format to download
# for users which do not have clang-format installed
CLANG_FORMAT_HTTP_LINUX_CACHE = 'http://llvm.org/releases/3.9.0/clang+llvm-3.9.0-x86_64-linux-gnu-ubuntu-16.04.tar.xz'

CLANG_FORMAT_HTTP_DARWIN_CACHE = 'http://llvm.org/releases/3.9.0/clang+llvm-3.9.0-x86_64-apple-darwin.tar.xz'

# Path in the tarball to the clang-format binary
CLANG_FORMAT_SOURCE_TAR_BASE = string.Template(
    'clang+llvm-$version-$tar_path/bin/' + CLANG_FORMAT_PROGNAME
)

# Path to extract clang-format to in source tree
BASE_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
CLANG_FORMAT_EXTRACT_DIR = os.path.join(BASE_DIR, 'bin')


def fetch_clang_format(url):
    dest = tempfile.gettempdir()
    tmp_tar = os.path.join(dest, 'temp.tar.xz')
    logging.info(
        'Downloading clang-format {} from {}, saving to {}'.
        format(CLANG_FORMAT_VERSION, url, tmp_tar)
    )
    urllib.request.urlretrieve(url, tmp_tar)
    return tmp_tar


def extract_clang_format(tarpath, final_cf_path):
    # Make the cf dir if it doesn't already exist
    if not os.path.isdir(CLANG_FORMAT_EXTRACT_DIR):
        os.mkdir(CLANG_FORMAT_EXTRACT_DIR)

    # Extract from tarfile to the final dir
    with tarfile.open(tarpath, 'r:xz') as t:
        logging.info(
            'Extracting {} from {}'.
            format(CLANG_FORMAT_PROGNAME, tarpath)
        )
        path_in_tar = list(
            filter(lambda n: n.endswith('/clang-format'), t.getnames())
        )[0]
        fullpath = os.path.join(CLANG_FORMAT_EXTRACT_DIR, CLANG_FORMAT_PROGNAME)
        extract_to = os.path.join(tempfile.gettempdir())
        t.extract(path_in_tar, path=extract_to)

        # Move to final location
        shutil.move(os.path.join(tempfile.gettempdir(), path_in_tar), fullpath)
        logging.info('Extracted {} to {}'.format(path_in_tar, fullpath))

        return fullpath


def callo(cmd):
    if isinstance(cmd, str):
        cmd = cmd.split()
    return subprocess.check_output(cmd).decode('utf-8').strip()


# Only return rc, not output
def call(cmd):
    if isinstance(cmd, list):
        cmd = ' '.join(cmd)
    return subprocess.getstatusoutput(cmd)


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

    # Check for the previously-extracted clang-format and make sure it's
    # executable
    extracted = os.path.join(CLANG_FORMAT_EXTRACT_DIR, CLANG_FORMAT_PROGNAME)
    if os.path.isfile(extracted) and callo(' '.join([extracted, '-h'])):
        return extracted

    # Check system $PATH
    try:
        return callo('which clang-format')
    except subprocess.CalledProcessError:
        msg = '''
            Could not find suitable clang-format in path. Attempting to download
            from LLVM release page
        '''
        logging.info(msg)

        # Fetch clang-format from LLVM servers
        tarpath = ''
        if platform.system() == 'Linux':
            tarpath = fetch_clang_format(CLANG_FORMAT_HTTP_LINUX_CACHE)
        elif platform.system() == 'Darwin':
            tarpath = fetch_clang_format(CLANG_FORMAT_HTTP_DARWIN_CACHE)
        else:
            logging.error('Platorm {} not supported'.format(platform.system()))
            sys.exit(1)

        # Extract to correct directory
        return extract_clang_format(tarpath, CLANG_FORMAT_EXTRACT_DIR)


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
        print('Suggested changes:\n')
        for line in result:
            print(line.strip())

    return formatted_text == original_text


if __name__ == '__main__':
    # Set up some logging
    logging.basicConfig(
        stream=sys.stdout,
        level=logging.DEBUG,
        format='%(asctime)s %(name)s %(levelname)s %(message)s',
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
    clean = all(lint_file(cf, f) for f in files)
    sys.exit(0) if clean else sys.exit(1)
