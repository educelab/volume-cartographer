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
import urllib

from distutils.version import StrictVersion
from os.path import join

# The version we care about
CF_VERSION = '3.8.0'
CF_VERSION_SHORT = '3.8'

# Name of clang-format as a binary
CF_PROGNAME = 'clang-format'

# URL location of the 'cached' copy of clang-format to download
# for users which do not have clang-format installed
CF_HTTP_LINUX_CACHE = 'http://llvm.org/releases/3.9.0/clang+llvm-3.9.0-x86_64-linux-gnu-ubuntu-16.04.tar.xz'

CF_HTTP_DARWIN_CACHE = 'http://llvm.org/releases/3.9.0/clang+llvm-3.9.0-x86_64-apple-darwin.tar.xz'

# Path in the tarball to the clang-format binary
CF_SOURCE_TAR_BASE = string.Template(
    join('clang+llvm-$version-$tar_path', 'bin', CF_PROGNAME)
)

# Path to extract clang-format to in source tree
BASE_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
CF_EXTRACT_DIR = join(BASE_DIR, 'bin')


def fetch_clang_format(url):
    dest = tempfile.gettempdir()
    tmp_tar = join(dest, 'temp.tar.xz')
    logging.info(
        'Downloading clang-format {} from {}, saving to {}'.
        format(CF_VERSION, url, tmp_tar)
    )
    urllib.request.urlretrieve(url, tmp_tar)
    return tmp_tar


def extract_clang_format(tarpath, final_cf):
    # Make the cf dir if it doesn't already exist
    if not os.path.isdir(CF_EXTRACT_DIR):
        os.mkdir(CF_EXTRACT_DIR)

    # Extract from tarfile to the final dir
    with tarfile.open(tarpath, 'r:xz') as t:
        logging.info('Extracting {} from {}'.format(CF_PROGNAME, tarpath))
        path_in_tar = list(
            filter(lambda n: n.endswith('/clang-format'), t.getnames())
        )[0]
        fullpath = join(CF_EXTRACT_DIR, CF_PROGNAME)
        extract_to = tempfile.gettempdir()
        t.extract(path_in_tar, path=extract_to)

        # Move to final location
        shutil.move(join(extract_to, path_in_tar), fullpath)
        logging.info('Extracted {} to {}'.format(path_in_tar, fullpath))

        return fullpath


def callo(cmd):
    if isinstance(cmd, str):
        cmd = cmd.split()
    return subprocess.check_output(cmd).decode('utf-8').strip()


# Get all changed files (even non-source ones)
def changed_files():
    current_branch = callo('git rev-parse --abbrev-ref @')
    develop = 'origin/develop'
    branch_point = callo('git merge-base {} {}'.format(develop, current_branch))
    diffcmd = 'git diff --name-only {}..{}'.format(branch_point, current_branch)
    return callo(diffcmd).splitlines()


# Find clang-format either in passed-in path or system path
def find_clang_format(argpath):
    if argpath:
        return argpath

    # Check for the previously-extracted clang-format and make sure it's
    # executable
    extracted = join(CF_EXTRACT_DIR, CF_PROGNAME)
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
            tarpath = fetch_clang_format(CF_HTTP_LINUX_CACHE)
        elif platform.system() == 'Darwin':
            tarpath = fetch_clang_format(CF_HTTP_DARWIN_CACHE)
        else:
            logging.error('Platorm {} not supported'.format(platform.system()))
            sys.exit(1)

        # Extract to correct directory
        return extract_clang_format(tarpath, CF_EXTRACT_DIR)


def cf_version(path):
    return callo(' '.join([path, '--version'])).split()[2]


# Lint a given file
def lint_file(cf, source_file, show_diff):
    # Read original text
    with open(source_file, 'r') as original_file:
        original_text = original_file.read()

    # clang-format file
    cmd = ' '.join([cf, '--style=file', source_file])
    formatted_text = callo(cmd)

    if formatted_text != original_text:
        original_lines = original_text.splitlines()
        formatted_lines = formatted_text.splitlines()
        result = difflib.unified_diff(original_lines, formatted_lines)

        print('Found formatting changes for file: ' + source_file)
        if show_diff:
            print('To fix, run "{} --style=file -i {}"'.format(cf, source_file))
            print('Suggested changes:')
            for line in result:
                print(line.strip())

    return formatted_text == original_text


if __name__ == '__main__':
    # Set up some logging
    logging.basicConfig(
        stream=sys.stdout,
        level=logging.DEBUG,
        format='%(asctime)s %(name)s %(levelname)s %(message)s',
        datefmt='%m-%d %H:%M',
    )

    parser = argparse.ArgumentParser('clang-format')
    parser.add_argument(
        '-c',
        '--clang-format-path',
        help='path to clang-format',
        metavar='PATH',
        dest='path',
    )
    parser.add_argument(
        '--no-diff',
        help='hide file diff generated by clang-format',
        default=False,
        action='store_true',
    )
    args = parser.parse_args()

    # Find clang-format, validate version
    cf = find_clang_format(args.path)
    if StrictVersion(cf_version(cf)) < StrictVersion(CF_VERSION):
        logging.error(
            'Incorrect version of clang-format: got {} but at least {} is required'.
            format(cf_version(cf), CF_VERSION)
        )
        sys.exit(1)

    # Then filter by extension
    changes = changed_files()
    if not changes:
        logging.info('No changed files, exiting')
        sys.exit(0)
    ext_re = re.compile(r'\.(h|hpp|c|cpp)$')
    cxx_changes = filter(lambda f: re.search(ext_re, f), changes)

    # Validate each with clang-format
    clean = True
    for f in cxx_changes:
        clean = clean and lint_file(cf, f, not args.no_diff)

    sys.exit(0) if clean else sys.exit(1)
