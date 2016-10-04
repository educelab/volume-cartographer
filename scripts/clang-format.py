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

from distutils.version import StrictVersion
from os.path import join
from typing import List, Union

# The version we care about
MIN_VERSION_REQUIRED = StrictVersion('3.8.0')

# Name of clang-format as a binary
PROGNAME = 'clang-format'

# URL location of the 'cached' copy of clang-format to download
# for users which do not have clang-format installed
HTTP_LINUX_URL = 'http://llvm.org/releases/3.9.0/clang+llvm-3.9.0-x86_64-linux-gnu-ubuntu-16.04.tar.xz'
HTTP_DARWIN_URL = 'http://llvm.org/releases/3.9.0/clang+llvm-3.9.0-x86_64-apple-darwin.tar.xz'

# Path to extract clang-format to in source tree
BASE_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
EXTRACT_DIR = join(BASE_DIR, 'bin')


class ClangFormatter:
    def __init__(self, path_to_cf: str) -> None:
        self.exepath = path_to_cf

    def __str__(self):
        return self.exepath

    @property
    def version(self) -> StrictVersion:
        return StrictVersion(
            callo(' '.join([self.exepath, '--version'])).split()[2]
        )

    @property
    def executable(self) -> bool:
        if not os.path.isfile(self.exepath):
            return False
        try:
            callo('{} -h'.format(self.exepath))
        except subprocess.CalledProcessError:
            return False
        return True

    # Format a given source_file
    def format(self, source_file: str) -> str:
        cmd = ' '.join([self.exepath, '--style=file', source_file])
        return callo(cmd)

    # Lint a given file - return whether or not the file is formatted correctly
    def lint(self, source_file: str, show_diff: bool=True) -> bool:
        with open(source_file, 'r') as original_file:
            original_text = original_file.read()

        original_lines = original_text.splitlines()
        formatted_lines = self.format(source_file).splitlines()

        # Create some nice git-like metadata for the diff
        base_git_dir = callo('git rev-parse --show-toplevel')
        relpath = os.path.relpath(source_file, base_git_dir)
        fromfile = join('a', relpath)
        tofile = join('b', relpath)

        diffs = list(
            difflib.unified_diff(
                original_lines,
                formatted_lines,
                fromfile=fromfile,
                tofile=tofile
            )
        )

        if diffs:
            print('Found formatting changes for file:', source_file)

            if show_diff:
                print(
                    'To fix, run "{} --style=file -i {}"'.
                    format(cf, source_file)
                )
                print('Suggested changes:')
                for line in diffs:
                    print(line.strip())
                print()

        return not bool(diffs)


# Fetches clang-format from llvm site, extracts binary and copies to
# bin/clang-format
def fetch_clang_format(url: str, extractdir: str=EXTRACT_DIR) -> ClangFormatter:
    dest = tempfile.gettempdir()
    tmptar = join(dest, 'temp.tar.xz')
    logging.info(
        'Downloading clang-format {} from {}, saving to {}'.
        format(MIN_VERSION_REQUIRED, url, tmptar)
    )
    urllib.request.urlretrieve(url, tmptar)

    if not os.path.isdir(extractdir):
        os.mkdir(extractdir)

    with tarfile.open(tmptar, 'r:xz') as tar:
        logging.info('Extracting {} from {}'.format(PROGNAME, tmptar))

        # Find clang-format inside tar, extract to tmp directory
        path_in_tar = next(
            filter(lambda n: n.endswith('/clang-format'), tar.getnames())
        )
        tar_extract_path = tempfile.gettempdir()
        tar.extract(path_in_tar, path=tar_extract_path)

        # Move to final location
        final_location = join(extractdir, PROGNAME)
        shutil.move(join(tar_extract_path, path_in_tar), final_location)
        logging.info('Extracted {} to {}'.format(path_in_tar, final_location))

    return ClangFormatter(final_location)


# Shell-out
def callo(cmd: Union[List[str], str]) -> str:
    if isinstance(cmd, str):
        cmd = cmd.split()
    return subprocess.check_output(cmd).decode('utf-8').strip()


# Get all changed files (even non-source ones)
def changed_files() -> List[str]:
    current_branch = callo('git rev-parse --abbrev-ref @')
    develop = 'origin/develop'
    branch_point = callo('git merge-base {} {}'.format(develop, current_branch))
    diffcmd = 'git diff --name-only {}..{}'.format(branch_point, current_branch)
    return callo(diffcmd).splitlines()


# Find clang-format. Look in three places:
# * argument path (argument to script)
# * system path
# * if nothing else, download and copy to known dir
def find_clang_format(argpath: str) -> ClangFormatter:
    if argpath:
        return ClangFormatter(argpath)

    # Check for the previously-extracted clang-format and make sure it's
    # executable
    cf = ClangFormatter(join(EXTRACT_DIR, PROGNAME))
    if cf.executable:
        return cf

    # Check system $PATH
    try:
        return ClangFormatter(callo('which clang-format'))
    except subprocess.CalledProcessError:
        msg = '''Could not find suitable clang-format in path. Attempting to \
                download from LLVM release page'''
        logging.info(msg)

        # Fetch clang-format from LLVM servers
        if platform.system() == 'Linux':
            return fetch_clang_format(HTTP_LINUX_URL)
        elif platform.system() == 'Darwin':
            return fetch_clang_format(HTTP_DARWIN_URL)
        else:
            logging.error('Platorm {} not supported'.format(platform.system()))
            sys.exit(1)


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
    show_diff = not args.no_diff

    # Find clang-format, validate version
    cf = find_clang_format(args.path)
    if cf.version < MIN_VERSION_REQUIRED:
        logging.error(
            '''Incorrect version of clang-format: got {} but at least {} is \
                    required'''.format(cf.version, MIN_VERSION_REQUIRED)
        )
        sys.exit(1)

    # Find changed files from last common ancestor with develop
    changes = changed_files()
    if not changes:
        logging.info('No changed files, exiting')
        sys.exit(0)

    # Then filter by extension
    ext_re = re.compile(r'\.(h|hpp|c|cpp)$')
    cxx_changes = filter(lambda f: re.search(ext_re, f), changes)
    if not cxx_changes:
        logging.info('No changed CXX files, exiting')
        sys.exit(0)

    # Validate each with clang-format
    clean = all([cf.lint(f, show_diff=show_diff) for f in cxx_changes])

    sys.exit(0) if clean else sys.exit(1)
