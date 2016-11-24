#!/usr/bin/env python3

import argparse
import logging
import sys
from distutils.version import LooseVersion

import common

MIN_VERSION_REQUIRED = LooseVersion('3.8.0')
PROGNAME = 'clang-tidy'


class ClangTidier:
    def __init__(self, path: str, build_dir: str) -> None:
        self.path = path
        self.build_dir = build_dir

    def __str__(self):
        return self.path

    @property
    def version(self) -> LooseVersion:
        lines = common.callo([self.path, '--version']).split('\n')
        return LooseVersion(lines[1].split()[2])

    def lint(self, source_file, print_output=False):
        '''
        Lints a given C++ `source_file` (as in ending in .cpp) with clang-tidy.
        '''
        base_dir = common.callo('git rev-parse --show-toplevel')
        compile_commands_dir = os.path.join(base_dir, self.build_dir)
        tidy_out = common.callo(
            [
                self.path,
                '-p={}'.format(compile_commands_dir),
                "-config=''",
                source_file,
            ]
        )

        if print_output:
            print(tidy_out)

        return not tidy_out


if __name__ == '__main__':
    # Set up some logging
    logging.basicConfig(
        stream=sys.stdout,
        level=logging.DEBUG,
        format='%(asctime)s %(name)s %(levelname)s %(message)s',
        datefmt='%m-%d %H:%M',
    )

    parser = argparse.ArgumentParser('clang-tidy')
    parser.add_argument(
        '-c',
        '--clang-tidy-path',
        help='path to clang-tidy',
        metavar='PATH',
        dest='path',
    )
    parser.add_argument(
        '--print-output',
        help='print output from clang-tidy',
        default=False,
        action='store_true',
    )
    parser.add_argument(
        '-b',
        '--build-dir',
        default='build',
        metavar='DIR',
        help='Build dir containing compile_commands.json',
    )
    args = parser.parse_args()

    # Find clang-tidy, validate version
    path_to_ct = common.find_binary(args.path, PROGNAME)
    ct = ClangTidier(path_to_ct, args.build_dir)
    if ct.version < MIN_VERSION_REQUIRED:
        logging.error(
            '''Incorrect version of {}: got {} but at least {} is required'''
            .format(PROGNAME, ct.version, MIN_VERSION_REQUIRED)
        )
        sys.exit(1)

    # clang-tidy operates only on c/cpp files (h/hpp files are linted
    # transitively). Only check c/cpp files.
    changes = common.changed_files(filter_regex=r'\.(c|cpp|cc)$')
    if not changes:
        logging.info('No changed files, exiting')
        sys.exit(0)

    # Validate each with clang-tidy
    clean = all([ct.lint(f, print_output=args.print_output) for f in changes])

    sys.exit(0) if clean else sys.exit(1)
