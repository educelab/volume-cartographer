#!/usr/bin/env python3

import argparse
import logging
import multiprocessing as mp
import os
import re
import subprocess
import sys
from distutils.version import LooseVersion
from fnmatch import fnmatch

import common

program_name = 'clang-tidy'
output_queue = mp.Queue()
nprocs = 4


class ClangTidier:
    # Blacklist certain files - tests, examples, etc. List entries should be in
    # the style of unix globs, e.g. *.cpp matches all files in the current dir
    # ending in .cpp
    BLACKLIST = [
        'apps/*',
        '*/test/*',
        'examples/*',
        'external/*',
    ]

    def __init__(self, path: str, build_dir: str) -> None:
        self.path = path
        self.toplevel = common.callo('git rev-parse --show-toplevel')
        self.build_dir = build_dir

    def __str__(self):
        return self.path

    @property
    def version(self) -> LooseVersion:
        lines = common.callo([self.path, '--version']).split('\n')
        return LooseVersion(lines[1].split()[2])

    def lint(self, source_file: str, print_output: bool=False) -> bool:
        '''
        Lints a given C++ `source_file` (as in ending in .cpp) with clang-tidy.
        '''
        # Check if source_file is blacklisted from checks
        if self.blacklisted_file(source_file):
            logging.debug(f'Skipping {source_file}, blacklisted')
            return True

        compile_commands_dir = os.path.join(self.toplevel, self.build_dir)
        cmd = [
            self.path,
            f'-p={compile_commands_dir}',
            '-config=',
            source_file,
        ]
        logging.debug(f'cmd: {" ".join(cmd)}')
        tidy_out = common.callo(cmd, stderr=subprocess.DEVNULL)

        # Only print if requested and if there's output
        if print_output and tidy_out:
            output_queue.put(tidy_out)

        return not tidy_out

    def blacklisted_file(self, source_file: str) -> bool:
        '''
        Checks whether `source_file` is blacklisted from clang-tidy checks.
        '''
        return any(fnmatch(source_file, pattern) for pattern in self.BLACKLIST)


def parse_arguments() -> argparse.Namespace:
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
    parser.add_argument(
        '-v',
        '--verbose',
        default=False,
        action='store_true',
        help='Print debug log statements',
    )
    return parser.parse_args()


def main() -> bool:
    args = parse_arguments()

    # Set up some logging
    level = logging.INFO if not args.verbose else logging.DEBUG
    logging.basicConfig(
        stream=sys.stdout,
        level=level,
        format='%(asctime)s %(name)s %(levelname)s %(message)s',
        datefmt='%m-%d %H:%M',
    )

    # Find clang-tidy, validate version
    path_to_ct = common.find_binary(program_name, args.path)
    ct = ClangTidier(path_to_ct, args.build_dir)
    if ct.version < common.MIN_VERSION_REQUIRED:
        logging.error(
            f'''Incorrect version of {program_name}: got {cf.version} but at \
                least {common.MIN_VERSION_REQUIRED} is required'''
        )
        sys.exit(1)

    # clang-tidy operates only on c/cpp files (h/hpp files are linted
    # transitively). Only check c/cpp files.
    cpp_files = re.compile(r'\.(c|cpp|cc|cxx)$')
    changes = [f for f in common.changed_files() if re.search(cpp_files, f)]

    # Validate each with clang-tidy in parallel
    with mp.Pool(nprocs) as pool:
        tasks = [
            pool.apply_async(
                ct.lint,
                args=(f,),
                kwds=dict(print_output=args.print_output),
            ) for f in changes
        ]
        result = all(task.get() for task in tasks)

        # Print output
        while args.print_output and not output_queue.empty():
            print(output_queue.get())

        return result


# Note: exit with `not main()` so that 0 return code means everything's fine
if __name__ == '__main__':
    sys.exit(int(not main()))
