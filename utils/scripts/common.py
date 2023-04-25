import logging
import os
import pprint
import shutil
import shlex
import subprocess
import sys
import tarfile
import tempfile
import urllib.request
from distutils.version import LooseVersion
from typing import Any, Dict, Iterator, List, Pattern, Union, Generator

# URL location of the 'cached' copy of clang-format to download
# for users which do not have clang-format installed
HTTP_LINUX_URL = 'http://llvm.org/releases/3.9.0/clang+llvm-3.9.0-x86_64-linux-gnu-ubuntu-16.04.tar.xz'
HTTP_DARWIN_URL = 'http://llvm.org/releases/3.9.0/clang+llvm-3.9.0-x86_64-apple-darwin.tar.xz'

# Path to extract clang-format to in source tree
BASE_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
EXTRACT_DIR = os.path.join(BASE_DIR, 'bin')

# Minimum required clang-tools version
MIN_VERSION_REQUIRED = LooseVersion('3.8.0')


class MissingBinaryException(Exception):
    '''
    Exception specifying that a binary could not be found.
    '''
    pass


def callo(cmd: Union[List[str], str], **kwargs: Any) -> str:
    '''
    Shell-out, returns stdout
    '''
    cmd = shlex.split(cmd) if isinstance(cmd, str) else cmd
    logging.debug(f'Running: {" ".join(cmd)}')
    return subprocess.run(cmd, **kwargs, stdout=subprocess.PIPE).stdout.decode('utf-8').strip()


def changed_files(compare_to: str) -> Generator[str, None, None]:
    '''
    Determines all changed files from HEAD to branch `compare_to`. Excludes files
    that were deleted for renamed.
    '''
    current_branch = callo('git rev-parse --abbrev-ref @')
    diffcmd = f'git diff --name-status --diff-filter=dr \
        {compare_to}...{current_branch}'

    # Return source files, only if they haven't been deleted. This prevents
    # trying to operate on a missing file.
    for line in callo(diffcmd).splitlines():
        status, *rest = line.split()

        # Renames look like:
        #     R000   <oldfile>   <newfile>
        if status.startswith('R'):
            yield rest[1]
        else:
            yield rest[0]


def all_files() -> Generator[str, None, None]:
    '''
    Lists all project files under version control.
    '''
    for f in callo('git ls-files').split():
        yield f


def fetch_clang_binary(
    url: str, binary: str,
    extract_dir: str=EXTRACT_DIR,
) -> str:
    '''
    Extract `binary` from `url` to `extract_dir`.
    '''
    dest = tempfile.gettempdir()
    tmptar = os.path.join(dest, 'temp.tar.xz')
    logging.info(f'Downloading {binary} from {url}, saving to {tmptar}')
    urllib.request.urlretrieve(url, tmptar)

    if not os.path.isdir(extract_dir):
        os.mkdir(extract_dir)

    with tarfile.open(tmptar, 'r:xz') as tar:
        logging.info(f'Extracting {binary} from {tmptar}')

        # Find clang-format inside tar, extract to tmp directory
        path_in_tar = next(
            filter(lambda n: n.endswith(f'/{binary}'), tar.getnames())
        )
        tar_extract_path = tempfile.gettempdir()
        tar.extract(path_in_tar, path=tar_extract_path)

        # Move to final location
        final_location = os.path.join(extract_dir, binary)
        shutil.move(os.path.join(tar_extract_path, path_in_tar), final_location)
        logging.info(f'Extracted {path_in_tar} to {final_location}')

    return final_location


def executable(path: str) -> bool:
    '''
    Check if the given `path` is executable
    '''
    if not os.path.isfile(path):
        return False
    try:
        callo(f'{path} --help')
    except subprocess.CalledProcessError:
        return False
    return True


def find_binary(binary: str, init_path: str=None) -> Union[str, None]:
    '''
    Search for `binary` in a few locations:
    1) `init_path`
    2) Previously-extracted path
    3) System path (e.g. $(which binary))
    4) If nothing else, download from pre-defined url
    '''
    if init_path:
        return init_path

    # Check system PATH
    try:
        path = callo(f'which {binary}')
        if len(path) == 0:
            return None
        else:
            return path
    except subprocess.CalledProcessError:
        msg = f'Could not find suitable {binary} in path.'
        logging.error(msg)
        raise MissingBinaryException(msg)
