import logging
import os
import shutil
import subprocess
import sys
import re
import tarfile
import urllib.request

from typing import Iterator, List, Union

# URL location of the 'cached' copy of clang-format to download
# for users which do not have clang-format installed
HTTP_LINUX_URL = 'http://llvm.org/releases/3.9.0/clang+llvm-3.9.0-x86_64-linux-gnu-ubuntu-16.04.tar.xz'
HTTP_DARWIN_URL = 'http://llvm.org/releases/3.9.0/clang+llvm-3.9.0-x86_64-apple-darwin.tar.xz'

# Path to extract clang-format to in source tree
BASE_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
EXTRACT_DIR = os.path.join(BASE_DIR, 'bin')


def callo(cmd: Union[List[str], str]) -> str:
    '''
    shell-out, returns stdout
    '''
    if isinstance(cmd, str):
        cmd = cmd.split()
    return subprocess.check_output(cmd).decode('utf-8').strip()


def changed_source_files() -> Iterator[str]:
    '''
    Determines all changed files from prior commit to HEAD.
    '''
    current_branch = callo('git rev-parse --abbrev-ref @')
    develop = 'origin/develop'
    branch_point = callo('git merge-base {} {}'.format(develop, current_branch))
    diffcmd = 'git diff --name-only {}..{}'.format(branch_point, current_branch)
    
    # Filter based on extension - only C/C++ source/header files
    ext = re.compile(r'\.(h|hpp|c|cpp)$')
    return filter(lambda f: re.search(ext, f), callo(diffcmd).splitlines())


def fetch_clang_binary(
    url: str, binary: str, extract_dir: str=EXTRACT_DIR
) -> str:
    '''
    Extract `binary` from `url` to `extract_dir`.
    '''
    dest = tempfile.gettempdir()
    tmptar = os.path.join(dest, 'temp.tar.xz')
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
            filter(lambda n: n.endswith('/{}'.format(binary)), tar.getnames())
        )
        tar_extract_path = tempfile.gettempdir()
        tar.extract(path_in_tar, path=tar_extract_path)

        # Move to final location
        final_location = os.path.join(extractdir, PROGNAME)
        shutil.move(os.path.join(tar_extract_path, path_in_tar), final_location)
        logging.info('Extracted {} to {}'.format(path_in_tar, final_location))

    return final_location


def executable(path: str) -> bool:
    '''
    Check if the given `path` is executable
    '''
    if not os.path.isfile(path):
        return False
    try:
        callo('{} -h'.format(path))
    except subprocess.CalledProcessError:
        return False
    return True


def find_binary(binary: str, init_path: str=None) -> str:
    '''
    Search for `binary` in a few locations:
    1) `init_path`
    2) Previously-extracted path
    3) System path (e.g. $(which binary))
    4) If nothing else, download from pre-defined url
    '''
    if init_path:
        return init_path

    # Check for the previously-extracted clang-format and make sure it's
    # executable
    prev_extracted = os.path.join(EXTRACT_DIR, binary)
    if executable(prev_extracted):
        return prev_extracted

    # Check system PATH
    try:
        return callo('which {}'.format(binary))
    except subprocess.CalledProcessError:
        msg = '''Could not find suitable {} in path. Attempting to \
                download from LLVM release page'''.format(binary)
        logging.info(msg)

        # Fetch clang-format from LLVM servers
        if sys.platform == 'linux':
            return fetch_clang_binary(HTTP_LINUX_URL, binary)
        elif sys.platform == 'darwin':
            return fetch_clang_binary(HTTP_DARWIN_URL, binary)
        else:
            logging.error('Platorm {} not supported'.format(platform.system()))
            sys.exit(1)
