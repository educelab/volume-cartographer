import argparse
import subprocess
import sys
from pathlib import Path

_TEXT_CHARS = (
        b''.join(
            i.to_bytes(1, byteorder=sys.byteorder) for i in range(32, 127)) +
        b'\n\r\t\f\b')

_CODESIGN_DEFAULT_ARGS = ['codesign', '-s', '-', '--force',
                          '--preserve-metadata=entitlements,requirements,flags,runtime']


def is_text_file(p: Path, block_size=512):
    """ Uses heuristics to guess whether the given file is text or binary,
        by reading a single block of bytes from the file.
        If more than 30% of the chars in the block are non-text, or there
        are NUL ('\x00') bytes in the block, assume this is a binary file.
        From:
        https://eli.thegreenplace.net/2011/10/19/perls-guess-if-file-is-text-or-binary-implemented-in-python/
    """
    with p.open('rb') as f:
        block = f.read(block_size)
        if b'\x00' in block:
            # Files with null bytes are binary
            return False
        elif not block:
            # An empty file is considered a valid text file
            return True

        # Use translate's 'deletechars' argument to efficiently remove all
        # occurrences of _text_characters from the block
        not_text = block.translate(None, _TEXT_CHARS)
        return float(len(not_text)) / len(block) <= 0.30


def is_binary_file(p: Path, block_size=512):
    return not is_text_file(p, block_size)


def codesign_bin_dir(bin_dir: Path):
    bins = []
    for p in bin_dir.rglob('*'):
        if not p.is_file():
            continue

        if is_binary_file(p):
            bins.append(str(p))

    if len(bins) > 0:
        args = [a for a in _CODESIGN_DEFAULT_ARGS]
        args.extend(bins)
        subprocess.run(args, check=True, text=True, stdout=sys.stdout,
                       stderr=sys.stdout)
    return len(bins)


def codesign_lib_dir(lib_dir: Path):
    libs = []
    frameworks = []
    for p in lib_dir.rglob('*.dylib'):
        libs.append(str(p))

        # add frameworks to separate list
        for idx, parent in enumerate(reversed(p.parents)):
            if '.framework' in str(parent):
                frameworks.append(str(p))

    for p in lib_dir.rglob('*.framework/'):
        frameworks.append(str(p))

    # Add to args
    if len(libs) > 0 or len(frameworks) > 0:
        args = [a for a in _CODESIGN_DEFAULT_ARGS]
        args.extend(libs)
        args.extend(set(frameworks))
        subprocess.run(args, check=True, text=True, stdout=sys.stdout,
                       stderr=sys.stdout)
    return len(libs) + len(frameworks)


def codesign_app_package(app_dir: Path):
    # sign libraries
    codesign_lib_dir(app_dir)
    codesign_bin_dir(app_dir / 'Contents' / 'MacOS')
    args = [a for a in _CODESIGN_DEFAULT_ARGS] + [str(app_dir)]
    subprocess.run(args, check=True, text=True, stdout=sys.stdout,
                   stderr=sys.stdout)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--input', required=True,
                        help='Path to release directory')
    args = parser.parse_args()

    input_dir = Path(args.input)
    if (input_dir / 'bin').exists():
        print('-- signing bin directory --')
        n = codesign_bin_dir(input_dir / 'bin')
        if n == 0:
            print('none')
        print()

    if (input_dir / 'lib').exists():
        print('-- signing lib directory --')
        n = codesign_lib_dir(input_dir / 'lib')
        if n == 0:
            print('none')
        print()

    print('-- signing app packages --')
    apps = list(input_dir.glob('*.app/'))
    if len(apps) > 0:
        print(f'* found {len(apps)} app packages')
        for app in apps:
            print(f'* signing {app.name}')
            codesign_app_package(app)
            print()
    else:
        print('none')


if __name__ == '__main__':
    main()
