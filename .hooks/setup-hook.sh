#!/usr/bin/env bash
set -e

ROOT=$(git rev-parse --show-toplevel)

if [ ! -d $ROOT/.git/hooks/ ]; then
	mkdir -p $ROOT/.git/hooks/
	echo "Directory .git/hooks does not exist! Creating directory."
fi

if [ -f $ROOT/.git/hooks/pre-commit-clang-format ];then
    echo "File .git/hooks/pre-commit-clang-format already exists! Will not attempt to overwrite."
else
    ln -s $ROOT/.hooks/pre-commit-clang-format $ROOT/.git/hooks/pre-commit-clang-format
    chmod +x $ROOT/.git/hooks/pre-commit-clang-format
fi

if [ -f $ROOT/.git/hooks/pre-commit ];then
    echo "File .git/hooks/pre-commit already exists! Will not attempt to overwrite."
else
    ln -s $ROOT/.hooks/pre-commit $ROOT/.git/hooks/pre-commit
    chmod +x $ROOT/.git/hooks/pre-commit
fi
