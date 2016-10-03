#!/usr/bin/env bash

set -eu

if [[ $# < 2 ]]; then
    echo "Usage:"
    echo "    $0 OLD NEW [SEP]"
    exit 1
fi

old="$1"
new="$2"

# Set separator - default to :
if [[ $# > 2 ]]; then
    sep="$3"
else
    sep=":"
fi

# Get git root repo
git_root=$(git rev-parse --show-toplevel)

# Construct regex
sed_cmd="s${sep}${old}${sep}${new}${sep}g"

# Do replace
ag -sl "$old" "$git_root" | xargs sed -i '' -e "$sed_cmd"
exit 0
