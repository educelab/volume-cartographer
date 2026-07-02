#!/usr/bin/env bash
# Installs and pins the fmt version known to work with spdlog, working
# around the incompatibility introduced in fmt 12.2.0. Run this before
# `brew bundle` (in CI or locally). See educelab/volume-cartographer#147.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
tap_name="educelab/volume-cartographer-pins"

if ! brew tap-info "$tap_name" &>/dev/null; then
  # `brew tap` clones its source with git, so stage the formula in a throwaway
  # git repo rather than nesting a real .git inside this repo's worktree.
  tap_src="$(mktemp -d)"
  trap 'rm -rf "$tap_src"' EXIT
  cp -R "$repo_root/ci/homebrew-pins/." "$tap_src/"
  git -C "$tap_src" init -q
  git -C "$tap_src" -c user.name=brew-pin-fmt -c user.email=brew-pin-fmt@localhost \
    add -A
  git -C "$tap_src" -c user.name=brew-pin-fmt -c user.email=brew-pin-fmt@localhost \
    commit -q -m "Pinned fmt formula"
  brew tap "$tap_name" "$tap_src"
fi

current="$(brew list --formula --full-name | grep -x -e fmt -e "$tap_name/fmt" || true)"
if [ "$current" = "fmt" ]; then
  # Installed from homebrew/core (or another tap) under the same name;
  # swap it for our pinned formula.
  brew unpin fmt 2>/dev/null || true
  brew uninstall --ignore-dependencies fmt
fi

brew install "$tap_name/fmt"
brew pin fmt
