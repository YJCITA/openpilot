#!/bin/env sh
echo "[-] Updating submodules T=$SECONDS"
target_dir=/data/openpilot

cd ${target_dir}
git submodule update --init --recursive --force

echo "[-] Submodules updated T=$SECONDS"