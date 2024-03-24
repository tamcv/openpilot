#!/usr/bin/env bash

set -Eeuo pipefail

script_name=$(basename "${BASH_SOURCE[0]}")

if [ $# -eq 0 ]; then
    echo "No argument supplied, must provide a branch"
    echo "$script_name <branch>"
    exit 1
fi

cd /data; rm -rf ./openpilot
git clone -b "$1" --recurse-submodules https://github.com/tamcv/openpilot.git
cd openpilot
sudo reboot
