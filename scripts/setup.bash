#!/usr/bin/env bash

# 获取当前被 source 或执行的脚本所在目录
_SETUP_DIR="$(cd -P "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"

if [[ -f "$_SETUP_DIR/setup.sh" ]]; then
    source "$_SETUP_DIR/setup.sh"
else
    echo "Error: $_SETUP_DIR/setup.sh not found!" >&2
    return 1
fi