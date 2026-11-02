#!/usr/bin/env zsh

# source setup.sh from same directory as this file
_MOTION_SETUP_DIR=$(builtin cd -q "`dirname "$0"`" > /dev/null && pwd)
emulate -R zsh -c 'source "$_MOTION_SETUP_DIR/setup.sh"'