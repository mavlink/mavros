#!/bin/sh

set -x

export PYTHONPATH=.

fgrep '[[[cog' --exclude 'cogall.sh' --exclude '*.md' --exclude '*.swp' -lr | python3 -m cogapp -cr @-
