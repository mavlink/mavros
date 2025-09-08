#!/bin/sh

set -x

grep -F '[[[cog' --exclude 'cogall.sh' --exclude '*.md' --exclude '*.swp' --exclude-dir '.venv' -lr | mr-cog -cr @-
