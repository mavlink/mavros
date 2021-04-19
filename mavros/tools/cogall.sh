#!/bin/sh

set -x

fgrep '[[[cog' --exclude 'cogall.sh' --exclude '*.md' --exclude '*.swp' -lr | python3 -m cogapp -cr @-
