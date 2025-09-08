#!/bin/sh

set -x

BASEDIR=$(dirname "$0")

find -name '*.h' -o -name '*.cpp' -print -exec uncrustify -c "$BASEDIR/uncrustify-cpp.cfg" --replace --no-backup {} \;
