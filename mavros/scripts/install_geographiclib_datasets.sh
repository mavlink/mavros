#!/bin/sh
# Script to install the model datasets required
# to GeographicLib apply certain conversions

set -x

if [[ $UID != 0 ]]; then
	echo "This script require root privilegies!" 1>&2
	exit 1
fi

# Install datasets
run_get() {
	local dir="$1"
	local tool="$2"
	local model="$3"

	if [[ -d "/usr/share/GeographicLib/$dir" or -d "/usr/local/share/GeographicLib/$dir" ]]; then
		return
	fi

	geographiclib-get-$tool $model
}

# check which command script is available
if which /usr/sbin/geographiclib-get-geoids; then
	run_get geoids geoids egm96-5
	run_get gravity gravity egm96
	run_get magnetic magnetic emm2015
elif which geographiclib-datasets-download; then # only allows install the goid model dataset
	geographiclib-datasets-download egm96_5;
else
	echo "OS not supported! Check GeographicLib page for supported OS and lib versions." 1>&2
fi
