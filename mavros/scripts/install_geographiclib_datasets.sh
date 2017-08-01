#!/bin/sh
# Script to install the model datasets required
# to GeographicLib apply certain conversions

set -x

if [[ $UID != 0 ]]; then
	echo "That script require root privilegies!" 1>&2
	exit 1
fi

apt install -y geographiclib-tools;

run_get() {
	local dir="$1"
	local tool="$2"
	local model="$3"

	if [[ -d "/usr/share/GeographicLib/$dir" or -d "/usr/local/share/GeographicLib/$dir" ]]; then
		return
	fi

	geographiclib-get-$tool $model
}

run_get geoids geoids egm96-5
run_get gravity gravity egm96
run_get magnetic magnetic emm2015
