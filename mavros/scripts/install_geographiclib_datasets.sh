#!/bin/sh
# Script to install the model datasets required
# to GeographicLib apply certain conversions

set -x

if [[ $UID != 0 ]]; then
	echo "This script require root privilegies!" 1>&2
	exit 1
fi

# Get OS name and release
if [ -f /etc/os-release ]; then
    # freedesktop.org and systemd
    . /etc/os-release
    OS=$NAME
    VER=$VERSION_ID
elif type lsb_release >/dev/null 2>&1; then
    # linuxbase.org
    OS=$(lsb_release -si)
    VER=$(lsb_release -sr)
elif [ -f /etc/lsb-release ]; then
    # For some versions of Debian/Ubuntu without lsb_release command
    . /etc/lsb-release
    OS=$DISTRIB_ID
    VER=$DISTRIB_RELEASE
elif [ -f /etc/debian_version ]; then
    # Older Debian/Ubuntu/etc.
    OS=Debian
    VER=$(cat /etc/debian_version)
else
    # Fall back to uname, e.g. "Linux <version>", also works for BSD, etc.
    OS=$(uname -s)
    VER=$(uname -r)
fi

VER_UTOPIC=14.10
VER_WHEEZZY=7
VER_FC22=22

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

if [ $OS = Ubuntu ]; then
	apt install -y geographiclib-tools;

	if test $(version $VER) -gt $(version $VER_UTOPIC) ; then # check if Ubuntu version is greater than 14.10
		run_get geoids geoids egm96-5
		run_get gravity gravity egm96
		run_get magnetic magnetic emm2015
	else
		geographiclib-datasets-download egm96_5;
	fi
elif [ $OS = Debian ]; then
	apt install -y geographiclib-tools;

	if test $(version $VER) -gt $(version $VER_WHEEZZY) ; then # check if Debian version is greater than 7
		run_get geoids geoids egm96-5
		run_get gravity gravity egm96
		run_get magnetic magnetic emm2015
	else
		geographiclib-datasets-download egm96_5;
	fi
elif [ $OS = Fedora ]; then
	if test $(version $VER) -gt $(version $VER_FC22) ; then # check if Fedora version is greater than 22
		yum install -y GeographicLib;

		run_get geoids geoids egm96-5
		run_get gravity gravity egm96
		run_get magnetic magnetic emm2015
	else
		echo "Fedora version not supported!" 1>&2
	fi
else
	echo "OS not supported! Check GeographicLib page for supported OS and lib versions." 1>&2
fi
