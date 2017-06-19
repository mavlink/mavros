#!/bin/sh
# Script to install the model datasets required
# to GeographicLib apply certain conversions

# check if geographiclib-tools is installed (for the Debian pkg only)
if [ $(dpkg-query -W -f='${Status}' geographiclib-tools 2>/dev/null | grep -c "ok installed") -eq 0 ]; then
  sudo apt-get --force-yes --yes install geographiclib-tools;
else
  echo "\ngeographiclib-tools install check: OK!\n"
fi

# Check if the geoid model dataset is already installed. If not, installs it
if [ ! -d "/usr/local/share/GeographicLib/geoids" ] && [ ! -d "/usr/share/GeographicLib/geoids" ]; then
  echo "\tGeoid Model dataset not installed. Installing..."
  sudo geographiclib-get-geoids egm96-5
else
  echo "\tThe Geoid Model dataset is already installed!"
fi

# Check if the gravity model dataset is already installed. If not, installs it
if [ ! -d "/usr/local/share/GeographicLib/gravity" ] && [ ! -d "/usr/share/GeographicLib/gravity" ]; then
  echo "\tGravity Model dataset not installed. Installing..."
  sudo geographiclib-get-gravity egm96
else
  echo "\tThe Gravity Model is already installed!"
fi

# Check if the magnetic model dataset is already installed. If not, installs it
if [ ! -d "/usr/local/share/GeographicLib/magnetic" ] && [ ! -d "/usr/share/GeographicLib/magnetic" ]; then
  echo "\tMagnetic Model dataset not installed. Installing..."
  sudo geographiclib-get-magnetic emm2015
else
  echo "\tThe Magnetic Model is already installed!\n"
fi
