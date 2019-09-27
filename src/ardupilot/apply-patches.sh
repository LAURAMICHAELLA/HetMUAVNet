#!/bin/bash
if [ "$1" != "safe12345678safe" ];
then
	echo "$0: This script is meant to be called by CMakeLists.txt, aborting" >&2
	exit 1
fi

# When we are called by CMakeLists.txt, we can safely assume that:
SOURCEDIR="$(dirname "$0")"
BUILDDIR="$PWD"

echo "Patching ArduPilot..."
set -ex

# Now replace some ArduPilot files with symbolic links to our version
for FILENAME in "$SOURCEDIR"/patches/*.patch;
do
	patch -p1 < "$FILENAME"
done
