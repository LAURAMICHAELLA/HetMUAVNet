#!/bin/bash
cd "$(dirname "$0")"
THISDIR="$PWD"

mkdir tmp
( cd ../.. && git archive HEAD --prefix=gzuav-src/ -o "$THISDIR/tmp/gzuav-src.tar" )

docker build . -t gzuav/build-image

docker run --rm -v "$THISDIR/tmp":/gzuav_tmp gzuav/build-image bash -c '
	set -ex
	tar xvf /gzuav_tmp/gzuav-src.tar
	cd gzuav-src
	dpkg-buildpackage -b
	cp -v ../* --target-directory=/gzuav_tmp/
'
