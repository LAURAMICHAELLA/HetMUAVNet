#!/bin/bash
cd "$(dirname "$0")"
THISDIR="$PWD"

mkdir tmp
( cd ../.. && git archive HEAD --prefix=gzuav-src/ -o "$THISDIR/tmp/gzuav-src.tar" )

docker build . -f Dockerfile.nographics -t gzuav/nographics
docker build . -f Dockerfile.intel -t gzuav/intel
