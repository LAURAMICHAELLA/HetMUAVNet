# Compilation
0) mercurial
1) ./setup-environment.sh
2) cd ns-3-allinone
3) ./build.py --enable-examples -- --build-profile=release

Example programs will be in:
 ns-3/ns-3-allinone/ns-3-dev/build/src/external-sync/examples

# MISC
cd ns-3-dev
./waf --run src/external-sync/examples/external-sync-example
