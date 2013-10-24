#!/bin/bash
# useful script to test all the different build types that we support.
# This helps when doing large merges
# Andrew Tridgell, November 2011

. config.mk

set -e
set -x

echo "Testing ArduPlane build"
pushd ArduPlane
for b in all apm2 apm2beta apm1-hil apm1-hilsensors apm2-hil apm2-hilsensors sitl sitl-mount linux; do
    pwd
    make clean
    make $b -j4
done
popd

echo "Testing ArduCopter build"
pushd ArduCopter
for b in all apm2 apm1-hil apm2-hil sitl heli dmp linux; do
    pwd
    make clean
    make $b -j4
done
popd

echo "Testing APMRover build"
pushd APMrover2
for b in all apm2 sitl apm2-hil linux; do
    pwd
    make clean
    make $b -j4
done
popd

echo "Testing build of examples"

examples="Tools/VARTest Tools/CPUInfo"
for d in $examples; do
    pushd $d
    make clean
    make -j4
    popd
done

test -d ../libmaple && {
echo "Testing flymaple build"
for d in ArduPlane ArduCopter APMrover2; do
    pushd $d
    make clean
    make flymaple -j4
    popd
done
}

test -n "$PX4_ROOT" && test -d "$PX4_ROOT" && {
    ./Tools/scripts/build_all_px4.sh
}

exit 0
