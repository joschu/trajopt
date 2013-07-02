#!/bin/sh

curdir=`pwd`
cd ~/build/trajopt
ctest > $curdir/cmd_output/ctest.txt

