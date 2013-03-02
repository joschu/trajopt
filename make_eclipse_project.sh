#!/bin/sh
### Set BUILD_DIR and SOURCE_DIR appropriately
BUILD_DIR=~/build
SOURCE_DIR=~/Proj/trajopt
set -e
cd $BUILD_DIR
rm -rf trajopt-eclipse
mkdir trajopt-eclipse
cd trajopt-eclipse
cmake -G"Eclipse CDT4 - Unix Makefiles"  -DBUILD_HUMANOIDS=ON -DBUILD_CLOUDPROC=OFF -DBUILD_SANDBOX=ON $SOURCE_DIR