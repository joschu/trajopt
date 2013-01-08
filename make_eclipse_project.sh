#!/bin/sh
set -e
cd ~/build
rm -rf trajopt-eclipse
mkdir trajopt-eclipse
cd trajopt-eclipse
cmake -G"Eclipse CDT4 - Unix Makefiles" ~/Proj/trajoptrave