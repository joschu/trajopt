#!/bin/sh
disabled=E1101,W0621
cd ..
pylint -f colorized --disable=C,R -r n -i y trajoptpy -d $disabled
cd python_examples
pylint -f colorized --disable=C,R -r n -i y *.py -d $disabled
