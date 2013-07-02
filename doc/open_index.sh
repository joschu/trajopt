#!/bin/sh

index=sphinx_build/html/index.html
if [ `uname` = Linux ] 
then
    google-chrome $index
else 
    open -a Google\ Chrome  $index
fi