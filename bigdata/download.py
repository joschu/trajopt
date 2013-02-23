#!/usr/bin/env python
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--rsync",action="store_true")
args = parser.parse_args()

import os, urllib2, zipfile, subprocess
assert os.getcwd().endswith("bigdata")
if args.rsync: 
    subprocess.check_call("rsync -azvu pabbeel@rll.berkeley.edu:/var/www/trajopt/bigdata/ ./ --exclude '*.py'", shell=True)

else:
    urlinfo = urllib2.urlopen("http://rll.berkeley.edu/trajopt/bigdata/all.zip")
    with open("all.zip","w") as fh:
        fh.write(urlinfo.read())
    with zipfile.ZipFile("all.zip","r") as myzip:
        myzip.extractall(".")