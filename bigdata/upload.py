#!/usr/bin/env python

import subprocess,os
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--delete", action="store_true")
args = parser.parse_args()

assert os.getcwd().endswith("bigdata")

print "creating zip file"
import zipfile
with zipfile.ZipFile("all.zip","w") as myzip:
    for fname in os.listdir("."):
        if not (fname.endswith("py") or fname.endswith("zip")):
            myzip.write(fname)
print "uploading"            
subprocess.check_call("rsync -azvu %s ./ pabbeel@rll.berkeley.edu:/var/www/trajopt/bigdata/ --exclude '*.py'"%("--delete" if args.delete else ""), shell=True)
            