#!/usr/bin/env python

import subprocess,os

assert os.getcwd().endswith("bigdata")
subprocess.check_call("rsync -azvu --delete ./ pabbeel@rll.berkeley.edu:/var/www/trajopt/bigdata/", shell=True)
