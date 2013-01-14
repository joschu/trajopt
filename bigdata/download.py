#!/usr/bin/env python
import subprocess, os

assert os.getcwd().endswith("bigdata")

subprocess.check_call("wget -r -nH --cut-dirs=2 -l1 --no-parent -e robots=off -A.pcd http://rll.berkeley.edu/trajopt/bigdata",shell=True)
