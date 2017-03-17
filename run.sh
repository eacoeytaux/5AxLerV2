#!/usr/bin/env python

import subprocess, shutil, os

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--coverage", help="open html coverage report", action='store_true')
args = parser.parse_args()

#set up
os.chdir("./build/CMakeFiles/_CuraEngine.dir/src/5axis/")
subprocess.call("rm *.gcda", shell=True)
subprocess.call("rm *.gcov", shell=True)

os.chdir("../../../../..")

#run tests
subprocess.call('./build/CuraEngine slice -v -j ./5axistests/fdmprinter.def.json -o "test.gcode" -e0 -l "./5axistests/simple.STL"', shell=True)
subprocess.call('./build/CuraEngine slice -v -j ./5axistests/fdmprinter.def.json -o "test.gcode" -e0 -l "./5axistests/radial_overhang.STL"', shell=True)
subprocess.call('./build/CuraEngine slice -v -j ./5axistests/fdmprinter.def.json -o "test.gcode" -e0 -l "./5axistests/double_overhang.STL"', shell=True)

if args.coverage:
	#generate HMTL coverage file
	os.chdir("./build/CMakeFiles/_CuraEngine.dir/src/5axis")
	subprocess.call("lcov --directory . --capture -o cov.info", shell=True)
	subprocess.call("genhtml cov.info -o output", shell=True)
	subprocess.call("open ./output/index.html", shell=True)

