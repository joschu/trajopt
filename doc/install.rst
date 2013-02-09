.. _install:


Installing trajopt
===================

This code has been tested on Linux and OSX. 

Dependencies:

- OpenRAVE
- CMake
- boost
- Eigen
- Gurobi (though in the future we plan to support other solvers and provide a custom solver)

Instructions:

- install CMake, boost, and Eigen using your package manager.
- download and install Gurobi from `<http://www.gurobi.com>`_. You'll need to make an account and obtain a license.
- set the environment variable GUROBI_HOME to point to the folder of your Gurobi directory that contains the folders :file:`include` and :file:`lib`. On my development machines, these are :file:`/home/username/Src/gurobi502/linux64` and :file:`/Library/gurobi501/mac64`.
- Follow the usual CMake procedure::

    mkdir build_trajopt
    cd build_trajopt
    cmake /path/to/trajopt
    make -j
  
The build type defaults to RelWithDebInfo.

You can check if the build worked by typing::

  make test
  
The output should look something like this::

  Test project /Users/joschu/build/trajopt-relwdeb
      Start 1: ipi-sco-unit
  1/4 Test #1: ipi-sco-unit .....................   Passed    0.07 sec
      Start 2: collision-checker-unit
  2/4 Test #2: collision-checker-unit ...........   Passed    0.23 sec
      Start 3: planning-unit
  3/4 Test #3: planning-unit ....................   Passed    1.42 sec
      Start 4: cast-cost-unit
  4/4 Test #4: cast-cost-unit ...................   Passed    0.07 sec

  100% tests passed, 0 tests failed out of 4

  Total Test time (real) =   1.80 sec

If one of the unit tests fails, you can get more diagnostic information by running the executable separately, e.g.::

  bin/ipi-sco-unit


You need to set your PYTHONPATH to call trajopt from python.  
Add the following two paths to your PYTHONPATH::

  /path/to/trajopt                # source directory, so "import trajoptpy" works
  /path/to/build_trajopt/lib      # here's where the extension modules (.so files) go

Now you should be able to run the scripts in the python_examples directory.
