.. _install:


Installing trajopt
===================

This code has been tested on Linux and OSX. 

Dependencies
------------

- OpenRAVE
- OpenSceneGraph
- CMake
- boost
- Eigen
- Gurobi (though in the future we plan to support other solvers and provide a custom solver)

Instructions
-------------

- install OpenRAVE 0.9 or above (currently, that means getting the latest sources) using `instructions on openrave.org <http://openrave.org/docs/latest_stable>`_

- install OpenSceneGraph, CMake, boost, and Eigen using your package manager. In Ubuntu, that's::

    sudo apt-get install libopenscene-graph cmake libboost-all-dev libeigen3-dev

- download and install Gurobi from `<http://www.gurobi.com>`_. You'll need to make an account and obtain a license.
- set the environment variable GUROBI_HOME to point to the folder of your Gurobi directory that contains the folders :file:`include` and :file:`lib`. On my development machines, these are :file:`/home/username/Src/gurobi502/linux64` and :file:`/Library/gurobi501/mac64`.
- Follow the usual CMake procedure::

    mkdir build_trajopt
    cd build_trajopt
    cmake /path/to/trajopt
    make -j
  
You need to set your ``PYTHONPATH`` to call trajopt from python.  
Add the following two paths to your ``PYTHONPATH``::

  /path/to/trajopt                # source directory, so "import trajoptpy" works
  /path/to/build_trajopt/lib      # here's where the extension modules (.so files) go

Now you should be able to run the scripts in the python_examples directory.


You can check if the build worked by typing

::

  ctest
  
in the build directory. The output should look something like this::

  Running tests...
  Test project /Users/joschu/build/trajopt-relwdeb
      Start 1: arm_to_cart_target.py
  1/7 Test #1: arm_to_cart_target.py ............   Passed    2.03 sec
      Start 2: arm_to_joint_target.py
  2/7 Test #2: arm_to_joint_target.py ...........   Passed    1.92 sec
      Start 3: position_base.py
  3/7 Test #3: position_base.py .................   Passed    2.59 sec
      Start 4: sco-unit
  4/7 Test #4: sco-unit .....................   Passed    0.04 sec
      Start 5: collision-checker-unit
  5/7 Test #5: collision-checker-unit ...........   Passed    0.05 sec
      Start 6: planning-unit
  6/7 Test #6: planning-unit ....................   Passed    1.40 sec
      Start 7: cast-cost-unit
  7/7 Test #7: cast-cost-unit ...................   Passed    0.06 sec

  100% tests passed, 0 tests failed out of 7

  Total Test time (real) =   8.09 sec

If one of the unit tests fails, you can get more diagnostic information by running with ``ctest -V``, or running the test scripts individually. The python executables are in ``SOURCE_DIR/python_examples`` and the compiled c++ executables are in ``BUILD_DIR/bin``. 


Common installation problems
-------------------------------

* *All tests fail, due to Gurobi license issue*. Make sure Gurobi license is set up properly.
* *You get an error from libbulletrave.so*, e.g.

  ::

    [plugindatabase.h:929] /usr/local/share/openrave-0.9/plugins/libbulletrave.so: /usr/local/share/openrave-0.9/plugins/libbulletrave.so: undefined symbol: _ZNK16btCollisionShape17getBoundingSphereER9btVector3Rf
    Segmentation fault (core dumped)

  One crude solution: ``rm /path/to/libbulletrave.so``. OpenRAVE uses ODE rather than Bullet by default, so there's no harm in removing the bullet plugin.

* *All the python tests fail*. You probably need to set your ``PYTHONPATH``

* *Almost all the tests fail, where OpenRAVE symbols aren't found*. Set ``LD_LIBRARY_PATH=/usr/local/lib`` or whereever libopenrave.so is. (Note: if you know how to fix this problem through RPATH settings or linker flags, please enlighten me.)