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
- Gurobi [#gurobi]_

.. [#gurobi] Good news! Trajopt now works with the excellent free solver BPMPD. It's not very well-tested at the moment, but you can use it now by checking out the devel branch and setting TRAJOPT_CONVEX_SOLVER=BPMPD.

Instructions
-------------

- install OpenRAVE 0.8 or above

.. note:: For best results, install a new version of OpenRAVE (version 0.9).  Due to a recent bugfix, optimization over affine DOFs won't work with 0.8. You can install from source or use the `openrave testing <https://launchpad.net/~openrave/+archive/testing>`_ PPA.


- install OpenSceneGraph, CMake, boost, and Eigen using your package manager. In Ubuntu, that's::

    sudo apt-get install libopenscene-graph cmake libboost-all-dev libeigen3-dev

- download and install Gurobi from `<http://www.gurobi.com>`_. You'll need to make an account and obtain a license. It's free for academic use, and non-academic users can get a free trial.
- set the environment variable GUROBI_HOME to point to the folder of your Gurobi directory that contains the folders :file:`include` and :file:`lib`. On my development machines, these are :file:`/home/username/Src/gurobi502/linux64` and :file:`/Library/gurobi501/mac64`.
- Clone trajopt from `github <https://github.com/joschu/trajopt>`_
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
  Test project /Users/joschu/build/trajopt-release
      Start 1: arm_to_cart_target.py
  1/6 Test #1: arm_to_cart_target.py ............   Passed    2.30 sec
      Start 2: arm_to_joint_target.py
  2/6 Test #2: arm_to_joint_target.py ...........   Passed    1.93 sec
      Start 3: sco-unit
  3/6 Test #3: sco-unit .........................   Passed    0.04 sec
      Start 4: collision-checker-unit
  4/6 Test #4: collision-checker-unit ...........   Passed    0.05 sec
      Start 5: planning-unit
  5/6 Test #5: planning-unit ....................   Passed    1.43 sec
      Start 6: cast-cost-unit
  6/6 Test #6: cast-cost-unit ...................   Passed    0.05 sec

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

  This is due to a name collision between a system installation of bullet and the local version in trajopt. One crude solution: ``sudo rm /usr/local/share/openrave-0.9/plugins/libbulletrave.so`` or whatever the path is to the bullet plugin. OpenRAVE uses ODE rather than Bullet by default, so there's no harm in removing the bullet plugin.

* *All the python tests fail* with an ``ImportError``, because ``trajoptpy`` is not found or ``ctrajoptpy`` is not found. That means your ``PYTHONPATH`` is not set correctly. It should have both the trajopt source directory ``/path/to/trajopt`` and the ``lib`` subdirectory of the build directory, ``/path/to/build_trajopt/lib``.

* *Almost all the tests fail, where OpenRAVE symbols aren't found*. Set ``LD_LIBRARY_PATH=/usr/local/lib`` or whereever libopenrave.so is. (Note: if you know how to fix this problem through RPATH settings or linker flags, please enlighten me.)