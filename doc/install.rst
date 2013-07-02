.. _install:


Installing trajopt
===================

This code has been tested on Linux and OSX. 

Dependencies
------------

- OpenRAVE (>= 0.8 required, >= 0.9 highly recommended)
- OpenSceneGraph
- CMake
- boost
- Eigen
- Gurobi [#gurobi]_ (recommended)

.. [#gurobi] On Linux, Trajopt now works with the free solver BPMPD, which is included with the source distribution (as a static library). However, Gurobi is better-tested with Trajopt and usually performs better.


Instructions
-------------

- install OpenRAVE. To obtain OpenRAVE 0.9, you can install from source or use the `openrave testing <https://launchpad.net/~openrave/+archive/testing>`_ PPA.


- install OpenSceneGraph, CMake, boost, and Eigen using your package manager. In Ubuntu, that's::

    sudo apt-get install libopenscenegraph-dev cmake libboost-all-dev libeigen3-dev python-numpy 

- Gurobi (optional):

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
  
in the build directory. The output should look something like this:

.. include:: cmd_output/ctest.txt
   :literal:

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
