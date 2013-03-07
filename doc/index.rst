
.. |paper| replace:: paper
.. _paper: ../../coming-soon.pdf

trajopt: Trajectory Optimization for Motion Planning
==================================================================

*trajopt* is a software framework for generating robot trajectories by local optimization.

The following core capabilities are included:

- a solver for non-convex optimization problems, using sequential convex optimization. 
- cost and constraint functions for kinematics and collision avoidance
- constructing problems from JSON-based specification format

The core libraries are implemented in C++ (`API docs <../../dox_build/index.html>`_), and python
bindings are generated using boost python.

The theory and technical details of this software are described in a |paper|_, which is under double-blind review (and will be posted after the review deadline).

Contents:

.. toctree::
   :maxdepth: 2

   install
   tutorial
   tips
   misc