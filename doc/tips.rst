.. _tips:

Tips and troubleshooting
=========================

Optimization gets stuck in a local minimum
---------------------------------------------

1. Do a better initialization, preferably one that reaches the goal. Use IK if possible.
2. Do multiple initializations, e.g. using random waypoints. See :ref:`benchmark` and :ref:`baseik` for examples.

Stationary initialization often doesn't cut it.
Here's how to initialize in several common types of problem:

- Arm planning problems with joint goal: use a straight line in joint space
- Arm planning problems with cartesian goal: do IK and then use a straight line in joint space. You may want to try multiple initializations for the different IK solutions.
- Arm + base problems: now it's not so straightforward to use analytic IK. Here's one easy-to-implement way to get an initialization that reaches the goal: first, with a stationary initialization, solve your optimization problem, minus the collision cost. That gives you a trajectory `Traj` that reaches the goal (hopefully). Then add in the collision cost, and initialize with `Traj`.

Note that numerical IK frequently fails, because forward kinematics is so nonlinear/non-convex. So if you can use analytic IK for initializing your trajectory, do it.

As for random initializations, one easy but effective trick is to make a random waypoint in configuration space, and then initialize with the trajectory Start-Waypoint-Goal, which linearly interpolates between the Start, Waypoint, and Goal configurations.


Optimization is slow
-----------------------

* Try reducing the number of timesteps
* Maybe some of the constraints are conflicting with the costs. Then the algorithm has to do an outer loop where it increases the constraint penalty until they're all satisfied. You'll see something like::

  [INFO optimizers.cpp:348] not all constraints are satisfied. increasing penalties

  If that happens you can try rescaling the cost or constraint functions so that this outer loop is not needed.

General troubleshooting tips
----------------------------

* Turn on plotting. From python, you do :code:`trajoptpy.SetInteractive(True)`.
* Look at the table printed out by the optimization algorithm at every iteration. e.g. for :file:`position_base.py` (which only optimizes over the robot configuration at a single timestep), the following table is printed::

                |   oldexact |    dapprox |     dexact |      ratio
          COSTS | -------------------------------------------------
      collision |  2.546e+00 |  1.504e-08 |  2.095e-08 |  1.394e+00
    CONSTRAINTS | -------------------------------------------------
     final_pose |  7.859e+00 |  1.055e+00 |  9.856e-01 |  9.344e-01
          TOTAL |  1.040e+01 |  1.055e+00 |  9.856e-01 |  9.344e-01
          
  This table describes how the cost and constraint violations are changed by each candidate step taken by the optimization.
  The meaning of the columns is as follows:
  
  - "oldexact": value of nonlinear cost / constraint penalty
  - "dapprox": change in the convexification of the cost / constraint penalty
  - "dexact": change in the exact cost / constraint penalty
  - "ratio": dexact / dapprox.
  
  All the constraint violations should become very small. (note that the values in the "oldexact" column are constraint_violation * penalty_coeff).
  If all is going well, the ratio should be somewhere near 1. If it's far from 1, it could be for one of the following reasons:
  
  1. if you defined your own convexification or analytic derivative, perhaps there is a math error
  2. the convexification does not approximate the function well over the trust region.
  
  Unforunately, the second case is fairly common, and there's not much you can do about it except change your cost or constraint function to something better behaved.