.. _tutorial:


Tutorial
===========

This tutorial will describe how to set up and solve optimization problems in python.

Move arm to joint target
-------------------------
Full source code:  :file:`python_examples/arm_to_joint_target.py`

The following script will plan to get the right arm to a joint-space target.
It is assumed that the robot's current state is the start state, so make sure you set the robots DOFs appropriately.

The optimization will pause at every iteration and plot the current trajectory. Press 'p' to un-pause. To disable plotting and pausing, change the statement to :code:`trajoptpy.SetInteractive(False)`.

.. literalinclude:: ../python_examples/arm_to_joint_target.py

Every problem description contains four sections: basic_info, costs, constraints, and init_info. (TODO: describe the syntax).

Here, there are two cost components: joint-space velocity, and the collision penalty. There is one constraint: the final joint state. (TODO: explain the parameters)

Let's inspect the terminal output after running the script. The following lines should appear at the end of the output.

::

  [INFO optimizers.cpp:225] iteration 9
  [INFO optimizers.cpp:294] 
                  |   oldexact |    dapprox |     dexact |      ratio
            COSTS | -------------------------------------------------
        joint_vel |  1.776e+00 |  2.249e-05 |  2.249e-05 |  1.000e+00
   cont_collision |  0.000e+00 |  0.000e+00 |  0.000e+00 | (     nan)
   cont_collision |  0.000e+00 |  0.000e+00 |  0.000e+00 | (     nan)
   cont_collision |  0.000e+00 |  0.000e+00 |  0.000e+00 | (     nan)
   cont_collision |  0.000e+00 | -3.796e-10 |  0.000e+00 | (-0.000e+00)
   cont_collision |  0.000e+00 | -3.801e-10 |  0.000e+00 | (-0.000e+00)
   cont_collision |  0.000e+00 | -3.947e-10 | -5.022e-04 | (1.272e+06)
   cont_collision |  0.000e+00 | -7.592e-10 |  0.000e+00 | (-0.000e+00)
   cont_collision |  0.000e+00 | -1.139e-09 |  0.000e+00 | (-0.000e+00)
   cont_collision |  0.000e+00 | -7.592e-10 |  0.000e+00 | (-0.000e+00)
            TOTAL |  1.776e+00 |  2.249e-05 | -4.797e-04 | -2.133e+01
  [INFO optimizers.cpp:305] converged because improvement was small (2.249e-05 < 1.000e-04)
  [INFO optimizers.cpp:343] woo-hoo! constraints are satisfied (to tolerance 1.00e-04)

Here, the optimization converged in 9 iterations. Each line in the table shows a cost or constraint term and it's value and improvement in the latest iteration.
The "joint_vel" line refers to the joint velocity cost.
The "cont_collision" lines each refer to the collision cost for a time interval [t,t+1].
As you can see, all of the collision costs are zero, meaning the trajectory is safely out of collision, with a safety margin of `dist_pen` meters.
Note that there's no term for the "joint" constraint. That's because this constraint is a linear constraint in the optimization problem, and the table only shows nonlinear constraints which must be approximated at each iteration.
As you can see, there's not a 1-1 correspondence between the "cost" and "constraint" items in the json file and the costs and constraints that the optimization algorithm sees.
(If you're digging into the C++ API, note that the table rows correspond to `Cost <../../dox_build/classsco_1_1_cost.html>`_ and `Constraint <../../dox_build/classsco_1_1_constraint.html>`_ objects).

.. note:: Why do we use a collision cost, rather than a constraint? In the sequential convex optimization procedure, constraints get converted into costs--specifically, :math:`\ell_1` penalties (see the paper). So the difference between a constraint and an :math:`\ell_1` cost is that for a constraint, the optimizer checks to see if it is satisfied (to some tolerance), and if not, it jacks up the penalty coefficient. The thing about collisions is that it's often necessary to violate the safety margin, e.g. when you need to contact an object. So you're best off handling the logic yourself of adjusting penalties and safety margins, based on your specific problem.



Move arm to pose target
-------------------------
Full source code:  :file:`python_examples/arm_to_cart_target.py`


Next, let's solve the same problem, but instead of specifying the target joint position, we'll specify the target pose.

Now we use a pose constraint instead of a joint constraint:

.. literalinclude:: ../python_examples/arm_to_cart_target.py
  :start-after: BEGIN pose_constraint
  :end-before: END pose_constraint

Initialize using collision-free IK:

.. literalinclude:: ../python_examples/arm_to_cart_target.py
  :start-after: BEGIN ik
  :end-before: END ik

...

.. literalinclude:: ../python_examples/arm_to_cart_target.py
  :start-after: BEGIN init
  :end-before: END init

Now you'll notice an extra couple lines of the optimizer output::

                |   oldexact |    dapprox |     dexact |      ratio
          COSTS | -------------------------------------------------
    ...
    CONSTRAINTS | -------------------------------------------------
           pose |  4.326e-05 |  4.326e-05 |  3.953e-05 |  9.138e-01

As you can see, the pose constraint is satisfied at convergence

Rolling your own costs and constraints in python
---------------------------------------------------
Full source code:  :file:`python_examples/this_side_up.py`

This next script shows how you can easily implement your own costs and constraints. The constraint we consider here says that a certain vector defined in the robot gripper frame must point up (in the world coordinates). For example, one might imagine that the robot is holding a glass of water, and we don't want it to spill.

The basic setup of the problem is similar to the previous examples.
We set a pose constraint at the end of the trajectory. This time, we ignore the rotation (by setting :code:`rot_coefs = [0,0,0]`).

.. literalinclude:: ../python_examples/this_side_up.py
  :start-after: BEGIN pose_target
  :end-before: END pose_target

We also add a constraint on the end-effector displacement at each step:

.. literalinclude:: ../python_examples/this_side_up.py
  :start-after: BEGIN vel
  :end-before: END vel

The parameter :code:`"distance_limit : .05"` means the position of the link (r_gripper_tool_frame) moves by less than .05 in each timestep, for each component (x,y,z).

Now, let's look at the user-defined costs and constraints. We define an vector-valued error function :code:`f(x)`, and the Jacobian of this function :code:`dfdx(x)`. f(x) returns the x and y components of the transformed vector (which both equal zero if the vector points up).

.. literalinclude:: ../python_examples/this_side_up.py
  :start-after: BEGIN python_funcs
  :end-before: END python_funcs

We add these error functions to the problem as constraints (i.e., that error = 0) with the following lines:

.. literalinclude:: ../python_examples/this_side_up.py
  :start-after: BEGIN add_constraints
  :end-before: END add_constraints
  
As you can see, we can provide only the function f (which will be numerically differentiated internally), or we can also provide the function and its analytic derivative. 
Note that in both calls to :code:`AddConstraint`, we pass in the following parameter: :code:`[(t,j) for j in xrange(7)]`. This argument is telling the optimizer which variables the provided function should be applied to. This parameter must be a list of ordered pairs (timestep, dof). These ordered pairs specify which trajectory variables make up the input vector for your provided functions. For example, we add a "up" cost at timestep :code:`t=3` by providing the indices :code:`[(3,0), (3,1), (3,2), (3,3), (3,4), (3,5), (3,6)]`. Then the optimizer knows that this constraint function should be applied to the vector :math:`(\theta_{3,0},\theta_{3,1},\theta_{3,2},\theta_{3,3},\theta_{3,4},\theta_{3,5},\theta_{3,6})`

Similarly to the way we added those constraints, we can also add the error functions as costs, as a hinge, abs, or squared loss, using the function :code:`TrajOptProb.AddErrCost`.
Or, we can provide a single scalar-valued cost function by using the function :code:`TrajOptProb.AddCost`.

More constraints have been implemented but are not currently documented here.
One way to find out about which ones have been implemented and what parameters are available is to look at the Doxygen documentation for the subclasses of `CostInfo <../../dox_build/structtrajopt_1_1_cost_info.html>`_ and `CntInfo <../../dox_build/structtrajopt_1_1_cnt_info.html>`_ because each item under "costs" or "constraints" in the JSON document is first converted to one of these structs when the JSON file is read.


Optimizing the base position and torso height
-----------------------------------------------

This next example shows how to jointly optimize over the base and torso degrees of freedom of a mobile robot, as well as the arms. This program considers a trajectory with one timestep, and ``start_fixed=False``, so it is really just doing collision-aware numerical IK. You can optimize over a trajectory containing all of these degrees of freedom by setting ``n_steps``. ``basic_info`` is set as follows:

.. literalinclude:: ../python_examples/position_base.py
  :start-after: BEGIN basic_info
  :end-before: END basic_info

Note that ``manip="active"``, which makes the selects the degrees of freedom of the optimization problem based on the degrees of freedom of the robot, which we set earlier:

.. literalinclude:: ../python_examples/position_base.py
  :start-after: BEGIN set_active
  :end-before: END set_active

Numerical IK is very unreliable since the forward kinematics function is so nonlinear. Therefore, we do a series of random initializations:

.. literalinclude:: ../python_examples/position_base.py
  :start-after: BEGIN random_init
  :end-before: END random_init

Walking with humanoid robot
----------------------------

.. note:: To run the example script, you'll need to download the :ref:`test data <bigdata>` and build with the CMake option ``BUILD_HUMANOIDS=ON``.

See :file:`python_examples/drc_walk.py` for a simple example of planning footsteps with the Atlas robot model (for DARPA robotics challenge).

The walk cycle consists of the following four phases, each of which is motion-planned independently:

1. Shift center of mass so it's over right foot
2. While keeping center of mass over right foot, swing left foot forward
3. Shift center of mass so it's over left foot
4. While keeping center of mass over left foot, swing right foot forward

These planning problems involve pose constraints, as well as a "ZMP" constraint, which constrains the robot's center of mass to lie above the convex hull of the one or two planted feet. This gait will provide a stable walk in the quasi-static regime that the robot moves very slowly (i.e., the forces and torques needed to accelerate the robot are negligible compared with the weight of the robot). 

Obstacle geometry from sensor data
----------------------------------------------

.. note:: To run the example script, you'll need to download the :ref:`test data <bigdata>` and build with the CMake option ``BUILD_CLOUDPROC=ON``.

We'll consider three different ways of representing the geometry of the environment:

(1) as a collection of boxes or spheres

  The advantage is that the preprocessing of the input point cloud or voxel data is very simple. The disadvantages are that (a) this representation is usually slow to collision check against, (b) for the purposes of trajectory optimization, the contact normals aren't very informative about how to get out of collision


(2) as a mesh

  There are a variety of algorithms for reconstructing surfaces from point clouds. The included script uses the simplest type of surface generation algorithm, which forms a mesh out of a single depth image by connecting neighboring points in the pixel array. After constructing the mesh, we decimate it, i.e., simplify it by iteratively merging vertices.

(3) convex decomposition

  The idea of convex decomposition is to break down a 3D shape into a set of convex pieces. The resulting convex decomposition allows fast collision checking and good surface normals. We use Khaled Mammou's Hierarchical Approximate Convex Decomposition (`HACD <http://sourceforge.net/projects/hacd>`_) code.

Here is a table summarizing the tradeoffs:

+-------------------+-------+------+---------------+
|                   | boxes | mesh | convex decomp |
+-------------------+-------+------+---------------+
| construction cost | low   | med  | high          |
+-------------------+-------+------+---------------+
| coll. check speed | low   | med  | high          |
+-------------------+-------+------+---------------+
| normal quality    | low   | med  | high          |
+-------------------+-------+------+---------------+

**We highly recommend using meshes or convex decomposition with trajopt**, since these representations will perform far better than boxes with regard to speed and likelihood of finding a solution.

A demonstration of using all three of these methods can be found in :file:`python_examples/kinect_drive_and_reach.py`. After driving a PR2 robot to several positions in an office, we acquired a single point cloud (xyz + rgb) from the robot's head-mounted Kinect and saved it, along with the joint state and camera transformation. For each scene, we have annotated a target pose for the right gripper (``pose_target.txt``). We then plan a trajectory which simultaneously moves the right arm, torso, and base (11 = 7 + 1 + 3 DOF) to reach this target.

To use the mesh or convex decomposition for collision geometry, we first construct a mesh from the point cloud.

.. literalinclude:: ../python_examples/kinect_drive_and_reach.py
  :start-after: BEGIN generate_mesh
  :end-before: END generate_mesh
  
Note that the module ``cloudprocpy`` is a set of python bindings to the Point Cloud Library (PCL), plus a few functions from other libraries.

For convex decomposition, we call a function to break the mesh into a set of convex pieces

.. code-block:: python

      convex_meshes = cloudprocpy.convexDecompHACD(big_mesh,30)

The second parameter (30) relates to the allowed concavity of each almost-convex subset; see the docstring for more information.