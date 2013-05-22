.. _misc:


Miscellaneous 
=======================

.. _bigdata: 

Downloading test data
-----------------------

First navigate to the ``bigdata`` directory, and then run the script ``download.py``.

Collision costs
------------------

There are two types of collision penalty provided: a discrete-time penalty and a continuous-time penalty. The discrete-time penalty is based on the signed distance between the robot's links and the obstacles (including robot links). The continuous-time penalty considers the volume swept-out by the robot's links as it moves from one timestep to the next. See the paper for more details. While the discrete-time penalty will often jump through a thin obstacle, the continuous-time penalty will properly penalize such events. The only downside of the continuous-time penalty is that it currently doesn't check self-collisions. If you want to avoid self-collisions, you can add a discrete-time and continuous-time penalty.

From python, both penalties have the same API. The optional ``first_step`` and ``last_step`` arguments are the first and last timestep (inclusive) for the costs.

Discrete-time collision:

.. code-block:: python

        {
            "type" : "collision",
            "params" : {"coeffs" : [10],"dist_pen" : [0.025], "first_step":0, "last_step":19}
        }

TODO: update this
Continuous-time collision:

.. code-block:: python

        {
            "type" : "continuous_collision",
            "params" : {"coeffs" : [10],"dist_pen" : [0.025], "first_step":0, "last_step":19}
        }
        
Finding out about available costs and constraints
----------------------------------------------------------------

Some costs and constraints have been implemented but are not currently documented here.
One way to find out about which ones have been implemented and what parameters are available is to look at the Doxygen documentation for the subclasses of `CostInfo <../../dox_build/structtrajopt_1_1_cost_info.html>`_ and `CntInfo <../../dox_build/structtrajopt_1_1_cnt_info.html>`_ because each item under "costs" or "constraints" in the JSON document is first converted to one of these structs when the JSON file is read.


