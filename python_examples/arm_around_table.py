import openravepy as rave
import trajoptpy
from trajoptpy import make_kinbodies

env = rave.Environment()
env.StopSimulation()
env.Load('robots/pr2-beta-static.zae')
trajoptpy.make_kinbodies.create_mesh_box(env, [1.1, 0, .65], [.85, .55, .06])

robot = env.GetRobots()[0]
robot.SetDOFValues([-1.832, -0.332, -1.011, -1.437, -1.1  , -2.106,  3.074], robot.GetManipulator('rightarm').GetArmIndices())
with open("../data/arm_around_table_continuous.json","r") as fh: s = fh.read()
trajoptpy.SetInteractive(True)
prob = trajoptpy.ConstructProblem(s, env)
result = trajoptpy.OptimizeProblem(prob)