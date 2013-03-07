import trajoptpy
import openravepy as rave
import numpy as np
import json
import atexit # avoid segfault  at exit
import trajoptpy.math_utils as mu
import trajoptpy.kin_utils as ku

def move_arm_straight_request(manip, n_steps, link_name, xyz_start, xyz_end, quat_start, quat_end, start_joints):
    manip_name = manip.GetName()
    request = {
        "basic_info" : {
            "n_steps" : n_steps,
            "manip" : manip_name,
            "start_fixed" : True
        },
        "costs" : [
        {
            "type" : "joint_vel",
            "params": {"coeffs" : [1]}
        },
        {
            "type" : "collision",
            "params" : {"coeffs" : [10],"dist_pen" : [0.025]}
        }                
        ],
        "constraints" : [
        {
            "type" : "pose",
            "name" : "final_pose",
            "params" : {
                "pos_coeffs" : [10,10,10],
                "rot_coeffs" : [10,10,10],
                "xyz" : list(xyz_end),
                "wxyz" : list(quat_end),
                "link" : link_name,
            },
        }
        ],
        "init_info" : {
        }
    }

    xyzs = mu.linspace2d(xyz_start, xyz_end, n_steps)
    quats = [rave.quatSlerp(quat_start, quat_end, t) for t in np.linspace(0,1,n_steps)]
    hmats = [rave.matrixFromPose(np.r_[quat,xyz]) for (xyz,quat) in zip(xyzs, quats)]
    def ikfunc(hmat):
        return ku.ik_for_link(hmat, manip, link_name, return_all_solns=True)
    def nodecost(joints, _):
        robot = manip.GetRobot()
        saver = rave.Robot.RobotStateSaver(robot)
        robot.SetDOFValues(joints, manip.GetArmJoints())
        return 100*robot.GetEnv().CheckCollision(robot)                
    paths, costs, timesteps = ku.traj_cart2joint(hmats, ikfunc, start_joints=start_joints, nodecost=nodecost)
    i_best = np.argmin(costs)
    print "lowest cost of initial trajs:", costs[i_best]
    if len(timesteps) < n_steps:
        print "timesteps with soln: ", timesteps
        print "linearly interpolating the rest"
        path_init = mu.interp2d(np.arange(n_steps), timesteps, paths[i_best])
    else:
        print "all timesteps have an ik solution"
        path_init = paths[i_best]
    for i in xrange(1, n_steps-1):
        print "waypoint xyz", xyzs[i]
        waypoint_term = {
            "type" : "pose",
            "name" : "waypoint_pose",
            "params" : {
                "xyz" : list(xyzs[i]),
                "wxyz" : list(quats[i]),
                "link" : link_name,
                "pos_coeffs" : [10,10,10],
                "rot_coeffs" : [10,10,10],
                "timestep" : i
            }}
        request["costs"].append(waypoint_term)
    request["init_info"]["type"] = "given_traj"
    request["init_info"]["data"] = [x.tolist() for x in path_init]

    
    return request


def main():
        
    ### Parameters ###
    ENV_FILE = "../data/pr2_table.env.xml"
    MANIP_NAME = "leftarm"
    N_STEPS = 15
    LINK_NAME = "l_gripper_tool_frame"
    XYZ_TARGET = [.5,0,1]
    QUAT_TARGET = [1,0,0,0]
    INTERACTIVE = True
    ##################
    
    
    ### Env setup ####
    env = rave.RaveGetEnvironment(1)
    if env is None:
        env = rave.Environment()
        env.StopSimulation()
        atexit.register(rave.RaveDestroy)
        env.Load(ENV_FILE)
    robot = env.GetRobots()[0]
    manip = robot.GetManipulator(MANIP_NAME)
    robot.SetDOFValues(np.zeros(len(manip.GetArmIndices())), manip.GetArmIndices())
    ##################
    
    T_start = robot.GetLink(LINK_NAME).GetTransform()
    quat_xyz = rave.poseFromMatrix(T_start)
    quat_start = quat_xyz[:4]
    xyz_start = quat_xyz[4:7]
    
    request = move_arm_straight_request(manip, N_STEPS, LINK_NAME, 
        xyz_start=xyz_start, xyz_end=XYZ_TARGET, quat_start=quat_start, quat_end=QUAT_TARGET,
        start_joints = robot.GetDOFValues(manip.GetArmJoints()))
    s = json.dumps(request)
    print "REQUEST:",s
    trajoptpy.SetInteractive(INTERACTIVE);
    prob = trajoptpy.ConstructProblem(s, env)
    result = trajoptpy.OptimizeProblem(prob)

if __name__ == "__main__":
    main()