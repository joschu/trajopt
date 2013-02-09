import humanoidspy
import trajoptpy
import openravepy as rave
import numpy as np
import json

def xyzQuatFromMatrix(T):
    wxyz_xyz = rave.poseFromMatrix(T)
    return wxyz_xyz[4:7], wxyz_xyz[0:4]

def request_skeleton(n_steps):
    request = {
        "basic_info" : {
            "n_steps" : n_steps,
            "manip" : "active",
            "start_fixed" : True
        },
        "costs" : [
        {
            "type" : "joint_vel",            
            "params": {"coeffs":[1]}
        },
        {
            "type":"collision",
            "params":{"coeffs":[1], "dist_pen":[.01]}
        }
        ],
        "constraints" : [
        ],
        "init_info" : {
            "type" : "stationary"
        }
    }
    for i in xrange(1,n_steps):
        request["costs"].extend([
        # {
        #     "type":"potential_energy",
        #     "params":{"coeff" : .01,"timestep":i}
        # },
        {
            "type":"static_torque",
            "params":{"coeff" : .1,"timestep":i}
        }                    
        ])
    return request    

def opposite_foot(foot):
    if foot == "l_foot": return "r_foot"
    elif foot == "r_foot": return "l_foot"
    else: raise Exception("foot must be l_foot or r_foot")
    

def step_forward_request(robot, n_steps, stepping_foot, dx, dy): 
    """
    Sets up the problem to step forward with one foot (stepping_foot)
    Suppose stepping_foot = "r_foot"    
    Then problem constrains r_foot to move by (dx, dy), while the center
    of mass lies above the support polygon of l_foot
    """
    
    
    
    planted_foot = opposite_foot(stepping_foot)
    
    request = request_skeleton(n_steps)
    
    # fixed pose of planted foot
    planted_link = robot.GetLink(planted_foot)
    planted_transform = planted_link.GetTransform()
    planted_xyz, planted_wxyz = xyzQuatFromMatrix(planted_transform)
    
    # final pose of stepping foot
    stepping_link = robot.GetLink(stepping_foot)
    stepping_init_transform = stepping_link.GetTransform()    
    stepping_init_xyz = stepping_init_transform[:3,3]    
    
    z_angle = np.arctan2(dy, dx)
    stepping_final_xyz = stepping_init_xyz + np.array([dx, dy, 0])
    stepping_final_wxyz = rave.quatFromAxisAngle([0,0,1], z_angle)
    
    for i in xrange(1, n_steps):
        request["constraints"].extend([
            {
                "type":"pose",
                "name":"planted_pose",
                "params":{
                    "xyz":list(planted_xyz),
                    "wxyz":list(planted_wxyz),
                    "link":planted_foot,
                    "timestep":i
                }
            },
            {
                "type":"zmp",
                "params":{"planted_links":[planted_foot], "timestep":i}
            }
        ])
        
    request["constraints"].append(
        {
            "type":"pose",
            "name":"step_pose",
            "params":{
                "xyz" : list(stepping_final_xyz),
                "wxyz" : list(stepping_final_wxyz),
                "link" : stepping_foot,
                "timestep":(n_steps-1)
            }
        }
    )
    return request
    
def shift_weight_request(robot, n_steps, to_foot):
    """
    Sets up the problem to safely shift the weight to the other foot (to_foot)
    Suppose to_foot = "r_foot"    
    Then problem constrains both feet to remain at fixed poses (their current poses)
    at all intermediate timesteps, the center of mass lies over the convex hull of l_foot and r_foot
    at the final timestep, the center of mass lies over r_foot
    """    
    from_foot = opposite_foot(to_foot)
    
    request = request_skeleton(n_steps)
    from_foot_xyz, from_foot_quat = xyzQuatFromMatrix(robot.GetLink(from_foot).GetTransform())
    to_foot_xyz, to_foot_quat = xyzQuatFromMatrix(robot.GetLink(to_foot).GetTransform())
        
    for i in xrange(1, n_steps):
        request["constraints"].extend([
            {
                "type":"pose",
                "name":"from_foot_pose",
                "params":{
                    "xyz":list(from_foot_xyz),
                    "wxyz":list(from_foot_quat),
                    "link":from_foot,
                    "timestep":i
                }
            },
            {
                "type":"pose",
                "name":"to_foot_pose",
                "params":{
                    "xyz":list(to_foot_xyz),
                    "wxyz":list(to_foot_quat),
                    "link":to_foot,
                    "timestep":i
                }
            }
        ])    
        if i < n_steps - 1:
            request["constraints"].append(
                {
                    "type":"zmp",
                    "params":{"planted_links":[from_foot, to_foot],"timestep":i}
                })
        else:
            request["constraints"].append(
                {
                    "type":"zmp",
                    "name":"final_zmp",
                    "params":{"planted_links":[to_foot], "timestep":i}
                })                    

    return request
    
    
if __name__ == "__main__":
    
    
    ### Env setup ####
    env = rave.RaveGetEnvironment(1)
    if env is None:
        env = rave.Environment()
        env.StopSimulation()
        loadsuccess = env.Load("/Users/joschu/Proj/drc/gfe.xml")
        loadsuccess = loadsuccess and env.Load("/Users/joschu/Proj/darpa-proposal/drclogs.env.xml")
        assert loadsuccess
        try:
            import IPython
            viewer = trajoptpy.GetViewer(env)
            IPython.lib.inputhook.set_inputhook(viewer.Step)
        except ImportError:
            print "can't set IPython input hook. you won't be able to interact with plot while IPython is idling"
            pass
        
        
    robot = env.GetRobots()[0]        
    init_transform = np.eye(4)
    init_transform[:3,3] = [-.35, 1, .92712]
    robot.SetTransform(init_transform)
    robot.SetDOFValues(np.zeros(robot.GetDOF()))
    robot.SetActiveDOFs(np.arange(robot.GetDOF()), rave.DOFAffine.Transform)
    
    ##################
    trajoptpy.SetInteractive(True)
    
    cc = trajoptpy.GetCollisionChecker(env)
    cc.ExcludeCollisionPair(robot.GetLink("l_foot"), env.GetKinBody("ProjectRoom").GetLink("Floor"))
    cc.ExcludeCollisionPair(robot.GetLink("r_foot"), env.GetKinBody("ProjectRoom").GetLink("Floor"))
    
    n_steps = 6
    
    request = step_forward_request(robot, n_steps, "r_foot",.1, 0)
    s = json.dumps(request)
    prob = trajoptpy.ConstructProblem(s, env)
    result = trajoptpy.OptimizeProblem(prob)
    
    
    for i in xrange(8):
    
        request = shift_weight_request(robot, n_steps, "r_foot")
        s = json.dumps(request)
        prob = trajoptpy.ConstructProblem(s, env)
        result = trajoptpy.OptimizeProblem(prob)
    
        request = step_forward_request(robot, n_steps, "l_foot",.2, 0)
        s = json.dumps(request)
        prob = trajoptpy.ConstructProblem(s, env)
        result = trajoptpy.OptimizeProblem(prob)
    
        request = shift_weight_request(robot, n_steps, "l_foot")
        s = json.dumps(request)
        prob = trajoptpy.ConstructProblem(s, env)
        result = trajoptpy.OptimizeProblem(prob)

        request = step_forward_request(robot, n_steps, "r_foot",.2, 0)
        s = json.dumps(request)
        prob = trajoptpy.ConstructProblem(s, env)
        result = trajoptpy.OptimizeProblem(prob)
        
        