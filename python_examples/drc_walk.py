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
            "type" : "joint_pos",
            "params" : {"coeffs": ([.1]*28 + [0]*7), "vals":standing_posture.tolist()}
        },
        {
            "type":"collision",
            "params":{"coeffs":[1], "dist_pen":[.04]}
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
         #{
             #"type":"potential_energy",
             #"params":{"coeff" : .0005,"timestep":i}
         #},
        #{
            #"type":"static_torque",
            #"params":{"coeff" : .01,"timestep":i}
        #}                    
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
                "type":"zmp","name":"zmp_%i"%i,
                "params":{"planted_links":[planted_foot], "timestep":i}
            }
        ])
        #if i < n_steps-1:
            #request["constraints"].append({
                #"type":"foot_height",
                #"params":{
                    #"height": .1,
                    #"timestep": i,
                    #"link":stepping_foot}})
        
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
                    "type":"zmp","name":"zmp%i"%i,
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

def press_button_request(robot, hand_xyz, hand_link, foot_links, n_steps):
    """
    Sets up the problem to safely shift the weight to the other foot (to_foot)
    Suppose to_foot = "r_foot"    
    Then problem constrains both feet to remain at fixed poses (their current poses)
    at all intermediate timesteps, the center of mass lies over the convex hull of l_foot and r_foot
    at the final timestep, the center of mass lies over r_foot
    """    
    
    from_foot, to_foot = foot_links
    
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
        request["constraints"].append(
            {
                "type":"zmp","name":"zmp_%i"%i,
                "params":{"planted_links":[from_foot, to_foot],"timestep":i}
            })
    request["constraints"].append(
        {
            "type":"pose",
            "name":"final_hand_pose",
            "params":{
                "xyz":list(hand_xyz),
                "wxyz":[1,0,0,0],
                "link":hand_link,
                "pos_coeffs":[1,1,1],
                "rot_coeffs":[0,0,0],
                "timestep":i
            }
        }        
    )

    return request


    
def animate_traj(viewer, robot, traj):
    for (i,row) in enumerate(traj):
        print "step",i
        robot.SetActiveDOFValues(row)
        viewer.Idle()
        
    
if __name__ == "__main__":
    
    
    ### Env setup ####
    env = rave.RaveGetEnvironment(1)
    if env is None:
        env = rave.Environment()
        env.StopSimulation()
        loadsuccess = env.Load("../bigdata/atlas.xml")
        loadsuccess = loadsuccess and env.Load("../data/obstructed_door.env.xml")
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
    init_transform[:3,3] = [.1, 1, .92712]
    #init_transform[:3,3] = [2.6, 1, .92712]
    robot.SetTransform(init_transform)
    robot.SetDOFValues(np.zeros(robot.GetDOF()))
    robot.SetActiveDOFs(np.arange(robot.GetDOF()), rave.DOFAffine.Transform)
    # move arms to side
    robot.SetDOFValues([-1.3],[robot.GetJoint("l_arm_shx").GetDOFIndex()])
    robot.SetDOFValues([1.3],[robot.GetJoint("r_arm_shx").GetDOFIndex()])
    standing_posture = robot.GetActiveDOFValues()
    ##################
    trajoptpy.SetInteractive(True)
    
    cc = trajoptpy.GetCollisionChecker(env)
    cc.ExcludeCollisionPair(robot.GetLink("l_foot"), env.GetKinBody("ProjectRoom").GetLink("Floor"))
    cc.ExcludeCollisionPair(robot.GetLink("r_foot"), env.GetKinBody("ProjectRoom").GetLink("Floor"))
    
    n_steps = 5


    x_button_press = 2.55 # if robot is at this x-coordinate, he can reach the button
    xyz_button = env.GetKinBody("bigredbutton").GetTransform()[:3,3]
    xyz_button[2] += .15;

    totaltraj = []

    i=0
    
    while robot.GetTransform()[0,3] < x_button_press:

        request = shift_weight_request(robot, n_steps, "l_foot")
        s = json.dumps(request)
        prob = trajoptpy.ConstructProblem(s, env)
        result = trajoptpy.OptimizeProblem(prob)
        robot.SetActiveDOFValues(result.GetTraj()[-1])
        totaltraj.extend(result.GetTraj())

        request = step_forward_request(robot, n_steps, "r_foot",.1 if i==0 else .2, 0)
        s = json.dumps(request)
        prob = trajoptpy.ConstructProblem(s, env)
        result = trajoptpy.OptimizeProblem(prob)
        robot.SetActiveDOFValues(result.GetTraj()[-1])
        totaltraj.extend(result.GetTraj())
    
        request = shift_weight_request(robot, n_steps, "r_foot")
        s = json.dumps(request)
        prob = trajoptpy.ConstructProblem(s, env)
        result = trajoptpy.OptimizeProblem(prob)
        robot.SetActiveDOFValues(result.GetTraj()[-1])
        totaltraj.extend(result.GetTraj())
    
        request = step_forward_request(robot, n_steps, "l_foot",.2, 0)
        s = json.dumps(request)
        prob = trajoptpy.ConstructProblem(s, env)
        result = trajoptpy.OptimizeProblem(prob)
        robot.SetActiveDOFValues(result.GetTraj()[-1])
        totaltraj.extend(result.GetTraj())
    
        i += 1
    
    request = press_button_request(robot, xyz_button, "l_hand", ["l_foot","r_foot"],10)
    s = json.dumps(request)
    prob = trajoptpy.ConstructProblem(s, env)
    result = trajoptpy.OptimizeProblem(prob)
    totaltraj.extend(result.GetTraj())

    viewer = trajoptpy.GetViewer(env)
    animate_traj(viewer, robot, totaltraj)