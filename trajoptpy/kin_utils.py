"""
Kinematics helpers for openrave
"""

import numpy as np

def shortest_paths(ncost_nk,ecost_nkk):
    """       
    Find minimum cost paths through graph (one path for each end point)
    where all nodes in nth row are connected to all nodes in (n+1)st row
    ncost_nk: node costs
    ecost_nkk: edge costs
    
    returns: (paths, costs) where
    paths is a K x N array of integers. Each row gives the minimum-cost path
      to its final node.
    costs has length K and gives the cost of each path
    """
    N = len(ncost_nk)
    assert len(ecost_nkk) == N-1
    
    cost_nk = [None for _ in xrange(N)]
    prev_nk = [None for _ in xrange(N-1)]
    cost_nk[0] = ncost_nk[0]
    for n in xrange(1,N):
        cost_kk = ecost_nkk[n-1] + cost_nk[n-1][:,None] + ncost_nk[n][None,:]
        cost_nk[n] = cost_kk.min(axis=0)
        prev_nk[n-1] = cost_kk.argmin(axis=0)

    
    path_costs = cost_nk[N-1]
    paths = [None for _ in xrange(N)]

    paths[N-1] = np.arange(len(ncost_nk[-1]))
    for n in xrange(N-1,0,-1):
        paths[n-1] = prev_nk[n-1][paths[n]]

    return np.array(paths).T, path_costs

def pairwise_squared_dist(x,y, timestep=None):
    "pairwise squared distance between rows of matrices x and y"
    return (x**2).sum(axis=1)[:,None]+(y**2).sum(axis=1)[None,:]-2*x.dot(y.T)    

def traj_cart2joint(hmats, ikfunc, start_joints = None, nodecost=None, edgecost = None):
    """
    hmats: poses at times t = 0,1,2,...T-1
    ikfunc: a function f: R^(4x4) -> [R^k]
        i.e., map from 4x4 matrix to a list of joint states
    start_joints: None, starting joint state, or list of starting joint states
        if start_joints is supplied, the first element of hmats is ignored
    nodecost (optional): the cost of a joint state
        function f : R^k, timestep -> R
    edgecost (optional): the cost of transitions
        function f : R^(n x k), R^(m x k), timestep -> R^(n x m)
        defaults to squared euclidean distance

    
    returns:  (trajectories, costs, timesteps)    
    where trajectories is a list of M 2d arrays, each of which has length S <= T
    costs is a list of M floats, each of which gives the cost of the corresponding trajectory
    and timesteps is a list of length S--the timesteps when an IK solution exists
    
    
    """
    iksolns = []
    timesteps = []
    for (i,hmat) in enumerate(hmats):
        if i==0 and start_joints is not None:
            solns = np.atleast_2d(start_joints)
        else:
            solns = ikfunc(hmat)
        if len(solns) > 0:
            iksolns.append(solns)
            timesteps.append(i)
    
    if edgecost is None:
        edgecost = pairwise_squared_dist
        
            
    N = len(iksolns)
    
    ncost_nk = []
    ecost_nkk = []
   
    for i in xrange(0,len(iksolns)):
        solns0 = iksolns[i]
        if nodecost is None: ncost_nk.append(np.zeros(len(solns0)))
        else: ncost_nk.append(np.array([nodecost(soln, timesteps[i]) for soln in solns0]))
        if i>0:
            solnsprev = iksolns[i-1]
            ecost_nkk.append(pairwise_squared_dist(solnsprev, solns0, i-1))
    
    paths, path_costs = shortest_paths(ncost_nk, ecost_nkk)
    return [np.array([iksolns[t][i] for (t,i) in enumerate(path)]) for path in paths], path_costs, timesteps
    
    

def ik_for_link(T_w_link, manip, link_name, filter_options = 18, return_all_solns = False):
    """
    Perform IK for an arbitrary link attached to the manipulator
    e.g. you might want ik for pr2 "r_gripper_tool_frame" instead of the openrave EE frame
    
    T_w_link: 4x4 matrix. "world frame from link frame"
    manip: openrave Manipulator
    link_name: (you know)
    filter_options: see openravepy.IkFilterOptions
    return_all_solns: if True, returns a list. if false, returns a single solution (openrave's default)
    if a solution exists, otherwise return None
    
    """
    
    robot = manip.GetRobot()

    link = robot.GetLink(link_name)

    if not robot.DoesAffect(manip.GetArmJoints()[-1], link.GetIndex()):
        raise Exception("link %s is not attached to end effector of manipulator %s"%(link_name, manip.GetName()))

    Tcur_w_link = link.GetTransform()
    Tcur_w_ee = manip.GetEndEffectorTransform()
    Tf_link_ee = np.linalg.solve(Tcur_w_link, Tcur_w_ee)
    
    T_w_ee = T_w_link.dot(Tf_link_ee)
    
    if return_all_solns:
        return manip.FindIKSolutions(T_w_ee, filter_options)
    else:
        return manip.FindIKSolution(T_w_ee, filter_options)
    
