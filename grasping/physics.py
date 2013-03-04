import openravepy as rave
#import trajoptpy
#import bulletsimpy

def get_bulletobj_state(bullet_obj):
  posevec = rave.poseFromMatrix(bullet_obj.GetTransform())
  return {"name": bullet_obj.GetName(), "xyz": posevec[4:7], "wxyz": posevec[0:4]}

def record_sim(bullet_env, bullet_rec_objs, n_timesteps, dt=0.01, max_substeps=100, internal_dt=0.01, prestep_fn=None, pause_per_iter=False, update_rave_env=False):
  assert all(not o.IsKinematic() for o in bullet_rec_objs) # not really necessary, but would be silly otherwise
  out = []
  rave_env = bullet_env.GetRaveEnv()

  for t in range(n_timesteps):
    if prestep_fn is not None:
      prestep_fn(t)

    bullet_env.Step(dt, max_substeps, internal_dt)

    if update_rave_env:
      for o in bullet_rec_objs:
        rave_env.GetKinBody(o.GetName()).SetTransform(o.GetTransform())
      rave_env.UpdatePublishedBodies()
      if pause_per_iter:
        raw_input('press enter to continue')

    obj_states = [get_bulletobj_state(o) for o in bullet_rec_objs]
    out.append({"timestep": t, "obj_states": obj_states})

  return out

def record_sim_with_traj(prob, robot_name, robot_traj, bullet_env, bullet_rec_objs, n_timesteps, **kwargs):
  assert n_timesteps == len(robot_traj)
  #assert rave.RaveGetEnvironmentId(bullet_env.GetRaveEnv()) == rave.RaveGetEnvironmentId(robot.GetEnv())

  rave_env = bullet_env.GetRaveEnv()
  robot, bullet_robot = rave_env.GetRobot(robot_name), bullet_env.GetObjectByName(robot_name)
  assert robot.IsKinematic() # if not, the prestep and record_sim could clash
  robot.SetActiveDOFs(prob.GetDOFIndices(), prob.GetAffineDOFs())
  ss = rave.RobotStateSaver(robot)

  def prestep(t):
    robot.SetActiveDOFValues(robot_traj[t,:])
    bullet_robot.UpdateFromRave()

  return record_simulation(bullet_env, bullet_rec_objs, n_timesteps, prestep_fn=prestep, **kwargs)

