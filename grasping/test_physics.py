import trajoptpy
import openravepy as rave
import bulletsimpy
import physics
from pprint import pprint

env = rave.Environment()
env.StopSimulation()
env.Load("robots/pr2-beta-static.zae")
env.Load("../data/table.xml")

env.SetViewer('qtcoin')

bullet_env = bulletsimpy.LoadFromRave(env, 'table')
bullet_env.SetGravity([0, 0, -9.8])

# objects to record
rec_obj_names = ['table']
rec_objs = [bullet_env.GetObjectByName(name) for name in rec_obj_names]

rec = physics.record_sim(bullet_env, rec_objs, n_timesteps=20, update_rave_env=True, pause_per_iter=True)
pprint(rec)

