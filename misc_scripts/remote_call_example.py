import xmlrpclib, json, time

server = xmlrpclib.ServerProxy('http://localhost:9000')

sesh_key =server.start_session("asdf")


xmldata = """
<Environment>
    <Robot file="robots/pr2-beta-static.zae"/>
</Environment>
"""
server.load_environment(sesh_key, xmldata)

joint_target = [0.062, 1.287, 0.1, -1.554, -3.011, -0.268, 2.988]

request = {
  "basic_info" : {
    "n_steps" : 10,
    "manip" : "rightarm", # see below for valid values
    "start_fixed" : True # i.e., DOF values at first timestep are fixed based on current robot state
  },
  "costs" : [
  {
    "type" : "joint_vel", # joint-space velocity cost
    "params": {"coeffs" : [1]} # a list of length one is automatically expanded to a list of length n_dofs
    # Also valid: "coeffs" : [7,6,5,4,3,2,1]
  },
  {
    "type" : "collision",
    "name" :"collision",
    "params" : {
      "coeffs" : [20], # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
      "dist_pen" : [0.025] # robot-obstacle distance that penalty kicks in. expands to length n_timesteps
    }
  }
  ],
  "constraints" : [
  {
    "type" : "joint", # joint-space target
    "params" : {"vals" : joint_target } # length of vals = # dofs of manip
  }
  ],
  "init_info" : {
      "type" : "straight_line", # straight line in joint space.
      "endpoint" : joint_target
  }
}

for i in xrange(10): 
    t_start = time.time()
    response = server.solve_problem(sesh_key, json.dumps(request))
    print "round trip time:", time.time() - t_start
    print "solve time:", response["solve_time"]