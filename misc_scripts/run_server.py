import argparse
parser = argparse.ArgumentParser()
parser.add_argument("keyfile", help="file with list of valid authentication keys")
args = parser.parse_args()

with open(args.keyfile) as fh:
    AUTH_KEYS = [line.strip() for line in fh.readlines()]
    AUTH_KEYS = filter(None, AUTH_KEYS)
    print "valid keys:", AUTH_KEYS



from SimpleXMLRPCServer import SimpleXMLRPCServer
import datetime
import openravepy, trajoptpy
import time
import signal

import threading
import random, string

def random_word(length):
   return ''.join(random.choice(string.lowercase) for i in range(length))


MAX_IDLE_TIME = 60
CLEANUP_PERIOD = 1 

class TrajoptManager(object):
    """
    Manages a bunch of environments    
    """
    
    def __init__(self):
        self.lock = threading.Lock() # XXX actually use this
        self.sesh2data = {} # session key 2 popen
        self.done = False
        
    def _update_last_used(self, sessionkey):
        self.sesh2data[sessionkey]["last_used"] = time.time()
        
    def start_session(self, authkey):
        if authkey not in AUTH_KEYS:
            raise Exception("invalid key")
        # todo: check authkey
        sessionkey = random_word(10)
        self.sesh2data[sessionkey] = {}
        self.sesh2data[sessionkey]["env"] = openravepy.Environment()
        self._update_last_used(sessionkey)
        return sessionkey
        
    def load_environment(self, sessionkey, xmlstr):
        """
        Load an openrave xml file
        """
        env = self.sesh2data[sessionkey]["env"]
        self._update_last_used(sessionkey)
        return bool(env.LoadData(xmlstr))
        
    def solve_problem(self, sessionkey, jsonstr):
        """
        Solve problem specified by a json file
        """
        env = self.sesh2data[sessionkey]["env"]
        prob = trajoptpy.ConstructProblem(jsonstr, env)
        t_start = time.time()
        result = trajoptpy.OptimizeProblem(prob) # do optimization
        out = {}
        out["solve_time"] = time.time() - t_start
        traj = result.GetTraj()
        costs = result.GetCosts()
        constraints = result.GetConstraints()
        out["traj"] = [row.tolist() for row in traj]
        out["costs"] = costs
        out["constraints"] = constraints
        self._update_last_used(sessionkey)
        return out
        
    def _cleanup_loop(self):
        while not self.done:
            for (seshname, seshdata) in self.sesh2data.items():
                t_now = time.time()
                if t_now - seshdata["last_used"] > MAX_IDLE_TIME:
                    print "deleting idle session %s"%seshname
                    seshdata["env"].Destroy()
                    del self.sesh2data[seshname]                
            time.sleep(CLEANUP_PERIOD)


def main():
    server = SimpleXMLRPCServer(('localhost', 9000), logRequests=True, allow_none=True)
    manager = TrajoptManager()
    server.register_introspection_functions()
    # server.register_instance(manager)

    cleanup_thread = threading.Thread(target = manager._cleanup_loop)
    cleanup_thread.start()
    try:
        print 'Use Control-C to exit'
        server.serve_forever()
    except KeyboardInterrupt:
        manager.done = True
        cleanup_thread.join()
        print 'Exiting'
        
main()