import argparse
parser = argparse.ArgumentParser()
parser.add_argument("scene")
args = parser.parse_args()
import subprocess
import signal
from shell_printing import call_and_print, Popen_and_print


scene = args.scene

if "counter" in scene: extraargs="--right"
else: extraargs=""

pop = subprocess.Popen("python planning_server.py ~/ros/moveit/scenes/%s.scene --discrete"%scene, shell=True)
shell.call_and_print("python move_group_battery.py ~/ros/moveit/scenes/%s.scene --planner=trajopt_disc_single %s"%(scene, extraargs))
print "interupt!"
pop.send_signal(signal.SIGINT)
print "waiting for termination!"
pop.wait()

pop = subprocess.Popen("python planning_server.py ~/ros/moveit/scenes/%s.scene --discrete --multi_init"%scene, shell=True)
shell.call_and_print("python move_group_battery.py ~/ros/moveit/scenes/%s.scene --planner=trajopt_disc_multi %s"%(scene, extraargs))
print "interupt!"
pop.send_signal(signal.SIGINT)
pop.wait()

pop = subprocess.Popen("python planning_server.py ~/ros/moveit/scenes/%s.scene"%scene, shell=True)
shell.call_and_print("python move_group_battery.py ~/ros/moveit/scenes/%s.scene --planner=trajopt_single %s"%(scene, extraargs))
print "interupt!"
pop.send_signal(signal.SIGINT)
pop.wait()

pop = subprocess.Popen("python planning_server.py ~/ros/moveit/scenes/%s.scene --multi_init"%scene, shell=True)
shell.call_and_print("python move_group_battery.py ~/ros/moveit/scenes/%s.scene --planner=trajopt_multi %s"%(scene, extraargs))
print "interupt!"
pop.send_signal(signal.SIGINT)
pop.wait()
