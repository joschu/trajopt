import argparse
parser = argparse.ArgumentParser()
parser.add_argument("scene")
args = parser.parse_args()
from jds_utils import shell

#OMPL_PLANNERS = ["RRTkConfigDefault", "RRTConnectkConfigDefault", "SBLkConfigDefault", "LBKPIECEkConfigDefault"]
OMPL_PLANNERS = ["RRTConnectkConfigDefault", "LBKPIECEkConfigDefault"]

if "counter" in args.scene: extraargs="--right"
else: extraargs=""
for planner in OMPL_PLANNERS:
    shell.call_and_print("python move_group_battery.py ~/ros/moveit/scenes/%s.scene --planner=%s --max_planning_time=100 --planner_id=%s %s"
                         %(args.scene, planner[:6], planner, extraargs))
