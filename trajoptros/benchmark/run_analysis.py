

import os
import h5py
import cPickle
import numpy as np
from numpy import asarray

def add_pkldata_to_h5(pklfile, h5obj):
    with open(os.path.join("../pklresults",pklfile),"r") as fh: (scene, planner, results) = cPickle.load(fh)
    success_count = 0
    fail_count = 0
    path_length_sum = 0
    
    path_lengths = []
    solved = []
    plantimes = []
    
    for (i,result) in enumerate(results):
        if result["returned"] and result["error_code"]==1:# and (result["safe"] or "chomp" not in planner): # we only have to check the safety of chomp trajectories
            solved.append(True)
            traj = np.asarray(result["traj"])
            path_lengths.append(np.abs(traj[1:] - traj[:-1]).sum())
            plantimes.append(result["planning_time"])
        else:
            solved.append(False)
            path_lengths.append(np.inf)
            plantimes.append(np.inf)
            
    
    h5path = "../results/results.h5"
    f = h5py.File(h5path,"r+" if os.path.exists(h5path) else "w")
    if scene not in f:
        f.create_group(scene)
    gscene = f[scene]
    if planner not in gscene:
        gscene.create_group(planner)
    gplanner = gscene[planner]
    gplanner["path_lengths"] = path_lengths
    gplanner["solved"] = solved
    gplanner["plantimes"] = plantimes    




h5path = "../results/results.h5"
if os.path.exists(h5path): os.unlink(h5path)
h5file = h5py.File(h5path, "w")


for fname in os.listdir("../pklresults"):
    add_pkldata_to_h5(fname, h5file)
    
    
table_cols = ["LBKPIE", "RRTCon", "RRTkCo", "SBLkCo", "chomp","trajopt_disc_single", "trajopt_disc_multi", "trajopt_single", "trajopt_multi"]
from collections import defaultdict

scene2tabledata = {}

total_probs = 0
for (namescene, groupscene) in h5file.items():
    
    tabledata = scene2tabledata[namescene] = {}
    
    numsolvers = None
    minplantimes = None
    minpathlengths = None
    for (nameplanner, groupplanner) in groupscene.items():
        if numsolvers is not None: numsolvers += np.asarray(groupplanner["solved"]).astype('int')
        else: numsolvers = groupplanner["solved"]
        plantimes = groupplanner["plantimes"]
        if minplantimes is not None: minplantimes = np.fmin(minplantimes, groupplanner["plantimes"])
        else: minplantimes = plantimes
        path_lengths = groupplanner["path_lengths"]
        if minpathlengths is not None: minpathlengths = np.fmin(minpathlengths, groupplanner["path_lengths"])
        else: minpathlengths = path_lengths
    print "scene: %s. %i/%i solved by somebody"%( namescene, asarray(numsolvers).sum(),len(numsolvers))
    goodprobinds = np.flatnonzero(asarray(numsolvers) > 0)
    
    for nameplanner in table_cols:
        groupplanner = groupscene[nameplanner]
        solvedinds = asarray(groupplanner["solved"])
        nsolved = asarray(groupplanner["solved"]).sum() / (len(goodprobinds) + 0.)
        print "planner %s scene %s. num solvable problems solved: %i/%i"%(nameplanner, namescene, nsolved, len(goodprobinds))
        avgnormedtime = (asarray(groupplanner["plantimes"])/minplantimes)[solvedinds].mean()
        print "avg time/best: %.3f"%( avgnormedtime )
        avgnormedlength = (asarray(groupplanner["path_lengths"])/minpathlengths)[solvedinds].mean()
        print "avg length/best: %.3f"%( avgnormedlength )
        avgtime = asarray(groupplanner["plantimes"][solvedinds]).mean()
        print "avg time: %.3f"%avgtime
        tabledata[nameplanner] = [nsolved, avgnormedlength, avgtime]
    total_probs += len(goodprobinds)
              
def pp(x):
    np.savetxt("/tmp/table.txt", x,fmt="%.3g")
    with open("/tmp/table.txt","r") as fh:
        x = fh.read()
    return x
    

tablesum = 0
for (scene, tabledata) in scene2tabledata.items():
    import numpy as np
    np.set_printoptions(precision=3)
    table = []
    for planner in table_cols:
        table.append(tabledata[planner])
    table = np.array(table).T
    print "scene",scene
    print table_cols 
    tablesum = tablesum + table
    print pp(table)
    
tablesum /= 3
print pp(tablesum)
print "total num problems", total_probs

def texprint(s):
    lines = s.split("\n")
    out = ""
    for line in lines:
        out += " & ".join(line.split()) + "\\\\ \n"
    return out

print texprint(pp(tablesum))