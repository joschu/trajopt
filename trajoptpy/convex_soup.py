import scipy.spatial
import numpy as np
import openravepy as rave
import itertools
import cloudprocpy

def calc_hull(points):
    if len(points) < 3: return points, np.zeros((0,3), int)
    elif len(points) == 3: return points, np.array([[0,1,2]], dtype=int)
    elif len(points) == 4: return points, np.array(list(itertools.combinations(range(4),3)), dtype=int)
    else:
        pertpts = points
        while True:
            try:
                delaunay = scipy.spatial.Delaunay(points)
                return delaunay.points, delaunay.convex_hull
            except Exception:
                print "qhull error, adding noise"
                pertpts = points + np.random.randn(*origpts.shape)*.001

def create_convex_soup(cloud, env, name = "convexsoup"):
    xyz = cloud.to2dArray()
    indss = cloudprocpy.convexDecomp(cloud, .02)
    geom_infos = []
    for (i,inds) in enumerate(indss):
        if len(inds) < 10: continue # openrave is slow with a lot of geometries

        origpts = xyz[inds]
        hullpoints, hullinds = calc_hull(origpts)
            
        gi = rave.KinBody.GeometryInfo()
        gi._meshcollision = rave.TriMesh(hullpoints, hullinds)
        gi._type = rave.GeometryType.Trimesh
        gi._vAmbientColor = np.random.rand(3)/2
        geom_infos.append(gi)

        body = rave.RaveCreateKinBody(env,'')
        body.SetName(name)
        body.InitFromGeometries(geom_infos)
    env.Add(body)
