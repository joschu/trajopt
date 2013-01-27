from __future__ import division
import openravepy as rave
import numpy as np

def create_box_from_bounds(env, bounds, name="box"):
  xmin, xmax, ymin, ymax, zmin, zmax = bounds
  xml_str = """
<Environment>
  <KinBody name="%s">
    <Body type="static">
      <Geom type="box">
	<Translation> %f %f %f </Translation>
	<extents> %f %f %f </extents>
      </Geom>
    </Body>
  </KinBody>

</Environment>
"""%( name,
      (xmin+xmax)/2, (ymin+ymax)/2, (zmin+zmax)/2,
      (xmax-xmin)/2, (ymax-ymin)/2, (zmax-zmin)/2 )
  
  
  fname = "/tmp/%s.xml"%name
  with open(fname,"w") as fh:
    fh.write(xml_str)
      
  return env.Load(fname)

def create_cylinder(env, center, radius, height, name = "cylinder"):
  xcenter, ycenter, zcenter = center
  xml_str = """
<Environment>
  <KinBody name="%s">
    <Body type="static">
      <Geom type="cylinder">
	<Translation> %f %f %f </Translation>
        <RotationAxis>1 0 0 90</RotationAxis>
	<radius> %f </radius>
        <height> %f </height>
      </Geom>
    </Body>
  </KinBody>

</Environment>
"""%( name,
      xcenter, ycenter, zcenter,
      radius, height )
  
  fname = "/tmp/%s.xml"%name
  with open(fname,"w") as fh:
    fh.write(xml_str)
      
  return env.Load(fname)

def create_mesh_box(env, center, half_extents, name="box"):
    box = rave.RaveCreateKinBody(env, '')
    rx, ry, rz = half_extents
    verts = np.array([
        [-rx, -ry, -rz],
        [-rx, -ry, rz],
        [-rx, ry, -rz],
        [-rx, ry, rz],
        [rx, -ry, -rz],
        [rx, -ry, rz],
        [rx, ry, -rz],
        [rx, ry, rz]])
    faces= [
        [0,1,2],
        [3,1,2],
        [0,1,4],
        [5,1,4],
        [0,2,4],
        [6,2,4],
        [7,6,5],
        [4,6,5],
        [7,6,3],
        [2,6,3],
        [7,5,3],
        [1,5,3]]
    box.SetName(name)
    box.InitFromTrimesh(rave.TriMesh(verts, faces), True)
    trans = np.eye(4)
    trans[:3,3] = center
    box.SetTransform(trans)
    env.Add(box)
    return box

