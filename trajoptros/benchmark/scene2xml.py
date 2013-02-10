

def scene2xml(s):
  lines = s.split('\n')
  it = iter(lines)
  out = """<Environment>\n"""

  
  title = it.next()
  while True:    
    name = it.next()
    if name == ".": break
    out += """<KinBody name="%s">\n"""%name.split()[1]
    out += """<Body type="static">\n"""
    geomcount = it.next()
    geomtype = it.next()
    assert geomtype == "box"
    out += """<Geom type="%s">\n"""%geomtype
    extents = map(float, it.next().split())
    out += """<extents>%s</extents>\n"""%(" ".join([str(a/2.) for a in extents]))
    translation = it.next()
    out += """<translation>%s</translation>\n"""%translation
    quat = it.next()
    xyzw = quat.split()
    out += """<quat>%s %s %s %s</quat>\n"""%(xyzw[3], xyzw[0], xyzw[1], xyzw[2])
    out += """</Geom>\n"""
    out += """</Body>\n"""
    out += """</KinBody>\n"""
    idunno = it.next()
  out += """</Environment>"""
  return out;

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("infile")
parser.add_argument("outfile")
args = parser.parse_args()

with open(args.infile,"r") as fh:
  sin = fh.read()
sout = scene2xml(sin)
with open(args.outfile,"w") as fh:
  sout = fh.write(sout)