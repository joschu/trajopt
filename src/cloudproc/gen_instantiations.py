#!/usr/bin/env python
import re
print "generating template instantiations"
with open("cloudproc.hpp","r") as infile:
    with open("autogen_instantiations.cpp","w") as outfile:
        while True:
            line = infile.readline()
            if not line: break
            if line.startswith("template"):
                funcline = infile.readline()
                if ";" not in funcline: continue # skip inline templates 
                if "disable_autogen" in funcline: continue
                if "instantiate:" in funcline:
                    types = [s.strip() for s in funcline.split("instantiate:")[1].split()]
                else:
                    types = ["pcl::PointXYZ", "pcl::PointXYZRGB", "pcl::PointNormal"]
                funcname = re.findall("(\w+)\(",funcline)[0]
                funcsig = funcline.split(";")[0]
                funcsig = funcsig.replace("typename","")
                funcsig = funcsig.replace("TRAJOPT_API","")
                funcsig = funcsig.replace(funcname, "%s<T>"%funcname)
                for type in types:
                    funcsig_specialized = funcsig.replace("<T>","<%s>"%type)
                    outline = "template %s;\n"%(funcsig_specialized)
                    outfile.write(outline)
        

                # outfile.write("PCL_INSTANTIATE(%s, CLOUDPROC_POINT_TYPES);\n"%(funcname))        
        
        