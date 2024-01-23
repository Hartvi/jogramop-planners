#!/usr/bin/env python3
import glob

fout = open("all-vis.sh", "wt")

for directory in glob.glob("results/*"):
    scenario = directory.replace("results/","")
    scenario = int(scenario)
    print(directory, "->", scenario)

    files = glob.glob("{}/*/*.try".format(directory))
    for filename in files:
        cmd = "python3.10 visualizationVojta.py  {} {} -1 -1".format(scenario,filename)
        fout.write(cmd + "\n")
fout.close()    

