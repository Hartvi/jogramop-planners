#!/usr/bin/env python3

import glob, sys
import matplotlib.pyplot as plt


DTG = 0.015

def getSRCurve(values, distances):
    if len(values) == 0 or len(distances) == 0:
        return []

    minValue = min(values)
    maxValue = max(values)

    steps = 100
    delta = (maxValue - minValue)/steps

    value = minValue
    srCurve = []  #each value is [value, sucess ratio]
    print("EE", len(values), len(distances))


    while value <= maxValue:
        finishedRuns = [ values[i] for i in range(len(values)) if distances[i] <= DTG and values[i] <= value ]
        
        srCurve.append([ value, 100*len(finishedRuns) / len(values) ] )
        value += delta

    return srCurve



fognuplot = open("res.gpl", "wt")
fognuplot.write("set term png size 1024,768\n")
fognuplot.write("set border 3\n")


for experimentDir in glob.glob("results/*"):

    files = glob.glob("{}/*.txt".format(experimentDir))
    outfile = experimentDir.replace("results/","")

    allLines = []
    for fn in files:
        with open(fn, "rt") as f:
            for line in f:
                allLines.append( line.strip().replace(","," ") )
    print("Loaded ", len(allLines), "lines from", len(allLines), "files in", experimentDir)

    #each measurement is [finished, distanceToGoal, time ,treeSize ]
    measurements = [ list(map(float, line.split() ) ) for line in allLines ]

    times = [ measurements[i][2] for i in range(len(measurements)) ]
    distances = [ measurements[i][1] for i in range(len(measurements)) ]

    srCurve = getSRCurve( times, distances )

    xvals = [ item[0] for item in srCurve ]
    yvals = [ item[1] for item in srCurve ]
    plt.clf()
    plt.plot(xvals, yvals)
    plt.xlabel("time [s]")
    plt.ylabel("sucess rate")
    plt.savefig("{}.png".format(outfile))

    """

    fo = open("res-{}.txt".format(outfile), "wt")
    for item in srCurve:
        fo.write("{} {}\n".format(item[0], item[1]) )
    fo.close()

    fognuplot.write("set output '{}.png'\n".format(outfile))
    fognuplot.write("plot 'res-{}.txt' u 1:2 w l lw 4 \n".format(outfile))
    fognuplot.write("set output\n\n")
    """
fognuplot.close()











