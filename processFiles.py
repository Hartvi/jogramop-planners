#!/usr/bin/env python3

import glob, sys, re
import matplotlib.pyplot as plt

if len(sys.argv) != 3:
    print("usage: ", sys.argv[0], " <resultDir> <prefix> ")
    quit()
    
    
plt.rcParams["figure.figsize"] = (12,8)

resultDir = sys.argv[1].replace("/","")
prefix = sys.argv[2]

DTG = 0.025

def getSRCurve(values, distances):
    if len(values) == 0 or len(distances) == 0:
        return []

    minValue = min(values)
    maxValue = max(values)

    steps = 100
    delta = (maxValue - minValue)/steps

    value = minValue
    srCurve = []  #each value is [value, sucess ratio]
    while value <= maxValue:
        finishedRuns = [ values[i] for i in range(len(values)) if distances[i] <= DTG and values[i] <= value ]
        
        srCurve.append([ value, 100*len(finishedRuns) / len(values) ] )
        value += delta

    return srCurve



results = {}

iks = []


for plannerDir in glob.glob("{}/*/*".format(resultDir)):
    res = re.search(r"{}\/(.*)\/(.*)".format(resultDir), plannerDir)
    if len(res.groups()) != 2:
        print("Cannot parse directory name ", plannerDir)
        quit()
    scenarioNum = res.group(1)
    plannerName = res.group(2)

    files = glob.glob("{}/out-*-*.txt".format(plannerDir))
    allLines = []
    for fn in files:
        res = re.search(r"out-(\d+)-(\d+)\.txt", fn)
        if len(res.groups()) != 2:
            print("Cannot read ikindex and iteration from ", fn)
            quit()
        ikindex = int(res.group(1))
        trial = int(res.group(2))
        iks.append( ikindex )
        with open(fn, "rt") as f:
            for line in f:
                allLines.append( line.strip().replace(","," ") )
                allLines[-1] += " {} {}".format(ikindex, trial)

    if len(allLines) == 0:
        continue
    #each measurement is [finished, distanceToGoal, time ,treeSize ]
    print("Loaded ", len(allLines), "lines from", len(allLines), "files in", plannerDir)
    mes = [ list(map(float, line.split() ) ) for line in allLines ]
    if not scenarioNum in results:
        results[ scenarioNum ] = {}
    results[scenarioNum][plannerName] = mes

print("Largest tested ikindex ", max(iks) )


for scenario in results:
    plt.clf()
    plt.xlabel("time [s]")
    plt.ylabel("sucess rate")
    srCurves = []

    for planner in results[scenario]:

        measurement = results[scenario][planner]

        times = [ measurement[i][2] for i in range(len(measurement)) ]
        distances = [ measurement[i][1] for i in range(len(measurement)) ]

        srCurve = getSRCurve( times, distances )

        xvals = [ item[0] for item in srCurve ]
        yvals = [ item[1] for item in srCurve ]

        if len(srCurve) != 0:
            srCurves.append([planner, srCurve])

        line = plt.plot( xvals, yvals, label="{}".format(planner) )
    plt.legend()
    outfile = "{}-scenario-{}.png".format(prefix, scenario)
    plt.savefig("{}.png".format(outfile))

    def val(x):
        return x[1][-1][1]

    srCurves.sort( key=val, reverse=True)
    plt.clf()

    plt.xlabel("time [s]")
    plt.ylabel("sucess rate")
    for i in range(min(5, len(srCurves))):
        srCurve = srCurves[i][1]
        planner = srCurves[i][0]
        xvals = [ item[0] for item in srCurve ]
        yvals = [ item[1] for item in srCurve ]
        line = plt.plot( xvals, yvals, label="{}".format(planner) )
    plt.legend()

    outfile = "{}-scenario-{}-best.png".format(prefix, scenario)
    plt.savefig("{}.png".format(outfile))






for scenario in results:
    plt.clf()
    plt.xlabel("ikindex [-]")
    plt.ylabel("sr")


    for planner in results[scenario]:
        if planner == "ikrrt" or planner=="ikrbt":
            pass
        else:
            continue
        pts = []
        for ikindex in range(max(iks)):
            measurement = results[scenario][planner]
            print("measurement", measurement)

            times = [ measurement[i][2] for i in range(len(measurement)) if measurement[i][-2] == ikindex ]
            distances = [ measurement[i][1] for i in range(len(measurement)) if measurement[i][-2] == ikindex ]
            print("planner", planner, ikindex, times, distances)
            srCurve = getSRCurve( times, distances )
            if len(srCurve) != 0:
                maxSR = srCurve[-1][1]
                pts.append([ ikindex, maxSR])
            else:
                pts.append([ ikindex, 0 ] )

        xvals = [ item[0] for item in pts ]
        yvals = [ item[1] for item in pts ]
        line = plt.plot( xvals, yvals, label="{}".format(planner) )
    plt.legend()

    outfile = "{}-scenarioIK-SR-{}.png".format(prefix, scenario)

    plt.savefig("{}.png".format(outfile))











