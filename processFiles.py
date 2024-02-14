

import glob, sys, re, math, json
import matplotlib.pyplot as plt

if len(sys.argv) != 4:
    print("usage: ", sys.argv[0], " <resultDir> <prefix> <maxReportedTime> ")
    quit()
    
    
plt.rcParams["figure.figsize"] = (12,8)

resultDir = sys.argv[1].replace("/","")
prefix = sys.argv[2]
maxReportedTime = float(sys.argv[3])
 

DTG = 0.07



def getSRCurve(values, distances):
    if len(values) == 0 or len(distances) == 0:
        return []

    minValue = min(values)
    maxValue = max(values)

    steps = 100
    delta = (maxValue - minValue)/steps

    if minValue == maxValue:
        return []

    value = minValue
    srCurve = []  #each value is [value, sucess ratio]
    while value <= maxValue:
        finishedRuns = [ values[i] for i in range(len(values)) if distances[i] <= DTG and values[i] <= value ]
        if maxReportedTime > 0 and value <= maxReportedTime:
            successRatio = len(finishedRuns) / len(values)
            srCurve.append([ value, 100*successRatio ] )

        value += delta

    return srCurve

def areaUnderCurve(srCurve, maxXvalue):
    area = 0
    for i in range(len(srCurve)-1):
        if srCurve[i][0] < maxXvalue:
            area += abs(srCurve[i][0]-srCurve[i-1][0]) * (srCurve[i][1] + srCurve[i-1][1])/2
    if srCurve[-1][0] <= maxXvalue:
        area += abs(maxXvalue-srCurve[-1][0]) * (srCurve[-1][1])

    return area        

def rankSRCurve(srCurve):
    mind = 1e6
    for item in srCurve:
        d= math.sqrt(item[0]**2 + (1-item[1])**2)
        if d < mind:
            mind = d
    return mind


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
    filesAll = glob.glob("{}/out-*-*.stdout".format(plannerDir))
#    allLines = []
    allMeasurements = []
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
                
                if line.find("{") != -1 and line.find("}") != -1:
                    y = json.loads(line)
                    if y["time"] < 1e-3 or y["treesize"] == 0:
                        y["time"] = 10*maxReportedTime
                        y["dtg"] = 1e10

                    print("Loading ", line, "->", y)
                    y["ikindex"] = ikindex
                    y["trial"] = trial
                    allMeasurements += [y]

    if len(allMeasurements) == 0:
        continue
    print("Loaded ", len(allMeasurements), "lines from", len(files), "files in", plannerDir)
    #each measurement is [finished, distanceToGoal, time ,treeSize ]
    #mes = [ list(map(float, line.split() ) ) for line in allLines ]
    if not scenarioNum in results:
        results[ scenarioNum ] = {}
    results[scenarioNum][plannerName] = allMeasurements


print("Largest tested ikindex ", max(iks) )

tableData = {}
usedPlanners = {}

for scenario in results:
    plt.clf()
    plt.xlabel("time [s]")
    plt.ylabel("sucess rate")
    srCurves = []

    tableData[scenario] = {}

    for planner in results[scenario]:
        usedPlanners[planner] = 1

        tableData[scenario][planner] = {}
        tableData[scenario][planner]["timeavg"] = None
        tableData[scenario][planner]["timedev"] = None
        tableData[scenario][planner]["sucess"] = None


        data = results[scenario][planner]  #data is list of hash 
        distances = [ item["dtg"] for item in data ]
        times = [ item["time"]  for item in data ]
        srCurve = getSRCurve( times, distances )

        xvals = [ item[0] for item in srCurve if item[0] < maxReportedTime ]
        yvals = [ item[1] for item in srCurve if item[0] < maxReportedTime ]

        if len(times) > 0:
            avgTime = sum(times) /len(times)
            stdDev = sum( [ (i-avgTime)**2 for i in times ] ) / len(times)
            stdDev = stdDev**(0.5)
            tableData[scenario][planner]["timeavg"] = avgTime
            tableData[scenario][planner]["timedev"] = stdDev

        if len(distances) > 0:
            goodTrials = [ 1 for item in distances if item <= DTG ]
            sr = len(goodTrials ) / len(distances)
            tableData[scenario][planner]["sucess"] = 100*sr
            
        if len(srCurve) != 0:
            srCurves.append([planner, srCurve])

        line = plt.plot( xvals, yvals, label="{}".format(planner) )
    plt.legend()
    outfile = "{}-scenario-{}.png".format(prefix, scenario)
    plt.savefig("{}.png".format(outfile))

    def val(x):
        return x[1][-1][1]

    def val2(x):
        return areaUnderCurve(x[1], maxReportedTime)

    def val3(x):
        return rankSRCurve(x[1])

    srCurves.sort( key=val3, reverse=True)
    print("num curves", len(srCurves))
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


    #plot worse ones
    srCurves.sort( key=val3, reverse=False)
    print("num curves", len(srCurves))
    plt.clf()

    plt.xlabel("time [s]")
    plt.ylabel("sucess rate")
    for i in range(min(20, len(srCurves))):
        srCurve = srCurves[i][1]
        planner = srCurves[i][0]
        xvals = [ item[0] for item in srCurve ]
        yvals = [ item[1] for item in srCurve ]
        line = plt.plot( xvals, yvals, label="{}".format(planner) )
    plt.legend()

    outfile = "{}-scenario-{}-worst.png".format(prefix, scenario)
    plt.savefig("{}.png".format(outfile))



#making table
fot = open("table.tex", "wt")
fot.write("\\begin{tabular}{llccc}\n")
fot.write("\\toprule\n")
fot.write("Scenario & Planner & Runtime avg & Runtime dev & Sucess \\\\ \n ")
for scenario in results:
    fot.write("\\midrule\n")
    fot.write("\\multirow{" + str(len(usedPlanners)) + "}{*}{" + str(scenario) + "} \n ")
    for planner in results[scenario]:

        timeavg = tableData[scenario][planner]["timeavg"]
        timedev = tableData[scenario][planner]["timedev"] 
        sucess = tableData[scenario][planner]["sucess"] 

        timeavg = "{:.2f}".format( timeavg ) if timeavg != None else "N/A"
        timedev = "{:.2f}".format( timedev ) if timeavg != None else "N/A"
        sucess = "{:.2f}".format( sucess ) if timeavg != None else "N/A"


        fot.write(" & {} & {} & {} & {}  \\\\ \n".format( planner, timeavg, timedev, sucess) )
fot.write("\\end{tabular}\n\n")
fot.close()


"""
for scenario in results:
    break
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

"""






