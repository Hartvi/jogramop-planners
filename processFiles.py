

import glob, sys, re, math, json
import matplotlib.pyplot as plt

if len(sys.argv) != 4:
    print("usage: ", sys.argv[0], " <resultDir> <prefix> <maxReportedTime> ")
    quit()
    
    
plt.rcParams["figure.figsize"] = (12,8)

resultDir = sys.argv[1].replace("/","")
prefix = sys.argv[2]
maxReportedTime = float(sys.argv[3])
 

DTG = 70

def val(x):
    return x[1][-1][1]

def val2(x):
    return areaUnderCurve(x[1], maxReportedTime)

def val3(x):
    return rankSRCurve(x[1])



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
allScenarios = {}
allPlanners = {}
allIKS = {}

for plannerDir in glob.glob("{}/*/*".format(resultDir)):
    res = re.search(r"{}\/(.*)\/(.*)".format(resultDir), plannerDir)
    if len(res.groups()) != 2:
        print("Cannot parse directory name ", plannerDir)
        quit()
    scenarioNum = res.group(1)
    plannerName = res.group(2)

    files = glob.glob("{}/out-*-*.txt".format(plannerDir))
    filesAll = glob.glob("{}/out-*-*.stdout".format(plannerDir))
    for fn in files:
        res = re.search(r"out-(\d+)-(\d+)\.txt", fn)
        if len(res.groups()) != 2:
            print("Cannot read ikindex and iteration from ", fn)
            quit()
        ikindex = int(res.group(1))
        trial = int(res.group(2))
        allScenarios[scenarioNum] = 1
        allPlanners[plannerName] = 1
        allIKS[ikindex] = 1

        key = (scenarioNum, plannerName, ikindex)

        if not key in results:
            results[key] = []
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
                    results[key] += [y]


for key in results:
    scenarioNum, plannerName, ikindex = key

    plt.clf()
    plt.xlabel("time [s]")
    plt.ylabel("sucess rate")
    srCurves = []

    data = results[key]
    distances = [ item["dtg"] for item in data ]
    times = [ item["time"]  for item in data ]
    srCurve = getSRCurve( times, distances )

    xvals = [ item[0] for item in srCurve if item[0] < maxReportedTime ]
    yvals = [ item[1] for item in srCurve if item[0] < maxReportedTime ]

    if len(times) > 0:
        avgTime = sum(times) /len(times)
        stdDev = sum( [ (i-avgTime)**2 for i in times ] ) / len(times)
        stdDev = stdDev**(0.5)

    if len(distances) > 0:
        goodTrials = [ 1 for item in distances if item <= DTG ]
        sr = len(goodTrials ) / len(distances)
            
    if len(srCurve) != 0:
        srCurves.append([plannerName, srCurve])

    line = plt.plot( xvals, yvals, label="{}".format(plannerName) )

        
    plt.legend()
    scenario = int(scenarioNum)
    ikindex = int(ikindex)
    outfile = "{}-scenario{:02d}-ik{:02d}-{}.png".format(prefix, scenario, ikindex, plannerName)
    plt.savefig("{}.png".format(outfile))


def summary(key):
    data = results[key]
    distances = [ item["dtg"] for item in data ]
    times = [ item["time"]  for item in data ]
    srCurve = getSRCurve( times, distances )

    xvals = [ item[0] for item in srCurve if item[0] < maxReportedTime ]
    yvals = [ item[1] for item in srCurve if item[0] < maxReportedTime ]
    avgTime = None
    stdDev = None
    sr = None
    if len(times) > 0:
        avgTime = sum(times) /len(times)
        stdDev = sum( [ (i-avgTime)**2 for i in times ] ) / len(times)
        stdDev = stdDev**(0.5)

    if len(distances) > 0:
        goodTrials = [ 1 for item in distances if item <= DTG ]
        sr = len(goodTrials ) / len(distances)
                
    return avgTime, stdDev, sr




fot = open("table.tex", "wo")
fot.write("\\begin{tabular}{lccccc}\n")
fot.write("\\toprule\n")
fot.write(" scene & jrrt & \\multicol{4}{c}{ikrrt} \\\\ \n")

scenes = list(allScenarios.keys())
scenes.sort()
for scene in scenes:
    fot.write("{\b " + str(scene) + "} & ")
    
    key = (scene, "jrrt", 0)
    if not key in results:
        print("Cannot find key", key)
        quit()


