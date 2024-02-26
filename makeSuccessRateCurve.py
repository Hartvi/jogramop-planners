

import glob, sys, re, math, json
import matplotlib.pyplot as plt

if len(sys.argv) != 5:
    print("usage: ", sys.argv[0], " <resultDir> <prefix> <maxReportedTime> <distanceToGoal>")
    quit()
    
    
plt.rcParams["figure.figsize"] = (12,8)

resultDir = sys.argv[1].replace("/","")
prefix = sys.argv[2]
MaxReportedTime = float(sys.argv[3])
DTG = float(sys.argv[4])

def getSRCurve(values, distances, maxReportedTime):
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
                    results[key] += [y]

print("allIKS", allIKS)

for planner in sorted(allPlanners):
    for scenePrefix in ["01", "02", "03", "04"]:
        plt.clf()
        plt.xlabel("time [s]")
        plt.ylabel("sucess rate")
        plt.figure(figsize=(10,4))
        #for scenePrefix in allScenarios:
        for i in range(1,6):
            scene = "{}{}".format(scenePrefix, i)
            srCurves = []
            for ik in allIKS:
                key = (scene, planner, ik)
                print("scene", scene, key, key in results)
                if not key in results:
                    continue
                data = results[key]
                distances = [ item["dtg"] for item in data ]
                times = [ item["time"]  for item in data ]
                treesize = [ item["treesize"]  for item in data ]
                if len(times) == 0:
                    continue
                meantime = sum(times) / len(times)
                srCurve = getSRCurve( times, distances, MaxReportedTime )
                srCurves.append( [srCurve, meantime ] )

                srCurve = getSRCurve( treesize, distances, 1e10 ) #for treesizes do not use MaxReportedTime
#                srCurves.append( [srCurve, meantime ] )

            srCurves.sort(key = lambda item: item[1]) #sort by time
            print("Sorted times", [ item[1] for item in srCurves ] )
            if len(srCurves) != 0:
                srCurve = srCurves[0][0]
                xvals = [ item[0] for item in srCurve if item[0] < MaxReportedTime ]
                yvals = [ item[1] for item in srCurve if item[0] < MaxReportedTime ]
                xvals = [ item[0] for item in srCurve  ]
                yvals = [ item[1] for item in srCurve  ]
                line = plt.plot( xvals, yvals, label="{}".format(scene) )
        
#        plt.xlim([0, maxReportedTime])
        plt.legend()
        plt.grid(axis='both', color='0.95')
#        plt.xlabel("Iterations")
        plt.xlabel("Runtime [s]")
        plt.ylabel("Probability of finding solution")
        ikindex = int(ikindex)
        outfile = "{}-scenario{}-planner{}-time.png".format(prefix, scenePrefix, planner) 
        plt.savefig("{}".format(outfile))



