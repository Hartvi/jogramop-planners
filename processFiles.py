

import glob, sys, re, math, json
import matplotlib.pyplot as plt

if len(sys.argv) != 4:
    print("usage: ", sys.argv[0], " <resultDir> <prefix> <maxReportedTime> ")
    quit()
    
    
plt.rcParams["figure.figsize"] = (12,8)

resultDir = sys.argv[1].replace("/","")
prefix = sys.argv[2]
MaxReportedTime = float(sys.argv[3])
 

DTG = 60

#NORUNTIME = 120 for all except 045
NORUNTIME = 900 #for 045



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
                        y["time"] = NORUNTIME
                        y["dtg"] = 1e10

                    print("Loading ", line, "->", y)
                    y["ikindex"] = ikindex
                    y["trial"] = trial
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
        outfile = "{}-scenario{}-planner{}-time.eps".format(prefix, scenePrefix, planner) 
        plt.savefig("{}".format(outfile))



def getNumTargets(scenario): #scenario is "011", "021" etc
    """ return number of ik-solutions for grasp and number of grasps """                              
    scenarioDir = "jogramop/scenarios/{}/export/".format(scenario)
    obstacleFile = "{}/obstacles.obj".format(scenarioDir)
    startFile = "{}/robot_start_conf.csv".format(scenarioDir)
    ikFile = "{}/grasp_IK_solutions.csv".format(scenarioDir)
    graspFile = "{}/grasps.csv".format(scenarioDir)
    f = open(ikFile, "rt")
    numIk = 0
    numGrasps = 0
    with open(ikFile, "rt") as f:
        for line in f:
            if len(line) > 2:
                numIk+=1
    with open(graspFile, "rt") as f:
        for line in f:
            if len(line) > 2:
                numGrasps+=1
    return numIk, numGrasps

def summary(key):
    if not key in results:
        print("Cannot find key!", key)
        quit()
    data = results[key]
    distances = [ item["dtg"] for item in data ]
    times = [ item["time"]  for item in data]
#    times = [ item for item in times if item != NORUNTIME ]
    print("runtimes", key, ",", times)

    avgTime = None
    stdDev = None
    sr = None
    if len(times) > 0:
        avgTime = sum(times) /len(times)
        stdDev = sum( [ (i-avgTime)**2 for i in times ] ) / len(times)
        stdDev = stdDev**(0.5)

    if len(distances) > 0:
        goodTrials = [ 1 for item in distances if item <= DTG ]
        sr = 100*len(goodTrials ) / len(distances)
    print("RES", avgTime, sr)                
    return avgTime, stdDev, sr


print("allIKS", allIKS)

fot = open("table.tex", "wt")
fot.write("\\begin{tabular}{lccll}\n")
fot.write("\\toprule\n")
fot.write(" \\multicolumn{1}{c}{\\bf Scenario } & {\\bf \\#IKS } & {\\bf \\#GP} & \\multicolumn{1}{c}{{\\bf JRRT }} & \\multicolumn{1}{c}{{\\bf IKRRT }} \\\\ \n")

scenes = list(allScenarios.keys())
scenes.sort()
for scene in scenes:
    if scene[-1] == "1":
        fot.write("\\midrule\n")

    numik, numgrasps = getNumTargets(scene)

    ikValues = []
    for ik in range(numik+1):
        key = (scene, "ikrrt", ik)
        if not key in results:
            continue
        tavg,timedev,sr = summary(key)
        if tavg != None:
            ikValues += [ [tavg, timedev, sr, ik ] ]
    if numik == 0:
        ikValues = []            
    print("ikval", ikValues)            
    #best time

    fot.write("{\\bf " + str(scene) + "} & " + str(numik) + "/" + str(len(ikValues)) + " & " + str(numgrasps) )
    
    key = (scene, "jrrt", 0)
    if not key in results:
        continue
    tavg,timedev,sr = summary(key)

    #write time line
    if tavg != None:
        fot.write("  & {}\%: ${:.2f}/{:.2f}$  ".format(int(sr),tavg,timedev))
    else:
        fot.write("  & ---   ")


    ikValues.sort(key=lambda item: item[0])

    if len(ikValues) > 0:
        fot.write(" &  {}\%  : {:.2f}/{:.2f}".format(int(ikValues[0][2]), ikValues[0][0], ikValues[0][1],  ))
    else:
        fot.write(" & --- " )
    fot.write("\\\\ \n ")

fot.write("\n\\bottomrule\n")
fot.write("\\end{tabular}\n")
fot.close()



