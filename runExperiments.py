#!/usr/bin/env python3

import sys, os, glob

#generate all-cmds.sh that contains individual commands to run the experiments
#you can run them: parallel < all-cmds.sh
#results will be in "results" directory

def loadGrasps(filename):
    fi = open(filename,"rt")
    lines = fi.readlines()
    fi.close()
    return lines

outputDir = "results"

planners = {}

planners["jrrt"] = "-planner 3 -d_crit 100000 " #rrt, alternating random expansion + goal bias 
planners["ikrrt"] = "-planner 5 -d_crit 100000 "  #goal is IK solution, goes to only single goal
#planners["jrbtNew"] = "-planner 8 -epsilon_q 0.1 -preheat_ratio 0.0 " #burs rrt + j+ expand

fout = open("all-cmds.sh", "wt")
    


rrtSize = 200 * 1000
distanceToGoal = 60.0  #should be 0.05!!
dcrit = 0.11
dcrit = 0.05

goalBiasRadius = 25
goalBiasProbability = 0.5 #goal bias neer the goal
prob_steer = 0.01 #steer
goalBiasProbability = 0.7 #goal bias neer the goal
    
#urdfFile = "jogramop/robots/franka_panda/mobile_panda.urdf"
#urdfFile = "jogramop/robots/franka_panda/mobile_panda_fingers.urdf"
urdfFile = "jogramop/robots/franka_panda/mobile_panda_fingersSmallMesh.urdf"
seed = 1



# urdfFile = "/home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/jogramop/robots/franka_panda/mobile_panda_fingersSmallMesh.urdf"
seed = -1
preheat_ratio=0.0

nameScenarios = {}
for scene in glob.glob("jogramop/scenarios/*"):
    scenename = scene.replace("jogramop/scenarios/", "")
    nameScenarios[ scenename ] = True
print("nameScenarios", nameScenarios)
nameScenarios = list(nameScenarios.keys())
nameScenarios.sort()

for scenario in nameScenarios:
    scenarioDir = "jogramop/scenarios/{}/export/".format(scenario)
    obstacleFile = "{}/obstacles.obj".format(scenarioDir)
    startFile = "{}/robot_start_conf.csv".format(scenarioDir)
    ikFile = "{}/grasp_IK_solutions.csv".format(scenarioDir)
    graspFile = "{}/grasps.csv".format(scenarioDir)

    
    for planner in planners:
        resultsDir = "{}/{}/{}".format(outputDir, scenario, planner)
        os.system("mkdir -p {}".format(resultsDir))

        for iteration in range(80):
            graspConfigurations = loadGrasps(ikFile)
            if len(graspConfigurations) == 0:
                graspConfigurations = [1]

            for ikindex in range(len(graspConfigurations)):
                if ikindex > 0 and planner == "jrrt":
                    break;

                outFile = "{}/out-{:03d}-{:03d}".format(resultsDir,ikindex,iteration)

                if os.path.isfile(outFile + ".txt"):
                    print("Result ", outFile, " finished ")
                    continue

                cmd = "timeout 60s ./burs_of_free_space test "
                cmd += " -grasp {} -urdf {} -obstacle {} -start_config {}".format(graspFile, urdfFile, obstacleFile, startFile)
                cmd += " -delta_q 3.14 -epsilon_q 0.1 -num_spikes 4  "
                cmd += " -render 0 -vis_script scripts/animate_scene.py -cx -1 -cy 3 -cz 6 -groundLevel 0.00 -minColSegIdx 6 "
                cmd += " -target_prefix {} ".format(outFile)
                cmd += " -d_crit {} ".format(dcrit)
                cmd += " -max_iters {} ".format(rrtSize)
                cmd += " -p_close_enough {} ".format(distanceToGoal)
                cmd += " -q_resolution 0.1 "
                cmd += " -goal_bias_radius {} ".format(goalBiasRadius)
                cmd += " -goal_bias_prob {} ".format(goalBiasProbability)
                cmd += " -prob_steer {} ".format(prob_steer)
                cmd += " -seed {} ".format(seed)
                cmd += " -preheat_ratio  {} ".format(preheat_ratio)
                cmd += " -preheat_type 0 "
                cmd += " -use_rot 100 "
                cmd += planners[ planner ]  #when some cmdline option is repearing, the last one is accepted, so this line must be last in cmd
                cmd += " -target_configs {} ".format(ikFile)
                cmd += " -bias_calculation 0 "
                cmd += " -ik_index {} ".format(ikindex)
                seed += 1
                fout.write("{} > {}.stdout \n".format( cmd, outFile ) )


fout.close()        
print("Commands written to all-cmds.sh")    
                
