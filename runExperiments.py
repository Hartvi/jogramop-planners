#!/usr/bin/env python3

import sys, os

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
#planners["rbt"] = "-planner 0 " #blind RTB: no goal bias, no goal-steer
#planners["rrt"] = "-planner 2 -d_crit 100000 "  #rrt, no goal bias

#planners["jrbt"] = "-planner 1 " #burs rrt + j+ expand
planners["jrrt"] = "-planner 3 -d_crit 100000 " #rrt, alternating random expansion + goal bias 
#planners["jrrt"] = "-planner 3 -d_crit 100000 " #rrt, alternating random expansion + goal bias 
#planners["ikrbt"] = "-planner 4 "   #goal is IK solution, goes to only single goal
#planners["ikrrt"] = "-planner 5 -d_crit 100000 "  #goal is IK solution, goes to only single goal
# planners["jrbtNew"] = "-planner 8 -epsilon_q 0.2 -preheat_ratio 0.2 " #burs rrt + j+ expand

fout = open("all-cmds.sh", "wt")
    


rrtSize = 40 * 1000
distanceToGoal = 15.0  #should be 0.05!!
dcrit = 0.11
dcrit = 0.05

goalBiasRadius = 0.2
goalBiasProbability = 0.8 #goal bias neer the goal, should be larger than goalBiasProbability2
prob_steer = 0.01 #steer
goalBiasProbability = 0.7 #goal bias neer the goal, should be larger than goalBiasProbability2
goalBiasProbability2 = 0.01
    
#urdfFile = "jogramop/robots/franka_panda/mobile_panda.urdf"
#urdfFile = "jogramop/robots/franka_panda/mobile_panda_fingers.urdf"
urdfFile = "jogramop/robots/franka_panda/mobile_panda_fingersSmallMesh.urdf"
seed = 1
"""
for sprob in [0.05, 0.1, 0.2, 0.3 ]:
    for gbr in [0.1, 0.2, 0.3, 0.5]:
        for gbprob in [0.1, 0.2, 0.7, 0.9 ]:
            planners["jrbt-steer{}-gbr{}-gprob{}".format(sprob,gbr,gbprob)] = " -planner 1 -goal_bias_radius {} -prob_steer {} -goal_bias_prob {} ".format(gbr,sprob,gbprob) 
"""
# urdfFile = "/home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/jogramop/robots/franka_panda/mobile_panda_fingersSmallMesh.urdf"
seed = -1
preheat_ratio=0.2


for scenario in range(1,12+1):
    scenarioDir = "jogramop/scenarios/{:03d}/export/".format(scenario)
    obstacleFile = "{}/obstacles.obj".format(scenarioDir)
    startFile = "{}/robot_start_conf.csv".format(scenarioDir)
    ikFile = "{}/grasp_IK_solutions.csv".format(scenarioDir)
    graspFile = "{}/grasps.csv".format(scenarioDir)

    
    for planner in planners:
        resultsDir = "{}/{}/{}".format(outputDir, scenario, planner)
        os.system("mkdir -p {}".format(resultsDir))

        for iteration in range(80):
            graspConfigurations = loadGrasps(ikFile)

            if planner == "jrbt":
                graspConfigurations = [1]; #it's not in fact used for this planner, this just forces that jrbt planner runs only once

            for ikindex in range(len(graspConfigurations)):
                if planner != "ikrrt" and planner != "ikrbt" and ikindex > 0:
                    break
                if ikindex > 0:
                    break

                outFile = "{}/out-{:03d}-{:03d}".format(resultsDir,ikindex,iteration)

                if os.path.isfile(outFile + ".txt"):
                    print("Result ", outFile, " finished ")
                    continue

                cmd = "timeout 30s ./build/burs_of_free_space test "
                cmd += " -grasp {} -urdf {} -obstacle {} -start_config {}".format(graspFile, urdfFile, obstacleFile, startFile)
                cmd += " -delta_q 3.3 -epsilon_q 0.1 -num_spikes 7  "
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
                cmd += planners[ planner ]  #when some cmdline option is repearing, the last one is accepted, so this line must be last in cmd
                seed += 1
                doBreak = True
                if planner == "ikrrt" or planner == "ikrbt":
                    cmd += "-target_configs {} ".format(ikFile)
                    cmd += "-ik_index {} ".format(ikindex)
                    doBreak = False

                fout.write("{} > {}.stdout \n".format( cmd, outFile ) )
                if doBreak:
                    break


fout.close()        
print("Commands written to all-cmds.sh")    
                
