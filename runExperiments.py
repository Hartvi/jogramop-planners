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



planners = {}
#planners["rbt"] = "-planner 0 "
planners["jrbt"] = "-planner 1 "
#planners["rrt"] = "-planner 2 -d_crit 100000 "
#planners["jrrt"] = "-planner 3 -d_crit 100000 "

#planners["ikrbt"] = "-planner 4 "
#planners["ikrrt"] = "-planner 5 -d_cirt 100000 "

fout = open("all-cmds.sh", "wt")
    
rrtSize = 50 * 1000
distanceToGoal = 0.122  #should be 0.05!!
dcrit = 0.11
goalBiasProbability = 0.9 #goal bias neer the goal, should be larger than goalBiasProbability2
goalBiasProbability2 = 0.1
    
#urdfFile = "jogramop/robots/franka_panda/mobile_panda.urdf"
urdfFile = "jogramop/robots/franka_panda/mobile_panda_fingers.urdf"
seed = 1

for scenario in range(1,4):
    scenarioDir = "jogramop/scenarios/{:03d}/export/".format(scenario)
    obstacleFile = "{}/obstacles.obj".format(scenarioDir)
    startFile = "{}/robot_start_conf.csv".format(scenarioDir)
    ikFile = "{}/grasp_IK_solutions.csv".format(scenarioDir)
    graspFile = "{}/grasps.csv".format(scenarioDir)

    graspConfigurations = loadGrasps(ikFile)
    
    for planner in planners:
        resultsDir = "results/{}/{}".format(scenario, planner)
        os.system("mkdir -p {}".format(resultsDir))

        for iteration in range(50):

            for ikindex in range(len(graspConfigurations)):

                outFile = "{}/out-{:03d}-{:03d}".format(resultsDir,ikindex,iteration)

                if os.path.isfile(outFile + ".txt"):
                    print("Result ", outFile, " finished ")
                    continue

                cmd = "./burs_of_free_space test "
                cmd += " -grasp {} -urdf {} -obstacle {} -start_config {}".format(graspFile, urdfFile, obstacleFile, startFile)
                cmd += " -delta_q 3.1415 -epsilon_q 0.05 -num_spikes 7  "
                cmd += " -render 0 -vis_script scripts/animate_scene.py -cx -1 -cy 3 -cz 6 -groundLevel 0.00 -minColSegIdx 6 "
                cmd += " -target_prefix {} ".format(outFile)
                cmd += " -d_crit {} ".format(dcrit)
                cmd += " -max_iters {} ".format(rrtSize)
                cmd += " -p_close_enough {} ".format(distanceToGoal)
                cmd += " -q_resolution 0.1 "
                cmd += " -goal_bias_radius 0.35 "
                cmd += " -goal_bias_prob {} ".format(goalBiasProbability)
                cmd += " -prob_steer {} ".format(goalBiasProbability2)
                cmd += planners[ planner ]
                cmd += " -seed {} ".format(seed)
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
                
