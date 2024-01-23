./build/burs_of_free_space test \
-grasp /home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/jogramop/scenarios/004/export/grasps.csv \
-urdf /home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/jogramop/robots/franka_panda/mobile_panda.urdf \
-obstacle /home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/jogramop/scenarios/004/export/obstacles.obj \
-start_config /home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/jogramop/scenarios/004/export/robot_start_conf.csv \
-planner 1 \
-max_iters 10000 \
-d_crit 0.1 \
-delta_q 3.1415 \
-epsilon_q 0.1 \
-num_spikes 7 \
-p_close_enough 0.015 \
-prob_steer 0.1 \
-target_prefix /home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/out.txt \
-render 1 \
-vis_script /home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/scripts/animate_scene.py \
-cx 1 \
-cy 0 \
-cz 7 \
-groundLevel 0.00 \
-minColSegIdx 6 \
-target_configs /home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/jogramop/scenarios/004/export/grasp_IK_solutions.csv