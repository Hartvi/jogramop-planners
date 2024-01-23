./build/burs_of_free_space test \
-grasp /home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/jogramop/scenarios/001/export/grasps.csv \
-urdf /home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/jogramop/robots/franka_panda/mobile_panda_fingers.urdf \
-obstacle /home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/jogramop/scenarios/001/export/obstacles.obj \
-start_config /home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/jogramop/scenarios/001/export/robot_start_conf.csv \
-planner 3 \
-max_iters 10000 \
-d_crit 0.05 \
-delta_q 1.1415 \
-epsilon_q 0.1 \
-num_spikes 7 \
-p_close_enough 0.015 \
-prob_steer 0.2 \
-target_prefix /home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/lel \
-render 1 \
-vis_script /home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/scripts/animate_scene.py \
-cx 3 \
-cy 1 \
-cz 7 \
-groundLevel 0.00 \
-minColSegIdx 6 \
-target_configs /home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/jogramop/scenarios/001/export/grasp_IK_solutions.csv \
-ik_index 0 \
-goal_bias_radius 0.15 \
-goal_bias_prob 0.5 \
-q_resolution 0.5 \
-seed -1