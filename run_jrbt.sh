./build/burs_of_free_space test \
-grasp /home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/jogramop/scenarios/004/export/grasps.csv \
-urdf /home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/jogramop/robots/franka_panda/mobile_panda_fingersSmallMesh.urdf \
-obstacle /home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/jogramop/scenarios/004/export/obstacles.obj \
-start_config /home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/jogramop/scenarios/004/export/robot_start_conf.csv \
-planner 1 \
-max_iters 50000 \
-d_crit 0.1 \
-delta_q 1.3 \
-epsilon_q 0.2 \
-num_spikes 7 \
-p_close_enough 0.07 \
-prob_steer 0.1 \
-target_prefix /home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/lel \
-render 1 \
-vis_script /home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/scripts/animate_scene.py \
-cx -3 \
-cy 2 \
-cz 7 \
-groundLevel 0.00 \
-minColSegIdx 6 \
-target_configs /home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/jogramop/scenarios/004/export/grasp_IK_solutions.csv \
-ik_index 0 \
-goal_bias_radius 0.35 \
-goal_bias_prob 0.9 \
-q_resolution 0.1 \
-seed -1 \
-render_tree 1000 \
-preheat_ratio 0.1