./build/burs_of_free_space test \
-grasp /home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/jogramop/scenarios/002/export/grasps.csv \
-urdf /home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/jogramop/robots/franka_panda/mobile_panda_fingersSmallMesh.urdf \
-obstacle /home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/jogramop/scenarios/002/export/obstacles.obj \
-start_config /home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/jogramop/scenarios/002/export/robot_start_conf.csv \
-planner 8 \
-max_iters 10000 \
-d_crit 0.1 \
-delta_q 3.1415 \
-epsilon_q 0.1 \
-num_spikes 7 \
-p_close_enough 50.0 \
-prob_steer 0.01 \
-q_resolution 0.1 \
-seed -1 \
-use_rot 100 \
-rot_ratio 0.5 \
-bias_calculation 0 \
-groundLevel 0.0 \
-minColSegIdx 6 \
-target_prefix /home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/lel \
-target_configs /home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/jogramop/scenarios/002/export/grasp_IK_solutions.csv \
-ik_index 0 \
-render 0 \
-vis_script /home/hartvi/Documents/CVUT/diploma_thesis/burs_of_free_space/scripts/animate_scene.py \
-cx -3 \
-cy 3 \
-cz 5 \
-render_tree 0 \
-goal_bias_radius 0.35 \
-goal_bias_prob 0.5 \
-preheat_ratio 0.0 \
-preheat_type 0 \