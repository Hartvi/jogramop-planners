./build/jogramop-framework-planners test \
-grasp jogramop-framework/scenarios/015/export/grasps.csv \
-urdf jogramop-framework/robots/franka_panda/mobile_panda_fingersSmallMesh.urdf \
-obstacle jogramop-framework/scenarios/015/export/obstacles.obj \
-start_config jogramop-framework/scenarios/015/export/robot_start_conf.csv \
-planner 0 \
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
-groundLevel 0.0 \
-minColSegIdx 6 \
-target_prefix lel \
-target_configs jogramop-framework/scenarios/015/export/grasp_IK_solutions.csv \
-ik_index 0 \
-bias_calculation 0 \
-render 0 \
-vis_script scripts/animate_scene.py \
-cx -3 \
-cy 3 \
-cz 5 \
-render_tree 0 \
-goal_bias_radius 0.35 \
-goal_bias_prob 0.5 \
-preheat_ratio 0.0 \
-preheat_type 0 \
