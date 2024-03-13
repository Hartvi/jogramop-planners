#include <string>
#include <sstream>
#include <vector>

#include "bur_funcs.h"
#include "j_rbt_planner.h"
#include "ut.h"

namespace Burs
{
    using namespace Eigen;

    JRbtPlanner::JRbtPlanner(std::string urdf_file) : IKRRTPlanner(urdf_file)
    {
        this->rng = std::make_shared<RandomNumberGenerator>(1, 1); // temporary seed is one, the proper seed is set at the begiging of JPlusRbt
    }

    std::optional<std::vector<VectorXd>>
    JRbtPlanner::JRbtBasic(const VectorXd &q_start, JPlusRbtParameters &plan_params, PlanningResult &plan_result)
    {
        std::cout << "RUNNING JRBT\n";
        if (plan_params.target_poses.size() < 1)
        {
            throw std::runtime_error("Target poses has length 0!");
        }
        RS start_state = this->NewState(q_start);

        auto tree = std::make_shared<BurTree>(start_state, q_start.size());

        // Random numbers
        // std::cout << "target poses: " << plan_params.target_poses.size() << "\n";
        this->rng = std::make_shared<RandomNumberGenerator>(plan_params.seed, plan_params.target_poses.size());

        // To prevent uninitialized vectors in plan_params
        this->InitGraspClosestConfigs(plan_params, tree, 0);

        double totalNNtime = 0;
        // double totalAddTime = 0;
        double totalRunTime = 0;
        // double totalCollisionTime = 0;
        double totalGetClosestDistTime = 0;
        double totalCollideAndAddTime = 0;
        struct rusage gt1, gt2;
        getTime(&gt1);

        for (unsigned int k = 0; k < plan_params.max_iters; ++k)
        {
            // LOGGING
            if (k % 1000 == 0)
            {
                getTime(&gt2);
                totalRunTime = getTime(gt1, gt2);
                Grasp &best_pose = plan_params.target_poses[this->GetBestGrasp(plan_params)];
                std::cout << "iter: " << k << "/" << plan_params.max_iters;
                std::cout << ", tree.size: " << tree->GetNumberOfNodes();
                std::cout << ", distToGoal: " << best_pose.best_dist;
                std::cout << ", prob_steer: " << plan_params.probability_to_steer_to_target;
                std::cout << ", p_close_enough: " << plan_params.p_close_enough;
                std::cout << ", totalNNtime: " << totalNNtime;
                std::cout << ", totalCollideAndAddTime: " << totalCollideAndAddTime;
                std::cout << ", totalGetClosestDistTime: " << totalGetClosestDistTime;
                std::cout << ", totalRunTime: " << totalRunTime << "\n";
                std::cout.flush();
            }
            // END LOGGING

            if (this->globalTrigger)
            {
                std::cerr << "Terminating planner as globalTrigger=" << globalTrigger << "\n";
                std::cout << "Terminating planner as globalTrigger=" << globalTrigger << "\n";
                break;
            }

            MatrixXd Qe = this->GetRandomQ(plan_params.num_spikes);

            struct rusage tt1, tt2;
            getTime(&tt1);
            // Random column
            int nearest_idx = tree->Nearest(Qe.col(0).data());
            RS *near_state = tree->Get(nearest_idx);
            getTime(&tt2);
            totalNNtime += getTime(tt1, tt2);

            getTime(&tt1);
            if (near_state->closest_distance_idx < 0)
            {
                auto [d_closest_idx, ds_closest] = this->GetClosestDistances(*near_state);
                near_state->closest_distance_idx = d_closest_idx;
                near_state->closest_dists = ds_closest;
            }
            else
            {
                // std::cout << "REUSING DISTANCE\n";
                // // WE DONT NOT WANT TO RECREATE THE BUR ? ? std::cout << "REUSING DISTANCE\n";
                // k--;
                // continue;
            }
            double d_closest = near_state->closest_dists[near_state->closest_distance_idx];
            // double d_closest = this->GetClosestDistance(*near_state);
            getTime(&tt2);
            totalGetClosestDistTime += getTime(tt1, tt2);

            bool too_close = d_closest < plan_params.d_crit;

            // Create only one state if too close
            std::vector<RS> Qe_states = this->NewStates((too_close ? Qe.col(0) : Qe));

            if (too_close)
            {
                int step_result = this->RRTStepInQ(tree, nearest_idx, Qe_states[0], plan_params.epsilon_q, plan_params.collision_resolution, true);
                if (step_result >= 0)
                {
                    // if (this->IsColliding(*tree->Get(step_result)))
                    // {
                    //     std::cout << "RRT STEP COLLIDING\n";
                    // }
                    this->SetGraspClosestConfigs(plan_params, tree, step_result);
                }
            }
            else // REGULAR BUR
            {
                double distance_to_move = d_closest;
                std::vector<RS> endpoints = this->GetEndpoints(*near_state, Qe_states, distance_to_move);

                for (unsigned int i = 0; i < endpoints.size(); ++i)
                {
                    if (this->IsColliding(endpoints[i]))
                    {
                        // std::cout << "ENDPOINT COLLIDING\n";
                        // exit(1);
                    }
                    else
                    {
                        int res = tree->AddNode(nearest_idx, endpoints[i]);
                        double tmp_dist = this->SetGraspClosestConfigs(plan_params, tree, res);
                    }
                }
            }

            // TRAVELLED DISTANCES ARE INDEED ALWAYS SMALLER THAN D_CLOSEST
            double rand_num = this->rng->getRandomReal();
            if (rand_num < plan_params.probability_to_steer_to_target)
            {
                // Steer until hit the target or obstacle or joint limits
                // AlgorithmState state = this->ExtendToGoalRbt(tree, plan_params);
                AlgorithmState state = this->ExtendToGoalRRT(tree, plan_params);

                // Get grasp with smallest distance
                // TODO: can keep grasps sorted by distance
                unsigned int best_grasp_idx = this->GetBestGrasp(plan_params);
                Grasp best_grasp = plan_params.target_poses[best_grasp_idx];
                if (state != AlgorithmState::Reached && best_grasp.best_dist <= plan_params.p_close_enough)
                {
                    state = AlgorithmState::Reached;
                }
                if (state == AlgorithmState::Reached)
                {
                    // Get idx in tree that leads to the best config
                    int best_idx = tree->Nearest(best_grasp.best_state);
                    // Take measurements
                    plan_result.distance_to_goal = best_grasp.best_dist;
                    plan_result.num_iterations = k;
                    plan_result.tree_size = tree->GetNumberOfNodes();
                    plan_result.success = true;

                    // Return best path
                    auto path = this->ConstructPathFromTree(tree, best_idx);
                    if (plan_params.visualize_tree > 0)
                    {
                        this->tree_csv = this->TreePoints(tree, plan_params.visualize_tree);
                    }
                    return path;
                }
            }
        }

        // Get grasp with minimal distance
        unsigned int best_grasp_idx = this->GetBestGrasp(plan_params);
        Grasp best_grasp = plan_params.target_poses[best_grasp_idx];

        // Get idx in tree that leads to the best config
        int best_idx = tree->Nearest(best_grasp.best_state);
        // Take measurements
        plan_result.distance_to_goal = best_grasp.best_dist;
        plan_result.num_iterations = plan_params.max_iters;
        plan_result.tree_size = tree->GetNumberOfNodes();
        plan_result.success = best_grasp.best_dist < plan_params.p_close_enough;

        // Return best path
        auto path = this->ConstructPathFromTree(tree, best_idx);
        if (plan_params.visualize_tree)
        {
            this->tree_csv = this->TreePoints(tree, plan_params.visualize_tree);
        }
        return path;
    }

    std::vector<Grasp>
    JRbtPlanner::GetBestAndRandomGrasps(JPlusRbtParameters &plan_params) const
    {
        std::vector<Grasp> grasps(plan_params.num_spikes);
        // Get best grasp IDX
        unsigned int best_grasp_idx = this->GetBestGrasp(plan_params);
        Grasp best_grasp = plan_params.target_poses[best_grasp_idx];
        grasps[0] = best_grasp;

        // Get shuffled integer vector
        auto non_repeating_ints = this->rng->getNonRepeatingInts();

        for (unsigned int i = 1; i < plan_params.num_spikes; ++i)
        {
            // i-th element from shuffled vector
            // unsigned int rand_int = *std::next(non_repeating_ints, i);
            unsigned int rand_int = non_repeating_ints[i];
            if (rand_int == best_grasp_idx)
            {
                // rand_int == best_idx => choose index 0 because we started at "i = 1"
                grasps[i] = plan_params.target_poses[0];
            }
            else
            {
                grasps[i] = plan_params.target_poses[rand_int];
            }
        }
        return grasps;
    }

    AlgorithmState
    JRbtPlanner::ExtendToGoalRbt(std::shared_ptr<BurTree> tree, JPlusRbtParameters &plan_params) const
    {
        /* OUTLINE:
        1. Get best grasp's best configuration
        2. From that best configuration expand towards the grasp and some random grasps
        3. Go to 1.
        */
        double delta_p = 1e14;
        double closest_dist = 1e14;

        do
        {
            // STEP 1.
            // Best grasp is in index 0
            std::vector<Grasp> tgt_grasps = this->GetBestAndRandomGrasps(plan_params);
            Grasp best_grasp = tgt_grasps[0];
            int best_state_idx = best_grasp.best_state;
            RS *best_state = tree->Get(best_state_idx);

            // STEP 2.
            // int idx_near = tree->Nearest(best_state_idx);
            delta_p = best_grasp.best_dist;
            double best_dist = best_grasp.best_dist;

            // Get closest obstacle distance
            closest_dist = this->GetClosestDistance(*best_state);

            // SWITCH TO RRT IF OBSTACLE TOO CLOSE
            bool too_close = false;
            // bool too_close = closest_dist < plan_params.d_crit;
            unsigned int num_extensions = too_close ? 1 : plan_params.num_spikes;

            // Target configs to extend to gained from the jacobian
            MatrixXd target_configs = MatrixXd(this->q_dim, num_extensions);

            KDL::Frame ee_frame = this->env->robot->GetEEFrame(*best_state);
            // double distance_to_move = too_close ? plan_params.epsilon_q : closest_dist;
            double distance_to_move = closest_dist;

            for (unsigned int i = 0; i < num_extensions; ++i)
            {
                // Max dist => closest_dist / dist to goal (maybe better to have distance to goal since it can be farther)
                KDL::Frame tgt_frame = tgt_grasps[i].frame;

                // From ee_frame (best config)
                // To tgt_frame (one of the randomly chosen goals)
                // Move `closest_dist` along that direction
                // Can be farther that the goal, but that's fine because we interpolate using `Densify`
                double metric_dist = (tgt_frame.p - ee_frame.p).Norm();
                auto [d, f_tgt] = this->BasicDistanceMetric(ee_frame, tgt_frame, plan_params.rotation_dist_ratio);
                delta_p = d;
                bool use_rotation = (delta_p <= plan_params.use_rotation);
                std::cout << "delta p: " << delta_p << " rotation threshold: " << plan_params.use_rotation << " userot: " << use_rotation << "\n";

                // Max dist => epsilon_q
                double dist_to_move = std::min(metric_dist, closest_dist);
                KDL::Twist twist = this->GetTwist(f_tgt, ee_frame, distance_to_move, plan_params.use_rotation);

                // TODO: reuse the jacobian for the pseudo-inverse J+
                KDL::JntArray q_dot = this->env->robot->ForwardJPlus(*best_state, twist);
                VectorXd delta_q = q_dot.data;
                target_configs.col(i) = best_state->config + delta_q;
            }

            std::vector<RS> target_states = this->NewStates(target_configs);

            // Iterate max `closest_dist` to `target_config`
            std::cout << "dist to move: " << distance_to_move << " res: " << plan_params.q_resolution << "\n";
            std::vector<std::vector<RS>> bur_endpoints = this->GetEndpointsInterstates(*best_state, target_states, distance_to_move, plan_params.q_resolution);

            // If closest obstacle was too close => check collisions for the RRT step
            for (unsigned int i = 0; i < bur_endpoints.size(); ++i)
            {
                std::vector<RS> line = bur_endpoints[i];
                // std::cout << "line: " << i << "\n";
                int prev_idx = best_state_idx;
                for (unsigned int j = 0; j < line.size(); ++j)
                {
                    if (too_close)
                    {
                        if (this->IsColliding(line[j]))
                        {
                            return AlgorithmState::Trapped;
                        }
                    }
                    // std::cout << "point: " << j << "\n";
                    prev_idx = tree->AddNode(prev_idx, line[j]);
                    this->SetGraspClosestConfigs(plan_params, tree, prev_idx);
                }
            }

            // Checks bounds
            // Interpolates with max resolution_q distance between points
            // Updates best config for each grasp

        } while (delta_p > plan_params.p_close_enough && closest_dist > plan_params.d_crit);

        if (delta_p <= plan_params.p_close_enough)
        {
            return AlgorithmState::Reached;
        }
        else
        {
            return AlgorithmState::Trapped;
        }
    }

}
