#include <string>
#include <sstream>
#include <vector>

#include "bur_funcs.h"
#include "j_rbt_planner.h"
#include "ut.h"

namespace Burs
{
    using namespace Eigen;

    JRbtPlanner::JRbtPlanner(std::string urdf_file) : JRRTPlanner(urdf_file)
    {
        // TODO: this planner needs to:
        //  - reuse forward-kinematics results
        //  - reuse the Jacobian from a single configuration

        // Idk how to random numbers in C++:

        this->rng = std::make_shared<RandomNumberGenerator>(1, 1); // temporary seed is one, the proper seed is set at the begiging of JPlusRbt
    }

    std::optional<std::vector<VectorXd>>
    JRbtPlanner::JRbtS(const VectorXd &q_start, JPlusRbtParameters &planner_parameters, PlanningResult &plan_result)
    {
        std::cout << "RUNNING JRBT\n";
        if (planner_parameters.target_poses.size() < 1)
        {
            throw std::runtime_error("Target poses has length 0!");
        }
        RS start_state = this->NewState(q_start);

        // Random numbers
        this->rng = std::make_shared<RandomNumberGenerator>(planner_parameters.seed, planner_parameters.target_poses.size());

        auto tree = std::make_shared<BurTree>(start_state, q_start.size());
        if (planner_parameters.preheat_type == 0)
        {
            this->PreheatNTrees(tree, q_start, planner_parameters);
        }

        // To prevent uninitialized vectors in planner_parameters
        this->InitGraspClosestConfigs(planner_parameters, tree, 0);
        int preheat_iters = planner_parameters.max_iters * planner_parameters.preheat_ratio;
        std::cout << "preheat iters: " << preheat_iters << "\n";

        if (planner_parameters.preheat_type == 1)
        {
            this->PreheatTree(tree, 0, preheat_iters, planner_parameters);
        }
        // exit(1);

        double totalNNtime = 0;
        // double totalAddTime = 0;
        double totalRunTime = 0;
        // double totalCollisionTime = 0;
        double totalGetClosestDistTime = 0;
        double totalCollideAndAddTime = 0;
        struct rusage gt1, gt2;
        getTime(&gt1);

        int special_steps = 0;
        for (unsigned int k = preheat_iters; k < planner_parameters.max_iters; ++k)
        {
            // LOGGING
            if (k % 1000 == 0)
            {
                getTime(&gt2);
                totalRunTime = getTime(gt1, gt2);
                auto &best_pose = planner_parameters.target_poses[this->GetBestGrasp(planner_parameters)];
                std::cout << "iter: " << k << "/" << planner_parameters.max_iters << ", tree.size: " << tree->GetNumberOfNodes() << ", distToGoal: " << best_pose.best_dist << ", prob_steer: " << planner_parameters.probability_to_steer_to_target;
                std::cout << ", p_close_enough: " << planner_parameters.p_close_enough << ", totalNNtime: " << totalNNtime << ", totalCollideAndAddTime: " << totalCollideAndAddTime
                          << ", totalGetClosestDistTime: " << totalGetClosestDistTime << ", totalRunTime: " << totalRunTime << "\n";
                std::cout.flush();
            }
            // END LOGGING

            MatrixXd Qe = this->GetRandomQ(planner_parameters.num_spikes);

            struct rusage tt1, tt2;
            getTime(&tt1);
            // Random column
            int nearest_idx = tree->Nearest(Qe.col(0).data());
            getTime(&tt2);
            totalNNtime += getTime(tt1, tt2);

            RS *near_state = tree->Get(nearest_idx);
            // VectorXd q_near = tree->GetQ(nearest_idx);

            getTime(&tt1);
            // Slow => maybe in the future use FCL and somehow compile it because it had a ton of compilation errors and version mismatches
            auto [d_closest_idx, closest_dists] = this->GetClosestDistances(*near_state);
            getTime(&tt2);
            totalGetClosestDistTime += getTime(tt1, tt2);

            double d_closest = closest_dists[d_closest_idx];
            bool too_close = d_closest < planner_parameters.d_crit;
            bool special_step = d_closest_idx < 4 && too_close;

            // if obstacle too close: try to see if higher-up joints are more free and try to move those
            if (special_step)
            {
                VectorXd config_mask = this->env->robot->segmentToJntCausality[d_closest_idx];
                // if the segment is low enough => zero out the joint changes that affect the lower segments
                for (int i = 0; i < Qe.cols(); ++i)
                {
                    Qe.col(i) = Qe.col(i).cwiseProduct(config_mask);
                }
                // Set it to the second lowest distance that can still move the rest of the robot
                d_closest = 1e14;
                for (int i = d_closest_idx + 1; i < closest_dists.size(); ++i)
                {
                    if (closest_dists[i] < d_closest)
                    {
                        d_closest = closest_dists[i];
                    }
                }
                // std::cout << "d_crit: " << planner_parameters.d_crit << "  second closest dist: " << d_closest << "\n";

                std::vector<RS> Qe_states = this->NewStates(Qe);
                double distance_to_move = std::min(d_closest, planner_parameters.delta_q);
                std::vector<RS> endpoints = this->GetEndpoints(*near_state, Qe_states, distance_to_move);

                std::vector<RS> collision_free_endpoints;
                VectorXd negative_mask = VectorXd::Ones(config_mask.size()) - config_mask;
                // std::cout << " negative mask: " << negative_mask.transpose() << "\n";
                for (unsigned int i = 0; i < endpoints.size(); ++i)
                {
                    // RRT STEP IN ADDITION TO RBT STEP
                    RS tmp_tgt = this->NewState(endpoints[i].config.cwiseProduct(negative_mask));
                    std::vector<RS> tmp_endpoint = this->GetEndpoints(endpoints[i], {tmp_tgt}, planner_parameters.epsilon_q);
                    if (this->IsColliding(tmp_endpoint[0]))
                    {
                        // std::cout << "JRBT COLLIDED IN RRT special STEP" << i << "/" << endpoints.size() << "\n";
                    }
                    else
                    {
                        collision_free_endpoints.push_back(tmp_endpoint[0]);
                    }
                }
                // std::cout << "length of new endpoints: " << collision_free_endpoints.size() << " old: " << endpoints.size() << "\n";
                this->AddDenseBur(tree, nearest_idx, collision_free_endpoints, planner_parameters);
                special_steps++;
                // std::cout << "SPECIAL STEPS: " << special_steps << "\n";
                // exit(1);
            }
            else if (too_close)
            {
                std::vector<RS> Qe_states = this->NewStates(Qe.col(0));
                std::vector<RS> endpoints = this->GetEndpoints(*near_state, Qe_states, planner_parameters.epsilon_q);
                if (this->IsColliding(endpoints[0]))
                {
                    // std::cout << "JRBT COLLIDED IN RRT RANDOM STEP\n";
                }
                else
                {
                    tree->AddNode(nearest_idx, endpoints[0]);
                }
            }
            else // REGULAR BUR
            {
                std::vector<RS> Qe_states = this->NewStates(Qe);
                double distance_to_move = std::min(d_closest, planner_parameters.delta_q);
                std::vector<RS> endpoints = this->GetEndpoints(*near_state, Qe_states, distance_to_move);

                this->AddDenseBur(tree, nearest_idx, endpoints, planner_parameters);
            }

            // TRAVELLED DISTANCES ARE INDEED ALWAYS SMALLER THAN D_CLOSEST

            double rand_num = this->rng->getRandomReal();
            // std::cout << "randnum: " << rand_num << "\n";
            if (rand_num < planner_parameters.probability_to_steer_to_target)
            {
                // Steer until hit the target or obstacle or joint limits
                AlgorithmState state = this->ExtendToGoalRbt(tree, planner_parameters);

                if (state != AlgorithmState::Reached)
                {
                    state = this->JumpToGoal(tree, planner_parameters);
                }
                // Get grasp with minimal distance
                unsigned int best_grasp_idx = this->GetBestGrasp(planner_parameters);
                Grasp best_grasp = planner_parameters.target_poses[best_grasp_idx];
                // std::cout << "best dist: " << best_grasp.best_dist << "\n";
                if (state != AlgorithmState::Reached)
                {
                    state = best_grasp.best_dist < planner_parameters.p_close_enough ? AlgorithmState::Reached : AlgorithmState::Trapped;
                }
                if (state == AlgorithmState::Reached)
                {
                    // Get grasp with minimal distance
                    unsigned int best_grasp_idx = this->GetBestGrasp(planner_parameters);
                    Grasp best_grasp = planner_parameters.target_poses[best_grasp_idx];
                    // Get idx in tree that leads to the best config
                    int best_idx = tree->Nearest(best_grasp.best_state);
                    // int best_idx = tree->Nearest(best_grasp.dv->v.data());
                    // Take measurements
                    plan_result.distance_to_goal = best_grasp.best_dist;
                    plan_result.num_iterations = k;
                    plan_result.tree_size = tree->GetNumberOfNodes();
                    plan_result.success = true;

                    // Return best path
                    auto path = this->ConstructPathFromTree(tree, best_idx);
                    if (planner_parameters.visualize_tree > 0)
                    {
                        this->tree_csv = this->TreePoints(tree, planner_parameters.visualize_tree);
                    }
                    return path;
                }
            }
        }

        // Get grasp with minimal distance
        unsigned int best_grasp_idx = this->GetBestGrasp(planner_parameters);
        Grasp best_grasp = planner_parameters.target_poses[best_grasp_idx];

        // Get idx in tree that leads to the best config
        int best_idx = tree->Nearest(best_grasp.best_state);
        // int best_idx = tree->Nearest(best_grasp.dv->v.data());
        // Take measurements
        plan_result.distance_to_goal = best_grasp.best_dist;
        // plan_result.distance_to_goal = best_grasp.dv->d;
        plan_result.num_iterations = planner_parameters.max_iters;
        plan_result.tree_size = tree->GetNumberOfNodes();
        plan_result.success = best_grasp.best_dist < planner_parameters.p_close_enough;

        // Return best path
        auto path = this->ConstructPathFromTree(tree, best_idx);
        if (planner_parameters.visualize_tree)
        {
            this->tree_csv = this->TreePoints(tree, planner_parameters.visualize_tree);
        }
        std::cout << "FINISHED ITERATIONS\n";
        return path;
    }

    std::optional<std::vector<VectorXd>>
    JRbtPlanner::JRbtBasic(const VectorXd &q_start, JPlusRbtParameters &planner_parameters, PlanningResult &plan_result)
    {
        std::cout << "RUNNING JRBT\n";
        if (planner_parameters.target_poses.size() < 1)
        {
            throw std::runtime_error("Target poses has length 0!");
        }
        RS start_state = this->NewState(q_start);

        auto tree = std::make_shared<BurTree>(start_state, q_start.size());

        // Random numbers
        // std::cout << "target poses: " << planner_parameters.target_poses.size() << "\n";
        this->rng = std::make_shared<RandomNumberGenerator>(planner_parameters.seed, planner_parameters.target_poses.size());

        // To prevent uninitialized vectors in planner_parameters
        this->InitGraspClosestConfigs(planner_parameters, tree, 0);

        double totalNNtime = 0;
        // double totalAddTime = 0;
        double totalRunTime = 0;
        // double totalCollisionTime = 0;
        double totalGetClosestDistTime = 0;
        double totalCollideAndAddTime = 0;
        struct rusage gt1, gt2;
        getTime(&gt1);

        for (unsigned int k = 0; k < planner_parameters.max_iters; ++k)
        {
            // LOGGING
            if (k % 1000 == 0)
            {
                getTime(&gt2);
                totalRunTime = getTime(gt1, gt2);
                Grasp &best_pose = planner_parameters.target_poses[this->GetBestGrasp(planner_parameters)];
                std::cout << "iter: " << k << "/" << planner_parameters.max_iters;
                std::cout << ", tree.size: " << tree->GetNumberOfNodes();
                std::cout << ", distToGoal: " << best_pose.best_dist;
                std::cout << ", prob_steer: " << planner_parameters.probability_to_steer_to_target;
                std::cout << ", p_close_enough: " << planner_parameters.p_close_enough;
                std::cout << ", totalNNtime: " << totalNNtime;
                std::cout << ", totalCollideAndAddTime: " << totalCollideAndAddTime;
                std::cout << ", totalGetClosestDistTime: " << totalGetClosestDistTime;
                std::cout << ", totalRunTime: " << totalRunTime << "\n";
                std::cout.flush();
            }
            // END LOGGING

            MatrixXd Qe = this->GetRandomQ(planner_parameters.num_spikes);
            std::vector<RS> Qe_states = this->NewStates(Qe);

            struct rusage tt1, tt2;
            getTime(&tt1);
            // Random column
            int nearest_idx = tree->Nearest(Qe.col(0).data());
            RS *near_state = tree->Get(nearest_idx);
            getTime(&tt2);
            totalNNtime += getTime(tt1, tt2);

            getTime(&tt1);
            // Slow => maybe in the future use FCL and somehow compile it because it had a ton of compilation errors and version mismatches
            double d_closest = this->GetClosestDistance(*near_state);
            getTime(&tt2);
            totalGetClosestDistTime += getTime(tt1, tt2);

            bool too_close = d_closest < planner_parameters.d_crit;

            if (too_close)
            {
                RS endpoint = this->GetEndpoints(*near_state, {Qe_states[0]}, planner_parameters.epsilon_q)[0];
                if (!this->IsColliding(endpoint))
                {
                    int new_id = tree->AddNode(nearest_idx, endpoint);
                    this->SetGraspClosestConfigs(planner_parameters, tree, new_id);
                }
                // RS Qe_state = this->NewState(Qe.col(0));
                // std::vector<int> ids = this->ExtendRandomConfig(tree, Qe_state, planner_parameters);
                // // std::cout << "RRT TOO CLOSE NUM NODES: " << ids.size() << "\n";
                // // std::cout << "extend to random point: " << ids.size() << "\n";
                // for (unsigned int i = 0; i < ids.size(); ++i)
                // {
                //     // std::cout << "q: " << tree->Get(ids[i])->config.transpose() << "\n";
                //     this->SetGraspClosestConfigs(planner_parameters, tree, ids[i]);
                // }
            }
            else // REGULAR BUR
            {
                double distance_to_move = d_closest;
                std::vector<std::vector<RS>> endpoints = this->GetEndpointsInterstates(*near_state, Qe_states, distance_to_move, planner_parameters.q_resolution);

                for (unsigned int i = 0; i < endpoints.size(); ++i)
                {
                    auto endpoint_vec = endpoints[i];
                    int prev_idx = nearest_idx; // idx of q_near

                    for (unsigned int l = 0; l < endpoint_vec.size(); ++l)
                    {
                        prev_idx = tree->AddNode(prev_idx, endpoint_vec[l]);
                        double tmp_dist = this->SetGraspClosestConfigs(planner_parameters, tree, prev_idx);
                    }
                }
            }

            // TRAVELLED DISTANCES ARE INDEED ALWAYS SMALLER THAN D_CLOSEST

            double rand_num = this->rng->getRandomReal();
            // std::cout << "randnum: " << rand_num << "\n";
            if (rand_num < planner_parameters.probability_to_steer_to_target)
            {
                // Steer until hit the target or obstacle or joint limits
                // AlgorithmState state = this->ExtendToGoalRbt(tree, planner_parameters);
                AlgorithmState state = this->ExtendToGoalRRT(tree, planner_parameters);

                // Get grasp with minimal distance
                unsigned int best_grasp_idx = this->GetBestGrasp(planner_parameters);
                Grasp best_grasp = planner_parameters.target_poses[best_grasp_idx];
                // std::cout << "best dist: " << best_grasp.best_dist << "\n";
                if (state != AlgorithmState::Reached)
                {
                    if (best_grasp.best_dist <= planner_parameters.p_close_enough)
                    {
                        state = AlgorithmState::Reached;
                    }
                }
                if (state == AlgorithmState::Reached)
                {
                    // Get idx in tree that leads to the best config
                    int best_idx = tree->Nearest(best_grasp.best_state);
                    // int best_idx = tree->Nearest(best_grasp.dv->v.data());
                    // Take measurements
                    plan_result.distance_to_goal = best_grasp.best_dist;
                    plan_result.num_iterations = k;
                    plan_result.tree_size = tree->GetNumberOfNodes();
                    plan_result.success = true;

                    // Return best path
                    auto path = this->ConstructPathFromTree(tree, best_idx);
                    if (planner_parameters.visualize_tree > 0)
                    {
                        this->tree_csv = this->TreePoints(tree, planner_parameters.visualize_tree);
                    }
                    return path;
                }
            }
        }

        // Get grasp with minimal distance
        unsigned int best_grasp_idx = this->GetBestGrasp(planner_parameters);
        Grasp best_grasp = planner_parameters.target_poses[best_grasp_idx];

        // Get idx in tree that leads to the best config
        int best_idx = tree->Nearest(best_grasp.best_state);
        // int best_idx = tree->Nearest(best_grasp.dv->v.data());
        // Take measurements
        plan_result.distance_to_goal = best_grasp.best_dist;
        // plan_result.distance_to_goal = best_grasp.dv->d;
        plan_result.num_iterations = planner_parameters.max_iters;
        plan_result.tree_size = tree->GetNumberOfNodes();
        plan_result.success = best_grasp.best_dist < planner_parameters.p_close_enough;

        // Return best path
        auto path = this->ConstructPathFromTree(tree, best_idx);
        if (planner_parameters.visualize_tree)
        {
            this->tree_csv = this->TreePoints(tree, planner_parameters.visualize_tree);
        }
        return path;
    }

    std::optional<std::vector<VectorXd>>
    JRbtPlanner::JRbt(const VectorXd &q_start, JPlusRbtParameters &planner_parameters, PlanningResult &plan_result)
    {
        std::cout << "RUNNING JRBT\n";
        if (planner_parameters.target_poses.size() < 1)
        {
            throw std::runtime_error("Target poses has length 0!");
        }
        RS start_state = this->NewState(q_start);

        auto tree = std::make_shared<BurTree>(start_state, q_start.size());
        if (planner_parameters.preheat_type == 0)
        {
            this->PreheatNTrees(tree, q_start, planner_parameters);
        }

        // Random numbers
        this->rng = std::make_shared<RandomNumberGenerator>(planner_parameters.seed, planner_parameters.target_poses.size());

        // To prevent uninitialized vectors in planner_parameters
        this->InitGraspClosestConfigs(planner_parameters, tree, 0);
        int preheat_iters = planner_parameters.max_iters * planner_parameters.preheat_ratio;
        std::cout << "preheat iters: " << preheat_iters << "\n";

        if (planner_parameters.preheat_type == 1)
        {
            this->PreheatTree(tree, 0, preheat_iters, planner_parameters);
        }
        // exit(1);

        double totalNNtime = 0;
        // double totalAddTime = 0;
        double totalRunTime = 0;
        // double totalCollisionTime = 0;
        double totalGetClosestDistTime = 0;
        double totalCollideAndAddTime = 0;
        struct rusage gt1, gt2;
        getTime(&gt1);

        for (unsigned int k = preheat_iters; k < planner_parameters.max_iters; ++k)
        {
            // LOGGING
            if (k % 1000 == 0)
            {
                getTime(&gt2);
                totalRunTime = getTime(gt1, gt2);
                Grasp &best_pose = planner_parameters.target_poses[this->GetBestGrasp(planner_parameters)];
                std::cout << "iter: " << k << "/" << planner_parameters.max_iters << ", tree.size: " << tree->GetNumberOfNodes() << ", distToGoal: " << best_pose.best_dist << ", prob_steer: " << planner_parameters.probability_to_steer_to_target;
                std::cout << ", p_close_enough: " << planner_parameters.p_close_enough << ", totalNNtime: " << totalNNtime << ", totalCollideAndAddTime: " << totalCollideAndAddTime
                          << ", totalGetClosestDistTime: " << totalGetClosestDistTime << ", totalRunTime: " << totalRunTime << "\n";
                std::cout.flush();
            }
            // END LOGGING

            MatrixXd Qe = this->GetRandomQ(planner_parameters.num_spikes);
            std::vector<RS> Qe_states = this->NewStates(Qe);

            struct rusage tt1, tt2;
            getTime(&tt1);
            // Random column
            int nearest_idx = tree->Nearest(Qe.col(0).data());
            getTime(&tt2);
            totalNNtime += getTime(tt1, tt2);

            RS *near_state = tree->Get(nearest_idx);
            // VectorXd q_near = tree->GetQ(nearest_idx);

            getTime(&tt1);
            // Slow => maybe in the future use FCL and somehow compile it because it had a ton of compilation errors and version mismatches
            double d_closest = this->GetClosestDistance(*near_state);
            getTime(&tt2);
            totalGetClosestDistTime += getTime(tt1, tt2);

            bool too_close = d_closest < planner_parameters.d_crit;

            // if obstacle too close: try to see if higher-up joints are more free and try to move those
            if (too_close)
            {
                std::vector<RS> Qe_states = this->NewStates(Qe.col(0));
                std::vector<RS> endpoints = this->GetEndpoints(*near_state, Qe_states, planner_parameters.epsilon_q);
                if (this->IsColliding(endpoints[0]))
                {
                    // std::cout << "JRBT COLLIDED IN RRT RANDOM STEP\n";
                }
                else
                {
                    tree->AddNode(nearest_idx, endpoints[0]);
                }
            }
            else // REGULAR BUR
            {
                std::vector<RS> Qe_states = this->NewStates(Qe);
                double distance_to_move = std::min(d_closest, planner_parameters.delta_q);
                std::vector<RS> endpoints = this->GetEndpoints(*near_state, Qe_states, distance_to_move);

                std::vector<double> bur_dists = this->AddDenseBur(tree, nearest_idx, endpoints, planner_parameters);
                // double r = this->rng->getRandomReal();
                // if (r < planner_parameters.goal_bias_probability)
                // {
                //     for (unsigned int i = 0; i < bur_dists.size(); ++i)
                //     {
                //         if (bur_dists[i] < planner_parameters.goal_bias_radius)
                //         {
                //             int endpoint_idx = tree->Nearest(endpoints[i]);
                //             while ()
                //                 auto [dists, states] = this->ExtendToGoalRbtStep(tree, endpoint_idx, planner_parameters);
                //         }
                //     }
                // }
            }

            // TRAVELLED DISTANCES ARE INDEED ALWAYS SMALLER THAN D_CLOSEST

            double rand_num = this->rng->getRandomReal();
            // std::cout << "randnum: " << rand_num << "\n";
            if (rand_num < planner_parameters.probability_to_steer_to_target)
            {
                // Steer until hit the target or obstacle or joint limits
                AlgorithmState state = this->ExtendToGoalRbt(tree, planner_parameters);

                if (state != AlgorithmState::Reached)
                {
                    state = this->JumpToGoal(tree, planner_parameters);
                }
                // Get grasp with minimal distance
                unsigned int best_grasp_idx = this->GetBestGrasp(planner_parameters);
                Grasp best_grasp = planner_parameters.target_poses[best_grasp_idx];
                // std::cout << "best dist: " << best_grasp.best_dist << "\n";
                if (state != AlgorithmState::Reached)
                {
                    state = best_grasp.best_dist < planner_parameters.p_close_enough ? AlgorithmState::Reached : AlgorithmState::Trapped;
                }
                if (state == AlgorithmState::Reached)
                {
                    // Get idx in tree that leads to the best config
                    int best_idx = tree->Nearest(best_grasp.best_state);
                    // int best_idx = tree->Nearest(best_grasp.dv->v.data());
                    // Take measurements
                    plan_result.distance_to_goal = best_grasp.best_dist;
                    plan_result.num_iterations = k;
                    plan_result.tree_size = tree->GetNumberOfNodes();
                    plan_result.success = true;

                    // Return best path
                    auto path = this->ConstructPathFromTree(tree, best_idx);
                    if (planner_parameters.visualize_tree > 0)
                    {
                        this->tree_csv = this->TreePoints(tree, planner_parameters.visualize_tree);
                    }
                    return path;
                }
            }
        }

        // Get grasp with minimal distance
        unsigned int best_grasp_idx = this->GetBestGrasp(planner_parameters);
        Grasp best_grasp = planner_parameters.target_poses[best_grasp_idx];

        // Get idx in tree that leads to the best config
        int best_idx = tree->Nearest(best_grasp.best_state);
        // int best_idx = tree->Nearest(best_grasp.dv->v.data());
        // Take measurements
        plan_result.distance_to_goal = best_grasp.best_dist;
        // plan_result.distance_to_goal = best_grasp.dv->d;
        plan_result.num_iterations = planner_parameters.max_iters;
        plan_result.tree_size = tree->GetNumberOfNodes();
        plan_result.success = best_grasp.best_dist < planner_parameters.p_close_enough;

        // Return best path
        auto path = this->ConstructPathFromTree(tree, best_idx);
        if (planner_parameters.visualize_tree)
        {
            this->tree_csv = this->TreePoints(tree, planner_parameters.visualize_tree);
        }
        return path;
    }

    std::vector<Grasp>
    JRbtPlanner::GetBestAndRandomGrasps(JPlusRbtParameters &planner_parameters) const
    {
        std::vector<Grasp> grasps(planner_parameters.num_spikes);
        // Get best grasp IDX
        unsigned int best_grasp_idx = this->GetBestGrasp(planner_parameters);
        Grasp best_grasp = planner_parameters.target_poses[best_grasp_idx];
        grasps[0] = best_grasp;

        // Get shuffled integer vector
        auto non_repeating_ints = this->rng->getNonRepeatingInts();

        // for (unsigned int i = 0; i < planner_parameters.num_spikes; ++i)
        // {
        //     std::cout << "int: " << non_repeating_ints[i] << " ";
        // }
        // std::cout << "\n";
        for (unsigned int i = 1; i < planner_parameters.num_spikes; ++i)
        {
            // i-th element from shuffled vector
            // unsigned int rand_int = *std::next(non_repeating_ints, i);
            unsigned int rand_int = non_repeating_ints[i];
            // std::cout << "rand int: " << rand_int << "\n";
            if (rand_int == best_grasp_idx)
            {
                // rand_int == best_idx => choose index 0 because we started at "i = 1"
                grasps[i] = planner_parameters.target_poses[0];
            }
            else
            {
                grasps[i] = planner_parameters.target_poses[rand_int];
            }
            // std::cout << "grasp: " << grasps[i].frame.p << "\n";
        }
        return grasps;
    }

    AlgorithmState
    JRbtPlanner::ExtendToGoalRbt(std::shared_ptr<BurTree> tree, JPlusRbtParameters &planner_parameters) const
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
            std::vector<Grasp> tgt_grasps = this->GetBestAndRandomGrasps(planner_parameters);
            // for (unsigned int i = 0; i < tgt_grasps.size(); ++i)
            // {
            //     std::cout << "tgt: " << tgt_grasps[i].frame.p << "\n";
            // }
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
            // bool too_close = closest_dist < planner_parameters.d_crit;
            unsigned int num_extensions = too_close ? 1 : planner_parameters.num_spikes;

            // Target configs to extend to gained from the jacobian
            MatrixXd target_configs = MatrixXd(this->q_dim, num_extensions);

            KDL::Frame ee_frame = this->env->robot->GetEEFrame(*best_state);
            // double distance_to_move = too_close ? planner_parameters.epsilon_q : closest_dist;
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
                auto [d, f_tgt] = this->BasicDistanceMetric(ee_frame, tgt_frame, planner_parameters.rotation_dist_ratio);
                delta_p = d;
                bool use_rotation = (delta_p <= planner_parameters.use_rotation);
                std::cout << "delta p: " << delta_p << " rotation threshold: " << planner_parameters.use_rotation << " userot: " << use_rotation << "\n";

                // Max dist => epsilon_q
                double dist_to_move = std::min(metric_dist, closest_dist);
                KDL::Twist twist = this->GetTwist(f_tgt, ee_frame, distance_to_move, planner_parameters.use_rotation);

                // TODO: reuse the jacobian for the pseudo-inverse J+
                KDL::JntArray q_dot = this->env->robot->ForwardJPlus(*best_state, twist);
                VectorXd delta_q = q_dot.data;
                target_configs.col(i) = best_state->config + delta_q;
            }

            std::vector<RS> target_states = this->NewStates(target_configs);

            // Iterate max `closest_dist` to `target_config`
            std::cout << "dist to move: " << distance_to_move << " res: " << planner_parameters.q_resolution << "\n";
            std::vector<std::vector<RS>> bur_endpoints = this->GetEndpointsInterstates(*best_state, target_states, distance_to_move, planner_parameters.q_resolution);

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
                    this->SetGraspClosestConfigs(planner_parameters, tree, prev_idx);
                }
            }

            // Checks bounds
            // Interpolates with max resolution_q distance between points
            // Updates best config for each grasp
            // this->AddDenseBur(tree, idx_near, bur_endpoints, planner_parameters);

        } while (delta_p > planner_parameters.p_close_enough && closest_dist > planner_parameters.d_crit);

        if (delta_p <= planner_parameters.p_close_enough)
        {
            return AlgorithmState::Reached;
        }
        else
        {
            return AlgorithmState::Trapped;
        }
    }

    std::pair<std::vector<double>, std::vector<RS>>
    JRbtPlanner::ExtendToGoalRbtStep(std::shared_ptr<BurTree> tree, const int &idx_near, JPlusRbtParameters &planner_parameters) const
    {
        /* OUTLINE:
        1. Get best grasp's best configuration
        2. From that best configuration expand towards the grasp and some random grasps
        3. Go to 1.
        */
        double closest_dist = 1e14;

        // STEP 1.
        // Best grasp is in index 0
        std::vector<Grasp> tgt_grasps = this->GetBestAndRandomGrasps(planner_parameters);
        Grasp best_grasp = tgt_grasps[0];

        RS *near_state = tree->Get(idx_near);

        // Get closest obstacle distance
        closest_dist = this->GetClosestDistance(*near_state);

        // SWITCH TO RRT IF OBSTACLE TOO CLOSE
        // bool too_close = false;
        bool too_close = closest_dist < planner_parameters.d_crit;
        unsigned int num_extensions = too_close ? 1 : planner_parameters.num_spikes;
        double distance_to_move = too_close ? planner_parameters.epsilon_q : closest_dist;
        // std::cout << "closest dist: " << closest_dist << " dist to move: " << distance_to_move << "\n";

        // closest_dist = this->GetClosestDistance(q_near);

        // Target configs to extend to gained from the jacobian
        MatrixXd target_configs = MatrixXd(this->q_dim, num_extensions);

        KDL::Frame ee_frame = this->env->robot->GetEEFrame(*near_state);
        for (unsigned int i = 0; i < num_extensions; ++i)
        {
            // Max dist => closest_dist / dist to goal (maybe better to have distance to goal since it can be farther)
            KDL::Frame tgt_frame = tgt_grasps[i].frame;
            // KDL::Frame ee_frame = best_grasp.best_frame;

            // From ee_frame (best config)
            // To tgt_frame (one of the randomly chosen goals)
            // Move `closest_dist` along that direction
            // Can be farther that the goal, but that's fine because we interpolate using `Densify`
            KDL::Twist twist = this->GetTwist(tgt_frame, ee_frame, distance_to_move, planner_parameters.use_rotation);

            // TODO: reuse the jacobian for the pseudo-inverse J+
            KDL::JntArray q_dot = this->env->robot->ForwardJPlus(*near_state, twist);
            VectorXd delta_q = q_dot.data;
            target_configs.col(i) = near_state->config + delta_q;
        }

        std::vector<RS> target_states = this->NewStates(target_configs);
        // DEBUG:
        // auto eeframe = this->env->robot->GetEEFrame(target_states[0]);

        // END DEBUG
        // Iterate max `closest_dist` to `target_config`
        std::vector<RS> bur_endpoints = this->GetEndpoints(*near_state, target_states, distance_to_move);
        // std::vector<double> distances(bur_endpoints.size());
        // for (unsigned int i = 0; i < bur_endpoints.size(); ++i)
        // {
        //     double tmpdist = this->env->robot->MaxDistance(*best_state, bur_endpoints[i]);
        //     std::cout << "max dist " << i << " in extend: " << tmpdist << "\n";
        // }

        // If closest obstacle was too close => check collisions for the RRT step
        if (too_close)
        {
            for (unsigned int i = 0; i < bur_endpoints.size(); ++i)
            {
                if (this->IsColliding(bur_endpoints[i]))
                {
                    // std::cout << "JRBT COLLIDED IN RRT EXTEND\n";
                    // break;
                    return {};
                }
            }
        }

        // Checks bounds
        // Interpolates with max resolution_q distance between points
        // Updates best config for each grasp
        return {this->AddDenseBur(tree, idx_near, bur_endpoints, planner_parameters), bur_endpoints};
    }
}