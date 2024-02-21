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
        this->rng = std::make_shared<RandomNumberGenerator>(1, 1); // temporary seed is one, the proper seed is set at the begiging of JPlusRbt
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
            if (rand_num < planner_parameters.probability_to_steer_to_target)
            {
                // Steer until hit the target or obstacle or joint limits
                // AlgorithmState state = this->ExtendToGoalRbt(tree, planner_parameters);
                AlgorithmState state = this->ExtendToGoalRRT(tree, planner_parameters);

                // Get grasp with minimal distance
                unsigned int best_grasp_idx = this->GetBestGrasp(planner_parameters);
                Grasp best_grasp = planner_parameters.target_poses[best_grasp_idx];
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
        // Take measurements
        plan_result.distance_to_goal = best_grasp.best_dist;
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

}