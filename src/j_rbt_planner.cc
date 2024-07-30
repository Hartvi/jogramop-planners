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
        assert(plan_params.distanceEstimateType != DistanceEstimateType::None);
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

        // LOGGING VARS
        double totalNNtime = 0;
        // double totalAddTime = 0;
        double totalRunTime = 0;
        // double totalCollisionTime = 0;
        double totalGetClosestDistTime = 0;
        double totalCollideAndAddTime = 0;
        double totalGetEndpointsTime = 0;
        double totalSetGraspConfigTime = 0;
        struct rusage gt1, gt2;
        getTime(&gt1);

        int num_bad_crashes = 0;
        int numberOfDistanceChecks = 0;
        // END LOGGING VARS

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

            if (this->finished)
            {
                break;
            }

            MatrixXd Qe = this->GetRandomQ(plan_params.num_spikes);

            struct rusage tt1, tt2;
            getTime(&tt1);
            // Random column
            int nearest_idx = tree->Nearest(Qe.col(0).data());
            RS *near_state = tree->Get(nearest_idx);
            for (size_t i = 0; i < Qe.cols(); ++i)
            {
                // normalize
                Qe.col(i).normalize();
                // stretch to cover the whole range
                // Qe.col(i) = Qe.col(i).cwiseProduct(this->bounds.col(1) - this->bounds.col(0));
                Qe.col(i) *= plan_params.delta_q;
                // add to nearest point to set it as the direction from q_near
                Qe.col(i) += near_state->config;
            }
            getTime(&tt2);
            totalNNtime += getTime(tt1, tt2);
            getTime(&tt1);
            if (!near_state->hasClosestDists)
            {
                auto [d_closest_idx, ds_closest] = this->GetClosestDistances(*near_state);
                near_state->hasClosestDists = true;
                near_state->closest_distance_ids = d_closest_idx;
                near_state->closest_dists = ds_closest;
                ++numberOfDistanceChecks;
                // std::cout << "checking dist\n";
            }
            double d_closest = near_state->closest_dists[near_state->closest_distance_ids[0]];
            getTime(&tt2);
            totalGetClosestDistTime += getTime(tt1, tt2);

            bool too_close = d_closest < plan_params.d_crit;

            // Create only one state if too close
            std::vector<RS> Qe_states = this->NewStates((too_close ? Qe.col(0) : Qe), DistanceEstimateType::None);

            if (too_close)
            {
                getTime(&tt1);
                int step_result = this->RRTStepInQ(tree, nearest_idx, Qe_states[0], plan_params.epsilon_q, plan_params.collision_resolution, DistanceEstimateType::None);
                getTime(&tt2);
                totalCollideAndAddTime += getTime(tt1, tt2);
                if (step_result >= 0)
                {
                    // std::cout << "added rrt step\n";
                    getTime(&tt1);
                    this->SetGraspClosestConfigs(plan_params, tree, step_result);
                    getTime(&tt2);
                    totalSetGraspConfigTime += getTime(tt1, tt2);
                }
            }
            else // REGULAR BUR
            {
                getTime(&tt1);
                std::vector<RS> endpoints = this->GetEndpointsGeneral(*near_state, Qe_states, plan_params.distanceEstimateType);
                getTime(&tt2);
                totalGetEndpointsTime += getTime(tt1, tt2);

                for (unsigned int i = 0; i < endpoints.size(); ++i)
                {
                    getTime(&tt1);
                    int res = tree->AddNode(nearest_idx, endpoints[i]);
                    totalCollideAndAddTime += getTime(tt1, tt2);
                    getTime(&tt2);
                    getTime(&tt1);
                    double tmp_dist = this->SetGraspClosestConfigs(plan_params, tree, res);
                    getTime(&tt2);
                    totalSetGraspConfigTime += getTime(tt1, tt2);
                }
            }

            // TRAVELLED DISTANCES ARE INDEED ALWAYS SMALLER THAN D_CLOSEST
            double rand_num = this->rng->getRandomReal();
            if (rand_num < plan_params.probability_to_steer_to_target)
            {
                // Steer until hit the target or obstacle or joint limits
                getTime(&tt1);
                AlgorithmState state = this->ExtendToGoalRRT(tree, plan_params);
                totalCollideAndAddTime += getTime(tt1, tt2);
                getTime(&tt2);

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

                    std::cout << "NUM DIST CHECKS: " << numberOfDistanceChecks << "\n";
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
        if (finished)
        {
            best_idx = tree->Nearest(this->last_state);
        }
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
        // std::cout << "NUM BAD CRASHES: " << num_bad_crashes << "\n";
        std::cout << "NUM DIST CHECKS: " << numberOfDistanceChecks << "\n";
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

}
