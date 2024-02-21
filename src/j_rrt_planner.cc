#include "ut.h"
#include "j_rrt_planner.h"
#include "bur_tree3d.h"
#include <numeric> // For std::iota

namespace Burs
{
    using namespace Eigen;

    JRRTPlanner::JRRTPlanner(std::string urdf_file) : RbtPlanner(urdf_file)
    {
        this->rng = std::make_shared<RandomNumberGenerator>(1, 1); // temporary seed is one, the proper seed is set from planner parameters
    }

    JRRTPlanner::JRRTPlanner() : RbtPlanner()
    {
        this->rng = std::make_shared<RandomNumberGenerator>(1, 1); // temporary seed is one, the proper seed is set from planner parameters
    }

    std::optional<std::vector<VectorXd>>
    JRRTPlanner::JRRT(VectorXd q_start, JPlusRbtParameters &planner_parameters, PlanningResult &plan_result)
    {
        // Setup rng:
        if (planner_parameters.target_poses.size() < 1)
        {
            throw std::runtime_error("Target poses has length 0!");
        }

        RS start_state = this->NewState(q_start);

        this->rng = std::make_shared<RandomNumberGenerator>(planner_parameters.seed, planner_parameters.target_poses.size());

        auto tree = std::make_shared<BurTree>(start_state, q_start.size());

        this->InitGraspClosestConfigs(planner_parameters, tree, 0);

        double totalNNtime = 0;
        // double totalAddTime = 0;
        double totalRunTime = 0;
        // double totalCollisionTime = 0;
        // double totalGetClosesDistTime = 0;
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
                auto &best_pose = planner_parameters.target_poses[this->GetBestGrasp(planner_parameters)];
                std::cout << "iter: " << k << "/" << planner_parameters.max_iters;
                std::cout << ", tree.size: " << tree->GetNumberOfNodes();
                std::cout << ", distToGoal: " << best_pose.best_dist << ", ";
                std::cout << ", p_close_enough: " << planner_parameters.p_close_enough;
                std::cout << ", totalNNtime: " << totalNNtime;
                std::cout << ", totalCollideAndAddTime: " << totalCollideAndAddTime;
                std::cout << ", totalRunTime: " << totalRunTime << "\n";
                std::cout.flush();
            }
            // END LOGGING

            VectorXd q_rand = this->GetRandomQ(1);
            RS tmp_state = this->NewState(q_rand);
            struct rusage tt1, tt2;
            getTime(&tt1);
            int idx_near = tree->Nearest(tmp_state);
            getTime(&tt2);
            totalNNtime += getTime(tt1, tt2);

            getTime(&tt1);
            int step_result = this->RRTStep(tree, idx_near, tmp_state, planner_parameters.epsilon_q);
            getTime(&tt2);
            totalCollideAndAddTime += getTime(tt1, tt2);

            if (step_result >= 0)
            {
                // Check distance to goal
                RS new_state = *tree->Get(step_result);
                this->SetGraspClosestConfigs(planner_parameters, tree, step_result);
            }

            if (this->rng->getRandomReal() < planner_parameters.probability_to_steer_to_target)
            {
                // Steer until hit the target or obstacle or joint limits
                AlgorithmState state = this->ExtendToGoalRRT(tree, planner_parameters);

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
                    // Get grasp with minimal distance
                    // Get idx in tree that leads to the best config
                    int best_idx = tree->Nearest(best_grasp.best_state);
                    // Take measurements
                    plan_result.distance_to_goal = best_grasp.best_dist;
                    plan_result.num_iterations = k;
                    plan_result.tree_size = tree->GetNumberOfNodes();
                    plan_result.success = true;

                    // Return best path
                    auto path = this->ConstructPathFromTree(tree, best_idx);
                    if (planner_parameters.visualize_tree)
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
        plan_result.success = false;

        // Return best path
        auto path = this->ConstructPathFromTree(tree, best_idx);
        if (planner_parameters.visualize_tree)
        {
            this->tree_csv = this->TreePoints(tree, planner_parameters.visualize_tree);
        }
        return path;
    }

    AlgorithmState
    JRRTPlanner::ExtendToGoalRRT(std::shared_ptr<BurTree> t_a, JPlusRbtParameters &planner_parameters) const
    {
        int randint = this->rng->getRandomInt();
        Grasp random_grasp = planner_parameters.target_poses[randint];
        KDL::Frame p_goal = random_grasp.frame;

        int best_state_idx = random_grasp.best_state;
        RS *best_state = t_a->Get(best_state_idx);
        RS near_state = *best_state;
        // Copy since we will change it

        int prev_idx = t_a->Nearest(best_state_idx);
        double delta_p = random_grasp.best_dist;
        KDL::Frame p_near = this->env->robot->GetEEFrame(*best_state);
        double delta_p_old = 0;
        // TESTING

        do
        {
            KDL::Vector delta_pos = (p_goal.p - p_near.p);
            double metric_dist = delta_pos.Norm();
            auto [d, f_tgt] = this->BasicDistanceMetric(p_near, p_goal, planner_parameters.rotation_dist_ratio);
            delta_p = d;
            bool use_rotation = (delta_p <= planner_parameters.use_rotation);

            // Max dist => epsilon_q
            double dist_to_move = std::min(metric_dist, planner_parameters.epsilon_q);
            RS new_state;
            MatrixXd p_inv = this->env->robot->JPlus(near_state);
            // .completeOrthogonalDecomposition().pseudoInverse();
            VectorXd delta_frame(6);
            delta_frame(0) = delta_pos(0);
            delta_frame(1) = delta_pos(1);
            delta_frame(2) = delta_pos(2);
            if (use_rotation)
            {
                double x, y, z;
                (f_tgt.M * p_near.M.Inverse()).GetEulerZYX(z, y, x);
                // RPY and euler return the same angle
                delta_frame(3) = x;
                delta_frame(4) = y;
                delta_frame(5) = z;
            }
            else
            {
                delta_frame(3) = 0;
                delta_frame(4) = 0;
                delta_frame(5) = 0;
            }
            VectorXd delta_q = p_inv * delta_frame;

            new_state = this->NewState(near_state.config + delta_q);
            // }
            near_state = this->GetEndpoints(near_state, {new_state}, dist_to_move)[0];

            if (this->IsColliding(near_state) || !this->InBounds(near_state.config))
            {
                return AlgorithmState::Trapped;
            }
            prev_idx = t_a->AddNode(prev_idx, near_state);
            this->SetGraspClosestConfigs(planner_parameters, t_a, prev_idx);

            p_near = this->env->robot->GetEEFrame(near_state);

            delta_p_old = delta_p;
        } while (delta_p > planner_parameters.p_close_enough);

        return AlgorithmState::Reached;
    }

}
