#include <cmath>
#include <optional>
#include "base_planner.h"
#include "bur_funcs.h"
#include "rbt_planner.h"
#include <flann/flann.hpp>
#include <Eigen/Dense>
#include <memory>
#include <iostream>
#include <fstream>
#include <vector>
#include "ut.h"

namespace Burs
{
    using namespace Eigen;

    RbtPlanner::RbtPlanner(std::string path_to_urdf_file)
        : RRTPlanner(path_to_urdf_file)
    {
    }

    RbtPlanner::RbtPlanner() : RRTPlanner()
    {
    }

    std::optional<std::vector<Eigen::VectorXd>>
    RbtPlanner::RbtConnectDenseBurs(const VectorXd &q_start, const VectorXd &q_goal, const RbtParameters &plan_parameters, PlanningResult &planning_result)
    {
        std::cout << "RUN DENSE BURS\n";
        // Givens:
        RS start_state = this->NewState(q_start);
        RS goal_state = this->NewState(q_goal);
        std::shared_ptr<BurTree> t_start = std::make_shared<BurTree>(start_state, q_start.size());
        std::shared_ptr<BurTree> t_goal = std::make_shared<BurTree>(goal_state, q_goal.size());

        // Changing
        auto t_a = t_start;
        auto t_b = t_goal;
        RS &best_state = start_state;
        double best_dist = 1e10;

        double totalNNtime = 0;
        double totalAddTime = 0;
        double totalRunTime = 0;
        double totalCollisionTime = 0;
        double totalGetClosestDistTime = 0;
        double totalCollideAndAddTime = 0;
        struct rusage gt1, gt2;
        getTime(&gt1);

        for (int k = 0; k < plan_parameters.max_iters; k++)
        {
            if (k % 1000 == 0)
            {
                getTime(&gt2);
                totalRunTime = getTime(gt1, gt2);
                std::cout << "iter: " << k << "/" << plan_parameters.max_iters;
                std::cout << ", tree.size: " << (t_start->GetNumberOfNodes() + t_goal->GetNumberOfNodes());
                std::cout << ", distToGoal: " << best_dist;
                std::cout << ", totalNNtime: " << totalNNtime;
                std::cout << ", totalCollisionTime: " << totalCollisionTime;
                std::cout << ", totalGetClosestDistTime: " << totalGetClosestDistTime;
                std::cout << ", totalAddTime: " << totalAddTime;
                std::cout << ", totalRunTime: " << totalRunTime << "\n\n";
                std::cout.flush();
            }
            MatrixXd Qe = this->GetRandomQ(plan_parameters.num_spikes);
            std::vector<RS> rand_states = this->NewStates(Qe);

            // Random column
            struct rusage tt1, tt2;
            getTime(&tt1);
            int nearest_idx = t_a->Nearest(rand_states[0]);
            getTime(&tt2);
            totalNNtime += getTime(tt1, tt2);
            RS near_state = *t_a->Get(nearest_idx);

            // Slow => maybe in the future use FCL and somehow compile it because it had a ton of compilation errors and version mismatches
            getTime(&tt1);
            double d_closest = this->GetClosestDistance(near_state);
            getTime(&tt2);
            totalGetClosestDistTime += getTime(tt1, tt2);

            if (d_closest < plan_parameters.d_crit)
            {
                RS q_t = this->GetEndpoints(near_state, {rand_states[0]}, plan_parameters.epsilon_q)[0];

                getTime(&tt1);
                if (this->IsColliding(q_t))
                {
                    continue;
                }
                getTime(&tt2);
                totalCollisionTime += getTime(tt1, tt2);

                getTime(&tt1);
                int step_result = t_a->AddNode(nearest_idx, q_t);
                getTime(&tt2);
                totalAddTime += getTime(tt1, tt2);

                if (t_a == t_start)
                {
                    RS tmp_state = *t_start->Get(step_result);
                    double tmp_dist = this->env->robot->EEDistance(goal_state, tmp_state);

                    if (tmp_dist < best_dist)
                    {
                        std::cout << "RRT best dist: " << best_dist << "\n";
                        best_dist = tmp_dist;
                        best_state = tmp_state;
                    }
                }
            }
            else
            {
                std::vector<std::vector<RS>> endpoints = this->GetEndpointsInterstates(near_state, rand_states, d_closest, plan_parameters.q_resolution);
                // TRAVELLED DISTANCES ARE INDEED ALWAYS SMALLER THAN D_CLOSEST

                for (unsigned int i = 0; i < endpoints.size(); ++i)
                {
                    auto endpoint_vec = endpoints[i];
                    int prev_idx = nearest_idx; // idx of q_near

                    for (unsigned int l = 0; l < endpoint_vec.size(); ++l)
                    {
                        prev_idx = t_a->AddNode(prev_idx, endpoint_vec[l]);
                        if (t_a == t_start)
                        {
                            RS tmp_state = *t_start->Get(prev_idx);
                            double tmp_dist = this->env->robot->EEDistance(tmp_state, goal_state);

                            if (tmp_dist < best_dist)
                            {
                                best_dist = tmp_dist;
                                std::cout << "RBT best dist: " << best_dist << "\n";
                                best_state = tmp_state;
                            }
                        }
                    }
                }
            }

            // It is either the one added through RRT, or in the bur
            int last_node_idx = t_a->GetNumberOfNodes() - 1;
            RS q_new = *t_a->Get(last_node_idx);

            auto [status, best_idx_t_b] = this->BurConnect(t_b, q_new, plan_parameters, goal_state, best_state, best_dist);
            if (status == AlgorithmState::Reached)
            {
                int start_closest;
                int goal_closest;
                if (t_b == t_goal)
                {
                    goal_closest = best_idx_t_b;
                    start_closest = last_node_idx;
                }
                else
                { // t_b == t_start
                    start_closest = best_idx_t_b;
                    goal_closest = last_node_idx;
                }

                planning_result.distance_to_goal = 0.0;
                planning_result.num_iterations = k;
                planning_result.tree_size = t_start->GetNumberOfNodes() + t_goal->GetNumberOfNodes();
                planning_result.success = true;

                auto path = this->Path(t_start, start_closest, t_goal, goal_closest);
                // std::cout << "path ";
                if (plan_parameters.visualize_tree > 0)
                {
                    this->tree_csv = this->TreePoints(t_start, plan_parameters.visualize_tree);
                }
                return path;
            }
            std::swap(t_a, t_b);
        }

        planning_result.num_iterations = plan_parameters.max_iters;
        planning_result.tree_size = t_start->GetNumberOfNodes() + t_goal->GetNumberOfNodes();
        planning_result.success = false;

        int best_idx = t_start->Nearest(best_state);
        // planning_result.distance_to_goal = this->GetDistToGoal(q_best, ee_goal);
        planning_result.distance_to_goal = this->env->robot->EEDistance(best_state, goal_state);

        if (plan_parameters.visualize_tree > 0)
        {
            std::cout << "visualizing tree\n";
            this->tree_csv = this->TreePoints(t_start, plan_parameters.visualize_tree);
        }
        return this->ConstructPathFromTree(t_start, best_idx);
    }

    std::pair<AlgorithmState, int>
    RbtPlanner::BurConnect(std::shared_ptr<BurTree> t, const RS &q, const RbtParameters &plan_parameters, const RS &goal_state, RS &best_state, double &best_dist)
    {
        RS tmp_q = q;
        int n_idx = t->Nearest(tmp_q);
        RS q_n = *t->Get(n_idx);
        const RS q_0 = q_n;

        double initial_distance = this->env->robot->MaxDistance(q_0, q) - 1e-6; // small tolerance

        double delta_s = 1e14;
        double threshold = 1e-2;
        int previous_step = n_idx;

        while (delta_s >= plan_parameters.delta_q)
        {
            double d_closest = this->GetClosestDistance(q_n);

            if (d_closest > plan_parameters.d_crit)
            {
                // if q_n is within the collision free bur of q, then we finish, game over
                std::vector<RS> q_t = this->GetEndpointsInterstates(q_n, {q}, d_closest, plan_parameters.q_resolution)[0];

                delta_s = (q_t.back().config - q_n.config).norm();

                // previous_step = this->AddPointsExceptFirst(t, previous_step, configs);
                for (unsigned int i = 0; i < q_t.size(); ++i)
                {
                    previous_step = t->AddNode(previous_step, q_t[i]);
                }
                q_n = q_t.back();

                if (this->env->robot->MaxDistance(q, q_n) < plan_parameters.epsilon_q)
                {
                    std::cout << "RBT reached\n";
                    return {AlgorithmState::Reached, previous_step};
                }
            }
            else
            {
                RS q_t = this->GetEndpoints(q_n, {q}, plan_parameters.epsilon_q)[0];

                if (this->IsColliding(q_t))
                {
                    return {AlgorithmState::Trapped, -1};
                }
                else
                { // not colliding
                    previous_step = t->AddNode(previous_step, q_t);
                    q_n = q_t;

                    double tmp_dist = this->env->robot->EEDistance(q_n, goal_state);
                    if (tmp_dist < best_dist)
                    {
                        best_dist = tmp_dist;
                        best_state = q_n;
                    }
                }
                // if ((q_n - q_0).norm() >= (q - q_0).norm())
                double qnq0_dist = this->env->robot->MaxDistance(q_n, q_0);
                if (qnq0_dist >= initial_distance)
                {
                    std::cout << "RRT reached\n";
                    return {AlgorithmState::Reached, previous_step};
                }
            }
        }
        return {AlgorithmState::Trapped, -1};
    }

    void
    RbtPlanner::InitGraspClosestConfigs(JPlusRbtParameters &planner_parameters, std::shared_ptr<BurTree> t, const int &start_idx) const
    {
        auto &tgts = planner_parameters.target_poses;
        KDL::Frame ee = this->env->robot->GetEEFrame(*t->Get(start_idx));

        for (unsigned int i = 0; i < tgts.size(); ++i)
        {
            auto &tgt = tgts[i];
            auto &goal = tgt.frame;
            auto [dist_to_goal, f] = this->BasicDistanceMetric(ee, goal, planner_parameters.rotation_dist_ratio);
            // double dist_to_goal = (goal.p - ee.p).Norm();
            tgt.best_dist = dist_to_goal;
            tgt.best_state = start_idx;
            // std::cout << "q: " << q.config << "\n";
        }
    }

    double
    RbtPlanner::SetGraspClosestConfigs(JPlusRbtParameters &planner_parameters, std::shared_ptr<BurTree> t, const int &state_idx) const
    {
        auto &tgts = planner_parameters.target_poses;
        KDL::Frame ee = this->env->robot->GetEEFrame(*t->Get(state_idx));

        double tmp_dist = 1e10;
        for (unsigned int i = 0; i < tgts.size(); ++i)
        {
            auto &tgt = tgts[i];
            auto &goal = tgt.frame;
            auto [dist_to_goal, f] = this->BasicDistanceMetric(ee, goal, planner_parameters.rotation_dist_ratio);
            if (dist_to_goal < tgt.best_dist)
            {
                tgt.best_dist = dist_to_goal;
                tgt.best_state = state_idx;
            }
            if (dist_to_goal < tmp_dist)
            {
                dist_to_goal = tmp_dist;
            }
        }
        return tmp_dist;
    }

    unsigned int
    RbtPlanner::GetBestGrasp(JPlusRbtParameters &planner_parameters) const
    {
        auto &tgts = planner_parameters.target_poses;
        double best_dist = 1e14;
        int best_idx = -1;

        for (unsigned int i = 0; i < tgts.size(); ++i)
        {
            auto &tgt = tgts[i];
            auto &goal = tgt.frame;
            if (tgt.best_dist < best_dist)
            {
                best_dist = tgt.best_dist;
                best_idx = i;
            }
        }
        return best_idx;
    }

    std::pair<double, KDL::Frame>
    RbtPlanner::BasicDistanceMetric(const KDL::Frame &ee, const KDL::Frame &tgt, const double &angle_ratio) const
    {
        // units [mm]
        double dist = (ee.p - tgt.p).Norm() * 1000;
        KDL::Rotation ee_inv = ee.M.Inverse();

        auto [changed_rot, closest_grasp_rot] = this->GetClosestSymmetricGrasp(tgt.M, ee.M);
        KDL::Rotation r = tgt.M * ee_inv;
        // units [deg]
        double angle_dist = acos((r(0, 0) + r(1, 1) + r(2, 2) - 1.0) * 0.5) * rad_to_deg;
        // if the symmetric rotation is a different one
        if (changed_rot)
        {
            KDL::Rotation r2 = closest_grasp_rot * ee_inv;
            double angle_dist2 = acos((r2(0, 0) + r2(1, 1) + r2(2, 2) - 1.0) * 0.5) * rad_to_deg;
            // take the smaller of the two distances
            if (angle_dist2 < angle_dist)
            {
                angle_dist = angle_dist2;
            }
            else
            {
            }
        }

        // 2 * cos + 1 on diagonal *always*
        KDL::Frame f(r, tgt.p);
        return {(1.0 - angle_ratio) * dist + angle_ratio * angle_dist, f};
    }

    std::pair<bool, KDL::Rotation>
    RbtPlanner::GetClosestSymmetricGrasp(const KDL::Rotation &rotMatGrasp, const KDL::Rotation &rotMatEE) const
    {
        // Extract the Y-axis (second column) from both rotation matrices
        KDL::Vector yGrasp = KDL::Vector(rotMatGrasp(0, 1), rotMatGrasp(1, 1), rotMatGrasp(2, 1));
        KDL::Vector yEE = KDL::Vector(rotMatEE(0, 1), rotMatEE(1, 1), rotMatEE(2, 1));

        // Initialize the closest grasp rotation matrix to the input grasp rotation matrix
        KDL::Rotation closestGrasp(rotMatGrasp);

        // Check if the dot product of the Y-axes is negative, indicating opposing directions
        double dot_product = (yGrasp(0) * yEE(0) + yGrasp(1) * yEE(1) + yGrasp(2) * yEE(2));
        bool changed_rot = dot_product < 0;
        if (changed_rot)
        {
            // Need to flip both the X (first column) and Y-axis (second column)
            closestGrasp(0, 0) = -closestGrasp(0, 0); // Flip X-axis
            closestGrasp(1, 0) = -closestGrasp(1, 0); // Flip X-axis
            closestGrasp(2, 0) = -closestGrasp(2, 0); // Flip X-axis
            closestGrasp(0, 1) = -closestGrasp(0, 1); // Flip Y-axis
            closestGrasp(1, 1) = -closestGrasp(1, 1); // Flip Y-axis
            closestGrasp(2, 1) = -closestGrasp(2, 1); // Flip Y-axis
        }

        return {changed_rot, closestGrasp};
    }

}
