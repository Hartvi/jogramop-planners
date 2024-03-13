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

    RbtPlanner::RbtPlanner() : RRTPlanner() {}

    std::optional<std::vector<Eigen::VectorXd>>
    RbtPlanner::RbtConnect(const VectorXd &q_start, const VectorXd &q_goal, const RbtParameters &plan_parameters, PlanningResult &planning_result)
    {
        // Givens:
        RS start_state = this->NewState(q_start);
        RS goal_state = this->NewState(q_goal);
        std::shared_ptr<BurTree> t_start = std::make_shared<BurTree>(start_state, q_start.size());
        std::shared_ptr<BurTree> t_goal = std::make_shared<BurTree>(goal_state, q_goal.size());

        if (this->IsColliding(start_state))
        {
            std::cout << "START COLLIDING\n";
            return {};
        }

        if (this->IsColliding(goal_state))
        {
            std::cout << "GOAL COLLIDING\n";
            return {};
        }
        // Changing
        auto t_a = t_start;
        auto t_b = t_goal;
        RS &best_state = start_state;
        double best_dist = 1e10;

        for (int k = 0; k < plan_parameters.max_iters; k++)
        {
            MatrixXd Qe = this->GetRandomQ(plan_parameters.num_spikes);
            std::vector<RS> rand_states = this->NewStates(Qe);

            // Random column
            int nearest_idx = t_a->Nearest(rand_states[0]);
            RS near_state = *t_a->Get(nearest_idx);

            // Slow => maybe in the future use FCL and somehow compile it because it had a ton of compilation errors and version mismatches
            // double d_closest = this->GetClosestDistance(near_state);
            if (near_state.closest_distance_idx < 0)
            {
                auto [d_closest_idx, ds_closest] = this->GetClosestDistances(near_state);
                near_state.closest_distance_idx = d_closest_idx;
                near_state.closest_dists = ds_closest;
            }
            else
            {
                std::cout << "REUSING DISTANCE\n";
            }
            double d_closest = near_state.closest_dists[near_state.closest_distance_idx];
            // std::cout << "d_closest: " << d_closest << "\n";

            if (d_closest < plan_parameters.d_crit)
            {
                // int step_result = this->RRTStepInQ(t_a, nearest_idx, near_state, plan_parameters.epsilon_q);
                int step_result = this->RRTStepInQ(t_a, nearest_idx, rand_states[0], plan_parameters.epsilon_q, plan_parameters.collision_resolution, true);
                if (step_result < 0)
                {
                    // If small basic rrt collides, then don't go here, hence the `continue`
                    std::cout << "RRT COLLIDE\n";
                    continue;
                }

                if (t_a == t_start)
                {
                    RS tmp_state = *t_start->Get(step_result);
                    double tmp_dist = this->env->robot->EEDistance(goal_state, tmp_state);

                    if (tmp_dist < best_dist)
                    {
                        std::cout << "RRT best dist: " << best_dist << "\n";
                        best_dist = tmp_dist;
                        best_state = tmp_state;
                        // q_best = tmp_vec;
                    }
                }
            }
            else
            {
                // Qe is scaled to max euclidean delta_q or closest obstacle distance
                std::vector<RS> endpoints = this->GetEndpoints(near_state, rand_states, d_closest);

                for (unsigned int i = 0; i < endpoints.size(); ++i)
                {
                    auto q_t = endpoints[i];
                    if (!this->IsColliding(q_t))
                    {
                        t_a->AddNode(nearest_idx, q_t);
                        // std::cout << "IS COLLIDING: " << q_t.config.transpose() << "\n  parent: " << near_state.config << "\n\n";
                        // exit(1);
                    }
                    // Measure distance to goal if this is the starting tree
                    if (t_a == t_start)
                    {
                        double tmp_dist = this->env->robot->EEDistance(q_t, goal_state);

                        if (tmp_dist < best_dist)
                        {
                            best_dist = tmp_dist;
                            std::cout << "RBT best dist: " << best_dist << "\n";
                            // q_best = tmp_vec;
                            best_state = q_t;
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
                // `q_new` is in `t_a`
                // `t_b` extends to `q_new` => it has a node near `q_new`

                planning_result.distance_to_goal = 0.0;
                planning_result.num_iterations = k;
                planning_result.tree_size = t_start->GetNumberOfNodes() + t_goal->GetNumberOfNodes();
                planning_result.success = true;

                auto path = this->Path(t_start, start_closest, t_goal, goal_closest);
                // std::cout << "path ";
                if (plan_parameters.visualize_tree)
                {
                    this->tree_csv = this->TreePoints(t_start, 100);
                }
                return path;
            }
            // exit(1);
            std::swap(t_a, t_b);
        }

        planning_result.num_iterations = plan_parameters.max_iters;
        planning_result.tree_size = t_start->GetNumberOfNodes() + t_goal->GetNumberOfNodes();
        planning_result.success = false;

        int best_idx = t_start->Nearest(best_state);
        // planning_result.distance_to_goal = this->GetDistToGoal(q_best, ee_goal);
        planning_result.distance_to_goal = this->env->robot->EEDistance(best_state, goal_state);

        if (plan_parameters.visualize_tree)
        {
            this->tree_csv = this->TreePoints(t_start, 100);
        }
        return this->ConstructPathFromTree(t_start, best_idx);
    }

    std::pair<AlgorithmState, int>
    RbtPlanner::BurConnect(std::shared_ptr<BurTree> t, const RS &q, const RbtParameters &plan_parameters, const RS &goal_state, RS &best_state, double &best_dist)
    {
        int debugcounter = 0;

        RS tmp_q = q;
        int n_idx = t->Nearest(tmp_q);
        RS q_n = *t->Get(n_idx);
        const RS q_0 = q_n;

        // MUST BE SLIGHTLy smaller than the ACTUAL DISTANCE, OTHERWISE IT DOESN'T FINISH
        double initial_distance = (q_0.config - q.config).norm() - 1e-6; // small tolerance

        double delta_s = 1e14;
        double threshold = 1e-2;
        int previous_step = n_idx;

        while (delta_s >= plan_parameters.epsilon_q)
        {
            // double d_closest = this->GetClosestDistance(q_n);
            if (q_n.closest_distance_idx < 0)
            {
                auto [d_closest_idx, ds_closest] = this->GetClosestDistances(q_n);
                q_n.closest_distance_idx = d_closest_idx;
                q_n.closest_dists = ds_closest;
            }
            else
            {
                std::cout << "REUSING DISTANCE\n";
            }
            double d_closest = q_n.closest_dists[q_n.closest_distance_idx];

            if (d_closest > plan_parameters.d_crit)
            {
                // if q_n is within the collision free bur of q, then we finish, game over
                std::vector<RS> q_t = this->GetEndpoints(q_n, {q}, d_closest);

                delta_s = (q_t.back().config - q_n.config).norm();

                for (unsigned int i = 0; i < q_t.size(); ++i)
                {
                    previous_step = t->AddNode(previous_step, q_t[i]);
                }
                q_n = q_t.back();

                // if (q_n.isApprox(q, threshold))
                if ((q.config - q_n.config).squaredNorm() < 1e-8)
                {
                    std::cout << "RBT reached\n";
                    return {AlgorithmState::Reached, previous_step};
                }
            }
            else
            {
                // std::cout << "q tgt: " << q.config.transpose() << "\n";
                int q_t_idx = this->RRTStepInQ(t, previous_step, q, plan_parameters.epsilon_q, plan_parameters.collision_resolution, true);

                if (q_t_idx < 0)
                {
                    std::cout << "burconnect rrt trapped\n";
                    return {AlgorithmState::Trapped, -1};
                }
                else
                { // not colliding
                    // q_n = q_t;
                    q_n = *t->Get(q_t_idx);
                    previous_step = q_t_idx;
                    // std::cout << "q_n: " << q_n.config.transpose() << "\n";

                    double tmp_dist = this->env->robot->EEDistance(q_n, goal_state);
                    if (tmp_dist < best_dist)
                    {
                        best_dist = tmp_dist;
                        best_state = q_n;
                    }
                }

                // if ((q_n - q_0).norm() >= (q - q_0).norm())
                double qnq0_dist = (q_n.config - q_0.config).norm();
                if (qnq0_dist >= initial_distance)
                {
                    std::cout << "RRT reached\n";
                    return {AlgorithmState::Reached, previous_step};
                }
            }
        }
        std::cout << "rbt step too small\n";
        return {AlgorithmState::Trapped, -1};
    }

    std::vector<RS>
    RbtPlanner::Densify(const RS &src, const RS &tgt, const RbtParameters &plan_params) const
    {
        // less than 2x upper distance between neighbouring positions
        double upper_dist = plan_params.q_resolution * 0.5;

        std::vector<RS> configs = {};
        double maxdist = this->env->robot->MaxDistance(src, tgt);
        VectorXd delta_vec = tgt.config - src.config;

        bool failed_distance = false;
        int num_splits = maxdist / plan_params.q_resolution;

        configs.push_back(src);
        for (int i = 1; i < num_splits; ++i)
        {
            VectorXd new_config(this->q_dim);
            for (int j = 0; j < this->q_dim; ++j)
            {
                new_config(j) = src.config(j) + ((double)i) * delta_vec(j) / (double)num_splits;
            }
            RS ns = this->NewState(new_config);
            double tmp_dist = this->env->robot->MaxDistance(src, ns);

            if (maxdist < tmp_dist)
            {
                failed_distance = true;
            }
            else
            {
                configs.push_back(ns);
            }
        }
        configs.push_back(tgt);
        return configs;
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
            tgt.best_dist = dist_to_goal;
            tgt.best_state = start_idx;
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

    std::optional<std::vector<Eigen::VectorXd>>
    RbtPlanner::TestCollisionVsDistanceTime(const VectorXd &q_start, const RbtParameters &plan_parameters, PlanningResult &planning_result)
    {
        struct rusage tt1, tt2;
        // getTime(&tt1);
        double totalCollideTime = 0.0;
        double totalDistanceCheckTime = 0.0;
        for (unsigned int k = 0; k < plan_parameters.max_iters; ++k)
        {
            RS rand_state = this->NewState(this->GetRandomQ(1));

            getTime(&tt1);
            this->IsColliding(rand_state);
            getTime(&tt2);
            totalCollideTime += getTime(tt1, tt2);

            getTime(&tt1);
            this->GetClosestDistance(rand_state);
            getTime(&tt2);
            totalDistanceCheckTime += getTime(tt1, tt2);
        }
        std::cout << "TOTAL COLLIDE TIME: " << totalCollideTime << "\n";
        std::cout << "TOTAL DISTANCE CHECK TIME: " << totalDistanceCheckTime << "\n";
        return {{q_start}};
    }

    std::pair<double, KDL::Frame>
    RbtPlanner::BasicDistanceMetric(const KDL::Frame &ee, const KDL::Frame &tgt, const double &angle_ratio) const
    {
        // units [meters]
        double dist = (ee.p - tgt.p).Norm() * 1000;
        KDL::Rotation ee_inv = ee.M.Inverse();

        auto [changed_rot, closest_grasp_rot] = this->GetClosestSymmetricGrasp(tgt.M, ee.M);
        KDL::Rotation r = tgt.M * ee_inv;
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
                // r = r2;
            }
            else
            {
                // std::cout << "default was smaller: " << angle_dist << " < " << angle_dist2 << "\n";
            }
        }
        // KDL::Rotation invm = tgt.M * ee_inv.Inverse();
        // KDL::Rotation r = (tgt.M * ee.M);
        // KDL::Rotation r2 = (closest_grasp_rot.Inverse() * ee.M);

        // 2 * cos + 1 on diagonal *always*
        // units: [degrees / 1000] to make it equivalent to [mm * deg]
        KDL::Frame f(r, tgt.p);
        // std::cout << "ratio: " << angle_ratio << " metric: " << dist << " angular: " << angle_dist << " total: " << (1.0 - angle_ratio) * dist + angle_ratio * angle_dist << "\n";
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
