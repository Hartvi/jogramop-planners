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

namespace Burs
{
    using namespace Eigen;

    // RbtPlanner::RbtPlanner(int q_dim, ForwardKinematicsParallel f, int max_iters, double d_crit, double delta_q, double epsilon_q, Eigen::MatrixXd bounds, RadiusFuncParallel radius_func, int num_spikes)
    //     : BasePlanner(q_dim, max_iters, epsilon_q, bounds), // Initialize the BasePlanner part of the object
    //       forwardKinematicsParallel(f), d_crit(d_crit), delta_q(delta_q), radius_func(radius_func), num_spikes(num_spikes)
    // {
    // }
    RbtPlanner::RbtPlanner(std::string path_to_urdf_file)
        : RRTPlanner(path_to_urdf_file)
    {
    }

    RbtPlanner::RbtPlanner() : RRTPlanner()
    {
    }

    // double
    // RbtPlanner::RhoR(const VectorXd &q1, const VectorXd &q2) const
    // {
    //     double max_distance = 0.0;

    //     auto res1 = this->forwardKinematicsParallel(q1);
    //     auto res2 = this->forwardKinematicsParallel(q2);

    //     for (unsigned int i = 0; i < res1.size(); ++i)
    //     {
    //         double tmp = (res1[i] - res2[i]).norm();
    //         if (tmp > max_distance)
    //         {
    //             max_distance = tmp;
    //         }
    //     }
    //     return max_distance;
    // }

    // double
    // RbtPlanner::GetDeltaTk(double phi_tk, double tk, const VectorXd &q_e, const VectorXd &q_k) const
    // {
    //     VectorXd r_vec = this->radius_func(q_k);

    //     double denominator = (q_e - q_k).cwiseAbs().dot(r_vec);
    //     return phi_tk * (1.0 - tk) / denominator;
    // }

    std::optional<std::vector<Eigen::VectorXd>>
    RbtPlanner::RbtConnect(const VectorXd &q_start, const VectorXd &q_goal, const RbtParameters &plan_parameters, PlanningResult &planning_result)
    {
        // Givens:
        RS start_state = this->NewState(q_start);
        RS goal_state = this->NewState(q_goal);
        std::shared_ptr<BurTree> t_start = std::make_shared<BurTree>(start_state, q_start.size());
        std::shared_ptr<BurTree> t_goal = std::make_shared<BurTree>(goal_state, q_goal.size());
        // TODO: check collision at the beginning
        // KDL::Frame ee_goal = this->GetEEPose(q_goal);

        // if (this->IsColliding(q_start))
        // {
        //     std::cout << "START COLLIDING\n";
        //     return {};
        // }

        // if (this->IsColliding(q_goal))
        // {
        //     std::cout << "GOAL COLLIDING\n";
        //     return {};
        // }
        // Changing
        auto t_a = t_start;
        auto t_b = t_goal;
        RS &best_state = start_state;
        // VectorXd q_best(q_start);
        double best_dist = 1e10;

        for (int k = 0; k < plan_parameters.max_iters; k++)
        {
            MatrixXd Qe = this->GetRandomQ(plan_parameters.num_spikes);
            std::vector<RS> rand_states = this->NewStates(Qe);

            // Random column
            int nearest_idx = t_a->Nearest(rand_states[0]);
            RS near_state = *t_a->Get(nearest_idx);
            // VectorXd q_near = t_a->Get(nearest_idx);
            // std::cout << "q_near:\n"
            //   << q_near.transpose() << "\n";

            // Slow => maybe in the future use FCL and somehow compile it because it had a ton of compilation errors and version mismatches
            double d_closest = this->GetClosestDistance(near_state);
            // std::cout << "d_closest: " << d_closest << "\n";

            if (d_closest < plan_parameters.d_crit)
            {
                int step_result = this->RRTStep(t_a, nearest_idx, near_state, plan_parameters.epsilon_q);
                if (step_result < 0)
                {
                    // If small basic rrt collides, then don't go here, hence the `continue`
                    // std::cout << "RRT COLLIDE\n";
                    continue;
                }

                if (t_a == t_start)
                {
                    RS tmp_state = *t_start->Get(step_result);
                    // VectorXd tmp_vec = t_start->GetQ(step_result);
                    // double tmp_dist = this->GetDistToGoal(tmp_vec, ee_goal);
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
                std::vector<RS> endpoints = this->GetEndpoints(near_state, rand_states, std::min(d_closest, plan_parameters.delta_q));
                // TRAVELLED DISTANCES ARE INDEED ALWAYS SMALLER THAN D_CLOSEST

                // this->AddDenseBur(t_a);

                for (unsigned int i = 0; i < endpoints.size(); ++i)
                {
                    auto endpoint = endpoints[i];
                    auto max_epsilon_separated_points = this->Densify(near_state, endpoint, plan_parameters);
                    int prev_idx = nearest_idx; // idx of q_near

                    this->AddPointsExceptFirst(t_a, prev_idx, max_epsilon_separated_points);

                    for (RS &point : max_epsilon_separated_points)
                    {
                        // Measure distance to goal if this is the starting tree
                        if (t_a == t_start)
                        {
                            RS tmp_state = *t_start->Get(prev_idx);
                            double tmp_dist = this->env->robot->EEDistance(tmp_state, goal_state);
                            // double tmp_dist = this->GetDistToGoal(tmp_vec, ee_goal);

                            if (tmp_dist < best_dist)
                            {
                                best_dist = tmp_dist;
                                std::cout << "RBT best dist: " << best_dist << "\n";
                                // q_best = tmp_vec;
                                best_state = tmp_state;
                            }
                        }
                    }
                }
            }

            // It is either the one added through RRT, or in the bur
            int last_node_idx = t_a->GetNumberOfNodes() - 1;
            RS q_new = *t_a->Get(last_node_idx);
            // VectorXd q_new = t_a->GetQ(last_node_idx);

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
    RbtPlanner::BurConnect(std::shared_ptr<BurTree> t, RS &state, const RbtParameters &plan_parameters, const RS &goal_state, RS &best_state, double &best_dist)
    {
        int nearest_index = t->Nearest(state);
        RS nearest_state = *t->Get(nearest_index);
        const RS state0 = nearest_state;

        double initial_distance = this->env->robot->MaxDistance(state0, state);
        // VectorXd q_n = t->GetQ(nearest_index);
        // VectorXd q_0(q_n);
        // double initial_distance = this->MaxMovedDistance(q_0, q);

        double delta_s = 1e14;
        double threshold = 1e-2;
        int previous_step = nearest_index;

        while (delta_s >= plan_parameters.delta_q)
        {
            double d_closest = this->GetClosestDistance(nearest_state);
            // std::cout << "BC CLOSE: " << d_closest << "\n";

            // TODO: interpolate burconnect as well
            if (d_closest > plan_parameters.d_crit)
            {
                // if q_n is within the collision free bur of q, then we finish, game over
                RS endpoint = this->GetEndpoints(nearest_state, {state}, d_closest)[0];
                // MatrixXd endpoint = this->GetEndpoints(q_n, q, d_closest);
                // VectorXd q_t = endpoint;

                delta_s = (endpoint.config - nearest_state.config).norm();
                // delta_s = (q_t - q_n).norm();

                std::vector<RS> configs = this->Densify(endpoint, nearest_state, plan_parameters);

                for (unsigned int i = 1; i < configs.size(); ++i)
                {
                    double tmp_dist = this->env->robot->EEDistance(configs[i], goal_state);
                    if (tmp_dist < best_dist)
                    {
                        best_dist = tmp_dist;
                        best_state = configs[i];
                    }
                }

                previous_step = this->AddPointsExceptFirst(t, previous_step, configs);
                nearest_state = endpoint;

                // if (q_n.isApprox(q, threshold))
                if (this->env->robot->MaxDistance(state, nearest_state) < plan_parameters.epsilon_q)
                {
                    std::cout << "RBT reached\n";
                    return {AlgorithmState::Reached, previous_step};
                }
            }
            else
            {
                RS endpoint = this->GetEndpoints(nearest_state, {state}, plan_parameters.epsilon_q)[0];
                // VectorXd q_t = this->GetEndpoints(q_n, q, plan_parameters.epsilon_q);

                int step_result = this->RRTStep(t, previous_step, nearest_state, plan_parameters.epsilon_q);
                if (step_result < 0)
                {
                    std::cout << "RRT trapped\n";
                    return {AlgorithmState::Trapped, previous_step};
                }
                else
                {
                    previous_step = step_result;
                    nearest_state = endpoint;

                    double tmp_dist = this->env->robot->EEDistance(endpoint, goal_state);
                    // double tmp_dist = this->GetDistToGoal(q_t, goal_ee);
                    if (tmp_dist < best_dist)
                    {
                        best_dist = tmp_dist;
                        // q_best = q_t;
                        best_state = endpoint;
                    }
                }

                // if ((q_n - q_0).norm() >= (q - q_0).norm())
                // double qnq0_dist = this->MaxMovedDistance(q_n, q_0);
                double qnq0_dist = this->env->robot->MaxDistance(nearest_state, state0);
                if (qnq0_dist > initial_distance)
                {
                    std::vector<RS> configs = this->Densify(nearest_state, state, plan_parameters);
                    // std::vector<VectorXd> configs = this->Densify(q_n, q, plan_parameters);
                    previous_step = this->AddPointsExceptFirst(t, previous_step, configs);
                    std::cout << "RRT reached\n";
                    return {AlgorithmState::Reached, previous_step};
                }
            }
        }
        return {AlgorithmState::Trapped, previous_step};
    }

    void
    RbtPlanner::AddDenseBur(std::shared_ptr<BurTree> tree, const int &idx_near, const std::vector<RS> &endpoints, JPlusRbtParameters &plan_params) const
    {
        RS near_state = *tree->Get(idx_near);
        // VectorXd q_near = tree->GetQ(idx_near);
        // std::cout << "end " << -1 << " : " << this->GetEEPose(q_near).p << "\n";

        for (unsigned int i = 0; i < endpoints.size(); ++i)
        {
            auto &endpoint = endpoints[i];
            // std::cout << "end " << i << " : " << this->GetEEPose(endpoint).p << "\n";
            auto max_epsilon_separated_points = this->Densify(near_state, endpoint, plan_params);
            int prev_idx = idx_near; // idx of q_near

            int last_idx = this->AddPointsExceptFirst(tree, prev_idx, max_epsilon_separated_points);
            // `last_idx` is the endpoint, all the other points inbetween lead to `idx_near`
            while (last_idx != idx_near)
            {
                // Get distance to goal from each
                this->SetGraspClosestConfigs(plan_params, tree, last_idx);
                last_idx = tree->GetParentIdx(last_idx);
            }
        }
    }

    int
    RbtPlanner::AddPointsExceptFirst(std::shared_ptr<BurTree> t, const int &first_el_idx, const std::vector<RS> vec) const
    {
        int prev_id = first_el_idx;
        for (unsigned int i = 1; i < vec.size(); ++i)
        {
            if (!this->InBounds(vec[i].config))
            {
                // Skip the endpoint if it's out of bounds
                continue;
            }
            prev_id = t->AddNode(prev_id, vec[i]);
        }
        return prev_id;
    }

    std::vector<RS>
    RbtPlanner::Densify(const RS &src, const RS &tgt, const RbtParameters &plan_params) const
    {
        // less than 2x upper distance between neighbouring positions
        double upper_dist = plan_params.q_resolution * 0.5;

        std::vector<RS> configs = {src, tgt};

        for (int i = 0; i + 1 < configs.size();)
        {
            VectorXd middle_config = (configs[i].config + configs[i + 1].config) * 0.5;
            RS ns = this->NewState(middle_config);
            double tmp_dist = this->env->robot->MaxDistance(configs[i], ns);
            // double tmp_dist = this->MaxMovedDistance(configs[i], middle_config);

            if (tmp_dist > upper_dist)
            {
                configs.insert(configs.begin() + i + 1, ns);
            }
            else
            {
                ++i;
            }
        }
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
            double dist_to_goal = (goal.p - ee.p).Norm();
            tgt.best_dist = dist_to_goal;
            tgt.best_state = start_idx;
            // std::cout << "state: " << state.config << "\n";
        }
    }

    void
    RbtPlanner::SetGraspClosestConfigs(JPlusRbtParameters &planner_parameters, std::shared_ptr<BurTree> t, const int &state_idx) const
    {
        auto &tgts = planner_parameters.target_poses;
        // auto ee_q = this->GetEEPose(q);
        KDL::Frame ee = this->env->robot->GetEEFrame(*t->Get(state_idx));

        for (unsigned int i = 0; i < tgts.size(); ++i)
        {
            auto &tgt = tgts[i];
            auto &goal = tgt.frame;
            double dist_to_goal = (goal.p - ee.p).Norm();
            // double dist_to_goal = this->DistanceToGoal(goal, ee_q);
            if (dist_to_goal < tgt.best_dist)
            {
                tgt.best_dist = dist_to_goal;
                tgt.best_state = state_idx;
                // std::cout << "new best config: " << tgt.dv->v.transpose() << "\n";
            }
        }
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

}
