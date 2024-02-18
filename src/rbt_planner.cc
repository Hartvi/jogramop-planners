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
                    auto q_t = endpoints[i];
                    auto max_epsilon_separated_points = this->Densify(near_state, q_t, plan_parameters);
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
    RbtPlanner::BurConnect(std::shared_ptr<BurTree> t, const RS &q, const RbtParameters &plan_parameters, const RS &goal_state, RS &best_state, double &best_dist)
    {
        // std::cout << "\n\ninitq: " << q.config.transpose() << "\n";
        int debugcounter = 0;

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
            // std::cout << "BC CLOSE: " << d_closest << "\n";

            if (d_closest > plan_parameters.d_crit)
            {
                // if q_n is within the collision free bur of q, then we finish, game over
                // RS q_t = this->GetEndpoints(q_n, {q}, d_closest)[0];
                std::vector<RS> q_t = this->GetEndpointsInterstates(q_n, {q}, d_closest, plan_parameters.q_resolution)[0];

                delta_s = (q_t.back().config - q_n.config).norm();

                // std::vector<RS> configs = this->Densify(q_n, q_t, plan_parameters);

                // for (unsigned int i = 1; i < configs.size(); ++i)
                // {
                //     double tmp_dist = this->env->robot->EEDistance(configs[i], goal_state);
                //     if (tmp_dist < best_dist)
                //     {
                //         best_dist = tmp_dist;
                //         best_state = configs[i];
                //     }
                // }

                // previous_step = this->AddPointsExceptFirst(t, previous_step, configs);
                for (unsigned int i = 0; i < q_t.size(); ++i)
                {
                    previous_step = t->AddNode(previous_step, q_t[i]);
                }
                q_n = q_t.back();

                // if (q_n.isApprox(q, threshold))
                if (this->env->robot->MaxDistance(q, q_n) < plan_parameters.epsilon_q)
                {
                    std::cout << "RBT reached\n";
                    return {AlgorithmState::Reached, previous_step};
                }
            }
            else
            {
                RS q_t = this->GetEndpoints(q_n, {q}, plan_parameters.epsilon_q)[0];
                // std::cout << "prev: " << t->Get(previous_step)->frames.back().p << "\n";

                if (this->IsColliding(q_t))
                {
                    // std::cout << "RRT trapped\n";
                    return {AlgorithmState::Trapped, -1};
                }
                else
                { // not colliding
                    previous_step = t->AddNode(previous_step, q_t);
                    // std::cout << "q_0: " << q_0.frames.back().p << "\n";
                    // std::cout << "prev: " << q_n.frames.back().p << "\n";
                    q_n = q_t;
                    // std::cout << "next: " << q_t.frames.back().p << "\n";

                    double tmp_dist = this->env->robot->EEDistance(q_n, goal_state);
                    if (tmp_dist < best_dist)
                    {
                        best_dist = tmp_dist;
                        best_state = q_n;
                    }
                }
                // if (debugcounter > 5)
                // {
                //     exit(1);
                // }
                // debugcounter++;

                // if ((q_n - q_0).norm() >= (q - q_0).norm())
                double qnq0_dist = this->env->robot->MaxDistance(q_n, q_0);
                // std::cout << "init dist: " << initial_distance << " qnq0 dist: " << qnq0_dist << "\n";
                if (qnq0_dist >= initial_distance)
                {
                    // std::vector<RS> configs = this->Densify(q_n, q, plan_parameters);
                    // std::vector<VectorXd> configs = this->Densify(q_n, q, plan_parameters);
                    // previous_step = this->AddPointsExceptFirst(t, previous_step, configs);
                    std::cout << "RRT reached\n";
                    return {AlgorithmState::Reached, previous_step};
                }
            }
        }
        return {AlgorithmState::Trapped, -1};
    }

    std::vector<double>
    RbtPlanner::AddDenseBur(std::shared_ptr<BurTree> tree, const int &idx_near, const std::vector<RS> &endpoints, JPlusRbtParameters &plan_params) const
    {
        RS near_state = *tree->Get(idx_near);
        // VectorXd q_near = tree->GetQ(idx_near);
        // std::cout << "end " << -1 << " : " << this->GetEEPose(q_near).p << "\n";
        std::vector<double> distances(endpoints.size(), 1e10);
        for (unsigned int i = 0; i < endpoints.size(); ++i)
        {
            auto &q_t = endpoints[i];
            // std::cout << "end " << i << " : " << this->GetEEPose(q_t).p << "\n";
            auto max_epsilon_separated_points = this->Densify(near_state, q_t, plan_params);
            // for (unsigned int k = 0; k < max_epsilon_separated_points.size(); ++k)
            // {
            //     if (this->IsColliding(max_epsilon_separated_points[k]))
            //     {
            //         std::cout << "IS COLLIDING\n";
            //     }
            // }
            int prev_idx = idx_near; // idx of q_near

            int last_idx = this->AddPointsExceptFirst(tree, prev_idx, max_epsilon_separated_points);
            // `last_idx` is the q_t, all the other points inbetween lead to `idx_near`
            while (last_idx != idx_near)
            {
                // Get distance to goal from each
                double tmp_dist = this->SetGraspClosestConfigs(plan_params, tree, last_idx);

                if (tmp_dist < distances[i])
                {
                    distances[i] = tmp_dist;
                }
                last_idx = tree->GetParentIdx(last_idx);
            }
        }
        return distances;
    }

    int
    RbtPlanner::AddPointsExceptFirst(std::shared_ptr<BurTree> t, const int &first_el_idx, const std::vector<RS> vec) const
    {
        int prev_id = first_el_idx;
        for (unsigned int i = 1; i < vec.size(); ++i)
        {
            if (!this->InBounds(vec[i].config))
            {
                // Skip the q_t if it's out of bounds
                continue;
            }

            // assert(!this->IsColliding(vec[i]));

            // if (this->IsColliding(vec[i]))
            // {
            //     std::cout << "IS COLLIDING AT STEP " << i << " / " << (vec.size() - 1) << "\n";
            //     std::cout << "distance travelled: " << this->env->robot->MaxDistance(vec[i], vec[0]) << "\n";
            //     std::cout << "start to end distance: " << this->env->robot->MaxDistance(vec.back(), vec[0]) << "\n";
            //     // std::cout << "diff: ";
            //     // for (unsigned int l = 0; l < vec[i].frames.size(); ++l)
            //     // {
            //     //     std::cout << (vec[0].frames[l].p - vec.back().frames[l].p).Norm() << ", ";
            //     // }
            //     // std::cout << "config 0: \n " << vec[0].config.transpose() << "\n";
            //     // std::cout << "config last: \n " << vec.back().config.transpose() << "\n";
            //     exit(1);
            // }

            // std::cout << " vecs: " << t->Get(prev_id)->config.transpose() << ", " << vec[i].config.transpose();
            // std::cout << " diff: " << (t->Get(prev_id)->config - vec[i].config).norm() << "\n";
            // The below seems to have gone
            // assert(!t->Get(prev_id)->config.isApprox(vec[i].config, 0.001));
            prev_id = t->AddNode(prev_id, vec[i]);
        }
        return prev_id;
    }

    std::vector<RS>
    RbtPlanner::Densify(const RS &src, const RS &tgt, const RbtParameters &plan_params) const
    {
        // less than 2x upper distance between neighbouring positions
        double upper_dist = plan_params.q_resolution * 0.5;

        std::vector<RS> configs = {};
        // return configs;
        double maxdist = this->env->robot->MaxDistance(src, tgt);
        // std::cout << "Densify: src and tgt dist: " << maxdist << "\n";
        VectorXd delta_vec = tgt.config - src.config;

        bool failed_distance = false;
        int num_splits = maxdist / plan_params.q_resolution;
        // configs.reserve(num_splits + 1);

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
                // std::cout << "halfway distance higher than final distance: " << tmp_dist << " > " << maxdist << "\n";
                // // exit(1);
                // std::cout << "ns: \n"
                //           << ns.config.transpose() << "\n";
                failed_distance = true;
            }
            else
            {
                // std::cout << "successfully added\n";
                configs.push_back(ns);
            }
        }
        // std::cout << "\n";
        configs.push_back(tgt);
        // for (int i = 0; i + 1 < configs.size();)
        // {
        //     VectorXd middle_config = (configs[i].config + configs[i + 1].config) * 0.5;
        //     RS ns = this->NewState(middle_config);
        //     double tmp_dist = this->env->robot->MaxDistance(configs[i], ns);

        //     if (maxdist < tmp_dist)
        //     {
        //         std::cout << "halfway distance higher than final distance: " << maxdist << " < " << tmp_dist << "\n";
        //         // exit(1);
        //         failed_distance = true;
        //     }
        //     // double tmp_dist = this->MaxMovedDistance(configs[i], middle_config);

        //     if (tmp_dist > upper_dist)
        //     {
        //         configs.insert(configs.begin() + i + 1, ns);
        //     }
        //     else
        //     {
        //         ++i;
        //     }
        // }

        // if (failed_distance)
        //     exit(1);
        // if (failed_distance)
        // {
        //     std::cout << "start: \n"
        //               << src.config.transpose() << "\n";
        //     std::cout << "end: \n"
        //               << tgt.config.transpose() << "\n";
        //     std::cout << "num interpoints: " << configs.size() << "\n";
        //     exit(1);
        // }
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
            auto [dist_to_goal, f] = this->BasicDistanceMetric(ee, goal);
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
        // auto ee_q = this->GetEEPose(q);
        KDL::Frame ee = this->env->robot->GetEEFrame(*t->Get(state_idx));

        double tmp_dist = 1e10;
        for (unsigned int i = 0; i < tgts.size(); ++i)
        {
            auto &tgt = tgts[i];
            auto &goal = tgt.frame;
            auto [dist_to_goal, f] = this->BasicDistanceMetric(ee, goal);
            // std::cout << "dist: " << dist_to_goal << "\n";
            // double dist_to_goal = (goal.p - ee.p).Norm();
            if (dist_to_goal < tgt.best_dist)
            {
                // std::cout << "new best dist: " << dist_to_goal << "\n";
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
            // std::cout << "current dist: " << tgt.best_dist << "\n";
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
    RbtPlanner::BasicDistanceMetric(const KDL::Frame &ee, const KDL::Frame &tgt) const
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
                // std::cout << "symmetric has smaller angle: " << angle_dist2 << " < " << angle_dist << "\n";
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
        // if (acos((r(0, 0) + r(1, 1) + r(2, 2) - 1.0) * 0.5) <= acos((r2(0, 0) + r2(1, 1) + r2(2, 2) - 1.0) * 0.5))
        // {
        // std::cout << "default angle dist: " << acos((r(0, 0) + r(1, 1) + r(2, 2) - 1.0) * 0.5) << "\n";
        // std::cout << "symmetr angle dist: " << acos((r2(0, 0) + r2(1, 1) + r2(2, 2) - 1.0) * 0.5) << "\n";
        // std::cout << "reverse def angle dist: " << acos((invm(0, 0) + invm(1, 1) + invm(2, 2) - 1.0) * 0.5) << "\n";
        // std::cout << "reverse sym angle dist: " << acos((invm2(0, 0) + invm2(1, 1) + invm2(2, 2) - 1.0) * 0.5) << "\n";
        // std::cout << "\n";
        // }
        // assert(acos((r(0, 0) + r(1, 1) + r(2, 2) - 1.0) * 0.5) <= acos((r2(0, 0) + r2(1, 1) + r2(2, 2) - 1.0) * 0.5));

        // 2 * cos + 1 on diagonal *always*
        // units: [degrees / 1000] to make it equivalent to [mm * deg]
        KDL::Frame f(r, tgt.p);
        return {dist + angle_dist, f};
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
        // std::cout << "dot product: " << dot_product << "\n";
        // std::cout << "arccos dot product: " << acos(dot_product) << "\n";
        // std::cout << "closest: \n"
        //           << closestGrasp << "\n";
        // std::cout << "rot mat: \n"
        //           << rotMatGrasp << "\n";
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
