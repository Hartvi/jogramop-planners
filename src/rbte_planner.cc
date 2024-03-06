#include <cmath>
#include <optional>
#include "base_planner.h"
#include "bur_funcs.h"
#include "rbte_planner.h"
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

    RbtePlanner::RbtePlanner(std::string path_to_urdf_file)
        : RbtPlanner(path_to_urdf_file)
    {
    }

    RbtePlanner::RbtePlanner() : RbtPlanner() {}

    std::optional<std::vector<Eigen::VectorXd>>
    RbtePlanner::RbteConnect(const VectorXd &q_start, const VectorXd &q_goal, const RbtParameters &plan_parameters, PlanningResult &planning_result)
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
            double d_closest = this->GetClosestDistance(near_state);

            if (d_closest < plan_parameters.d_crit)
            {
                int step_result = this->RRTStep(t_a, nearest_idx, near_state, plan_parameters.epsilon_q);
                if (step_result < 0)
                {
                    // If small basic rrt collides, then don't go here, hence the `continue`
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
                    }
                }
            }
            else
            {
                // Qe is scaled to max euclidean delta_q or closest obstacle distance
                std::vector<RS> endpoints = this->GetEndpoints(near_state, rand_states, std::min(d_closest, plan_parameters.delta_q));
                // TRAVELLED DISTANCES ARE INDEED ALWAYS SMALLER THAN D_CLOSEST

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
                // `q_new` is in `t_a`
                // `t_b` extends to `q_new` => it has a node near `q_new`

                planning_result.distance_to_goal = 0.0;
                planning_result.num_iterations = k;
                planning_result.tree_size = t_start->GetNumberOfNodes() + t_goal->GetNumberOfNodes();
                planning_result.success = true;

                auto path = this->Path(t_start, start_closest, t_goal, goal_closest);
                if (plan_parameters.visualize_tree)
                {
                    this->tree_csv = this->TreePoints(t_start, 100);
                }
                return path;
            }
            std::swap(t_a, t_b);
        }

        planning_result.num_iterations = plan_parameters.max_iters;
        planning_result.tree_size = t_start->GetNumberOfNodes() + t_goal->GetNumberOfNodes();
        planning_result.success = false;

        int best_idx = t_start->Nearest(best_state);
        planning_result.distance_to_goal = this->env->robot->EEDistance(best_state, goal_state);

        if (plan_parameters.visualize_tree)
        {
            this->tree_csv = this->TreePoints(t_start, 100);
        }
        return this->ConstructPathFromTree(t_start, best_idx);
    }

    void
    RbtePlanner::TestFunctions()
    {
        this->env->robot->segmentToJntCausality;
    }

    std::vector<RS>
    RbtePlanner::CreateExtendedBur()
    {
        return {};
    }

}
