#include "rrt_planner.h"
#include "collision_env.h"

namespace Burs
{
    using namespace Eigen;

    RRTPlanner::RRTPlanner(std::string path_to_urdf_file)
        : BasePlanner(path_to_urdf_file)
    {
    }

    RRTPlanner::RRTPlanner() : BasePlanner()
    {
    }

    double
    RRTPlanner::GetDistToGoal(const VectorXd &q, const KDL::Vector &goal_pos) const
    {
        auto ee = this->GetEEPose(q).p;
        return (ee - goal_pos).Norm();
    }

    KDL::Frame
    RRTPlanner::GetEEPose(const VectorXd &q) const
    {
        auto config_rt = this->myRobot->CachedForwardPass(q);
        auto ee = config_rt[config_rt.size() - 1];
        return ee;
    }

    int
    RRTPlanner::RRTStep(std::shared_ptr<BurTree> t, int node_idx, VectorXd rand_q, const Meters &epsilon_q) const
    {
        // VectorXd rand_q = this->GetRandomQ(1);
        VectorXd q_new = this->GetEndpoint(rand_q, t->GetQ(node_idx), epsilon_q);

        if (!this->IsColliding(q_new))
        {
            t->AddNode(node_idx, q_new);
            // The result is the INDEX of the new node => 0 to N-1
            return t->GetNumberOfNodes() - 1;
        }
        return -1;
    }

    std::optional<std::vector<VectorXd>>
    RRTPlanner::RRTConnect(const VectorXd &q_start, const VectorXd &q_goal, const RRTParameters &plan_parameters, PlanningResult &planning_result)
    {
        // start of actual algorithm
        std::shared_ptr<BurTree> t_start = std::make_shared<BurTree>(q_start, q_start.rows());
        std::shared_ptr<BurTree> t_goal = std::make_shared<BurTree>(q_goal, q_goal.rows());
        // TODO: check collision at the beginning
        auto ee_goal = this->GetEEPose(q_goal).p;

        VectorXd q_best(q_start);

        for (int k = 0; k < plan_parameters.max_iters; k++)
        {
            // Get random configuration
            VectorXd q_rand = this->GetRandomQ(1);

            auto status_a = this->GreedyExtendRandomConfig(t_start, q_rand, plan_parameters, ee_goal, q_best);

            VectorXd tmp_vec(q_start);
            // we do not want the closest config in the goal tree since it already leads to the goal
            auto status_b = this->GreedyExtendRandomConfig(t_goal, q_rand, plan_parameters, ee_goal, tmp_vec);

            if (status_a == AlgorithmState::Reached && status_b == AlgorithmState::Reached)
            {
                int start_closest = t_start->Nearest(q_rand.data());
                int goal_closest = t_goal->Nearest(q_rand.data());

                planning_result.num_iterations = plan_parameters.max_iters;
                planning_result.tree_size = t_start->GetNumberOfNodes() + t_goal->GetNumberOfNodes();
                planning_result.success = true;
                planning_result.distance_to_goal = 0.0;
                return this->Path(t_start, start_closest, t_goal, goal_closest);
            }
        }

        planning_result.num_iterations = plan_parameters.max_iters;
        planning_result.tree_size = t_start->GetNumberOfNodes() + t_goal->GetNumberOfNodes();
        planning_result.success = false;

        planning_result.distance_to_goal = this->GetDistToGoal(q_best, ee_goal);

        int best_idx = t_start->Nearest(q_best.data());

        return this->ConstructPathFromTree(t_start, best_idx);
    }

    AlgorithmState
    RRTPlanner::GreedyExtendRandomConfig(std::shared_ptr<BurTree> t_a, VectorXd rand_q, const RRTParameters &planner_parameters, const KDL::Vector &goal_ee, VectorXd &q_best) const
    {
        int nearest_idx = t_a->Nearest(rand_q.data());
        auto step_result = this->RRTStep(t_a, nearest_idx, rand_q, planner_parameters.epsilon_q);

        double best_dist = this->GetDistToGoal(q_best, goal_ee);

        while (step_result >= 0)
        {
            // if stepped in tree: new node added and crashless
            // if finished: return result
            VectorXd step_q = t_a->GetQ(step_result);
            double max_dist = this->MaxMovedDistance(step_q, rand_q);

            double tmp_dist = this->GetDistToGoal(step_q, goal_ee);

            if (tmp_dist < best_dist)
            {
                q_best = step_q;
                best_dist = tmp_dist;
            }

            // if reached the random config:
            if (max_dist <= planner_parameters.epsilon_q)
            {
                return AlgorithmState::Reached;
            }

            // then step again from newly added node: step_result
            step_result = this->RRTStep(t_a, step_result, rand_q, planner_parameters.epsilon_q);
        }
        return AlgorithmState::Trapped;
    }

    // AlgorithmState
    // RRTPlanner::GreedyExtend(std::shared_ptr<BurTree> t_a, std::shared_ptr<BurTree> t_b, VectorXd q_a, const RRTParameters &planner_parameters)
    // {
    //     // Get closest point in tree b to point q_a from tree a
    //     int nearest_in_b = t_b->Nearest(q_a.data());
    //     VectorXd rand_q = t_b->GetQ(nearest_in_b);

    //     VectorXd new_q = this->GetEndpoint(rand_q, q_a, planner_parameters.epsilon_q);
    //     while (!this->IsColliding(new_q))
    //     {
    //         // Check if new point isn't too close to already existing points in the tree
    //         auto [n_a, d_a] = t_a->NearestIdxAndDist(new_q.data());
    //         // TODO CONVERT EPSILONQ TO SQUARED AS WELL TO COMPARE WITH THE KDTREE DISTANCES
    //         // if (d_a > planner_parameters.epsilon_q)
    //         // {
    //         //     t_a->AddNode(n_a, new_q);
    //         // }
    //         // else
    //         // {
    //         //     new_q = t_a->GetQ(n_a);
    //         // }
    //         t_a->AddNode(n_a, new_q);
    //         // If close enough to target, finish
    //         if ((new_q - rand_q).norm() <= planner_parameters.epsilon_q)
    //         {
    //             return AlgorithmState::Reached;
    //         }
    //         new_q = this->GetEndpoint(rand_q, new_q, planner_parameters.epsilon_q);
    //     }
    //     return AlgorithmState::Trapped;
    // }
}