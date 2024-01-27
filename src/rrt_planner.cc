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
            return t->GetNumberOfNodes();
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

        VectorXd q_best(q_start);

        for (int k = 0; k < plan_parameters.max_iters; k++)
        {
            // Get random configuration
            VectorXd q_rand = this->GetRandomQ(1);

            auto status_a = this->GreedyExtendRandomConfig(t_start, q_rand, plan_parameters, q_goal, q_best);

            VectorXd tmp_vec(q_start);
            auto status_b = this->GreedyExtendRandomConfig(t_goal, q_rand, plan_parameters, q_goal, tmp_vec);

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

        auto best_position = this->GetEEPose(q_best).p;
        auto goal_position = this->GetEEPose(q_goal).p;

        planning_result.distance_to_goal = (best_position - goal_position).Norm();

        int last_idx = t_start->Nearest(q_best.data());
        return this->ConstructPathFromTree(t_start, last_idx);
    }

    AlgorithmState
    RRTPlanner::GreedyExtendRandomConfig(std::shared_ptr<BurTree> t_a, VectorXd rand_q, const RRTParameters &planner_parameters, const VectorXd &q_goal, VectorXd &q_best) const
    {
        int nearest_idx = t_a->Nearest(rand_q.data());
        auto step_result = this->RRTStep(t_a, nearest_idx, rand_q, planner_parameters.epsilon_q);

        // auto goal_config_rt = this->myRobot->CachedForwardPass(q_goal);
        // KDL::Vector goal_ee = goal_config_rt[goal_config_rt.size() - 1].p;
        KDL::Vector goal_ee = this->GetEEPose(q_goal).p;

        KDL::Vector best_ee = this->GetEEPose(q_best).p;
        double best_dist = (best_ee - goal_ee).Norm();

        while (step_result >= 0)
        {
            // if stepped in tree: new node added and crashless
            // if finished: return result
            VectorXd step_q = t_a->GetQ(step_result);
            double max_dist = this->MaxMovedDistance(step_q, rand_q);

            KDL::Vector tmp_ee = this->GetEEPose(step_q).p;
            double tmp_dist = (tmp_ee - goal_ee).Norm();

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

        // int nearest_in_a = t_a->Nearest(rand_q.data());
        // VectorXd q_a = t_a->GetQ(nearest_in_a);

        // VectorXd new_q = this->GetEndpoint(rand_q, q_a, planner_parameters.epsilon_q);
        // double smaller_epsilon = 99 * planner_parameters.epsilon_q;
        // while (!this->IsColliding(new_q))
        // {
        //     auto [n_a, d_a] = t_a->NearestIdxAndDist(new_q.data());
        //     t_a->AddNode(n_a, new_q);
        //     // // Check if new point isn't too close to already existing points in the tree
        //     // auto [n_a, d_a] = t_a->NearestIdxAndDistSqr(new_q.data());

        //     // // if >= epsilon then it will never add it
        //     // if (d_a > smaller_epsilon)
        //     // {
        //     //     t_a->AddNode(n_a, new_q);
        //     // }
        //     // else // d_a <= epsilon => with step size epsilon_q it would never add it
        //     // {
        //     //     new_q = t_a->GetQ(n_a);
        //     // }
        //     // If close enough to target, finish
        //     if ((new_q - rand_q).norm() <= planner_parameters.epsilon_q)
        //     {
        //         return AlgorithmState::Reached;
        //     }
        //     // Do not have to check if it's out of bounds because we are going towards a point that is inbounds and we will end if we reach it
        //     new_q = this->GetEndpoint(rand_q, new_q, planner_parameters.epsilon_q);
        //     std::cout << "new_q: " << new_q.transpose() << "\n";
        // }
        // return AlgorithmState::Trapped;
    }

    AlgorithmState
    RRTPlanner::GreedyExtend(std::shared_ptr<BurTree> t_a, std::shared_ptr<BurTree> t_b, VectorXd q_a, const RRTParameters &planner_parameters)
    {
        // Get closest point in tree b to point q_a from tree a
        int nearest_in_b = t_b->Nearest(q_a.data());
        VectorXd rand_q = t_b->GetQ(nearest_in_b);

        VectorXd new_q = this->GetEndpoint(rand_q, q_a, planner_parameters.epsilon_q);
        while (!this->IsColliding(new_q))
        {
            // Check if new point isn't too close to already existing points in the tree
            auto [n_a, d_a] = t_a->NearestIdxAndDist(new_q.data());
            // TODO CONVERT EPSILONQ TO SQUARED AS WELL TO COMPARE WITH THE KDTREE DISTANCES
            // if (d_a > planner_parameters.epsilon_q)
            // {
            //     t_a->AddNode(n_a, new_q);
            // }
            // else
            // {
            //     new_q = t_a->GetQ(n_a);
            // }
            t_a->AddNode(n_a, new_q);
            // If close enough to target, finish
            if ((new_q - rand_q).norm() <= planner_parameters.epsilon_q)
            {
                return AlgorithmState::Reached;
            }
            new_q = this->GetEndpoint(rand_q, new_q, planner_parameters.epsilon_q);
        }
        return AlgorithmState::Trapped;
    }
}