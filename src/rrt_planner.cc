#include "ut.h"
#include "rrt_planner.h"

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

    int
    RRTPlanner::RRTStep(std::shared_ptr<BurTree> t, int node_idx, const RS &rand_state, const Meters &epsilon_q) const
    {
        RS new_state = this->GetEndpoints(*t->Get(node_idx), {rand_state}, epsilon_q)[0];

        if (!this->IsColliding(new_state) && this->InBounds(new_state.config))
        {
            // The result is the INDEX of the new node => 0 to N-1
            return t->AddNode(node_idx, new_state);
        }
        return -1;
    }

    std::optional<std::vector<VectorXd>>
    RRTPlanner::RRTConnect(const VectorXd &q_start, const VectorXd &q_goal, const RRTParameters &plan_parameters, PlanningResult &planning_result)
    {
        std::cout << "start q: " << q_start.transpose() << "\n";
        std::cout << "goal q: " << q_goal.transpose() << "\n\n";
        // start of actual algorithm
        RS goal_state = this->NewState(q_goal);
        RS start_state = this->NewState(q_start);

        std::shared_ptr<BurTree> t_start = std::make_shared<BurTree>(start_state, q_start.size());
        std::shared_ptr<BurTree> t_goal = std::make_shared<BurTree>(goal_state, q_goal.size());

        VectorXd q_best(q_start);
        RS &best_state = start_state;
        RS &tmp_state = goal_state;

        for (int k = 0; k < plan_parameters.max_iters; k++)
        {
            if (k % 1000 == 0)
            {
                std::cout << "tree: " << (t_start->GetNumberOfNodes() + t_goal->GetNumberOfNodes()) << "\n";
            }

            if (this->globalTrigger)
            {
                std::cerr << "Terminating planner as globalTrigger=" << globalTrigger << "\n";
                std::cout << "Terminating planner as globalTrigger=" << globalTrigger << "\n";
                break;
            }

            // Get random configuration
            VectorXd q_rand = this->GetRandomQ(1);
            // std::cout << "new state\n";
            RS rand_state = this->NewState(q_rand);

            // std::cout << "start tree:\n";
            // std::cout << "greedy extend\n";
            auto status_a = this->GreedyExtendRandomConfig(t_start, rand_state, plan_parameters, goal_state, best_state);

            VectorXd tmp_vec(q_start);
            // we do not want the closest config in the goal tree since it already leads to the goal
            // std::cout << "goal tree:\n";
            auto status_b = this->GreedyExtendRandomConfig(t_goal, rand_state, plan_parameters, goal_state, tmp_state);

            if (status_a == AlgorithmState::Reached && status_b == AlgorithmState::Reached)
            {
                int start_closest = t_start->Nearest(q_rand.data());
                int goal_closest = t_goal->Nearest(q_rand.data());

                planning_result.num_iterations = k;
                planning_result.tree_size = t_start->GetNumberOfNodes() + t_goal->GetNumberOfNodes();
                planning_result.success = true;
                planning_result.distance_to_goal = 0.0;
                if (plan_parameters.visualize_tree > 0)
                {
                    this->tree_csv = this->TreePoints(t_start, plan_parameters.visualize_tree);
                    std::cout << "inside vis tree\n";
                }
                return this->Path(t_start, start_closest, t_goal, goal_closest);
            }
        }

        planning_result.num_iterations = plan_parameters.max_iters;
        planning_result.tree_size = t_start->GetNumberOfNodes() + t_goal->GetNumberOfNodes();
        planning_result.success = false;

        planning_result.distance_to_goal = this->env->robot->EEDistance(best_state, goal_state);
        // planning_result.distance_to_goal = this->GetDistToGoal(q_best, ee_goal);

        int best_idx = t_start->Nearest(best_state);

        if (plan_parameters.visualize_tree > 0)
        {
            this->tree_csv = this->TreePoints(t_start, plan_parameters.visualize_tree);
            std::cout << "inside vis tree\n";
        }
        return this->ConstructPathFromTree(t_start, best_idx);
    }

    std::vector<int>
    RRTPlanner::ExtendRandomConfig(std::shared_ptr<BurTree> t_a, RS rand_state, const RRTParameters &planner_parameters) const
    {
        std::vector<int> ids;
        int nearest_idx = t_a->Nearest(rand_state);
        auto step_result = this->RRTStep(t_a, nearest_idx, rand_state, planner_parameters.epsilon_q);

        while (step_result >= 0)
        {
            ids.push_back(step_result);
            // if stepped in tree: new node added and crashless
            // if finished: return result
            RS step_state = *t_a->Get(step_result);
            double max_dist = this->env->robot->MaxDistance(step_state, rand_state);

            // if reached the random config:
            if (max_dist <= planner_parameters.epsilon_q)
            {
                return ids;
            }

            // then step again from newly added node: step_result
            step_result = this->RRTStep(t_a, step_result, rand_state, planner_parameters.epsilon_q);
        }
        return ids;
    }

    AlgorithmState
    RRTPlanner::GreedyExtendRandomConfig(std::shared_ptr<BurTree> t_a, RS rand_state, const RRTParameters &planner_parameters, const RS &goal_state, RS &best_state) const
    {
        int nearest_idx = t_a->Nearest(rand_state);
        // std::cout << "last tree config: " << t_a->GetQ(t_a->GetNumberOfNodes() - 1).transpose() << "\n\n";
        auto step_result = this->RRTStep(t_a, nearest_idx, rand_state, planner_parameters.epsilon_q);

        double best_dist = this->env->robot->EEDistance(best_state, goal_state);
        // double best_dist = this->GetDistToGoal(q_best, goal_ee);

        while (step_result >= 0)
        {
            // if stepped in tree: new node added and crashless
            // if finished: return result
            RS step_state = *t_a->Get(step_result);
            double max_dist = this->env->robot->MaxDistance(step_state, rand_state);

            double tmp_dist = this->env->robot->EEDistance(step_state, goal_state);

            if (tmp_dist < best_dist)
            {
                best_state = step_state;
                best_dist = tmp_dist;
            }

            // if reached the random config:
            if (max_dist <= planner_parameters.epsilon_q)
            {
                return AlgorithmState::Reached;
            }

            // then step again from newly added node: step_result
            step_result = this->RRTStep(t_a, step_result, rand_state, planner_parameters.epsilon_q);
        }
        return AlgorithmState::Trapped;
    }

    // BELOW NOT IN USE
    std::optional<std::vector<VectorXd>>
    RRTPlanner::TestSampling(const VectorXd &q_start, const RRTParameters &plan_parameters, PlanningResult &planning_result)
    {
        // start of actual algorithm
        RS start_state = this->NewState(q_start);

        std::shared_ptr<BurTree> t_start = std::make_shared<BurTree>(start_state, q_start.size());

        this->GenerateRandomSamples(t_start, plan_parameters.max_iters);

        if (plan_parameters.visualize_tree > 0)
        {
            this->tree_csv = this->TreePoints(t_start, plan_parameters.visualize_tree);
        }
        return {{q_start}};
    }

    void
    RRTPlanner::GenerateRandomSamples(std::shared_ptr<BurTree> t, int num_samples)
    {
        MatrixXd rand_Q = this->GetRandomQ(num_samples);
        auto states = this->NewStates(rand_Q);
        for (unsigned int i = 0; i < num_samples; ++i)
        {
            t->AddNode(0, states[i]);
        }
    }

}
