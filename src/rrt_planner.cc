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
    RRTPlanner::RRTStepInQ(std::shared_ptr<BurTree> t, int node_idx, const RS &rand_state, const Qunit &epsilon_q, const Meters &p_step) const
    {
        // p_step in the bur paper is roughly 0.006
        RS near_state = *t->Get(node_idx);
        // shifted in configuration space
        VectorXd new_config = near_state.config + epsilon_q * (rand_state.config - near_state.config);
        // end state
        RS new_state(new_config, this->env->robot->ForwardPass(new_config));
        double max_dist = this->env->robot->MaxDistance(new_state, near_state);

        // interpolate base on workspace distance
        for (unsigned int i = 1; i < (unsigned int)(max_dist / p_step + 2.0); ++i)
        {
            VectorXd interconfig = near_state.config + ((double)i) * p_step * (new_state.config - near_state.config);
            RS interstate(interconfig, this->env->robot->ForwardPass(interconfig));
            if (this->IsColliding(interstate))
            {
                return -1;
            }
        }

        if (this->InBounds(new_state.config))
        {
            // The result is the INDEX of the new node => 0 to N-1
            return t->AddNode(node_idx, new_state);
        }
        return -1;
    }

    int
    RRTPlanner::RRTStep(std::shared_ptr<BurTree> t, int node_idx, const RS &rand_state, const Meters &epsilon_q) const
    {
        // VectorXd rand_q = this->GetRandomQ(1);
        // std::cout << "parent node: " << t->GetQ(node_idx).transpose() << "\n";
        RS new_state = this->GetEndpoints(*t->Get(node_idx), {rand_state}, epsilon_q)[0];

        if (!this->IsColliding(new_state) && this->InBounds(new_state.config))
        {
            // The result is the INDEX of the new node => 0 to N-1
            return t->AddNode(node_idx, new_state);
        }
        return -1;
    }

    std::optional<std::vector<VectorXd>>
    RRTPlanner::RRTConnectQStep(const VectorXd &q_start, const VectorXd &q_goal, const RRTParameters &plan_parameters, PlanningResult &planning_result)
    {
        // start of actual algorithm
        RS goal_state = this->NewState(q_goal);
        RS start_state = this->NewState(q_start);

        if (this->IsColliding(goal_state))
        {
            std::cout << "RRTConnectQStep: goal state is colliding\n";
            return {{q_start}};
        }
        if (this->IsColliding(start_state))
        {
            std::cout << "RRTConnectQStep: start state is colliding\n";
            return {{q_start}};
        }

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
            RS rand_state = this->NewState(q_rand);

            auto status_a = this->GreedyExtendRandomConfigInQ(t_start, rand_state, plan_parameters, goal_state, best_state);

            VectorXd tmp_vec(q_start);
            // we do not want the closest config in the goal tree since it already leads to the goal
            auto status_b = this->GreedyExtendRandomConfigInQ(t_goal, rand_state, plan_parameters, goal_state, tmp_state);

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

        int best_idx = t_start->Nearest(best_state);

        if (plan_parameters.visualize_tree > 0)
        {
            this->tree_csv = this->TreePoints(t_start, plan_parameters.visualize_tree);
            std::cout << "inside vis tree\n";
        }
        return this->ConstructPathFromTree(t_start, best_idx);
    }

    std::optional<std::vector<VectorXd>>
    RRTPlanner::RRTConnect(const VectorXd &q_start, const VectorXd &q_goal, const RRTParameters &plan_parameters, PlanningResult &planning_result)
    {
        RS goal_state = this->NewState(q_goal);
        RS start_state = this->NewState(q_start);

        std::shared_ptr<BurTree> t_start = std::make_shared<BurTree>(start_state, q_start.size());
        std::shared_ptr<BurTree> t_goal = std::make_shared<BurTree>(goal_state, q_goal.size());

        VectorXd q_best(q_start);
        RS &best_state = start_state;
        RS &tmp_state = goal_state;

        if (this->IsColliding(goal_state))
        {
            std::cout << "RRTConnect: goal state is colliding\n";
            return {{q_start}};
        }
        if (this->IsColliding(start_state))
        {
            std::cout << "RRTConnect: start state is colliding\n";
            return {{q_start}};
        }

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
            RS rand_state = this->NewState(q_rand);

            auto status_a = this->GreedyExtendRandomConfig(t_start, rand_state, plan_parameters, goal_state, best_state);

            VectorXd tmp_vec(q_start);
            // We do not want the closest config in the goal tree since it already leads to the goal => use tmp_state which can be changed with no harm
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

    AlgorithmState
    RRTPlanner::GreedyExtendRandomConfig(std::shared_ptr<BurTree> t_a, RS rand_state, const RRTParameters &planner_parameters, const RS &goal_state, RS &best_state) const
    {
        int nearest_idx = t_a->Nearest(rand_state);
        auto step_result = this->RRTStep(t_a, nearest_idx, rand_state, planner_parameters.epsilon_q);

        double best_dist = this->env->robot->EEDistance(best_state, goal_state);

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

    AlgorithmState
    RRTPlanner::GreedyExtendRandomConfigInQ(std::shared_ptr<BurTree> t_a, RS rand_state, const RRTParameters &planner_parameters, const RS &goal_state, RS &best_state) const
    {
        int nearest_idx = t_a->Nearest(rand_state);
        auto step_result = this->RRTStepInQ(t_a, nearest_idx, rand_state, planner_parameters.epsilon_q, planner_parameters.collision_resolution);

        double best_dist = this->env->robot->EEDistance(best_state, goal_state);

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
            step_result = this->RRTStepInQ(t_a, step_result, rand_state, planner_parameters.epsilon_q, planner_parameters.collision_resolution);
        }
        return AlgorithmState::Trapped;
    }

    // NOT IN USE
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

    // NOT IN USE
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
