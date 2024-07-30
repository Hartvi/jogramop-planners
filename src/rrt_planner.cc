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
    RRTPlanner::RRTStepInQ(std::shared_ptr<BurTree> t, int node_idx, const RS &rand_state, const Qunit &epsilon_q, const Meters &p_step, const DistanceEstimateType &det) const
    {
        // RRTExtend

        // p_step in the bur paper is roughly 0.005
        RS near_state = *t->Get(node_idx);

        // shifted in configuration space
        VectorXd new_config = near_state.config + epsilon_q * (rand_state.config - near_state.config).normalized();
        // end state
        RS new_state = this->NewState(new_config, det);
        double max_dist = this->env->robot->MaxDistance(new_state, near_state);

        // interpolate base on workspace distance
        unsigned int steps = (unsigned int)(max_dist / p_step + 1.0);
        // check the farthest state first
        if (this->IsColliding(new_state))
        {
            return -1;
        }
        for (unsigned int i = 1; i < steps; ++i)
        {
            VectorXd interconfig = new_state.config + ((double)i) * p_step * (near_state.config - new_state.config);
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

    std::optional<std::vector<VectorXd>>
    RRTPlanner::RRTConnectBasic(const VectorXd &q_start, const VectorXd &q_goal, const RRTParameters &plan_parameters, PlanningResult &planning_result)
    {
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
        auto t_a = t_start;
        auto t_b = t_goal;

        // best_state will keep how far the initial tree has gotten
        RS &best_state = start_state;
        RS &tmp_state = goal_state;
        // for i from 1 to maxIterations do:
        //     q_rand ← SampleRandomConfiguration()

        //     if Extend(treeA, q_rand, stepSize) ≠ TRAPPED then:
        //         if Connect(treeB, q_new, stepSize) = REACHED then:
        //             return Path(treeA, treeB)

        //     Swap(treeA, treeB)

        // return FAILURE
        for (int k = 0; k < plan_parameters.max_iters; ++k)
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
            // get random q in jointspace
            auto q_rand = this->GetRandomQ(1);
            RS rand_state = this->NewState(q_rand);

            // get nearest points in tree
            int nearest_idx = t_a->Nearest(rand_state);
            int new_idx = this->RRTStepInQ(t_a, nearest_idx, rand_state, plan_parameters.epsilon_q, plan_parameters.collision_resolution);
            // step in random direction
            if (new_idx != -1)
            {
                RS *new_state = t_a->Get(new_idx);
                if (this->ExtendRandomConfigInQ(t_b, *new_state, plan_parameters) == AlgorithmState::Reached)
                {
                    int start_closest = t_start->Nearest(new_idx);
                    int goal_closest = t_goal->Nearest(new_idx);

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
            std::swap(t_a, t_b);
        }
        planning_result.num_iterations = plan_parameters.max_iters;
        planning_result.tree_size = t_start->GetNumberOfNodes() + t_goal->GetNumberOfNodes();
        planning_result.success = false;
        planning_result.distance_to_goal = 1000.0;
        if (plan_parameters.visualize_tree > 0)
        {
            this->tree_csv = this->TreePoints(t_start, plan_parameters.visualize_tree);
            std::cout << "inside vis tree\n";
        }
        std::cout << "TODO FINISH BASIC RRT\n";
        return this->ConstructPathFromTree(t_start, t_start->GetNumberOfNodes() - 1);
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

    AlgorithmState
    RRTPlanner::GreedyExtendRandomConfigInQ(std::shared_ptr<BurTree> t_a, RS rand_state, const RRTParameters &planner_parameters, const RS &goal_state, RS &best_state) const
    {
        int nearest_idx = t_a->Nearest(rand_state);
        auto step_result = this->RRTStepInQ(t_a, nearest_idx, rand_state, planner_parameters.epsilon_q, planner_parameters.collision_resolution, DistanceEstimateType::None);

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
            step_result = this->RRTStepInQ(t_a, step_result, rand_state, planner_parameters.epsilon_q, planner_parameters.collision_resolution, DistanceEstimateType::None);
        }
        return AlgorithmState::Trapped;
    }

    AlgorithmState
    RRTPlanner::ExtendRandomConfigInQ(std::shared_ptr<BurTree> t_a, RS rand_state, const RRTParameters &planner_parameters) const
    {
        int nearest_idx = t_a->Nearest(rand_state);
        auto step_result = this->RRTStepInQ(t_a, nearest_idx, rand_state, planner_parameters.epsilon_q, planner_parameters.collision_resolution, DistanceEstimateType::None);

        while (step_result >= 0)
        {
            // if stepped in tree: new node added and crashless
            // if finished: return result
            RS step_state = *t_a->Get(step_result);
            double max_dist = this->env->robot->MaxDistance(step_state, rand_state);

            // if reached the random config:
            if (max_dist < planner_parameters.epsilon_q)
            {
                return AlgorithmState::Reached;
            }

            // then step again from newly added node: step_result
            step_result = this->RRTStepInQ(t_a, step_result, rand_state, planner_parameters.epsilon_q, planner_parameters.collision_resolution, DistanceEstimateType::None);
        }
        return AlgorithmState::Trapped;
    }

}
