#include <cmath>
#include <optional>
#include "base_planner.h"
#include "bur_funcs.h"
#include "ik_rrt_planner.h"
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

    IKRRTPlanner::IKRRTPlanner(std::string path_to_urdf_file)
        : JRRTPlanner(path_to_urdf_file)
    {
    }

    std::optional<std::vector<VectorXd>>
    IKRRTPlanner::IKRRT(const VectorXd &q_start, const JPlusRbtParameters &plan_parameters, PlanningResult &planning_result)
    {
        this->rng = std::make_shared<RandomNumberGenerator>(plan_parameters.seed, plan_parameters.target_poses.size());

        // start of actual algorithm
        RS start_state = this->NewState(q_start);
        if (this->IsColliding(start_state))
        {
            std::cout << "IKRRT: start state is colliding\n";
            return {{q_start}};
        }

        std::shared_ptr<BurTree> t_start = std::make_shared<BurTree>(start_state, q_start.size());

        VectorXd q_best(q_start);
        RS &best_state = start_state;

        // IK:
        KDL::ChainIkSolverPos_LMA ik_solver(this->env->robot->kdl_chain);
        KDL::JntArray init_joints(q_start.size());
        init_joints.data = q_start;
        VectorXd q_goal(q_start);
        int ik_attempts = 0;
        int num_goals = 0;
        auto res = this->env->robot->GetInverseKinematics(ik_solver, init_joints, plan_parameters.target_poses[this->rng->getRandomInt()].frame);
        while (ik_attempts < plan_parameters.max_iters)
        {
            ik_attempts++;
            init_joints.data = q_start + 0.1 * MatrixXd::Random(this->q_dim, 1);
            res = this->env->robot->GetInverseKinematics(ik_solver, init_joints, plan_parameters.target_poses[this->rng->getRandomInt()].frame);
            if (res)
            {
                RS tmp = this->NewState(res.value());
                if (!this->IsColliding(tmp) && this->InBounds(tmp.config))
                {
                    std::cout << "collision free IK solution FOUND\n";
                    break;
                    // res = {};
                }
            }
        }
        int ik_solutions = 1;
        q_goal = res.value();

        // END IK
        RS goal_state = this->NewState(q_goal);
        std::cout << "IK FOUND SOLUTION: " << goal_state.frames.back() << "\n\n";
        RS &tmp_state = goal_state;
        std::shared_ptr<BurTree> t_goal = std::make_shared<BurTree>(goal_state, q_goal.size());

        for (int k = ik_attempts; k < plan_parameters.max_iters; k++)
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

            // SOME SMALL PROBABILITY => FIND IK
            if (rng->getRandomReal() < plan_parameters.probability_to_steer_to_target)
            {
                init_joints.data = q_start + 0.1 * MatrixXd::Random(this->q_dim, 1);
                res = this->env->robot->GetInverseKinematics(ik_solver, init_joints, plan_parameters.target_poses[this->rng->getRandomInt()].frame);
                if (res)
                {
                    RS tmp = this->NewState(res.value());
                    if (!this->IsColliding(tmp) && this->InBounds(tmp.config))
                    {
                        t_goal->AddNode(0, tmp);
                        ik_solutions++;
                        std::cout << "IK SOLUTIONS: " << ik_solutions << "\n";
                    }
                }
            }
            else // ELSE REGULAR RRT
            {
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
        }

        planning_result.num_iterations = plan_parameters.max_iters;
        planning_result.tree_size = t_start->GetNumberOfNodes() + t_goal->GetNumberOfNodes();
        planning_result.success = false;

        planning_result.distance_to_goal = this->env->robot->EEDistance(best_state, goal_state);

        int best_idx = t_goal->Nearest(best_state);

        if (plan_parameters.visualize_tree > 0)
        {
            this->tree_csv = this->TreePoints(t_goal, plan_parameters.visualize_tree);
            std::cout << "inside vis tree\n";
        }
        return this->ConstructPathFromTree(t_goal, best_idx);
    }

}
