#include "ut.h"
#include "j_rrt_planner.h"

namespace Burs
{
    using namespace Eigen;

    JRRTPlanner::JRRTPlanner(std::string urdf_file) : RbtPlanner(urdf_file)
    {
        this->rng = std::make_shared<RandomNumberGenerator>(1, 1); // temporary seed is one, the proper seed is set from planner parameters
    }

    JRRTPlanner::JRRTPlanner() : RbtPlanner()
    {
        this->rng = std::make_shared<RandomNumberGenerator>(1, 1); // temporary seed is one, the proper seed is set from planner parameters
    }

    std::optional<std::vector<VectorXd>>
    JRRTPlanner::JRRT(VectorXd q_start, JPlusRbtParameters &planner_parameters, PlanningResult &plan_result)
    {
        // Setup rng:
        // std::cout << "target poses: " << planner_parameters.target_poses.size() << "\n";
        if (planner_parameters.target_poses.size() < 1)
        {
            throw std::runtime_error("Target poses has length 0!");
        }

        RS start_state = this->NewState(q_start);
        // td::cout << "start state: " << start_state.config << "\n";
        // To prevent uninitialized vectors in planner_parameters
        // exit(1);

        this->rng = std::make_shared<RandomNumberGenerator>(planner_parameters.seed, planner_parameters.target_poses.size() - 1);

        auto tree = std::make_shared<BurTree>(start_state, q_start.size());

        this->InitGraspClosestConfigs(planner_parameters, tree, 0);

        // for (auto &s : planner_parameters.target_poses)
        // {
        //     std::cout << "best state: " << s.best_state << "\n";
        //     std::cout << "config: " << tree->Get(s.best_state)->config.transpose() << "\n";
        //     // std::cout << s.best_state->config.transpose() << "\n";
        // }s

        double totalNNtime = 0;
        // double totalAddTime = 0;
        double totalRunTime = 0;
        // double totalCollisionTime = 0;
        // double totalGetClosesDistTime = 0;
        double totalCollideAndAddTime = 0;
        struct rusage gt1, gt2;
        getTime(&gt1);

        for (unsigned int k = 0; k < planner_parameters.max_iters; ++k)
        {
            // LOGGING
            if (k % 1000 == 0)
            {
                getTime(&gt2);
                totalRunTime = getTime(gt1, gt2);
                auto &best_pose = planner_parameters.target_poses[this->GetBestGrasp(planner_parameters)];
                std::cout << "iter: " << k << "/" << planner_parameters.max_iters << ", tree.size: " << tree->GetNumberOfNodes() << ", distToGoal: " << best_pose.best_dist << ", ";
                std::cout << ", p_close_enough: " << planner_parameters.p_close_enough << ", totalNNtime: " << totalNNtime << ", totalCollideAndAddTime: " << totalCollideAndAddTime
                          << ", totalRunTime: " << totalRunTime << "\n";
                std::cout.flush();
            }
            // END LOGGING

            VectorXd q_rand = this->GetRandomQ(1);
            RS tmp_state = this->NewState(q_rand);
            struct rusage tt1, tt2;
            getTime(&tt1);
            int idx_near = tree->Nearest(tmp_state);
            getTime(&tt2);
            totalNNtime += getTime(tt1, tt2);

            getTime(&tt1);
            int step_result = this->RRTStep(tree, idx_near, tmp_state, planner_parameters.epsilon_q);
            getTime(&tt2);
            totalCollideAndAddTime += getTime(tt1, tt2);

            if (step_result >= 0)
            {
                // Check distance to goal
                RS new_state = *tree->Get(step_result);
                // auto eepose = this->env->robot->GetEEFrame(new_state);
                // VectorXd q_new = tree->Get(step_result);
                // auto ee_pose = this->GetEEPose(q_new);
                this->SetGraspClosestConfigs(planner_parameters, tree, step_result);
            }

            if (this->rng->getRandomReal() < planner_parameters.probability_to_steer_to_target)
            {
                // Steer until hit the target or obstacle or joint limits
                // std::cout << "tree: " << tree->GetNumberOfNodes() << "\n";
                // std::cout << "planner parameters: " << planner_parameters.target_poses[0].dv->v.transpose() << "\n";
                AlgorithmState state = this->ExtendToGoalRRT(tree, planner_parameters);

                if (state == AlgorithmState::Reached)
                {
                    // Get grasp with minimal distance
                    unsigned int best_grasp_idx = this->GetBestGrasp(planner_parameters);
                    Grasp best_grasp = planner_parameters.target_poses[best_grasp_idx];
                    // Get idx in tree that leads to the best config
                    int best_idx = tree->Nearest(best_grasp.best_state);
                    // int best_idx = tree->Nearest(best_grasp.best_state->config.data());
                    // int best_idx = tree->Nearest(best_grasp.dv->v.data());
                    // Take measurements
                    plan_result.distance_to_goal = best_grasp.best_dist;
                    // plan_result.distance_to_goal = best_grasp.dv->d;
                    plan_result.num_iterations = k;
                    plan_result.tree_size = tree->GetNumberOfNodes();
                    plan_result.success = true;

                    // Return best path
                    auto path = this->ConstructPathFromTree(tree, best_idx);
                    if (planner_parameters.visualize_tree)
                    {
                        this->tree_csv = this->TreePoints(tree, 100);
                    }
                    return path;
                }
            }
        }

        // Get grasp with minimal distance
        unsigned int best_grasp_idx = this->GetBestGrasp(planner_parameters);
        Grasp best_grasp = planner_parameters.target_poses[best_grasp_idx];
        // Get idx in tree that leads to the best config
        int best_idx = tree->Nearest(best_grasp.best_state);
        // int best_idx = tree->Nearest(best_grasp.best_state->config.data());
        // int best_idx = tree->Nearest(best_grasp.dv->v.data());
        // Take measurements
        plan_result.distance_to_goal = best_grasp.best_dist;
        // plan_result.distance_to_goal = best_grasp.dv->d;
        plan_result.num_iterations = planner_parameters.max_iters;
        plan_result.tree_size = tree->GetNumberOfNodes();
        plan_result.success = false;

        // Return best path
        auto path = this->ConstructPathFromTree(tree, best_idx);
        if (planner_parameters.visualize_tree)
        {
            this->tree_csv = this->TreePoints(tree, 100);
        }
        return path;
    }

    KDL::Twist
    JRRTPlanner::GetTwist(const KDL::Frame &tgt, const KDL::Frame &src, const double &max_dist) const
    {
        KDL::Twist twist;
        auto delta_p = tgt.p - src.p;
        delta_p.Normalize();
        twist.vel = delta_p * max_dist;

        // TODO ROTATION

        return twist;
    }

    AlgorithmState
    JRRTPlanner::ExtendToGoalRRT(std::shared_ptr<BurTree> t_a, JPlusRbtParameters &planner_parameters) const
    {
        // std::cout << "inside extend to goal rrt\n";
        int randint = this->rng->getRandomInt();
        // std::cout << "num tgts: " << planner_parameters.target_poses.size() << " randint: " << randint << "\n";
        Grasp random_grasp = planner_parameters.target_poses[randint];
        // RS p_goal = random_grasp.frame;
        KDL::Frame p_goal = random_grasp.frame;

        // std::cout << "random grasp config: " << random_grasp.dv->v.transpose() << "\n";
        // std::cout << "random grasp config: " << random_grasp.dv->v.size() << "\n";
        // std::cout << "best config: " << random_grasp.best_state->frames.size() << "\n";
        // std::cout << "best config: " << random_grasp.best_state->config << "\n";
        int best_state_idx = random_grasp.best_state;
        RS *best_state = t_a->Get(best_state_idx);
        RS near_state = *best_state;
        // RS near_state(*random_grasp.best_state);
        // Copy since we will change it
        // VectorXd q_near(random_grasp.dv->v);

        int prev_idx = t_a->Nearest(best_state_idx);
        // int prev_idx = t_a->Nearest(q_near.data());
        // std::cout << "prev_idx: " << prev_idx << "\n";
        double delta_p = random_grasp.best_dist;
        // double delta_p = random_grasp.dv->d;
        KDL::Frame p_near = this->env->robot->GetEEFrame(*best_state);
        // RS p_near = random_grasp.best_state;
        // KDL::Frame p_near = random_grasp.best_frame;

        do
        {
            delta_p = (p_goal.p - p_near.p).Norm();
            // delta_p = this->env->robot->EEDistance(p_goal, p_near);
            // delta_p = this->DistanceToGoal(p_goal, p_near);

            // Max dist => epsilon_q
            KDL::Twist twist = this->GetTwist(p_goal, p_near, std::min(delta_p, planner_parameters.epsilon_q));

            // std::cout << "q_near: " << q_near.transpose() << "\n";
            // std::cout << "twist: " << twist << "\n";
            KDL::JntArray q_dot = this->env->robot->ForwardJPlus(near_state, twist);
            // KDL::JntArray q_dot = this->myRobot->ForwardJPlus(q_near, twist);
            VectorXd delta_q = q_dot.data;
            near_state = this->NewState(near_state.config + delta_q);
            // q_near = q_near + delta_q;

            if (this->IsColliding(near_state) || !this->InBounds(near_state.config))
            {
                return AlgorithmState::Trapped;
            }
            prev_idx = t_a->AddNode(prev_idx, near_state);
            this->SetGraspClosestConfigs(planner_parameters, t_a, prev_idx);
            p_near = this->env->robot->GetEEFrame(near_state);
            // p_near = this->GetEEPose(q_near);

        } while (delta_p > planner_parameters.p_close_enough);

        return AlgorithmState::Reached;
    }

}
