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

        // To prevent uninitialized vectors in planner_parameters
        this->InitGraspClosestConfigs(planner_parameters, q_start);

        this->rng = std::make_shared<RandomNumberGenerator>(planner_parameters.seed, planner_parameters.target_poses.size() - 1);

        auto tree = std::make_shared<BurTree>(q_start, q_start.size());

        for (unsigned int k = 0; k < planner_parameters.max_iters; ++k)
        {
            VectorXd q_rand = this->GetRandomQ(1);
            int idx_near = tree->Nearest(q_rand.data());

            int step_result = this->RRTStep(tree, idx_near, q_rand, planner_parameters.epsilon_q);
            if (step_result >= 0)
            {
                // Check distance to goal
                VectorXd q_new = tree->GetQ(step_result);
                auto ee_pose = this->GetEEPose(q_new);
                this->SetGraspClosestConfigs(planner_parameters, q_new);
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
                    int best_idx = tree->Nearest(best_grasp.dv->v.data());
                    // Take measurements
                    plan_result.distance_to_goal = best_grasp.dv->d;
                    plan_result.num_iterations = k;
                    plan_result.tree_size = tree->GetNumberOfNodes();
                    plan_result.success = true;

                    // Return best path
                    auto path = this->ConstructPathFromTree(tree, best_idx);
                    return path;
                }
            }
        }

        // Get grasp with minimal distance
        unsigned int best_grasp_idx = this->GetBestGrasp(planner_parameters);
        Grasp best_grasp = planner_parameters.target_poses[best_grasp_idx];
        // Get idx in tree that leads to the best config
        int best_idx = tree->Nearest(best_grasp.dv->v.data());
        // Take measurements
        plan_result.distance_to_goal = best_grasp.dv->d;
        plan_result.num_iterations = planner_parameters.max_iters;
        plan_result.tree_size = tree->GetNumberOfNodes();
        plan_result.success = false;

        // Return best path
        auto path = this->ConstructPathFromTree(tree, best_idx);
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
        Grasp random_grasp = planner_parameters.target_poses[this->rng->getRandomInt()];
        KDL::Frame p_goal = random_grasp.frame;

        // std::cout << "random grasp config: " << random_grasp.dv->v.transpose() << "\n";
        // std::cout << "random grasp config: " << random_grasp.dv->v.size() << "\n";
        // Copy since we will change it
        VectorXd q_near(random_grasp.dv->v);
        // std::cout << "q_near: " << q_near.transpose() << "\n";
        int prev_idx = t_a->Nearest(q_near.data());
        // std::cout << "prev_idx: " << prev_idx << "\n";
        double delta_p = random_grasp.dv->d;
        KDL::Frame p_near = random_grasp.best_frame;

        do
        {
            delta_p = this->DistanceToGoal(p_goal, p_near);

            // Max dist => epsilon_q
            KDL::Twist twist = this->GetTwist(p_goal, p_near, std::min(delta_p, planner_parameters.epsilon_q));

            // std::cout << "q_near: " << q_near.transpose() << "\n";
            // std::cout << "twist: " << twist << "\n";
            KDL::JntArray q_dot = this->myRobot->ForwardJPlus(q_near, twist);
            VectorXd delta_q = q_dot.data;
            q_near = q_near + delta_q;

            if (this->IsColliding(q_near) || !this->InBounds(q_near))
            {
                return AlgorithmState::Trapped;
            }
            prev_idx = t_a->AddNode(prev_idx, q_near);
            this->SetGraspClosestConfigs(planner_parameters, q_near);
            p_near = this->GetEEPose(q_near);

        } while (delta_p > planner_parameters.p_close_enough);

        return AlgorithmState::Reached;
    }

}
