#include "ut.h"
#include "j_rrt_planner.h"
#include "bur_tree3d.h"
#include <numeric> // For std::iota

namespace Burs
{
    using namespace Eigen;

    JRRTPlanner::JRRTPlanner(std::string urdf_file) : RbtePlanner(urdf_file)
    {
        this->rng = std::make_shared<RandomNumberGenerator>(1, 1); // temporary seed is one, the proper seed is set from planner parameters
    }

    JRRTPlanner::JRRTPlanner() : RbtePlanner()
    {
        this->rng = std::make_shared<RandomNumberGenerator>(1, 1); // temporary seed is one, the proper seed is set from planner parameters
    }

    std::optional<std::vector<VectorXd>>
    JRRTPlanner::JRRT(VectorXd q_start, JPlusRbtParameters &planner_parameters, PlanningResult &plan_result)
    {
        // Setup rng:
        if (planner_parameters.target_poses.size() < 1)
        {
            throw std::runtime_error("Target poses has length 0!");
        }

        RS start_state = this->NewState(q_start);

        this->rng = std::make_shared<RandomNumberGenerator>(planner_parameters.seed, planner_parameters.target_poses.size());

        auto tree = std::make_shared<BurTree>(start_state, q_start.size());

        this->InitGraspClosestConfigs(planner_parameters, tree, 0);

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
                // std::cout << "target poses: " << planner_parameters.target_poses.size() << "\n";
                // std::cout << "best grasp: " << this->GetBestGrasp(planner_parameters) << "\n";
                auto &best_pose = planner_parameters.target_poses[this->GetBestGrasp(planner_parameters)];
                std::cout << "iter: " << k << "/" << planner_parameters.max_iters;
                std::cout << ", tree.size: " << tree->GetNumberOfNodes();
                std::cout << ", distToGoal: " << best_pose.best_dist << ", ";
                std::cout << ", p_close_enough: " << planner_parameters.p_close_enough;
                std::cout << ", totalNNtime: " << totalNNtime;
                std::cout << ", totalCollideAndAddTime: " << totalCollideAndAddTime;
                std::cout << ", totalRunTime: " << totalRunTime << "\n";
                std::cout.flush();
            }
            // END LOGGING
            if (this->globalTrigger)
            {
                std::cerr << "Terminating planner as globalTrigger=" << globalTrigger << "\n";
                std::cout << "Terminating planner as globalTrigger=" << globalTrigger << "\n";
                break;
            }

            VectorXd q_rand = this->GetRandomQ(1);
            RS tmp_state = this->NewState(q_rand);
            struct rusage tt1, tt2;
            getTime(&tt1);
            int idx_near = tree->Nearest(tmp_state);
            getTime(&tt2);
            totalNNtime += getTime(tt1, tt2);

            getTime(&tt1);
            int step_result = this->RRTStepInQ(tree, idx_near, tmp_state, planner_parameters.epsilon_q, planner_parameters.collision_resolution, DistanceEstimateType::None);
            getTime(&tt2);
            totalCollideAndAddTime += getTime(tt1, tt2);

            if (step_result >= 0)
            {
                // Check distance to goal
                RS new_state = *tree->Get(step_result);
                this->SetGraspClosestConfigs(planner_parameters, tree, step_result);
            }

            if (this->rng->getRandomReal() < planner_parameters.probability_to_steer_to_target)
            {
                // Steer until hit the target or obstacle or joint limits
                AlgorithmState state = this->ExtendToGoalRRT(tree, planner_parameters);

                unsigned int best_grasp_idx = this->GetBestGrasp(planner_parameters);
                Grasp best_grasp = planner_parameters.target_poses[best_grasp_idx];
                if (state != AlgorithmState::Reached)
                {
                    if (best_grasp.best_dist <= planner_parameters.p_close_enough)
                    {
                        state = AlgorithmState::Reached;
                    }
                }
                if (state == AlgorithmState::Reached)
                {
                    // Get grasp with minimal distance
                    // Get idx in tree that leads to the best config
                    int best_idx = tree->Nearest(best_grasp.best_state);
                    // Take measurements
                    plan_result.distance_to_goal = best_grasp.best_dist;
                    plan_result.num_iterations = k;
                    plan_result.tree_size = tree->GetNumberOfNodes();
                    plan_result.success = true;

                    // Return best path
                    auto path = this->ConstructPathFromTree(tree, best_idx);
                    if (planner_parameters.visualize_tree)
                    {
                        this->tree_csv = this->TreePoints(tree, planner_parameters.visualize_tree);
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
        // Take measurements
        plan_result.distance_to_goal = best_grasp.best_dist;
        plan_result.num_iterations = planner_parameters.max_iters;
        plan_result.tree_size = tree->GetNumberOfNodes();
        plan_result.success = false;

        // Return best path
        auto path = this->ConstructPathFromTree(tree, best_idx);
        if (planner_parameters.visualize_tree)
        {
            this->tree_csv = this->TreePoints(tree, planner_parameters.visualize_tree);
        }
        return path;
    }

    KDL::Twist
    JRRTPlanner::GetTwist(const KDL::Frame &tgt, const KDL::Frame &src, const double &max_dist, const bool &use_rot) const
    {
        KDL::Twist twist;
        auto delta_p = tgt.p - src.p;
        delta_p.Normalize();
        twist.vel = delta_p * max_dist;

        if (use_rot)
        {
            // TODO ROTATION
            // src rot: R1
            // tgt rot: R2
            // difference: subtract R1 then add R2
            // calculation: R2 * R1^T

            KDL::Rotation inv_src_rot = src.M.Inverse();
            double x, y, z;
            // tgt.M.GetRPY(x, y, z);
            // std::cout << "tgt: " << x << ", " << y << ", " << z << "\n";
            // src.M.GetRPY(x, y, z);
            // std::cout << "src: " << x << ", " << y << ", " << z << "\n";
            // exit(1);

            // KDL::Rotation diff_rot = src.M * tgt.M;
            // KDL::Rotation diff_rot = src.M * tgt.M.Inverse();
            // KDL::Rotation diff_rot = tgt.M.Inverse() * src.M;
            // KDL::Rotation diff_rot = inv_src_rot * tgt.M;
            KDL::Rotation diff_rot = tgt.M * inv_src_rot;
            diff_rot.GetRPY(x, y, z);
            // diff_rot.GetEulerZYX(z, y, x);
            KDL::Vector global_xyz(x, y, z);
            KDL::Vector rot_vel = global_xyz;
            // std::cout << "diff: " << x << ", " << y << ", " << z << "\n";

            twist.rot = rot_vel;
            // twist.rot = KDL::Vector(x, y, z);
        }

        return twist;
    }

    AlgorithmState
    JRRTPlanner::ExtendToGoalRRT(std::shared_ptr<BurTree> t_a, JPlusRbtParameters &planner_parameters) const
    {
        // std::cout << "extend to goal\n";
        int randint = this->rng->getRandomInt();
        Grasp random_grasp = planner_parameters.target_poses[randint];
        KDL::Frame p_goal = random_grasp.frame;

        int best_state_idx = random_grasp.best_state;
        RS *best_state = t_a->Get(best_state_idx);
        if (!best_state->hasDistanceEstimate || !best_state->hasJacobian)
        {
            switch (planner_parameters.distanceEstimateType)
            {
            case (DistanceEstimateType::JacPos):
            case (DistanceEstimateType::Projection):
            {
                this->AddDistanceEstimates(*best_state, DistanceEstimateType::JacPos);
                break;
            }
            case (DistanceEstimateType::JacPosRot):
            {
                this->AddDistanceEstimates(*best_state, DistanceEstimateType::JacPosRot);
                break;
            }
            }
        }
        RS near_state = *best_state;
        // std::cout << "has jac: " << near_state.jac.data << "\n";
        // Copy since we will change it

        int prev_idx = t_a->Nearest(best_state_idx);
        double delta_p = random_grasp.best_dist;
        KDL::Frame p_near = this->env->robot->GetEEFrame(*best_state);

        int extension = 0;

        do
        {
            // std::cout << "extension: " << extension << "\n";
            KDL::Vector delta_pos = (p_goal.p - p_near.p);
            // std::cout << "extension dist: " << delta_pos.Norm() << "\n";
            double metric_dist = delta_pos.Norm();
            auto [d, f_tgt] = this->BasicDistanceMetric(p_near, p_goal, planner_parameters.rotation_dist_ratio);
            delta_p = d;
            // if close enough => BREAK
            if (delta_p <= planner_parameters.p_close_enough)
            {
                break;
            }
            bool use_rotation = (delta_p <= planner_parameters.use_rotation);

            // Max dist => epsilon_q
            double dist_to_move = std::min(metric_dist, planner_parameters.epsilon_q);

            MatrixXd p_inv = this->env->robot->JPlus(near_state);
            // .completeOrthogonalDecomposition().pseudoInverse();
            VectorXd delta_frame(6);
            // Is this
            delta_frame.head<3>() << delta_pos[0], delta_pos[1], delta_pos[2];
            // the same as this?
            if (use_rotation)
            {
                double x, y, z;
                (f_tgt.M * p_near.M.Inverse()).GetEulerZYX(z, y, x);
                // RPY and euler return the same angle
                delta_frame(3) = x;
                delta_frame(4) = y;
                delta_frame(5) = z;
            }
            else
            {
                delta_frame(3) = 0;
                delta_frame(4) = 0;
                delta_frame(5) = 0;
            }
            VectorXd delta_q = p_inv * delta_frame;

            // std::cout << "near config: " << near_state.config << "\n";
            RS tmp_state = this->NewState(near_state.config + delta_q);
            prev_idx = this->RRTStepInQ(t_a, prev_idx, tmp_state, planner_parameters.epsilon_q, planner_parameters.collision_resolution, planner_parameters.distanceEstimateType);
            if (prev_idx < 0)
            {
                return AlgorithmState::Trapped;
            }
            near_state = *t_a->Get(prev_idx);
            // if (this->IsColliding(near_state))
            // {
            //     throw std::runtime_error("RRT EXTEND TO GOAL COLLIDING");
            // }
            switch (planner_parameters.distanceEstimateType)
            {
            case (DistanceEstimateType::JacPos):
            case (DistanceEstimateType::Projection):
            {
                this->AddDistanceEstimates(near_state, DistanceEstimateType::JacPos);
                break;
            }
            case (DistanceEstimateType::JacPosRot):
            {
                this->AddDistanceEstimates(near_state, DistanceEstimateType::JacPosRot);
                break;
            }
            }
            // TOOD ADD DISTANCE EStIMATE TO NEARstATE

            this->SetGraspClosestConfigs(planner_parameters, t_a, prev_idx);

            p_near = this->env->robot->GetEEFrame(near_state);
            if (extension++ > planner_parameters.max_extensions)
            {
                return AlgorithmState::Trapped;
            }
        } while (true);
        // std::cout << "extensions: " << extension << " max extensions: " << planner_parameters.max_extensions << " delta_p: " << delta_p << "\n";

        return AlgorithmState::Reached;
    }

    KDL::Vector
    JRRTPlanner::GetRotVec(const KDL::Frame &tgt, const KDL::Frame &src) const
    {
        // TODO ROTATION
        // src rot: R1
        // tgt rot: R2
        // difference: subtract R1 then add R2
        // calculation: R2 * R1^T

        KDL::Rotation inv_src_rot = src.M.Inverse();
        double x, y, z;
        KDL::Rotation diff_rot = tgt.M * inv_src_rot;
        diff_rot.GetRPY(x, y, z);

        return KDL::Vector(x, y, z);
    }

    Eigen::Matrix3d
    JRRTPlanner::ProjectApproachDirection(const Eigen::Matrix3d &rotMatGrasp, const Eigen::Matrix3d &rotMatEE) const
    {
        // Extract the Z-axis (approach vector) of the EE and the Y-axis of the grasp
        Eigen::Vector3d zEE = rotMatEE.col(2);
        Eigen::Vector3d yGrasp = rotMatGrasp.col(1);

        // Project the EE's Z-axis onto the grasp's approach plane
        Eigen::Vector3d zProjected = zEE - (zEE.dot(yGrasp) * yGrasp);
        double norm = zProjected.norm();

        // Check if the projected vector's norm is 0 (i.e., if it's orthogonal to the grasp's approach plane)
        if (norm == 0)
        {
            // If orthogonal, return the original grasp rotation matrix
            return rotMatGrasp;
        }

        // Normalize the projected Z-axis
        Eigen::Vector3d zNew = zProjected / norm;

        // Compute the new X-axis as the cross product of Y-axis of grasp and the new Z-axis
        Eigen::Vector3d xNew = yGrasp.cross(zNew);

        // Assemble the new grasp rotation matrix from the new X, original Y, and new Z axes
        Eigen::Matrix3d newGraspRotMat;
        newGraspRotMat.col(0) = xNew;
        newGraspRotMat.col(1) = yGrasp;
        newGraspRotMat.col(2) = zNew;

        return newGraspRotMat;
    }

    std::optional<std::vector<VectorXd>>
    JRRTPlanner::RotTest(VectorXd q_start, JPlusRbtParameters &planner_parameters, PlanningResult &plan_result)
    {
        if (planner_parameters.target_poses.size() < 1)
        {
            throw std::runtime_error("Target poses has length 0!");
        }

        VectorXd first_state = this->GetRandomQ(1);
        RS start_state = this->NewState(first_state);
        // RS start_state = this->NewState(q_start);

        this->rng = std::make_shared<RandomNumberGenerator>(planner_parameters.seed, planner_parameters.target_poses.size());

        auto tree = std::make_shared<BurTree>(start_state, q_start.size());

        this->InitGraspClosestConfigs(planner_parameters, tree, 0);

        int last_idx = 0;
        for (unsigned int k = 0; k < planner_parameters.max_iters; ++k)
        {
            std::cout << "iter: " << k << "\n";
            int bestgraspid = this->GetBestGrasp(planner_parameters);
            // this->env->robot->KDLFrameToEigen();

            Grasp best_grasp = planner_parameters.target_poses[bestgraspid];
            KDL::Frame grasp_frame = best_grasp.frame;
            RS *best_state = tree->Get(last_idx);
            std::cout << "grasp before: " << grasp_frame << "\n";
            auto [changed, fr] = this->GetClosestSymmetricGrasp(grasp_frame.M, best_state->frames.back().M);
            grasp_frame.M = fr;
            std::cout << "grasp after: " << grasp_frame << "\n";

            // exit(1);

            KDL::Vector twist_vec = this->GetRotVec(best_grasp.frame, best_state->frames.back());
            KDL::Twist twist;
            twist.rot = twist_vec;
            twist.vel = KDL::Vector::Zero();

            KDL::JntArray q_dot = this->env->robot->ForwardJPlus(*best_state, twist);
            VectorXd delta_q = q_dot.data;

            RS new_state = this->NewState(best_state->config + delta_q);
            int step_result = this->RRTStepInQ(tree, last_idx, new_state, planner_parameters.epsilon_q, planner_parameters.collision_resolution);

            if (step_result >= 0)
            {
                // Check distance to goal
                RS new_state = *tree->Get(step_result);
                this->SetGraspClosestConfigs(planner_parameters, tree, step_result);
                last_idx = step_result;
            }
            else
            {
                break;
            }
        }

        // Get grasp with minimal distance
        unsigned int best_grasp_idx = this->GetBestGrasp(planner_parameters);
        Grasp best_grasp = planner_parameters.target_poses[best_grasp_idx];
        // Take measurements
        plan_result.distance_to_goal = best_grasp.best_dist;
        plan_result.num_iterations = planner_parameters.max_iters;
        plan_result.tree_size = tree->GetNumberOfNodes();
        plan_result.success = false;

        // Return best path
        auto path = this->ConstructPathFromTree(tree, last_idx);
        if (planner_parameters.visualize_tree)
        {
            this->tree_csv = this->TreePoints(tree, 100);
        }
        std::cout << "TEST PATH LENGTH: " << path.size() << "\n";
        return path;
    }
}
