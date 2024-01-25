#include "bur_funcs.h"
#include "j_plus_rbt_planner.h"
#include <string>
#include <sstream>
#include <algorithm>

#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include "printing.h"
#include <typeinfo>
#include <memory>
#include <stdexcept>

#include <random>
#include <cmath>

namespace Burs
{
    JPlusRbtPlanner::JPlusRbtPlanner(std::string urdf_file) : RbtPlanner(urdf_file)
    {
        // TODO: this planner needs to:
        //  - reuse forward-kinematics results
        //  - reuse the Jacobian from a single configuration

        // Idk how to random numbers in C++:
        this->rng = std::make_shared<RandomNumberGenerator>(1);
    }

    AlgorithmState
    JPlusRbtPlanner::CheckGoalStatus(const std::vector<KDL::Frame> &current_poses, const JPlusRbtParameters &planner_parameters, int &closest_index, double &distance_to_goal)
    {
        std::shared_ptr<BurTree> t = planner_parameters.target_poses;
        // t.Nearest();
        // if too far from grasps, check the current p_tree

        // Maximal cost: k*180 meters + (1-k) * 180 degrees
        double translation_cost = 180;
        double rotation_cost = 180;
        double best_total_cost = 180;

        unsigned int best_cost_index = -1; // select from `i`, i.e. current_poses

        // TODO: handle rotations in the tree-search
        // for (unsigned int i = 0; i < current_poses.size(); ++i)
        // {
        //     auto current_pose = current_poses[i];
        //     double x1, y1, z1;
        //     current_pose.M.GetRPY(x1, y1, z1);

        //     for (unsigned int k = 0; k < planner_parameters.target_poses->mNodes.size(); ++k)
        //     {
        //         auto target_pose = target_poses[k];
        //         double tmp_tr_cost = 1000.0 * (current_pose.p - target_pose.p).Norm();

        //         double x2, y2, z2;
        //         // Difference in RPY can be used in a Twist command???
        //         target_pose.M.GetRPY(x2, y2, z2);

        //         // pitch (y) lies in +- PI/2
        //         // Note: norm is linear => can take out rad_to_deg outside of each element
        //         double tmp_rot_cost = rad_to_deg * Eigen::Vector3d(normalizeAngle(x1 - x2), 0.5 * normalizeAngle(2 * (y1 - y2)), normalizeAngle(z1 - z2)).norm();
        //         // As per Martin Rudorfer :
        //         double total_cost = tmp_tr_cost + tmp_rot_cost;
        //         if (total_cost < best_total_cost)
        //         {
        //             best_total_cost = total_cost;
        //             best_cost_index = i;
        //         }
        //     }
        // }

        std::shared_ptr<BurTree> tgp = planner_parameters.target_poses;
        int best_grasp_id = -1;
        for (unsigned int i = 0; i < current_poses.size(); ++i)
        {
            KDL::Vector v = current_poses[i].p;
            double *vdata = v.data;
            auto [n_i, dist] = tgp->NearestIdxAndDist(vdata);

            double tmp_rot_cost = 0.0;
            double tmp_tr_cost = dist;
            double total_cost = tmp_tr_cost + tmp_rot_cost;

            if (total_cost < best_total_cost)
            {
                best_total_cost = total_cost;
                best_cost_index = i;
                best_grasp_id = n_i;
            }
        }

        closest_index = best_cost_index;
        distance_to_goal = best_total_cost;
        if (best_total_cost < planner_parameters.p_close_enough)
        {
            // std::ofstream pos_file("pos_file.txt");
            // if (pos_file.is_open())
            // {
            //     pos_file << "grasp," << tgp->GetQ(best_grasp_id).transpose() << "\n";
            //     pos_file << "current_pose," << current_poses[best_cost_index].p << "\n";
            //     pos_file << "best_total_cost," << best_total_cost << "\n";
            //     pos_file.close();
            // }
            // exit(1);
            return AlgorithmState::Reached;
        }
        else
        {
            return AlgorithmState::Trapped;
        }
    }

    std::optional<std::vector<Eigen::VectorXd>>
    JPlusRbtPlanner::JPlusRbt(const VectorXd &q_start, const JPlusRbtParameters &planner_parameters, PlanningResult &planning_result)
    {
        int num_nodes = planner_parameters.target_poses->GetNumberOfNodes();
        assert(planner_parameters.num_spikes < num_nodes);

        this->rng = std::make_shared<RandomNumberGenerator>(num_nodes - 1);

        auto my_env = this->GetEnv<URDFEnv>();
        auto chain = my_env->myURDFRobot->kdl_chain;

        KDL::JntArray q_kdl(this->q_dim);
        KDL::JntArray q_kdl_dot(this->q_dim);

        q_kdl.data = q_start;

        KDL::ChainFkSolverPos_recursive fk_solver(chain);
        KDL::ChainIkSolverVel_pinv pinv_solver(chain);

        std::shared_ptr<BurTree> q_tree = std::make_shared<BurTree>(q_start, this->q_dim);
        // Eigen::VectorXd q_new;

        AlgorithmState algorithm_state = AlgorithmState::Trapped;
        // int last_idx = -1;
        Eigen::VectorXd last_q;
        planning_result.distance_to_goal = 1e10;

        std::vector<KDL::Frame> newest_pose(1);

        int exploit = 0;

        // // KDL::Frame p_out;
        // std::vector<KDL::Frame> p_out(chain.getNrOfSegments());
        // std::cout << "number of segments: " << p_out.size() << "\n";
        // q_kdl.data = q_start;
        // if (fk_solver.JntToCart(q_kdl, p_out) < 0)
        // {
        //     throw std::runtime_error("Failed forward kinematics in goal status checking");
        // }
        // newest_poses[0] = p_out[chain.getNrOfSegments() - 1];
        // for (int i = 0; i < chain.getNrOfSegments(); ++i)
        // {
        //     std::cout << "segment " << i << ": " << chain.getSegment(i).getName() << " " << p_out[i].p << "\n";
        // }

        // if (fk_solver.JntToCart(q_kdl, p_out[0]) < 0)
        // {
        //     throw std::runtime_error("Failed forward kinematics in goal status checking");
        // }

        // int closest_index = -1;
        // double distance_to_goal;
        // check only once in a while perhaps
        // algorithm_state = this->CheckGoalStatus(newest_poses, planner_parameters, closest_index, distance_to_goal);
        // std::cout << "initial distance to goal:" << distance_to_goal << " \n";

        for (int k = 0; k < planner_parameters.max_iters; k++)
        {
            if (k % 512 == 0)
            {
                std::cout << "          iter: " << k << "\n";
                std::cout.flush();
            }
            algorithm_state = AlgorithmState::Trapped;

            Eigen::VectorXd q_rand = this->GetRandomQ(1);

            // q_near <- NEAREST(q_{e1}, T_a)
            int nearest_index = this->NearestIndex(q_tree, q_rand);

            const VectorXd q_near = q_tree->GetQ(nearest_index);

            // dc(q_near)
            double d_closest = this->GetClosestDistance(q_near);
            // std::cout << "d closest " << d_closest << "\n";

            // If obstacles close, use RRT steps => d_crit
            if (d_closest < planner_parameters.d_crit)
            {
                Eigen::VectorXd q_new;
                // Biased towards goal
                double rand_num = this->rng->getRandomReal();
                if (rand_num < planner_parameters.probability_to_steer_to_target || exploit * planner_parameters.goal_bias_probability)
                // if (this->rng->getRandomReal() < planner_parameters.probability_to_steer_to_target)
                {
                    KDL::ChainIkSolverVel_pinv pinv_solver(chain);

                    KDL::Frame p_out;

                    // directed approach to goal
                    q_kdl.data = q_near;

                    // Get Cartesian end-effector position & rotation
                    if (fk_solver.JntToCart(q_kdl, p_out) < 0)
                    {
                        throw std::runtime_error("JPlusRbt: couldn't perform forward kinematics inside steer-to-target");
                    }

                    // Prepare bur target endpoints container
                    Eigen::MatrixXd Qe(this->q_dim, planner_parameters.num_spikes);

                    // Get random target poses from the workspace targets to extend to
                    std::shared_ptr<BurTree> target_poses = planner_parameters.target_poses;

                    std::vector<RRTNode> target_nodes = target_poses->mNodes;

                    int randint = this->rng->getRandomInt();

                    RRTNode random_node = target_nodes[randint];

                    auto new_twist_target = KDL::Twist();

                    Eigen::VectorXd pose_target = random_node.q;
                    KDL::Vector tgt_pos(pose_target(0), pose_target(1), pose_target(2));
                    new_twist_target.vel = tgt_pos - p_out.p;
                    // TODO: include rotation
                    // For now no rotation
                    new_twist_target.rot = KDL::Vector::Zero();

                    // TODO: optimize this to reuse the jacobian
                    // Possibly fork the library:
                    // https://github.com/orocos/orocos_kinematics_dynamics/blob/master/orocos_kdl/src/chainiksolvervel_pinv.cpp
                    if (pinv_solver.CartToJnt(q_kdl, new_twist_target, q_kdl_dot) >= 0)
                    {
                        Eigen::VectorXd delta_q = q_kdl_dot.data;
                        // q_new = this->GetEndpoint(q_dir, q_near, planner_parameters.epsilon_q);
                        q_new = (q_near + delta_q).normalized() * planner_parameters.epsilon_q;
                    }
                    else
                    {
                        std::runtime_error("RRT section: couldn't do p_inv forward kinematics");
                    }
                    // std::cout << "RRT: biased dir\n";
                }
                else
                {
                    // Random direction
                    Eigen::VectorXd q_dir = this->GetRandomQ(1);
                    q_new = this->GetEndpoint(q_dir, q_near, planner_parameters.epsilon_q);
                }

                if (!this->IsColliding(q_new))
                {
                    q_tree->AddNode(nearest_index, q_new);

                    KDL::Frame p_out;
                    q_kdl.data = q_new;
                    if (fk_solver.JntToCart(q_kdl, p_out) < 0)
                    {
                        throw std::runtime_error("Failed forward kinematics in goal status checking");
                    }
                    newest_pose[0] = p_out;
                    int closest_index = -1;
                    double distance_to_goal;
                    // check only once in a while perhaps
                    algorithm_state = this->CheckGoalStatus(newest_pose, planner_parameters, closest_index, distance_to_goal);

                    if (distance_to_goal < planning_result.distance_to_goal)
                    {
                        planning_result.distance_to_goal = distance_to_goal;
                        last_q = q_new;
                        std::cout << "RRT nearest\n";
                    }

                    if (algorithm_state == AlgorithmState::Reached)
                    {
                        std::cout << "finished in RRT section\n";
                        // last_idx = q_tree->Nearest(q_new.data());
                    }
                }
                else
                {
                    std::cout << "RRT colliding\n";
                }
            }
            else
            { // Grow biased/random bur
                Bur new_bur;
                double rand_num = this->rng->getRandomReal();
                if (rand_num < planner_parameters.probability_to_steer_to_target || exploit * planner_parameters.goal_bias_probability)
                {
                    // Biased bur in target directions

                    new_bur = this->ExtendTowardsCartesian(q_near, planner_parameters, d_closest);
                }
                else
                {

                    // Random point, since we don't know where we're going in configuration space
                    Eigen::MatrixXd Qe = this->GetRandomQ(planner_parameters.num_spikes);

                    // Instead of biased direction calculate random bur and check if have hit the goal
                    // Bur b = this->ExtendTowardsCartesian(q_near, planner_parameters, d_closest);

                    // limit to max pi rotation
                    this->GetEndpoints(Qe, q_near, planner_parameters.delta_q);
                    new_bur = this->GetBur(q_near, Qe, d_closest);
                }

                for (int i = 0; i < planner_parameters.num_spikes; ++i)
                {

                    auto tgt = new_bur.endpoints.col(i);
                    auto dense_tgts = this->Densify(q_near, tgt, planner_parameters);

                    // The first node is obviously the child of the nearest q in the tree
                    q_tree->AddNode(nearest_index, dense_tgts[1]);

                    for (int l = 2; l < dense_tgts.size(); ++l)
                    {
                        auto t = dense_tgts[l];
                        // TODO: Is there a way I can check indices if I know the last added one is gonna be the parent?
                        int nd = q_tree->Nearest(t.data());
                        q_tree->AddNode(nd, t);
                    }
                }

                // This should be the same for directed and randomm expansion
                // Get newest workspace poses
                std::vector<KDL::Frame> newest_poses(planner_parameters.num_spikes);
                // for (unsigned int i = 0; i < planner_parameters.num_spikes; ++i)
                for (int i = 0; i < 1; ++i)
                {
                    KDL::Frame p_out;
                    q_kdl.data = new_bur.endpoints.col(i);
                    if (fk_solver.JntToCart(q_kdl, p_out) < 0)
                    {
                        throw std::runtime_error("Failed forward kinematics in goal status checking");
                    }
                    newest_pose[i] = p_out;
                    // newest_poses[i] = p_out;
                }
                int closest_index = -1;
                double distance_to_goal;
                // check only once in a while perhaps
                algorithm_state = this->CheckGoalStatus(newest_pose, planner_parameters, closest_index, distance_to_goal);
                // algorithm_state = this->CheckGoalStatus(newest_poses, planner_parameters, closest_index, distance_to_goal);
                // IF CLOSE TO TARGET GO FOR IT
                if (distance_to_goal < planner_parameters.goal_bias_radius && exploit == 0)
                {
                    exploit = 1;
                    std::cout << "SWITCH TO EXPLOIT\n";
                    // new_bur = this->ExtendTowardsCartesian(q_near, planner_parameters, d_closest);
                }

                if (distance_to_goal < planning_result.distance_to_goal)
                {
                    planning_result.distance_to_goal = distance_to_goal;
                    last_q = new_bur.endpoints.col(closest_index);
                    std::cout << "RBT nearest\n";
                }

                if (algorithm_state == AlgorithmState::Reached)
                {
                    std::cout << "finished in RBT section\n";

                    // last_idx = q_tree->Nearest(new_bur.endpoints.col(closest_index).data());
                }
            }

            if (algorithm_state == AlgorithmState::Reached)
            {
                // std::cout << "finished in directed section\n";
                planning_result.num_iterations = k;
                planning_result.tree_size = q_tree->GetNumberOfNodes();
                planning_result.success = true;
                // TODO: add time measurement

                std::cout << "FINISHED\n";
                planning_result.distance_to_goal = sqrt(planning_result.distance_to_goal);
                int last_idx = q_tree->Nearest(last_q.data());
                return this->ConstructPathFromTree(q_tree, last_idx);
            }
        }
        planning_result.num_iterations = planner_parameters.max_iters;
        planning_result.tree_size = q_tree->GetNumberOfNodes();
        planning_result.success = false;
        int last_idx = q_tree->Nearest(last_q.data());

        planning_result.distance_to_goal = sqrt(planning_result.distance_to_goal);
        std::cout << "FAILED\n";
        return this->ConstructPathFromTree(q_tree, last_idx);
    }

    Bur
    JPlusRbtPlanner::ExtendTowardsCartesian(const VectorXd &q_near, const JPlusRbtParameters &planner_parameters, const double &closest_distance)
    {
        auto my_env = this->GetEnv<URDFEnv>();
        auto chain = my_env->myURDFRobot->kdl_chain;
        // unsigned int num_of_targets = target_rotations.size();

        KDL::JntArray q_kdl(this->q_dim);
        KDL::JntArray q_kdl_dot(this->q_dim);

        KDL::ChainFkSolverPos_recursive fk_solver(chain);
        KDL::ChainIkSolverVel_pinv pinv_solver(chain);

        KDL::Frame p_out;

        // update current configuration to a random one
        q_kdl.data = q_near;

        // Get Cartesian end-effector position & rotation
        if (fk_solver.JntToCart(q_kdl, p_out) < 0)
        {
            throw std::runtime_error("JPlusRbt: couldn't perform forward kinematics inside steer-to-target");
        }
        // Later we will use this for setting the target rotation velocity
        // REASON: target_rotation * current_rotation^(-1) = the difference in rotations.
        //         Analogical to: target_vector - current_vector
        // p_out.M.SetInverse(); // transpose in-place

        // Prepare bur target endpoints container
        Eigen::MatrixXd Qe(this->q_dim, planner_parameters.num_spikes);

        // Get random target poses from the workspace targets to extend to
        std::shared_ptr<BurTree> target_poses = planner_parameters.target_poses;
        // std::cout << "target pose tree " << target_poses->GetNumberOfNodes() << "\n";
        std::vector<RRTNode> target_nodes = target_poses->mNodes;

        // Get random target poses; don't care about non-repeating I guess
        std::vector<RRTNode> random_nodes(planner_parameters.num_spikes);
        // std::cout << "ExtendTOwardCartesian target nodes: " << target_nodes.size() << "\n";
        for (int i = 0; i < planner_parameters.num_spikes; ++i)
        {
            auto randint = this->rng->getRandomInt();
            // std::cout << "random element idx: " << randint << "\n";
            random_nodes[i] = target_nodes[randint];
        }

        // set the direction for each endpoint in the bur
        for (unsigned int i = 0; i < planner_parameters.num_spikes; ++i)
        {
            auto new_twist_target = KDL::Twist();

            Eigen::VectorXd pose_target = random_nodes[i].q;
            KDL::Vector tgt_pos(pose_target(0), pose_target(1), pose_target(2));
            new_twist_target.vel = tgt_pos - p_out.p;
            // TODO: include rotation
            // For now no rotation
            new_twist_target.rot = KDL::Vector::Zero();

            /*
            KDL::Frame pose_target = target_poses[i];
            KDL::Vector tgt_pos = pose_target.p;

            // target translation matrix * inv(my translation matrix) = target_position - my_position
            new_twist_target.vel = tgt_pos - p_out.p;

            // NOTE: rotation is immediate rotation about x, y, z axes
            // RPY = roll - x, pitch - y, yaw (heading) - z
            double x, y, z;
            (pose_target.M * p_out.M).GetRPY(x, y, z);

            new_twist_target.rot = KDL::Vector(x, y, z);
            */

            // TODO: optimize this to reuse the jacobian
            // Possibly fork the library:
            // https://github.com/orocos/orocos_kinematics_dynamics/blob/master/orocos_kdl/src/chainiksolvervel_pinv.cpp
            if (pinv_solver.CartToJnt(q_kdl, new_twist_target, q_kdl_dot) >= 0)
            {
                Eigen::VectorXd delta_q = q_kdl_dot.data;
                Qe.col(i) = q_near + delta_q;
            }
        }
        // Bur target endpoints should be populated now

        Bur b = this->GetBur(q_near, Qe, closest_distance);

        return b;
    }

    std::pair<AlgorithmState, Eigen::VectorXd>
    JPlusRbtPlanner::ExtendTowardsGoalRRT(std::shared_ptr<BurTree> q_tree, const Eigen::VectorXd q_near, const JPlusRbtParameters &plan_params, PlanningResult &planning_result)
    {
        // begin use once:
        auto env = this->GetEnv<URDFEnv>();
        auto chain = env->myURDFRobot->kdl_chain;

        KDL::ChainFkSolverPos_recursive fk_solver(chain);
        KDL::ChainIkSolverVel_pinv pinv_solver(chain);

        KDL::Frame p_out;
        KDL::JntArray q_kdl;
        KDL::JntArray q_kdl_dot;

        // Get random target poses from the workspace targets to extend to
        std::shared_ptr<BurTree> target_poses = plan_params.target_poses;
        std::vector<RRTNode> target_nodes = target_poses->mNodes;
        int randint = this->rng->getRandomInt();
        RRTNode random_node = target_nodes[randint];
        Eigen::VectorXd pose_target = random_node.q;

        auto new_twist_target = KDL::Twist();
        KDL::Vector tgt_pos(pose_target(0), pose_target(1), pose_target(2));

        std::vector<KDL::Frame> newest_poses(1);
        // :end use once
        double dist_to_goal = 1e10;

        Eigen::VectorXd q_old(q_near);
        Eigen::VectorXd q_new(q_near);

        Eigen::VectorXd final_q(q_near);

        // while (dist_to_goal > plan_params.p_close_enough)
        for (int i = 0; dist_to_goal > plan_params.p_close_enough && i < 100; ++i)
        {
            // directed approach to goal
            q_kdl.data = q_old;

            // Get Cartesian end-effector position & rotation
            if (fk_solver.JntToCart(q_kdl, p_out) < 0)
            {
                throw std::runtime_error("JPlusRbt: couldn't perform forward kinematics inside steer-to-target");
            }
            new_twist_target.vel = tgt_pos - p_out.p;

            // if (new_twist_target.vel.Norm() < plan_params.p_close_enough)
            { // FINISH

                int closest_index = -1;

                // check only once in a while perhaps
                newest_poses[0] = p_out;
                AlgorithmState algorithm_state = this->CheckGoalStatus(newest_poses, plan_params, closest_index, dist_to_goal);

                if (dist_to_goal < planning_result.distance_to_goal)
                {
                    planning_result.distance_to_goal = dist_to_goal;
                    final_q = q_new;
                    std::cout << "nearest\n";
                }

                if (algorithm_state == AlgorithmState::Reached)
                {
                    return std::pair<AlgorithmState, Eigen::VectorXd>(AlgorithmState::Reached, final_q);
                    // last_idx = q_tree->Nearest(q_new.data());
                }
            }
            // TODO: include rotation
            // For now no rotation
            new_twist_target.rot = KDL::Vector::Zero();

            // TODO: optimize this to reuse the jacobian
            // Possibly fork the library:
            // https://github.com/orocos/orocos_kinematics_dynamics/blob/master/orocos_kdl/src/chainiksolvervel_pinv.cpp
            if (pinv_solver.CartToJnt(q_kdl, new_twist_target, q_kdl_dot) >= 0)
            {
                Eigen::VectorXd delta_q = q_kdl_dot.data;
                // q_new = this->GetEndpoint(q_dir, q_near, planner_parameters.epsilon_q);
                q_new = (q_old + delta_q).normalized() * plan_params.epsilon_q;
            }
            else
            {
                std::runtime_error("RRT section: couldn't do p_inv forward kinematics");
            }
            if (!this->IsColliding(q_new))
            {
                int n_i = q_tree->Nearest(q_new.data());
                q_tree->AddNode(n_i, q_new);
                q_old = q_new;
            }
            else
            {
                return std::pair<AlgorithmState, Eigen::VectorXd>(AlgorithmState::Trapped, q_near);
            }
        }
    }

    std::pair<AlgorithmState, Eigen::VectorXd>
    JPlusRbtPlanner::ExtendTowardsGoalRBT(std::shared_ptr<BurTree> q_tree, const Eigen::VectorXd q_near, const double &d_closest, const JPlusRbtParameters &plan_params, PlanningResult &planning_result)
    {
        // begin use once:
        auto env = this->GetEnv<URDFEnv>();
        auto chain = env->myURDFRobot->kdl_chain;

        KDL::ChainFkSolverPos_recursive fk_solver(chain);
        KDL::ChainIkSolverVel_pinv pinv_solver(chain);

        KDL::Frame p_out;
        KDL::JntArray q_kdl;
        KDL::JntArray q_kdl_dot;

        // Get random target poses from the workspace targets to extend to
        std::shared_ptr<BurTree> target_poses = plan_params.target_poses;
        std::vector<RRTNode> target_nodes = target_poses->mNodes;
        int randint = this->rng->getRandomInt();
        RRTNode random_node = target_nodes[randint];
        Eigen::VectorXd pose_target = random_node.q;

        auto new_twist_target = KDL::Twist();
        KDL::Vector tgt_pos(pose_target(0), pose_target(1), pose_target(2));

        std::vector<KDL::Frame> newest_poses(1);
        // :end use once
        double dist_to_goal = 1e10;

        Eigen::VectorXd q_old(q_near);
        Eigen::VectorXd q_new(q_near);

        Eigen::VectorXd final_q(q_near);

        for (int i = 0; dist_to_goal > plan_params.p_close_enough && i < 100; ++i)
        {
            // directed approach to goal
            q_kdl.data = q_old;

            // Get Cartesian end-effector position & rotation
            if (fk_solver.JntToCart(q_kdl, p_out) < 0)
            {
                throw std::runtime_error("JPlusRbt: couldn't perform forward kinematics inside steer-to-target");
            }
            new_twist_target.vel = tgt_pos - p_out.p;
            if (new_twist_target.vel.Norm() < plan_params.d_crit)
            {
                return std::pair<AlgorithmState, Eigen::VectorXd>(AlgorithmState::Trapped, q_near);
            }

            { // FINISH
                int closest_index = -1;

                // check only once in a while perhaps
                newest_poses[0] = p_out;
                AlgorithmState algorithm_state = this->CheckGoalStatus(newest_poses, plan_params, closest_index, dist_to_goal);

                if (dist_to_goal < planning_result.distance_to_goal)
                {
                    planning_result.distance_to_goal = dist_to_goal;
                    final_q = q_new;
                    std::cout << "nearest\n";
                }

                if (algorithm_state == AlgorithmState::Reached)
                {
                    return std::pair<AlgorithmState, Eigen::VectorXd>(AlgorithmState::Reached, final_q);
                    // last_idx = q_tree->Nearest(q_new.data());
                }
            }
            // TODO: include rotation
            // For now no rotation
            new_twist_target.rot = KDL::Vector::Zero();

            // Calculate bur and go as far as possible:
            // TODO: optimize this to reuse the jacobian
            // Possibly fork the library:
            // https://github.com/orocos/orocos_kinematics_dynamics/blob/master/orocos_kdl/src/chainiksolvervel_pinv.cpp
            if (pinv_solver.CartToJnt(q_kdl, new_twist_target, q_kdl_dot) >= 0)
            {
                Eigen::VectorXd delta_q = q_kdl_dot.data;
                // RRT step:
                // q_new = (q_old + delta_q).normalized() * plan_params.epsilon_q;
                // RBT step:
                Bur b = this->GetBur(q_old, q_old + delta_q, d_closest);
                q_new = b.endpoints.col(0);
            }
            else
            {
                std::runtime_error("RRT section: couldn't do p_inv forward kinematics");
            }
            // if (!this->IsColliding(q_new))
            // {
            //     int n_i = q_tree->Nearest(q_new.data());
            //     q_tree->AddNode(n_i, q_new);
            //     q_old = q_new;
            // }
            // else
            // {
            //     return std::pair<AlgorithmState, Eigen::VectorXd>(AlgorithmState::Trapped, q_near);
            // }
        }
    }

    int JPlusRbtPlanner::AddObstacle(std::string obstacle_file, Eigen::Matrix3d R, Eigen::Vector3d t)
    {
        // std::cout << "JPlusRbtPlanner: adding obstacle " << obstacle_file << std::endl;
        return this->GetEnv<URDFEnv>()->AddObstacle(obstacle_file, R, t);
    }

    void JPlusRbtPlanner::SetObstacleRotation(int id, Eigen::Matrix3d R, Eigen::Vector3d t)
    {
        std::shared_ptr<RtModels::RtModel> model = this->GetEnv<URDFEnv>()->obstacle_models[id];
        model->SetRotation(R);
        model->SetTranslation(t);
    }

    std::string
    JPlusRbtPlanner::ToString(const Eigen::VectorXd &q_in, bool obstacles)
    {
        std::ostringstream output;
        auto env = this->GetEnv<URDFEnv>();
        env->SetPoses(q_in);

        if (obstacles)
        {
            // std::cout << "JPlusRbtPlanner: number of obstacles: " << env->obstacle_models.size() << std::endl;
            for (int i = 0; i < env->obstacle_models.size(); ++i)
            {
                output << "obstacle," << i << std::endl;
                output << env->obstacle_models[i]->ToString();
            }
        }
        else
        {
            for (int i = 0; i < env->robot_models.size(); ++i)
            {
                // environment has the OBJs
                output << "robot," << i << std::endl;
                output << env->robot_models[i]->ToString();
            }
        }
        return output.str();
    }

    std::string
    JPlusRbtPlanner::StringifyPath(std::vector<Eigen::VectorXd> path)
    {
        std::cout << "Path length: " << path.size() << "\n";
        std::ostringstream output;

        // first set the obstacles. Planning is time independent, so the obstacles are set once before planning.
        output << this->ToString(path[0], true);

        // go through all intermediate configurations to visualize the path
        for (Eigen::VectorXd &point : path)
        {
            output << this->ToString(point, false);
        }
        return output.str();
    }

    std::string
    JPlusRbtPlanner::ConfigsToString(const std::vector<Eigen::VectorXd> &path)
    {
        std::ostringstream output;

        // go through all intermediate configurations to visualize the path
        for (const Eigen::VectorXd &point : path)
        {
            output << point(0);
            for (int k = 1; k < point.size(); ++k)
            {
                output << "," << point(k);
            }
            output << "\n";
        }
        return output.str();
    }

    // static methods
    std::vector<Eigen::VectorXd>
    JPlusRbtPlanner::InterpolatePath(std::vector<Eigen::VectorXd> path, Qunit threshold)
    {
        std::vector<Eigen::VectorXd> dense_path;
        dense_path.push_back(path[0]);
        for (int i = 0; i < path.size() - 1; ++i)
        {
            Eigen::VectorXd last_point = dense_path[dense_path.size() - 1];
            Eigen::VectorXd delta_path = path[i + 1] - last_point;
            while (delta_path.norm() > threshold)
            {
                Eigen::VectorXd new_point = last_point + threshold * delta_path.normalized();
                // std::cout << "Dense path length: " << dense_path.size() << std::endl;
                dense_path.push_back(new_point);

                last_point = new_point;
                delta_path = path[i + 1] - last_point;
            }
            dense_path.push_back(path[i + 1]);
        }

        return dense_path;
    }

}
