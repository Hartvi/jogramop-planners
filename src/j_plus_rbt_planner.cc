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

    std::shared_ptr<BurTree>
    JPlusRbtPlanner::ConstructTreeFromTargets(std::vector<KDL::Frame> &target_poses)
    {
        // TODO: add rotation
        // Eigen::VectorXd root_node(/*load translation rotation vector*/);
        Eigen::Vector3d root_node(target_poses[0].p.x(), target_poses[0].p.y(), target_poses[0].p.z());

        std::shared_ptr<BurTree> t = std::make_shared<BurTree>(root_node, root_node.size());

        for (int i = 1; i < target_poses.size(); ++i)
        {
            Eigen::Vector3d new_node(target_poses[i].p.x(), target_poses[i].p.y(), target_poses[i].p.z());

            t->AddNode(i - 1, new_node);
        }

        return t;
    }

    Eigen::Vector3d
    JPlusRbtPlanner::GetMeanTranslation(std::vector<KDL::Frame> &target_poses)
    {
        Eigen::Vector3d mean_vector(0, 0, 0);
        for (int i = 0; i < target_poses.size(); ++i)
        {
            Eigen::Vector3d v(target_poses[i].p.x(), target_poses[i].p.y(), target_poses[i].p.z());
            mean_vector += v;
        }
        mean_vector /= target_poses.size();
        return mean_vector;
    }

    AlgorithmState
    JPlusRbtPlanner::CheckGoalStatus(const std::vector<KDL::Frame> &current_poses, const JPlusRbtParameters &planner_parameters, unsigned int &closest_index)
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
            }
        }

        closest_index = best_cost_index;
        if (best_total_cost < planner_parameters.p_close_enough)
        {
            return AlgorithmState::Reached;
        }
        else
        {
            return AlgorithmState::Trapped;
        }
    }

    std::optional<std::vector<Eigen::VectorXd>>
    JPlusRbtPlanner::JPlusRbt(const VectorXd &q_start, const JPlusRbtParameters &planner_parameters)
    {

        int num_nodes = planner_parameters.target_poses->GetNumberOfNodes();
        assert(planner_parameters.num_spikes < num_nodes);

        this->rng = std::make_shared<RandomNumberGenerator>(num_nodes);

        auto my_env = this->GetEnv<URDFEnv>();
        auto chain = my_env->myURDFRobot->kdl_chain;
        // unsigned int num_of_targets = target_rotations.size();

        KDL::JntArray q_kdl(this->q_dim);
        KDL::JntArray q_kdl_dot(this->q_dim);

        q_kdl.data = q_start;

        KDL::ChainFkSolverPos_recursive fk_solver(chain);
        KDL::ChainIkSolverVel_pinv pinv_solver(chain);

        std::shared_ptr<BurTree> q_tree = std::make_shared<BurTree>(q_start, this->q_dim);
        // Eigen::VectorXd q_new;

        for (int k = 0; k < planner_parameters.max_iters; k++)
        {
            // IF selecting directed expansion:
            if (this->rng->getRandomReal() < planner_parameters.probability_to_steer_to_target)
            {
                // Biased bur in target directions
                // Random point, since we don't know where we're going in configuration space
                Eigen::VectorXd q_rand = this->GetRandomQ(1);

                // q_near <- NEAREST(q_{e1}, T_a)
                int nearest_index = this->NearestIndex(q_tree, q_rand);

                const VectorXd q_near = q_tree->GetQ(nearest_index);

                // dc(q_near)
                double d_closest = this->GetClosestDistance(q_near);

                Bur b = this->ExtendTowardsCartesian(q_near, planner_parameters, d_closest);

                for (int i = 0; i < planner_parameters.num_spikes; ++i)
                {
                    q_tree->AddNode(nearest_index, b.endpoints.col(i));
                }

                // This should be the same for directed and randomm expansion
                // Get newest workspace poses
                std::vector<KDL::Frame> newest_poses(planner_parameters.num_spikes);
                for (unsigned int i = 0; i < planner_parameters.num_spikes; ++i)
                {
                    KDL::Frame p_out;
                    q_kdl.data = b.endpoints.col(i);
                    if (fk_solver.JntToCart(q_kdl, p_out) < 0)
                    {
                        throw std::runtime_error("Failed forward kinematics in goal status checking");
                    }
                    newest_poses[i] = p_out;
                }
                unsigned int closest_index = -1;
                // check only once in a while perhaps
                AlgorithmState status = this->CheckGoalStatus(newest_poses, planner_parameters, closest_index);

                if (status == AlgorithmState::Reached)
                {

                    int a_closest = q_tree->Nearest(b.endpoints.col(closest_index).data());

                    return this->ConstructPathFromTree(q_tree, a_closest);
                }
            }
            else // Random expansion
            {
                // TODO fill in the random expansion

                // Random point, since we don't know where we're going in configuration space
                Eigen::MatrixXd Qe = this->GetRandomQ(planner_parameters.num_spikes);
                // Random anyway
                Eigen::VectorXd q_rand = Qe.col(0);

                // q_near <- NEAREST(q_{e1}, T_a)
                int nearest_index = this->NearestIndex(q_tree, q_rand);

                const VectorXd q_near = q_tree->GetQ(nearest_index);

                // dc(q_near)
                double d_closest = this->GetClosestDistance(q_near);

                // Instead of biased direction calculate random bur and check if have hit the goal
                // Bur b = this->ExtendTowardsCartesian(q_near, planner_parameters, d_closest);

                Bur b = this->GetBur(q_near, Qe, d_closest);

                for (int i = 0; i < planner_parameters.num_spikes; ++i)
                {
                    q_tree->AddNode(nearest_index, b.endpoints.col(i));
                }

                // Get newest workspace poses
                std::vector<KDL::Frame> newest_poses(planner_parameters.num_spikes);
                for (unsigned int i = 0; i < planner_parameters.num_spikes; ++i)
                {
                    KDL::Frame p_out;
                    q_kdl.data = b.endpoints.col(i);
                    if (fk_solver.JntToCart(q_kdl, p_out) < 0)
                    {
                        throw std::runtime_error("Failed forward kinematics in goal status checking");
                    }
                    newest_poses[i] = p_out;
                }
                unsigned int closest_index = -1;
                // check only once in a while perhaps
                AlgorithmState status = this->CheckGoalStatus(newest_poses, planner_parameters, closest_index);

                if (status == AlgorithmState::Reached)
                {

                    int a_closest = q_tree->Nearest(b.endpoints.col(closest_index).data());

                    return this->ConstructPathFromTree(q_tree, a_closest);
                }
            }
        }
        return {};
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
        p_out.M.SetInverse(); // transpose in-place

        // Prepare bur target endpoints container
        Eigen::MatrixXd Qe(this->q_dim, planner_parameters.num_spikes);

        // Get random target poses from the workspace targets to extend to
        std::shared_ptr<BurTree> target_poses = planner_parameters.target_poses;
        std::vector<RRTNode> target_nodes = target_poses->mNodes;

        // Get random target poses; don't care about non-repeating I guess
        std::vector<RRTNode> random_nodes(planner_parameters.num_spikes);
        for (int i = 0; i < planner_parameters.num_spikes; ++i)
        {
            random_nodes[i] = target_nodes[this->rng->getRandomInt()];
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

    std::vector<Eigen::VectorXd>
    JPlusRbtPlanner::ConstructPathFromTree(std::shared_ptr<BurTree> q_tree, int final_node_id)
    {
        std::vector<Eigen::VectorXd> res_a;

        // connect the two path from the two trees, NODE B and NODE A to each tree's roots respectively
        int node_id_a = final_node_id;
        do
        {
            res_a.push_back(q_tree->GetQ(node_id_a));
            node_id_a = q_tree->GetParentIdx(node_id_a);
        } while (node_id_a != -1);

        std::reverse(res_a.begin(), res_a.end());

        return res_a;
    }

    int
    JPlusRbtPlanner::AddObstacle(std::string obstacle_file, Eigen::Matrix3d R, Eigen::Vector3d t)
    {
        // std::cout << "JPlusRbtPlanner: adding obstacle " << obstacle_file << std::endl;
        return this->GetEnv<URDFEnv>()->AddObstacle(obstacle_file, R, t);
    }

    void
    JPlusRbtPlanner::SetObstacleRotation(int id, Eigen::Matrix3d R, Eigen::Vector3d t)
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