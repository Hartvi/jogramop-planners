#include "bur_related/bur_funcs.h"
#include "bur_related/urdf_planner.h"
#include <string>
#include <sstream>

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
    URDFPlanner::URDFPlanner(std::string urdf_file, int max_iters, double d_crit, double delta_q, double epsilon_q, int num_spikes) : BasePlanner()
    // : urdf_file(urdf_file), max_iters(max_iters), d_crit(d_crit), delta_q(delta_q), epsilon_q(epsilon_q), num_spikes(num_spikes)
    {
        // 5/9
        this->max_iters = max_iters;
        this->d_crit = d_crit;
        this->delta_q = delta_q;
        this->epsilon_q = epsilon_q;
        this->num_spikes = num_spikes;

        auto collision_env = std::make_shared<CollisionEnv>(urdf_file);
        this->SetBurEnv(collision_env);

        int q_dim = this->GetNrOfJoints();
        // std::cout << "Starting URDFPlanner..\n File: " << urdf_file << "\nq_dim: " << q_dim << std::endl;

        // ForwardKinematics fk = this->mCollisionEnv->myURDFRobot->GetForwardPointFunc();
        auto my_collision_env = this->GetBurEnv<CollisionEnv>();
        auto my_urdf_robot = my_collision_env->myURDFRobot;

        ForwardKinematicsParallel fkp = my_urdf_robot->GetForwardPointParallelFunc();
        RadiusFuncParallel rf = my_urdf_robot->GetRadiusFunc();
        ForwardRt frt = my_urdf_robot->GetSelectedForwardRtFunc();

        std::vector<std::vector<double>> min_max_bounds = my_urdf_robot->GetMinMaxBounds();

        if (min_max_bounds.size() != q_dim)
        {
            throw std::runtime_error("Min max bounds length ()" + std::to_string(min_max_bounds.size()) + ") doesn't equal number of joints (" + std::to_string(q_dim) + ")\n");
        }

        Eigen::MatrixXd minMaxBounds(q_dim, 2);

        for (int i = 0; i < q_dim; ++i)
        {
            for (int k = 0; k < 2; ++k)
            {
                minMaxBounds(i, k) = min_max_bounds[i][k];
            }
        }
        // 4/9
        this->q_dim = q_dim;
        this->forwardKinematicsParallel = fkp;
        this->bounds = minMaxBounds;
        this->radius_func = rf;

        // BasePlanner(int q_dim, ForwardKinematics f, int max_iters, double d_crit, double delta_q, double epsilon_q, MatrixXd bounds, RadiusFunc radius_func, int num_spikes);
        // this->mBasePlanner = std::make_shared<BasePlanner>(
        // q_dim, fkp, max_iters, d_crit, delta_q, epsilon_q, minMaxBounds, rf, num_spikes);

        // this->mBasePlanner->SetBurEnv(this->mCollisionEnv);
    }

    unsigned int
    URDFPlanner::GetNrOfJoints()
    {
        return this->GetBurEnv<CollisionEnv>()->myURDFRobot->kdl_chain.getNrOfJoints();
    }

    template <typename T>
    std::vector<T>
    URDFPlanner::SelectRandomElements(std::vector<T> &vec, size_t N)
    {
        std::random_device rd;
        std::mt19937 eng(rd());

        std::shuffle(vec.begin(), vec.end(), eng);

        return std::vector<T>(vec.begin(), vec.begin() + N);
    }

    AlgorithmState
    URDFPlanner::CheckGoalStatus(const std::vector<KDL::Frame> &current_poses, const std::vector<KDL::Frame> &target_poses, const double &p_close_enough, unsigned int &closest_index)
    {
        // Maximal cost: k*180 meters + (1-k) * 180 degrees
        double translation_cost = 180;
        double rotation_cost = 180;
        double best_total_cost = 180;

        unsigned int best_cost_index = 0; // select from `i`, i.e. current_poses

        for (unsigned int i = 0; i < current_poses.size(); ++i)
        {
            auto current_pose = current_poses[i];
            double x1, y1, z1;
            current_pose.M.GetRPY(x1, y1, z1);

            for (unsigned int k = 0; k < target_poses.size(); ++k)
            {
                auto target_pose = target_poses[k];
                double tmp_tr_cost = 1000.0 * (current_pose.p - target_pose.p).Norm();

                double x2, y2, z2;
                // Difference in RPY can be used in a Twist command???
                target_pose.M.GetRPY(x2, y2, z2);

                // pitch (y) lies in +- PI/2
                // Note: norm is linear => can take out rad_to_deg outside of each element
                double tmp_rot_cost = rad_to_deg * Eigen::Vector3d(normalizeAngle(x1 - x2), 0.5 * normalizeAngle(2 * (y1 - y2)), normalizeAngle(z1 - z2)).norm();
                // As per Martin Rudorfer :
                double total_cost = tmp_tr_cost + tmp_rot_cost;
                if (total_cost > best_total_cost)
                {
                    best_total_cost = total_cost;
                    best_cost_index = i;
                }
            }
        }

        if (best_total_cost < p_close_enough)
        {
            closest_index = best_cost_index;
            return AlgorithmState::Reached;
        }
        else
        {
            return AlgorithmState::Trapped;
        }
    }

    std::optional<std::vector<Eigen::VectorXd>>
    URDFPlanner::JPlusRbt(const VectorXd &q_start, std::vector<KDL::Frame> &target_poses, const double &probability_to_steer_to_target, const double &p_close_enough)
    {
        auto my_env = this->GetBurEnv<CollisionEnv>();
        auto chain = my_env->myURDFRobot->kdl_chain;
        // unsigned int num_of_targets = target_rotations.size();

        KDL::JntArray q_kdl(this->q_dim);
        KDL::JntArray q_kdl_dot(this->q_dim);

        // for (unsigned int i = 0; i < this->q_dim; ++i)
        // {
        //     q_kdl(i) = q_start(i);
        // }
        q_kdl.data = q_start;

        KDL::ChainFkSolverPos_recursive fk_solver(chain);
        KDL::ChainIkSolverVel_pinv pinv_solver(chain);

        std::random_device rd;                            // Obtain a random number from hardware
        std::mt19937 eng(rd());                           // Seed the generator
        std::uniform_real_distribution<> distr(0.0, 1.0); // Define the range

        std::shared_ptr<BurTree> t_a = std::make_shared<BurTree>(q_start, this->q_dim);
        // Eigen::VectorXd q_new;

        for (int k = 0; k < this->max_iters; k++)
        {
            // IF selecting directed expansion:
            if (distr(eng) < probability_to_steer_to_target)
            {
                // virtual int JntToCart(const JntArray& q_in, Frame& p_out, int segmentNr=-1);
                KDL::Frame p_out;

                // random growth direction; can be any other among the random vectors from Qe
                Eigen::VectorXd q_rand = this->GetRandomQ(1);

                // q_near <- NEAREST(q_{e1}, T_a)
                int nearest_index = this->NearestIndex(t_a, q_rand);

                const VectorXd q_near = t_a->GetQ(nearest_index);
                // update current configuration to a random one
                q_kdl.data = q_near;

                // Get Cartesian end-effector position & rotation
                if (fk_solver.JntToCart(q_kdl, p_out) >= 0)
                {
                    // IGNORE ROTATIONS FOR NOW
                    // p_out.M.;
                    // process M into the same format as in Twist
                    // p_out.p
                    /*
                    #include <cmath> // For M_PI and trigonometric functions

                    // Assuming Rotation class and Vector class are defined as in your description

                    Rotation rotation1;     // First rotation
                    Rotation rotation2;     // Second rotation
                    double deltaTime = 1.0; // For calculating velocity (x1 - x2)/dt

                    // Step 1: Calculate the Rotational Difference
                    // NOTE: The rotation difference goes from the inverse one to the proper one
                    Rotation rotationDiff = rotation2 * rotation1.Inverse();

                    // Step 2: Convert to Axis-Angle
                    Vector axis;
                    double angle = rotationDiff.GetRotAngle(axis); // GetRotAngle should modify 'axis' and return the angle

                    // Step 3: Calculate Angular Velocity
                    Vector angularVelocity = axis * (angle / deltaTime); // Assuming 'Vector' supports scalar multiplicationV
                    */
                }
                else
                {
                    throw std::runtime_error("JPlusRbt: couldn't perform forward kinematics inside steer-to-target");
                }
                Eigen::MatrixXd Qe(this->q_dim, this->num_spikes);

                // Get random target poses from the workspace targets:
                std::vector<KDL::Frame> pose_targets = this->SelectRandomElements(target_poses, (size_t)this->num_spikes);

                for (unsigned int i = 0; i < this->num_spikes; ++i)
                {
                    auto new_twist_target = KDL::Twist();

                    auto pose_target = pose_targets[i];
                    KDL::Vector tgt_pos = pose_target.p;

                    // target translation matrix * inv(my translation matrix) = target_position - my_position
                    new_twist_target.vel = tgt_pos - p_out.p;

                    // NOTE: rotation is immediate rotation about x, y, z axes
                    // RPY = roll - x, pitch - y, yaw (heading) - z
                    p_out.M.SetInverse(); // transpose in-place
                    double x, y, z;
                    (pose_target.M * p_out.M).GetRPY(x, y, z);

                    new_twist_target.rot = KDL::Vector(x, y, z);
                    // new_twist_target.rot = (target_rotation * p_out.M.inv()).ToAnglesXYZ();
                    // IGNORE rotation for now???

                    // TODO: optimize this to reuse the jacobian
                    // Possibly fork the library:
                    // https://github.com/orocos/orocos_kinematics_dynamics/blob/master/orocos_kdl/src/chainiksolvervel_pinv.cpp
                    if (pinv_solver.CartToJnt(q_kdl, new_twist_target, q_kdl_dot) >= 0)
                    {
                        Eigen::VectorXd delta_q = q_kdl_dot.data;
                        Qe.col(i) = q_near + delta_q;
                    }
                }
                // Bur endpoints should be populated now

                // dc(q_near)
                double d_closest = this->GetClosestDistance(q_near);

                if (d_closest < 1e-3)
                {
                    std::cout << "CLOSEST DISTANCE TOO SMALL" << std::endl;
                    return {};
                }

                Eigen::MatrixXd Qnew;

                if (d_closest < this->d_crit)
                {
                    std::cout << "d < d_crit" << std::endl;
                    // Qnew from above, will be used as the new endpoint for BurConnect
                    // small step RRT
                    Qnew = this->GetEndpoint(q_rand, q_near, this->epsilon_q);
                    std::cout << "Qnew: " << Qnew.transpose() << std::endl;

                    if (!this->IsColliding(Qnew))
                    {
                        t_a->AddNode(nearest_index, Qnew);
                    }
                    else
                    {
                        // if small basic rrt collides continue
                        continue;
                    }
                }
                else
                {
                    // HERE it does a forward kinematics pass that I will use for evaluating if I have reached the goal
                    Bur b = this->GetBur(q_near, Qe, d_closest);

                    for (int i = 0; i < Qe.cols(); ++i)
                    {
                        t_a->AddNode(nearest_index, b.endpoints.col(i));
                    }
                    // doesn't matter which column, since they all go in random directions
                    Qnew = b.endpoints.col(0);
                }

                std::vector<KDL::Frame> newest_poses(Qnew.cols());
                for (unsigned int i = 0; i < Qnew.cols(); ++i)
                {
                    // For each new point in the tree see how close it is in the workspace to the goals
                    q_kdl.data = Qnew.col(i);

                    // This forward kinematics has already been done when calculating the bur in the bur's last iteration
                    if (fk_solver.JntToCart(q_kdl, newest_poses[i]) < 0)
                    {
                        throw std::runtime_error("JPlusRbt: couldn't perform forward kinematics.");
                    }
                }
                // CheckGoalStatus(std::vector<KDL::Frame> cur, std::vector<KDL::Frame> tgt, double close)
                unsigned int closest_index = -1;
                AlgorithmState status = this->CheckGoalStatus(newest_poses, target_poses, p_close_enough, closest_index);
                if (status == AlgorithmState::Reached)
                {
                    int a_closest = t_a->Nearest(Qnew.col(closest_index).data());

                    return this->ConstructPathFromTree(t_a, a_closest);
                }
            }
            else // Random expansion
            {
                // TODO fill in the random expansion
            }
        }
        return {};
    }

    std::vector<Eigen::VectorXd>
    URDFPlanner::ConstructPathFromTree(std::shared_ptr<BurTree> t_a, int final_node_id)
    {
        std::vector<Eigen::VectorXd> res_a;

        // connect the two path from the two trees, NODE B and NODE A to each tree's roots respectively
        int node_id_a = final_node_id;
        do
        {
            res_a.push_back(t_a->GetQ(node_id_a));
            node_id_a = t_a->GetParentIdx(node_id_a);
        } while (node_id_a != -1);

        std::reverse(res_a.begin(), res_a.end());

        return res_a;
    }

    std::optional<std::vector<Eigen::VectorXd>> URDFPlanner::PlanPath(Eigen::VectorXd start, Eigen::VectorXd goal)
    {
        std::optional<std::vector<Eigen::VectorXd>> path_opt = this->RbtConnect(start, goal);

        return path_opt;
    }

    int
    URDFPlanner::AddObstacle(std::string obstacle_file, Eigen::Matrix3d R, Eigen::Vector3d t)
    {
        // std::cout << "URDFPlanner: adding obstacle " << obstacle_file << std::endl;
        return this->GetBurEnv<CollisionEnv>()->AddObstacle(obstacle_file, R, t);
    }

    void
    URDFPlanner::SetObstacleRotation(int id, Eigen::Matrix3d R, Eigen::Vector3d t)
    {
        std::shared_ptr<RtModels::RtModel> model = this->GetBurEnv<CollisionEnv>()->obstacle_models[id];
        model->SetRotation(R);
        model->SetTranslation(t);
    }

    std::string
    URDFPlanner::ToString(const Eigen::VectorXd &q_in, bool obstacles)
    {
        std::ostringstream output;
        auto env = this->GetBurEnv<CollisionEnv>();
        env->SetPoses(q_in);

        if (obstacles)
        {
            // std::cout << "URDFPlanner: number of obstacles: " << env->obstacle_models.size() << std::endl;
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
    URDFPlanner::StringifyPath(std::vector<Eigen::VectorXd> path)
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
    URDFPlanner::InterpolatePath(std::vector<Eigen::VectorXd> path, Qunit threshold)
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
