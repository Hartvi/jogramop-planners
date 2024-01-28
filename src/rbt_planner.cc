#include <optional>
#include "collision_env.h"
#include "base_planner.h"
#include "bur_funcs.h"
#include "rbt_planner.h"
#include <flann/flann.hpp>
#include <Eigen/Dense>
#include <memory>
#include <iostream>
#include <fstream>
#include <vector>

namespace Burs
{
    using namespace Eigen;

    // RbtPlanner::RbtPlanner(int q_dim, ForwardKinematicsParallel f, int max_iters, double d_crit, double delta_q, double epsilon_q, Eigen::MatrixXd bounds, RadiusFuncParallel radius_func, int num_spikes)
    //     : BasePlanner(q_dim, max_iters, epsilon_q, bounds), // Initialize the BasePlanner part of the object
    //       forwardKinematicsParallel(f), d_crit(d_crit), delta_q(delta_q), radius_func(radius_func), num_spikes(num_spikes)
    // {
    // }
    RbtPlanner::RbtPlanner(std::string path_to_urdf_file)
        : RRTPlanner(path_to_urdf_file)
    {
        auto my_env = this->GetEnv<URDFEnv>();
        auto my_robot = my_env->myURDFRobot;

        this->forwardKinematicsParallel = my_robot->GetForwardPointParallelFunc();
        this->radius_func = my_robot->GetRadiusFunc();
    }

    RbtPlanner::RbtPlanner() : RRTPlanner()
    {
    }

    double
    RbtPlanner::RhoR(const VectorXd &q1, const VectorXd &q2) const
    {
        double max_distance = 0.0;

        auto res1 = this->forwardKinematicsParallel(q1);
        auto res2 = this->forwardKinematicsParallel(q2);

        for (unsigned int i = 0; i < res1.size(); ++i)
        {
            double tmp = (res1[i] - res2[i]).norm();
            if (tmp > max_distance)
            {
                max_distance = tmp;
            }
        }
        return max_distance;
    }

    double
    RbtPlanner::GetDeltaTk(double phi_tk, double tk, const VectorXd &q_e, const VectorXd &q_k) const
    {
        VectorXd r_vec = this->radius_func(q_k);

        double denominator = (q_e - q_k).cwiseAbs().dot(r_vec);
        return phi_tk * (1.0 - tk) / denominator;
    }

    std::optional<std::vector<Eigen::VectorXd>>
    RbtPlanner::RbtConnect(const VectorXd &q_start, const VectorXd &q_goal, const RbtParameters &plan_parameters, PlanningResult &planning_result)
    {
        // Givens:
        std::shared_ptr<BurTree> t_start = std::make_shared<BurTree>(q_start, q_start.rows());
        std::shared_ptr<BurTree> t_goal = std::make_shared<BurTree>(q_goal, q_goal.rows());
        // TODO: check collision at the beginning
        auto ee_goal = this->GetEEPose(q_goal).p;

        // Changing
        auto t_a = t_start;
        auto t_b = t_goal;
        VectorXd q_best(q_start);
        double best_dist = 1e10;

        for (int k = 0; k < plan_parameters.max_iters; k++)
        {
            MatrixXd Qe = this->GetRandomQ(plan_parameters.num_spikes);

            // random column
            int nearest_idx = t_a->Nearest(Qe.col(0).data());
            VectorXd q_near = t_a->GetQ(nearest_idx);

            SetEndpoints(Qe, q_near, plan_parameters.delta_q);

            // Slow => maybe in the future use FCL and somehow compile it because it had a ton of compilation errors and version mismatches
            double d_closest = this->GetClosestDistance(q_near);

            if (d_closest < plan_parameters.d_crit)
            {
                int step_result = this->RRTStep(t_a, nearest_idx, Qe.col(0), plan_parameters.epsilon_q);
                if (step_result < 0)
                {
                    // If small basic rrt collides, then don't go here, hence the `continue`
                    continue;
                }

                if (t_a == t_start)
                {
                    VectorXd tmp_vec = t_start->GetQ(step_result);
                    double tmp_dist = this->GetDistToGoal(tmp_vec, ee_goal);

                    if (tmp_dist < best_dist)
                    {
                        best_dist = tmp_dist;
                        q_best = tmp_vec;
                    }
                }
            }
            else
            {
                // Qe is already scaled to max euclidean delta_q
                Bur b = this->GetBur(q_near, Qe, d_closest);

                for (int i = 0; i < Qe.cols(); ++i)
                {
                    t_a->AddNode(nearest_idx, b.endpoints.col(i));
                }
            }
            // It is either the one added through RRT, or the last node in the bur
            int last_node_idx = t_a->GetNumberOfNodes() - 1;
            VectorXd q_new = t_a->GetQ(last_node_idx);

            AlgorithmState status = this->BurConnect(t_b, q_new, plan_parameters);
            if (status == AlgorithmState::Reached)
            {
                // `q_new` is in `t_a`
                // `t_b` extends to `q_new` => it has a node near `q_new`
                int a_closest = t_start->Nearest(q_new.data());
                int b_closest = t_goal->Nearest(q_new.data());

                planning_result.distance_to_goal = 0.0;
                planning_result.num_iterations = k;
                planning_result.tree_size = t_start->GetNumberOfNodes() + t_goal->GetNumberOfNodes();
                planning_result.success = true;
                return this->Path(t_start, a_closest, t_goal, b_closest);
            }

            std::swap(t_a, t_b);
        }

        planning_result.num_iterations = plan_parameters.max_iters;
        planning_result.tree_size = t_start->GetNumberOfNodes() + t_goal->GetNumberOfNodes();
        planning_result.success = false;

        int best_idx = t_start->Nearest(q_best.data());
        planning_result.distance_to_goal = this->GetDistToGoal(q_best, ee_goal);

        return this->ConstructPathFromTree(t_start, best_idx);

        // //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // auto env = this->GetEnv<URDFEnv>();
        // KDL::ChainFkSolverPos_recursive fk_solver(env->myURDFRobot->kdl_chain);
        // planning_result.distance_to_goal = 1e10;
        // KDL::Frame p_out;
        // KDL::Vector p_goal;
        // KDL::JntArray q_kdl(this->q_dim);
        // Eigen::VectorXd best_q;

        // q_kdl.data = q_goal;
        // if (fk_solver.JntToCart(q_kdl, p_out) >= 0)
        // {
        //     p_goal = p_out.p;
        // }
        // else
        // {
        //     throw std::runtime_error("RbtConnect: couldn't perform forward kinematics for goal position");
        // }
        // // std::cout << "goal pos: " << p_goal << "\n";

        // q_kdl.data = q_start;
        // // std::cout << "qkdl data: " << q_kdl.data.transpose() << " p_pout: " << p_out << "\n";
        // if (fk_solver.JntToCart(q_kdl, p_out) >= 0)
        // {
        //     double new_distance = (p_out.p - p_goal).Norm();
        //     if (planning_result.distance_to_goal > new_distance)
        //     {
        //         planning_result.distance_to_goal = new_distance;
        //         best_q = q_start;
        //     }
        // }
        // else
        // {
        //     throw std::runtime_error("RbtConnect: couldn't perform forward kinematics for start position");
        // }

        // for (int k = 0; k < plan_parameters.max_iters; k++)
        // {
        //     VectorXd q_new(this->q_dim);
        //     /*
        //     for i in 1:N:
        //         Qe |= random_config()
        //     */
        //     Eigen::MatrixXd Qe = this->GetRandomQ(plan_parameters.num_spikes);

        //     // random growth direction; can be any other among the random vectors from Qe
        //     VectorXd q_e_0 = Qe.col(0);
        //     // q_near <- NEAREST(q_{e1}, T_a)
        //     int nearest_index = this->NearestIndex(t_a, q_e_0);

        //     const VectorXd q_near = t_a->GetQ(nearest_index);

        //     for (int i = 0; i < plan_parameters.num_spikes; i++)
        //     {
        //         VectorXd q_e_i = Qe.col(i);
        //         q_e_i = this->GetEndpoint(q_e_i, q_near, plan_parameters.delta_q);
        //         Qe.col(i).array() = q_e_i;
        //     }

        //     // dc(q_near)
        //     double d_closest = this->GetClosestDistance(q_near);
        //     // std::cout << "d_closest: " << d_closest << std::endl;

        //     // if (d_closest < 1e-3)
        //     // {
        //     //     std::cout << "CLOSEST DISTANCE TOO SMALL" << std::endl;

        //     //     return {};
        //     // }

        //     if (d_closest < plan_parameters.d_crit)
        //     {
        //         std::cout << "d < d_crit" << std::endl;
        //         // q_new from above, will be used as the new endpoint for BurConnect
        //         // small step RRT
        //         q_new = this->GetEndpoint(q_e_0, q_near, plan_parameters.epsilon_q);
        //         std::cout << "q_new: " << q_new.transpose() << std::endl;

        //         if (!this->IsColliding(q_new))
        //         {
        //             t_a->AddNode(nearest_index, q_new);
        //         }
        //         else
        //         {
        //             // if small basic rrt collides, then don't go here, hence the `continue`
        //             continue;
        //         }
        //     }
        //     else
        //     {
        //         Bur b = this->GetBur(q_near, Qe, d_closest);

        //         for (int i = 0; i < Qe.cols(); ++i)
        //         {
        //             t_a->AddNode(nearest_index, b.endpoints.col(i));
        //         }
        //         // doesn't matter which column, since they all go in random directions
        //         q_new = b.endpoints.col(0);
        //     }

        //     q_kdl.data = q_new;
        //     if (fk_solver.JntToCart(q_kdl, p_out) >= 0)
        //     {
        //         double new_distance = (p_out.p - p_goal).Norm();
        //         if (planning_result.distance_to_goal > new_distance)
        //         {
        //             planning_result.distance_to_goal = new_distance;
        //             best_q = q_start;
        //         }
        //     }
        //     else
        //     {
        //         throw std::runtime_error("RbtConnect: couldn't perform forward kinematics for progress check");
        //     }

        //     // if reached, then index is the closest node in `t_b` to `q_new` in `t_a`
        //     AlgorithmState status = this->BurConnect(t_b, q_new, plan_parameters);
        //     if (status == AlgorithmState::Reached)
        //     {
        //         int a_closest = t_start->Nearest(q_new.data());
        //         int b_closest = t_goal->Nearest(q_new.data());

        //         planning_result.num_iterations = k;
        //         planning_result.tree_size = t_start->GetNumberOfNodes() + t_goal->GetNumberOfNodes();
        //         planning_result.success = true;
        //         return this->Path(t_start, a_closest, t_goal, b_closest);
        //     }

        //     std::swap(t_a, t_b);
        // }

        // planning_result.num_iterations = plan_parameters.max_iters;
        // planning_result.tree_size = t_start->GetNumberOfNodes() + t_goal->GetNumberOfNodes();
        // planning_result.success = false;

        // int last_idx = t_start->Nearest(best_q.data());
        // return this->ConstructPathFromTree(t_start, last_idx);
    }

    std::optional<std::vector<Eigen::VectorXd>>
    // RbtPlanner::RbtConnect(const VectorXd &q_start, const VectorXd &q_goal)
    RbtPlanner::RbtConnect(const VectorXd &q_start, const VectorXd &q_goal, const RbtParameters &plan_parameters)
    {
        // start of actual algorithm
        std::shared_ptr<BurTree> t_start = std::make_shared<BurTree>(q_start, q_start.rows());
        std::shared_ptr<BurTree> t_goal = std::make_shared<BurTree>(q_goal, q_goal.rows());
        auto t_a = t_start;
        auto t_b = t_goal;
        // TODO: check collision at the beginning

        for (int k = 0; k < plan_parameters.max_iters; k++)
        {
            VectorXd q_new(this->q_dim);
            /*
            for i in 1:N:
                Qe |= random_config()
            */
            Eigen::MatrixXd Qe = this->GetRandomQ(plan_parameters.num_spikes);

            // random growth direction; can be any other among the random vectors from Qe
            VectorXd q_e_0 = Qe.col(0);
            // q_near <- NEAREST(q_{e1}, T_a)
            int nearest_index = this->NearestIndex(t_a, q_e_0);

            const VectorXd q_near = t_a->GetQ(nearest_index);

            for (int i = 0; i < plan_parameters.num_spikes; i++)
            {
                VectorXd q_e_i = Qe.col(i);
                q_e_i = this->GetEndpoint(q_e_i, q_near, plan_parameters.delta_q);
                Qe.col(i).array() = q_e_i;
            }

            // dc(q_near)
            double d_closest = this->GetClosestDistance(q_near);
            // std::cout << "d_closest: " << d_closest << std::endl;

            // if (d_closest < 1e-3)
            // {
            //     std::cout << "CLOSEST DISTANCE TOO SMALL" << std::endl;

            //     return {};
            // }

            if (d_closest < plan_parameters.d_crit)
            {
                std::cout << "d < d_crit" << std::endl;
                // q_new from above, will be used as the new endpoint for BurConnect
                // small step RRT
                q_new = this->GetEndpoint(q_e_0, q_near, plan_parameters.epsilon_q);
                std::cout << "q_new: " << q_new.transpose() << std::endl;

                if (!this->IsColliding(q_new))
                {
                    t_a->AddNode(nearest_index, q_new);
                }
                else
                {
                    // if small basic rrt collides, then don't go here, hence the `continue`
                    continue;
                }
            }
            else
            {
                Bur b = this->GetBur(q_near, Qe, d_closest);

                for (int i = 0; i < Qe.cols(); ++i)
                {
                    t_a->AddNode(nearest_index, b.endpoints.col(i));
                }
                // doesn't matter which column, since they all go in random directions
                q_new = b.endpoints.col(0);
            }

            // if reached, then index is the closest node in `t_b` to `q_new` in `t_a`
            AlgorithmState status = this->BurConnect(t_b, q_new, plan_parameters);
            if (status == AlgorithmState::Reached)
            {
                int a_closest = t_start->Nearest(q_new.data());
                int b_closest = t_goal->Nearest(q_new.data());

                return this->Path(t_start, a_closest, t_goal, b_closest);
            }

            std::swap(t_a, t_b);
        }
        return {};
    }

    AlgorithmState
    RbtPlanner::BurConnect(std::shared_ptr<BurTree> t, VectorXd &q, const RbtParameters &plan_parameters)
    {
        int nearest_index = t->Nearest(q.data());

        VectorXd q_n = t->GetQ(nearest_index);
        VectorXd q_0(q_n);

        double delta_s = 1e14;
        double threshold = 1e-2;

        while (delta_s >= plan_parameters.epsilon_q)
        {
            double d_closest = this->GetClosestDistance(q_n);

            if (d_closest > plan_parameters.d_crit)
            {
                // if q_n is within the collision free bur of q, then we finish, game over
                Bur b = this->GetBur(q_n, q, d_closest);

                VectorXd q_t = b.endpoints.col(0);

                delta_s = (q_t - q_n).norm();

                q_n = q_t;

                if (q_n.isApprox(q, threshold))
                {
                    return AlgorithmState::Reached;
                }
            }
            else
            {
                VectorXd q_t = this->GetEndpoint(q, q_n, plan_parameters.epsilon_q);

                // if not colliding then proceed
                if (!this->IsColliding(q_t))
                {
                    q_n = q_t;
                }
                else
                {
                    return AlgorithmState::Trapped;
                }

                if ((q_n - q_0).norm() >= (q - q_0).norm())
                {
                    return AlgorithmState::Reached;
                }
            }
        }
        return AlgorithmState::Trapped;
    }

    Bur
    RbtPlanner::GetBur(const VectorXd &q_near, const MatrixXd &Q_e, double d_closest)
    {
        double d_small = 0.1 * d_closest;
        MatrixXd endpoints = MatrixXd::Zero(this->q_dim, Q_e.cols());

        for (int i = 0; i < Q_e.cols(); ++i)
        {
            double tk = 0;

            // always start out from the center
            VectorXd q_k(q_near);
            double phi_result = d_closest;

            const VectorXd q_e = Q_e.col(i);

            // They said 4-5 iterations to reach 0.1*closest_distance
            // So either:
            //  1. iterate until 0.1*dc
            //  2. 4-5 iterations
            for (unsigned int k = 0; k < 5; ++k)
            // while (phi_result > d_small)
            {
                // CHECK: this is indeed PI away from q_near
                phi_result = d_closest - this->RhoR(q_near, q_k);
                double delta_tk = this->GetDeltaTk(phi_result, tk, q_e, q_k);
                tk = tk + delta_tk;
                // has actually never reached > 1
                // if (tk > 1.0) // some tolerance
                // {
                //     q_k = q_e;
                //     // std::runtime_error("t_k was greater than 1. This shouldn't happen.");
                //     break;
                // }
                q_k = q_near + tk * (q_e - q_near);
            }
            endpoints.col(i) = q_k;
        }
        Bur myBur(q_near, endpoints);
        return myBur;
    }

    std::vector<Eigen::VectorXd>
    RbtPlanner::Densify(const Eigen::VectorXd &src, const Eigen::VectorXd &tgt, const RbtParameters &plan_params)
    {
        std::vector<Eigen::VectorXd> dense_path;
        dense_path.push_back(src);

        Eigen::VectorXd delta_path = tgt - src;
        double threshold = plan_params.q_resolution;
        double delta_path_norm = delta_path.norm();

        // Calculate the number of steps needed
        int steps = static_cast<int>(std::ceil(delta_path_norm / threshold));
        // 2>1 => 2 steps
        // Adjust threshold to evenly distribute points
        threshold = delta_path_norm / steps; // => split in half

        for (int i = 1; i < steps; ++i)
        {
            Eigen::VectorXd new_point = src + i * threshold * delta_path.normalized();
            dense_path.push_back(new_point);
        }
        dense_path.push_back(tgt);

        return dense_path;
    }

}
