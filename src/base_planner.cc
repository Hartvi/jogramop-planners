#include "base_planner.h"
#include <flann/flann.hpp>
#include <Eigen/Dense>
#include <memory>
#include <iostream>
#include <fstream>
#include <vector>

namespace Burs
{
    using namespace Eigen;

    BasePlanner::BasePlanner(std::string path_to_urdf_file) : MinPlanner()
    {
        //   int q_dim, int max_iters, double epsilon_q, MatrixXd bounds}
        std::shared_ptr<BaseEnv> my_env = std::make_shared<BaseEnv>(path_to_urdf_file);

        this->SetEnv(my_env);
        // this->epsilon_q = some random number
        // this->max_iters = some random number

        std::vector<std::vector<double>>
            min_max_bounds = this->env->robot->GetMinMaxBounds();

        this->q_dim = this->env->robot->minMaxBounds.size();

        Eigen::MatrixXd minMaxBounds(q_dim, 2);

        for (int i = 0; i < q_dim; ++i)
        {
            for (int k = 0; k < 2; ++k)
            {
                minMaxBounds(i, k) = min_max_bounds[i][k];
            }
        }
        this->bounds = minMaxBounds;

        // this->myEnv = this->GetEnv<URDFEnv>();
        // this->myRobot = this->myEnv->robot;
    }

    BasePlanner::BasePlanner() {}

    double
    BasePlanner::GetDeltaTk(double phi_tk, double tk, const RS &end_state, const RS &k_state) const
    {
        // VectorXd r_vec = this->radius_func(q_k);

        // std::cout << "end state: " << end_state.frames.size() << "\n";
        // std::cout << "end state: " << end_state.config << "\n";
        // std::cout << "k state: " << k_state.config << "\n";
        double denominator = (end_state.config - k_state.config).cwiseAbs().dot(k_state.radii);
        return phi_tk * (1.0 - tk) / denominator;
    }
    // double
    // BasePlanner::GetDeltaTk(double phi_tk, double tk, const VectorXd &q_e, const VectorXd &q_k) const
    // {
    //     VectorXd r_vec = this->radius_func(q_k);

    //     double denominator = (q_e - q_k).cwiseAbs().dot(r_vec);
    //     return phi_tk * (1.0 - tk) / denominator;
    // }

    std::vector<RS>
    BasePlanner::QToStates(const MatrixXd &Q) const
    {

        std::vector<RS> rand_states;
        rand_states.reserve(Q.cols());

        for (unsigned int i = 0; i < Q.cols(); ++i)
        {
            auto new_state = this->env->robot->FullFK(Q.col(i));
            rand_states.push_back(new_state);
        }
        return rand_states;
    }

    // std::vector<RS>
    // BasePlanner::GetEndpoints(const RS &state_near, const std::vector<RS> &rand_states, std::vector<double> d_maxes, double freeze_dist) const
    // {

    //     double d_small = 0.1 * d_max;

    //     std::vector<RS> new_states;
    //     new_states.reserve(rand_states.size());
    //     // MatrixXd endpoints = MatrixXd::Zero(this->q_dim, rand_Q.cols());

    //     // std::cout << "start q: " << q_near.transpose() << "\n";
    //     for (int i = 0; i < rand_states.size(); ++i)
    //     {
    //         // std::cout << "getendpoints rand state: " << rand_states[i].config.transpose() << "\n";
    //         // // If this won't move further that it is allowed
    //         double maxPossibleDist = this->env->robot->MaxDistance(state_near, rand_states[i]);
    //         if (maxPossibleDist < d_max)
    //         {
    //             new_states.push_back(rand_states[i]);
    //             continue;
    //         }

    //         double tk = 0;

    //         // always start out from the center
    //         RS state_k = state_near;
    //         double phi_result = d_max;
    //         // VectorXd q_k(q_near);

    //         const RS &end_state = rand_states[i];
    //         // const VectorXd q_e = rand_Q.col(i);

    //         // They said 4-5 iterations to reach 0.1*closest_distance
    //         // So either:
    //         //  1. iterate until 0.1*dc
    //         //  2. 4-5 iterations
    //         // for (unsigned int k = 0; k < 5; ++k)
    //         for (unsigned int k = 0; k < 5 && phi_result > d_small; ++k)
    //         // for (unsigned int k = 0; phi_result > d_small; ++k)
    //         // while (phi_result > d_small)
    //         {
    //             // CHECK: this is indeed PI away from q_near

    //             double delta_tk = this->GetDeltaTk(phi_result, tk, end_state, state_k);

    //             tk = tk + delta_tk;
    //             if (tk >= 1.0)
    //             {
    //                 std::cout << "MOVED TOO MUCH IN ENDPOINT CALC\n";
    //                 break;
    //             }
    //             // has actually never reached > 1
    //             // if (tk > 1.0) // some tolerance
    //             // {
    //             //     q_k = q_e;
    //             //     // std::runtime_error("t_k was greater than 1. This shouldn't happen.");
    //             //     break;
    //             // }
    //             // q_k = q_near + tk * (q_e - q_near);
    //             VectorXd q_k = state_near.config + tk * (end_state.config - state_near.config);
    //             state_k = this->QToStates(q_k)[0];
    //             // state_k = this->env->robot->FullFK(q_k);
    //             phi_result = d_max - this->env->robot->MaxDistance(state_near, state_k);
    //             // std::cout << "dist travelled at step " << k << ": " << (d_max - phi_result) << "\n";
    //             // phi_result = d_max - this->MaxMovedDistance(q_near, q_k);
    //         }
    //         new_states.push_back(state_k);
    //         // endpoints.col(i) = q_k;
    //         // double max_moved_dist = this->MaxMovedDistance(q_near, q_k);
    //         // std::cout << "q_k: " << q_k.transpose() << "\n";
    //         // std::cout << "maxmoved dist: " << max_moved_dist << " epsilon: " << d_max << "\n";
    //         // if (max_moved_dist > d_max)
    //         // {
    //         //     std::cout << "MOVED MORE THAN SHOULD HAVE: " << max_moved_dist << " > " << d_max << "\n";
    //         //     // exit(1);
    //         // }
    //     }
    //     // return endpoints;
    //     // for (auto &s : new_states)
    //     // {
    //     //     std::cout << "s: " << s.config.transpose() << "\n";
    //     // }
    //     // assert(new_states.size() < 2);
    //     return new_states;
    // }

    std::vector<RS>
    BasePlanner::GetEndpoints(const RS &state_near, const std::vector<RS> &rand_states, double d_max) const
    {
        // std::cout << "getendpoints state near: " << state_near.config.transpose() << "\n";
        double d_small = 0.1 * d_max;

        std::vector<RS> new_states;
        new_states.reserve(rand_states.size());
        // MatrixXd endpoints = MatrixXd::Zero(this->q_dim, rand_Q.cols());

        // std::cout << "start q: " << q_near.transpose() << "\n";
        for (int i = 0; i < rand_states.size(); ++i)
        {
            // std::cout << "getendpoints rand state: " << rand_states[i].config.transpose() << "\n";
            // // If this won't move further that it is allowed
            double maxPossibleDist = this->env->robot->MaxDistance(state_near, rand_states[i]);
            if (maxPossibleDist < d_max)
            {
                new_states.push_back(rand_states[i]);
                continue;
            }

            double tk = 0;

            // always start out from the center
            RS state_k = state_near;
            double phi_result = d_max;
            // VectorXd q_k(q_near);

            const RS &end_state = rand_states[i];
            // const VectorXd q_e = rand_Q.col(i);

            // They said 4-5 iterations to reach 0.1*closest_distance
            // So either:
            //  1. iterate until 0.1*dc
            //  2. 4-5 iterations
            // for (unsigned int k = 0; k < 5; ++k)
            for (unsigned int k = 0; k < 5 && phi_result > d_small; ++k)
            // for (unsigned int k = 0; phi_result > d_small; ++k)
            // while (phi_result > d_small)
            {
                // CHECK: this is indeed PI away from q_near

                double delta_tk = this->GetDeltaTk(phi_result, tk, end_state, state_k);

                tk = tk + delta_tk;
                if (tk >= 1.0)
                {
                    std::cout << "MOVED TOO MUCH IN ENDPOINT CALC\n";
                    break;
                }
                // has actually never reached > 1
                // if (tk > 1.0) // some tolerance
                // {
                //     q_k = q_e;
                //     // std::runtime_error("t_k was greater than 1. This shouldn't happen.");
                //     break;
                // }
                // q_k = q_near + tk * (q_e - q_near);
                VectorXd q_k = state_near.config + tk * (end_state.config - state_near.config);
                state_k = this->QToStates(q_k)[0];
                // state_k = this->env->robot->FullFK(q_k);
                phi_result = d_max - this->env->robot->MaxDistance(state_near, state_k);
                // std::cout << "dist travelled at step " << k << ": " << (d_max - phi_result) << "\n";
                // phi_result = d_max - this->MaxMovedDistance(q_near, q_k);
            }
            new_states.push_back(state_k);
            // endpoints.col(i) = q_k;
            // double max_moved_dist = this->env->robot->MaxMovedDistance(state_near, state_k);
            // std::cout << "q_k: " << q_k.transpose() << "\n";
            // std::cout << "maxmoved dist: " << max_moved_dist << " epsilon: " << d_max << "\n";
            // if (max_moved_dist > d_max)
            // {
            //     std::cout << "MOVED MORE THAN SHOULD HAVE: " << max_moved_dist << " > " << d_max << "\n";
            //     // exit(1);
            // }
        }
        // return endpoints;
        // for (auto &s : new_states)
        // {
        //     std::cout << "s: " << s.config.transpose() << "\n";
        // }
        // assert(new_states.size() < 2);
        return new_states;
    }

    int
    BasePlanner::AddObstacle(std::string obstacle_file, Eigen::Matrix3d R, Eigen::Vector3d t)
    {
        // std::cout << "JPlusRbtPlanner: adding obstacle " << obstacle_file << std::endl;
        return this->env->AddObstacle(obstacle_file, R, t);
    }

    void
    BasePlanner::SetObstacleRotation(int id, Eigen::Matrix3d R, Eigen::Vector3d t)
    {
        std::shared_ptr<RtModels::RtModel> model = this->env->obstacle_models[id];
        model->SetRotation(R);
        model->SetTranslation(t);
    }

    std::string
    BasePlanner::ToString(const VectorXd &state, bool obstacles)
    {
        std::ostringstream output;
        auto env = this->env;
        env->SetPoses(this->NewState(state));

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
    BasePlanner::StringifyPath(std::vector<VectorXd> path)
    {
        std::cout << "Path length: " << path.size() << "\n";
        std::ostringstream output;

        // first set the obstacles. Planning is time independent, so the obstacles are set once before planning.
        output << this->ToString(path[0], true);

        // go through all intermediate configurations to visualize the path
        for (VectorXd &point : path)
        {
            output << this->ToString(point, false);
        }
        return output.str();
    }

    std::string
    BasePlanner::ConfigsToString(const std::vector<VectorXd> &path)
    {
        std::ostringstream output;

        // go through all intermediate configurations to visualize the path
        for (const VectorXd &point : path)
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

    std::string
    BasePlanner::TreePoints(const std::shared_ptr<BurTree> t, const int &one_out_of) const
    {
        std::ostringstream output;

        // go through all intermediate configurations to visualize the path
        int i = 0;
        for (const auto &node : t->mNodes)
        {
            if (i % one_out_of == 0)
            {
                KDL::Frame EE = this->env->robot->GetEEFrame(node.state);
                auto pos = EE.p;
                output << pos(0);
                for (int k = 1; k < 3; ++k)
                {
                    output << "," << pos(k);
                }
                output << "\n";
            }
            ++i;
        }
        return output.str();
    }

    void
    BasePlanner::ExampleFunctions(const VectorXd &q_start, const VectorXd &q_goal)
    {

        this->env->AddObstacle("path/to/obstacle.obj", Matrix3d::Identity(), Vector3d::Ones());

        RS new_state = this->NewState(q_start);
        // BurTree(VectorXd q_location, int q_dim);
        std::shared_ptr<BurTree> t_a = std::make_shared<BurTree>(new_state, this->q_dim);
        VectorXd Qe = this->GetRandomQ(1);

        // q_near <- NEAREST(q_{e1}, T_a)
        // int nearest_index = this->NearestIndex(t_a, Qe);

        // const VectorXd q_near = t_a->GetQ(nearest_index);
        const double some_delta_q = 0.1;
        // VectorXd q_new = this->GetEndpoints(Qe, q_near, some_delta_q);

        // if (!this->IsColliding(q_new))
        // {
        //     t_a->AddNode(nearest_index, q_new);
        // }

        // CLOSEST DISTANCE
        // double d_closest = this->GetClosestDistance(q_near);
        std::cout << "d < d_crit" << std::endl;
    }

}
