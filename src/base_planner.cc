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
        std::shared_ptr<BaseEnv> my_env = std::make_shared<BaseEnv>(path_to_urdf_file);

        this->SetEnv(my_env);

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
    }

    BasePlanner::BasePlanner() {}

    double
    BasePlanner::GetDeltaTk(double phi_tk, double tk, const RS &end_state, const RS &k_state) const
    {
        double denominator = (end_state.config - k_state.config).cwiseAbs().dot(k_state.radii);
        return phi_tk * (1.0 - tk) / denominator;
    }

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

    std::vector<std::vector<RS>>
    BasePlanner::GetEndpointsInterstates(const RS &state_near, const std::vector<RS> &rand_states, double d_max, double q_resolution) const
    {
        double half_resolution = q_resolution * 0.5;
        double d_small = 0.1 * d_max;

        std::vector<std::vector<RS>> new_states(rand_states.size());

        for (int i = 0; i < rand_states.size(); ++i)
        {
            // If this won't move further that it is allowed
            double maxPossibleDist = this->env->robot->MaxDistance(state_near, rand_states[i]);
            if (maxPossibleDist < d_max)
            {
                new_states[i].push_back(rand_states[i]);
                continue;
            }

            double tk = 0;
            double final_dist = 0;

            // always start out from the center
            RS state_k = state_near;
            double phi_result = d_max;

            const RS &end_state = rand_states[i];
            VectorXd config_delta = end_state.config - state_near.config;
            // They said 4-5 iterations to reach 0.1*closest_distance
            // So either:
            //  1. iterate until 0.1*dc
            //  2. 4-5 iterations
            for (unsigned int k = 0; phi_result > d_small; ++k)
            {
                double delta_tk = this->GetDeltaTk(phi_result, tk, end_state, state_k);

                tk = tk + delta_tk;
                // q_k = q_near + tk * (q_e - q_near);
                VectorXd q_k = state_near.config + tk * config_delta;
                state_k = this->QToStates(q_k)[0];
                final_dist = this->env->robot->MaxDistance(state_near, state_k);
                // std::cout << "intermediate dist: " << final_dist << "\n";
                phi_result = d_max - final_dist;
            }
            // 0.5 => 5 segments => 6 points
            // 0.5/0.1 = 5
            // 5 - 1 = 4 BUT we want 0.1, 0.2, 0.3, 0.4, not 0.0 0.1 0.2 0.3
            // std::cout << "final dist: " << final_dist << " resolution: " << q_resolution << "\n";
            int segments = (final_dist / q_resolution);
            // 1 2 3 4
            for (unsigned int l = 1; l < segments; ++l) // segments = 5 => 1 2 3 4 OK
            {
                VectorXd q_k_tmp = state_near.config + static_cast<double>(l) / static_cast<double>(segments) * tk * (end_state.config - state_near.config);
                RS inter_state = this->NewState(q_k_tmp);

                double tmp_max_dist = this->env->robot->MaxDistance(state_near, inter_state);
                new_states[i].push_back(inter_state);
            }
            new_states[i].push_back(state_k);
        }

        return new_states;
    }

    std::vector<RS>
    BasePlanner::GetEndpoints(const RS &state_near, const std::vector<RS> &rand_states, double d_max) const
    {
        double d_small = 0.1 * d_max;

        std::vector<RS> new_states;
        new_states.reserve(rand_states.size());

        for (int i = 0; i < rand_states.size(); ++i)
        {
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

            const RS &end_state = rand_states[i];

            // They said 4-5 iterations to reach 0.1*closest_distance
            // So either:
            //  1. iterate until 0.1*dc
            //  2. 4-5 iterations
            for (unsigned int k = 0; phi_result > d_small; ++k)
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
                // q_k = q_near + tk * (q_e - q_near);
                VectorXd q_k = state_near.config + tk * (end_state.config - state_near.config);
                state_k = this->QToStates(q_k)[0];
                phi_result = d_max - this->env->robot->MaxDistance(state_near, state_k);
            }
            new_states.push_back(state_k);
        }
        return new_states;
    }

    int
    BasePlanner::AddObstacle(std::string obstacle_file, Eigen::Matrix3d R, Eigen::Vector3d t)
    {
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

}
