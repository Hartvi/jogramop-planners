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
    BasePlanner::GetDeltaTkGeometry(double phi_tk, double tk, const RS &end_state, const RS &k_state) const
    {
        VectorXd deltaConfigs = (end_state.config - k_state.config).cwiseAbs();
        // radii are positive => can add the vectors then dot product
        VectorXd radiiSum = k_state.radii + k_state.rigidRadii;
        double denominator = deltaConfigs.dot(radiiSum);
        return phi_tk * (1.0 - tk) / denominator;
    }

    double
    BasePlanner::GetDeltaTk(double phi_tk, double tk, const RS &end_state, const RS &k_state) const
    {
        double denominator = (end_state.config - k_state.config).cwiseAbs().dot(k_state.radii);
        return phi_tk * (1.0 - tk) / denominator;
    }

    double
    BasePlanner::GetDeltaTkGeneral(const RS &state_near, double tk, const RS &end_state, const RS &k_state, const DistanceEstimateType &det, const bool &expanded_bubble) const
    {
        VectorXd deltaConfigs = (end_state.config - k_state.config).cwiseAbs();
        // radii are positive => can add the vectors then do dot product
        VectorXd radiiSum; // = k_state.radii + k_state.rigidRadii;
        switch (det)
        {
        case (DistanceEstimateType::JacPos):
        case (DistanceEstimateType::Projection):
        {
            radiiSum = k_state.radii;
            break;
        }
        case (DistanceEstimateType::JacPosRot):
        {
            radiiSum = k_state.radii + k_state.rigidRadii;
            break;
        }
        case (DistanceEstimateType::None):
        {
            throw std::runtime_error("GetDeltaTkGeneral: cannot compute deltatk for None distance estimate type");
            break;
        }
        }
        if (expanded_bubble)
        {
            std::vector<double> rho_profile = this->env->robot->MaxDistanceMeshPositions(state_near, k_state);
            std::vector<double> distance_profile = state_near.closest_dists;
            VectorXd phi_tk(this->q_dim);
            int j = 0;
            for (size_t i = 0; i < rho_profile.size(); ++i)
            {
                double denominator = radiiSum.head(j + 1).dot(deltaConfigs.head(j + 1));
                phi_tk(j) = (distance_profile[i] - rho_profile[i]) / denominator;
                if (this->env->robot->kdl_chain.getSegment(i).getJoint().getType() != KDL::Joint::JointType::None)
                {
                    ++j;
                }
            }
            return phi_tk.minCoeff() * (1.0 - tk);
        }
        else
        {
            double rho = this->env->robot->MaxDistance(state_near, k_state);
            double d_c = state_near.closest_dists[state_near.closest_distance_ids[0]];
            double denominator = radiiSum.dot(deltaConfigs);
            double phi_tk = (d_c - rho) / denominator;
            return phi_tk * (1.0 - tk);
        }
    }

    std::vector<RS>
    BasePlanner::GetEndpointsGeneral(RS &state_near, const std::vector<RS> &rand_states, const DistanceEstimateType &det, const size_t &max_iters, const bool &expanded_bubble)
    {
        // double d_small = 0.1 * d_max;

        std::vector<RS> new_states;
        if (!state_near.hasDistanceEstimate)
        {
            this->AddDistanceEstimates(state_near, det);
        }

        for (int i = 0; i < rand_states.size(); ++i)
        {
            double tk = 0;

            // always start out from the center
            RS state_k = state_near;
            // double phi_result = d_max;
            const RS &end_state = rand_states[i];
            // double delta_tk = this->GetDeltaTkGeneral(phi_result, tk, end_state, state_near, det, expanded_bubble);
            // double max_travelled_dist = 0;

            size_t k = 0;
            while (true)
            {
                double delta_tk = this->GetDeltaTkGeneral(state_near, tk, end_state, state_k, det, expanded_bubble);
                tk = tk + delta_tk;
                VectorXd q_k = state_near.config + tk * (end_state.config - state_near.config);
                // max_iters = 5
                // k=1 k=2 k=3 k=4 (k=5)=>None
                state_k = this->NewState(q_k, (++k < max_iters) ? det : DistanceEstimateType::None);
                // to prevent unnecessary computations below
                if (k == max_iters)
                {
                    break;
                }
            }
            if (det != DistanceEstimateType::JacPosRot)
            {
                if (this->IsColliding(state_k))
                {
                    continue;
                }
            }
            // state_k.distanceFromParent = max_travelled_dist;
            // state_k.hasDistFromParent = true;
            new_states.push_back(state_k);
        }
        return new_states;
    }

    std::vector<RS>
    BasePlanner::GetEndpointsGeometry(const RS &state_near, const std::vector<RS> &rand_states, double d_max)
    {
        double d_small = 0.1 * d_max;

        std::vector<RS> new_states;
        new_states.reserve(rand_states.size());

        for (int i = 0; i < rand_states.size(); ++i)
        {
            double tk = 0;

            // always start out from the center
            RS state_k = state_near;
            double phi_result = d_max;
            double max_travelled_dist = 0;

            const RS &end_state = rand_states[i];

            // They said 4-5 iterations to reach 0.1*closest_distance
            // So either:
            //  1. iterate until 0.1*dc
            //  2. 4-5 iterations
            size_t k = 0;
            // for (; phi_result > d_small; ++k)
            for (; k < 6; ++k)
            {
                double delta_tk = this->GetDeltaTkGeometry(phi_result, tk, end_state, state_k);

                tk = tk + delta_tk;
                VectorXd q_k = state_near.config + tk * (end_state.config - state_near.config);
                state_k = this->NewState(q_k, DistanceEstimateType::JacPosRot);
                max_travelled_dist = this->env->robot->MaxDistanceMeshes(state_near, state_k);
                phi_result = d_max - max_travelled_dist;
            }
            // std::cout << "k: " << k << " max travelled dist: " << max_travelled_dist << " d_c: " << d_max << "\n";
            // the below has never actually happened
            // if (max_travelled_dist > d_max)
            // {
            //     std::cout << "k: " << k << " max travelled dist: " << max_travelled_dist << " d_c: " << d_max << "\n";
            //     throw std::runtime_error("MAX TRAVELED DIST > DMAX");
            // }
            state_k.distanceFromParent = max_travelled_dist;
            state_k.hasDistFromParent = true;
            new_states.push_back(state_k);
        }
        return new_states;
    }

    std::vector<RS>
    BasePlanner::GetEndpointsCollisionCheck(const RS &state_near, const std::vector<RS> &rand_states, double d_max) const
    {
        double d_small = 0.1 * d_max;

        std::vector<RS> new_states;
        // new_states.reserve(rand_states.size());

        for (int i = 0; i < rand_states.size(); ++i)
        {
            double tk = 0;
            double max_travelled_dist = 0;

            // always start out from the center
            RS state_k = state_near;
            double phi_result = d_max;

            const RS &end_state = rand_states[i];

            // They said 4-5 iterations to reach 0.1*closest_distance
            // So either:
            //  1. iterate until 0.1*dc
            //  2. 4-5 iterations
            size_t k = 0;
            bool hasCollided = false;
            for (; k < 5; ++k)
            {
                double delta_tk = this->GetDeltaTk(phi_result, tk, end_state, state_k);

                // has actually never reached > 1
                tk = tk + delta_tk;
                // q_k = q_near + tk * (q_e - q_near);
                VectorXd q_k = state_near.config + tk * (end_state.config - state_near.config);
                state_k = this->NewState(q_k, DistanceEstimateType::Projection);
                max_travelled_dist = this->env->robot->MaxDistance(state_near, state_k);
                phi_result = d_max - max_travelled_dist;
            }
            if (this->IsColliding(state_k))
            {
                hasCollided = true;
                break;
            }
            if (!hasCollided)
            {
                state_k.distanceFromParent = max_travelled_dist;
                state_k.hasDistFromParent = true;
                new_states.push_back(state_k);
            }
        }
        return new_states;
    }

    std::vector<RS>
    BasePlanner::GetEndpointsCollisionCheckOriginal(const RS &state_near, const std::vector<RS> &rand_states, double d_max) const
    {
        double d_small = 0.1 * d_max;

        std::vector<RS> new_states;
        // new_states.reserve(rand_states.size());

        for (int i = 0; i < rand_states.size(); ++i)
        {
            double tk = 0;
            double max_travelled_dist = 0;

            // always start out from the center
            RS state_k = state_near;
            double phi_result = d_max;

            const RS &end_state = rand_states[i];

            // They said 4-5 iterations to reach 0.1*closest_distance
            // So either:
            //  1. iterate until 0.1*dc
            //  2. 4-5 iterations
            size_t k = 0;
            bool hasCollided = false;
            for (; k < 5; ++k)
            {
                double delta_tk = this->GetDeltaTk(phi_result, tk, end_state, state_k);
                // has actually never reached > 1
                tk = tk + delta_tk;
                // q_k = q_near + tk * (q_e - q_near);
                VectorXd q_k = state_near.config + tk * (end_state.config - state_near.config);
                state_k = this->NewState(q_k, DistanceEstimateType::Projection);
                max_travelled_dist = this->env->robot->MaxDistance(state_near, state_k);
                phi_result = d_max - max_travelled_dist;
            }
            if (this->IsColliding(state_k))
            {
                hasCollided = true;
                break;
            }
            if (!hasCollided)
            {
                state_k.distanceFromParent = max_travelled_dist;
                state_k.hasDistFromParent = true;
                new_states.push_back(state_k);
            }
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

            double tk = 0;
            double max_travelled_dist = 0;

            // always start out from the center
            RS state_k = state_near;
            double phi_result = d_max;

            const RS &end_state = rand_states[i];

            // They said 4-5 iterations to reach 0.1*closest_distance
            // So either:
            //  1. iterate until 0.1*dc
            //  2. 4-5 iterations
            // for (unsigned int k = 0; k < 5; ++k)
            for (unsigned int k = 0; k < 5 && phi_result > d_small; ++k)
            {
                double delta_tk = this->GetDeltaTk(phi_result, tk, end_state, state_k);

                tk = tk + delta_tk;
                // has actually never reached > 1
                // q_k = q_near + tk * (q_e - q_near);
                VectorXd q_k = state_near.config + tk * (end_state.config - state_near.config);
                state_k = this->NewState(q_k, DistanceEstimateType::Projection);
                max_travelled_dist = this->env->robot->MaxDistance(state_near, state_k);
                phi_result = d_max - max_travelled_dist;
            }
            state_k.distanceFromParent = max_travelled_dist;
            state_k.hasDistFromParent = true;
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

    // BELOW NOT IN USE /////////////////////////////////////////////////////////////////////////////////////
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
