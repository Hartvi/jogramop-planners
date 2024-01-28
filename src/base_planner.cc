#include "collision_env.h"
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
        std::shared_ptr<URDFEnv> my_urdf_env = std::make_shared<URDFEnv>(path_to_urdf_file);

        this->SetEnv(my_urdf_env);
        // this->epsilon_q = some random number
        // this->max_iters = some random number

        std::vector<std::vector<double>>
            min_max_bounds = my_urdf_env->myURDFRobot->GetMinMaxBounds();

        this->q_dim = my_urdf_env->myURDFRobot->minMaxBounds.size();

        Eigen::MatrixXd minMaxBounds(q_dim, 2);

        for (int i = 0; i < q_dim; ++i)
        {
            for (int k = 0; k < 2; ++k)
            {
                minMaxBounds(i, k) = min_max_bounds[i][k];
            }
        }
        this->bounds = minMaxBounds;

        this->myEnv = this->GetEnv<URDFEnv>();
        this->myRobot = this->myEnv->myURDFRobot;
    }

    BasePlanner::BasePlanner()
    {
    }

    double
    BasePlanner::MaxMovedDistance(const VectorXd &q1, const VectorXd &q2) const
    {

        auto fk1 = myRobot->GetForwardPointParallel(q1);
        auto fk2 = myRobot->GetForwardPointParallel(q2);

        double max_dist = 0;
        for (unsigned int i = 0; i < fk1.size(); ++i)
        {
            // dist is in meters
            double dist = (fk2[i] - fk1[i]).norm();
            if (dist > max_dist)
            {
                max_dist = dist;
            }
        }
    }

    void
    BasePlanner::SetEndpoints(MatrixXd &Qe, const VectorXd &q_near, const double &factor) const
    {
        // MatrixXd normalized_Qe = Qe; // Create a copy of Qe to store the normalized results
        for (int j = 0; j < Qe.cols(); ++j)
        {
            Qe.col(j) = GetEndpoint(Qe.col(j), q_near, factor);
        }
    }

    VectorXd
    BasePlanner::GetEndpoint(const VectorXd &q_ei, const VectorXd &q_near, double factor) const
    {
        // 'factor' is in meters:
        // 1. do forward kinematics on current node q_near => fk1
        // 2. do forward kinematics on target node q_ei => fk2
        // 3. if (fk1 - fk2).norm() < factor: return q_ei
        //    else get the ratio so the new config is < factor distance away
        // auto robot = this->GetEnv<URDFEnv>()->myURDFRobot;

        double max_dist = this->MaxMovedDistance(q_near, q_ei);
        if (max_dist < factor)
        {
            return q_ei;
        }
        // dist > factor => scale down => factor/dist < 1
        // => (factor/dist) [m/m] unitless
        // q_new [config] = q_near [config] + [m / m * config = config]
        VectorXd q_new = q_near + factor / max_dist * (q_ei - q_near);
        return q_new;
    }

    void
    BasePlanner::ExampleFunctions(const VectorXd &q_start, const VectorXd &q_goal)
    {
        // ADD OBSTACLE:
        auto my_env = this->GetEnv<URDFEnv>();

        my_env->AddObstacle("path/to/obstacle.obj", Matrix3d::Identity(), Vector3d::Ones());

        // BurTree(VectorXd q_location, int q_dim);
        std::shared_ptr<BurTree> t_a = std::make_shared<BurTree>(q_start, this->q_dim);
        VectorXd Qe = this->GetRandomQ(1);

        // q_near <- NEAREST(q_{e1}, T_a)
        int nearest_index = this->NearestIndex(t_a, Qe);

        const VectorXd q_near = t_a->GetQ(nearest_index);
        const double some_delta_q = 0.1;
        VectorXd q_new = this->GetEndpoint(Qe, q_near, some_delta_q);

        if (!this->IsColliding(q_new))
        {
            t_a->AddNode(nearest_index, q_new);
        }

        // CLOSEST DISTANCE
        double d_closest = this->GetClosestDistance(q_near);
        std::cout << "d < d_crit" << std::endl;
    }

}
