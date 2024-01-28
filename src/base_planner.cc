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

        this->forwardKinematicsParallel = this->myRobot->GetForwardPointParallelFunc();
        this->radius_func = this->myRobot->GetRadiusFunc();
    }

    BasePlanner::BasePlanner()
    {
    }

    double
    BasePlanner::GetDistToGoal(const VectorXd &q, const KDL::Vector &goal_pos) const
    {
        auto ee = this->GetEEPose(q).p;
        return (ee - goal_pos).Norm();
    }

    KDL::Frame
    BasePlanner::GetEEPose(const VectorXd &q) const
    {
        auto config_rt = this->myRobot->CachedForwardPass(q);
        auto ee = config_rt[config_rt.size() - 1];
        return ee;
    }

    double
    BasePlanner::MaxMovedDistance(const VectorXd &q1, const VectorXd &q2) const
    {

        // std::cout << "q1: " << q1.transpose() << " q2: " << q2.transpose() << "\n";
        auto fk1 = myRobot->GetForwardPointParallel(q1);
        // std::cout << "fk1: " << fk1.size() << "\n";
        auto fk2 = myRobot->GetForwardPointParallel(q2);
        // std::cout << "fk2: " << fk2.size() << "\n";

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
        return max_dist;
    }

    double
    BasePlanner::GetDeltaTk(double phi_tk, double tk, const VectorXd &q_e, const VectorXd &q_k) const
    {
        VectorXd r_vec = this->radius_func(q_k);

        double denominator = (q_e - q_k).cwiseAbs().dot(r_vec);
        return phi_tk * (1.0 - tk) / denominator;
    }

    MatrixXd
    BasePlanner::GetEndpoints(const VectorXd &q_near, const MatrixXd &Q_e, double d_max) const
    {
        double d_small = 0.1 * d_max;
        MatrixXd endpoints = MatrixXd::Zero(this->q_dim, Q_e.cols());
        // std::cout << "start q: " << q_near.transpose() << "\n";
        for (int i = 0; i < Q_e.cols(); ++i)
        {
            // // If this won't move further that it is allowed
            double maxPossibleDist = this->MaxMovedDistance(q_near, Q_e.col(i));
            if (maxPossibleDist < d_max)
            {
                endpoints.col(i) = Q_e.col(i);
            }

            double tk = 0;

            // always start out from the center
            VectorXd q_k(q_near);
            double phi_result = d_max;

            const VectorXd q_e = Q_e.col(i);

            // They said 4-5 iterations to reach 0.1*closest_distance
            // So either:
            //  1. iterate until 0.1*dc
            //  2. 4-5 iterations
            for (unsigned int k = 0; k < 5; ++k)
            // while (phi_result > d_small)
            {
                // CHECK: this is indeed PI away from q_near

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
                phi_result = d_max - this->MaxMovedDistance(q_near, q_k);
            }
            endpoints.col(i) = q_k;
            // double max_moved_dist = this->MaxMovedDistance(q_near, q_k);
            // std::cout << "q_k: " << q_k.transpose() << "\n";
            // std::cout << "maxmoved dist: " << max_moved_dist << " epsilon: " << d_max << "\n";
            // if (max_moved_dist > d_max)
            // {
            //     std::cout << "MOVED MORE THAN SHOULD HAVE: " << max_moved_dist << " > " << d_max << "\n";
            //     // exit(1);
            // }
        }
        return endpoints;
    }

    // VectorXd
    // BasePlanner::GetEndpoints(const VectorXd &q_ei, const VectorXd &q_near, double factor) const
    // {
    //     double max_dist = this->MaxMovedDistance(q_near, q_ei);

    //     if (max_dist < factor)
    //     {
    //         return q_ei;
    //     }

    //     double stepScale = 0.1; // Start with a small fraction of the step
    //     double maxScale = 1.0;  // Maximum scale is the full step
    //     VectorXd q_new = q_near;
    //     double newDist;

    //     VectorXd q_new = q_near + factor / max_dist * (q_ei - q_near);
    //     auto newdist = this->MaxMovedDistance(q_new, q_near);
    //     while (newdist > factor)
    //     {
    //         newdist
    //     }

    //     while (stepScale <= maxScale)
    //     {
    //         q_new = q_near + stepScale * (q_ei - q_near);
    //         newDist = this->MaxMovedDistance(q_new, q_near);

    //         if (newDist >= factor)
    //         {
    //             // If the distance is larger than factor, reduce the step
    //             maxScale = stepScale - 0.01; // Reduce the upper bound
    //             stepScale = 0.1;             // Reset to a smaller step
    //         }
    //         else
    //         {
    //             // Incrementally increase the step
    //             stepScale += 0.01;
    //         }
    //     }
    //     return q_new;
    // }

    // VectorXd
    // BasePlanner::GetEndpoints(const VectorXd &q_ei, const VectorXd &q_near, double factor) const
    // {
    //     // 'factor' is in meters:
    //     // 1. do forward kinematics on current node q_near => fk1
    //     // 2. do forward kinematics on target node q_ei => fk2
    //     // 3. if (fk1 - fk2).norm() < factor: return q_ei
    //     //    else get the ratio so the new config is < factor distance away
    //     // auto robot = this->GetEnv<URDFEnv>()->myURDFRobot;

    //     double max_dist = this->MaxMovedDistance(q_near, q_ei);

    //     if (max_dist < factor)
    //     {
    //         return q_ei;
    //     }
    //     // /\ THE ABOVE WORKS FINE

    //     std::cout << "max moved dist: " << max_dist << " factor: " << factor << " ratio: " << (factor / max_dist) << "\n";
    //     // dist > factor => scale down => factor/dist < 1
    //     // => (factor/dist) [m/m] unitless
    //     // q_new [config] = q_near [config] + [m / m * config = config]
    //     VectorXd q_new = q_near + factor / max_dist * (q_ei - q_near);

    //     auto newdist = this->MaxMovedDistance(q_new, q_near);
    //     std::cout << "new moved distance: " << newdist << "\n";

    //     // THIS IS ALMOST ALWAY TRUE
    //     if (newdist > factor)
    //     {
    //         exit(1);
    //     }
    //     return q_new;
    // }

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
        VectorXd q_new = this->GetEndpoints(Qe, q_near, some_delta_q);

        if (!this->IsColliding(q_new))
        {
            t_a->AddNode(nearest_index, q_new);
        }

        // CLOSEST DISTANCE
        double d_closest = this->GetClosestDistance(q_near);
        std::cout << "d < d_crit" << std::endl;
    }

}
