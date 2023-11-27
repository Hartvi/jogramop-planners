#include "bur_related/urdf_planner.h"
#include <string>

namespace Burs
{
    URDFPlanner::URDFPlanner(std::string urdf_file, int max_iters, double d_crit, double delta_q, double epsilon_q, int num_spikes)
    // : urdf_file(urdf_file), max_iters(max_iters), d_crit(d_crit), delta_q(delta_q), epsilon_q(epsilon_q), num_spikes(num_spikes)
    {
        // this->urdf_file
        // BasePlanner(int q_dim, ForwardKinematics f, int max_iters, double d_crit, double delta_q, double epsilon_q, MatrixXd bounds, RadiusFunc radius_func, int num_spikes);
        // BasePlanner(int q_dim, ForwardKinematics f, MatrixXd bounds, RadiusFunc radius_func);
        this->mCollisionEnv = std::make_shared<CollisionEnv>(urdf_file);

        int q_dim = this->GetNrOfJoints();
        std::cout << "Starting URDFPlanner..\n File: " << urdf_file << "\nq_dim: " << q_dim << std::endl;

        ForwardKinematics fk = this->mCollisionEnv->myURDFRobot->GetForwardPointFunc();
        RadiusFunc rf = this->mCollisionEnv->myURDFRobot->GetRadiusFunc();
        ForwardRt frt = this->mCollisionEnv->myURDFRobot->GetSelectedForwardRtFunc();

        std::vector<std::vector<double>> min_max_bounds = this->mCollisionEnv->myURDFRobot->GetMinMaxBounds();

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
        // BasePlanner(int q_dim, ForwardKinematics f, int max_iters, double d_crit, double delta_q, double epsilon_q, MatrixXd bounds, RadiusFunc radius_func, int num_spikes);
        this->mBasePlanner = std::make_shared<BasePlanner>(
            q_dim, fk, max_iters, d_crit, delta_q, epsilon_q, minMaxBounds, rf, num_spikes);

        this->mBasePlanner->SetBurEnv(this->mCollisionEnv);
    }

    unsigned int
    URDFPlanner::GetNrOfJoints()
    {
        return this->mCollisionEnv->myURDFRobot->kdl_chain.getNrOfJoints();
    }

    std::optional<std::vector<Eigen::VectorXd>>
    URDFPlanner::PlanPath(Eigen::VectorXd start, Eigen::VectorXd goal)
    {
        std::optional<std::vector<Eigen::VectorXd>> path_opt = this->mBasePlanner->RbtConnect(start, goal);

        return path_opt;
    }

}