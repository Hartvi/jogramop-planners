#include "bur_related/urdf_planner.h"
#include <string>
#include <sstream>
#include "printing.h"

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
        // std::cout << "Starting URDFPlanner..\n File: " << urdf_file << "\nq_dim: " << q_dim << std::endl;

        // ForwardKinematics fk = this->mCollisionEnv->myURDFRobot->GetForwardPointFunc();
        ForwardKinematicsParallel fkp = this->mCollisionEnv->myURDFRobot->GetForwardPointParallelFunc();
        RadiusFuncParallel rf = this->mCollisionEnv->myURDFRobot->GetRadiusFunc();
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
            q_dim, fkp, max_iters, d_crit, delta_q, epsilon_q, minMaxBounds, rf, num_spikes);

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

    int
    URDFPlanner::AddObstacle(std::string obstacle_file, Eigen::Matrix3d R, Eigen::Vector3d t)
    {
        // std::cout << "URDFPlanner: adding obstacle " << obstacle_file << std::endl;
        return this->mCollisionEnv->AddObstacle(obstacle_file, R, t);
    }

    void
    URDFPlanner::SetObstacleRotation(int id, Eigen::Matrix3d R, Eigen::Vector3d t)
    {
        std::shared_ptr<RtModels::RtModel> model = this->mCollisionEnv->obstacle_models[id];
        model->SetRotation(R);
        model->SetTranslation(t);
    }

    std::string
    URDFPlanner::ToString(const Eigen::VectorXd &q_in, bool obstacles)
    {
        std::ostringstream output;
        auto env = this->mCollisionEnv;
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
