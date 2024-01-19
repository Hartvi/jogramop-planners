#include <memory>
#include <stdexcept>
#include "burs.h"
#include "rt_model.h"
#include "PQP.h"

namespace Burs
{

    using namespace Eigen;

    void
    BaseEnv::SetPoses(VectorXd q)
    {
        auto [rotations, translations] = this->forwardRt(q);

        for (int i = 0; i < rotations.size(); ++i)
        {
            this->robot_models[i]->SetRotation(rotations[i]);
            this->robot_models[i]->SetTranslation(translations[i]);
            // std::cout << "Setting robot position to " << translations[i].transpose() << std::endl;
        }
        this->poses_are_set = true;
    }

    // void BaseEnv::SetObstaclePose(Matrix3d R, Vector3d t)
    // {
    //     for (int i = 0; i < rotations.size(); ++i)
    //     {
    //         this->robot_models[i]->SetRotation(rotations[i]);
    //         this->robot_models[i]->SetTranslation(translations[i]);
    //         // std::cout << "Setting robot position to " << translations[i].transpose() << std::endl;
    //     }
    //     this->obstacle_poses_are_set = true;
    // }

    bool
    BaseEnv::IsColliding() const
    {
        if (!this->poses_are_set)
        {
            std::runtime_error("Poses are not set. Set them before getting closest distance.\n");
        }

        double min_dist = 1e14;

        PQP_CollideResult res;

        for (int i = 0; i < this->robot_models.size(); i++)
        {
            std::shared_ptr<RtModels::RtModel> current_robot_part = this->robot_models[i];
            for (int k = 0; k < this->obstacle_models.size(); k++)
            {
                auto obs = this->obstacle_models[k];
                current_robot_part->Collide(&res, obs.get());

                if (res.Colliding())
                {
                    return true;
                }
            }
        }
        return false;
    }

    double
    BaseEnv::GetClosestDistance() const
    {
        if (!this->poses_are_set)
        {
            std::runtime_error("Poses are not set. Set them before getting closest distance.\n");
        }

        double min_dist = 1e14;

        PQP_DistanceResult res;

        for (int i = 0; i < this->robot_models.size(); i++)
        {
            std::shared_ptr<RtModels::RtModel> current_robot_part = this->robot_models[i];
            for (int k = 0; k < this->obstacle_models.size(); k++)
            {
                auto obs = this->obstacle_models[k];
                current_robot_part->CheckDistance(&res, 1e-3, 1e-3, obs.get());

                if (res.distance < min_dist)
                {
                    min_dist = res.distance;
                }
            }
        }
        return min_dist;
    }

    void
    BaseEnv::AddRobotModel(std::shared_ptr<RtModels::RtModel> m)
    {
        this->robot_models.push_back(m);
    }

    void
    BaseEnv::SetForwardRt(Burs::ForwardRt forwardRt)
    {
        this->forwardRt = forwardRt;
    }

    /*If you want to add other robots, make an environment for them and add the other robot as an obstacle to this one.*/
    int
    BaseEnv::AddObstacle(std::string obstacle_file, Eigen::Matrix3d R, Eigen::Vector3d t)
    {
        std::shared_ptr<RtModels::RtModel> obstacle_model = std::make_shared<RtModels::RtModel>(obstacle_file);

        obstacle_model->SetRotation(R);
        obstacle_model->SetTranslation(t);

        this->obstacle_models.push_back(obstacle_model);
        this->obstacle_map.push_back(obstacle_file);
        return this->obstacle_models.size() - 1;
    }

    void
    BaseEnv::SetObstacleRotation(int id, Eigen::Matrix3d R, Eigen::Vector3d t)
    {
        std::shared_ptr<RtModels::RtModel> model = this->obstacle_models[id];
        model->SetRotation(R);
        model->SetTranslation(t);
    }
}
