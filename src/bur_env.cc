#include <memory>
#include <stdexcept>
#include "bur_env.h"
#include "model.h"
#include "PQP.h"

namespace Burs
{

    using namespace Eigen;

    void BurEnv::SetPoses(VectorXd q)
    {
        auto [rotations, translations] = this->forwardRt(q);

        for (int i = 0; i < rotations.size(); ++i)
        {
            this->robot_models[i]->SetRotation(rotations[i]);
            this->robot_models[i]->SetTranslation(translations[i]);
        }
        this->poses_are_set = true;
    }

    bool BurEnv::IsColliding() const
    {
        if (!this->poses_are_set)
        {
            std::runtime_error("Poses are not set. Set them before getting closest distance.\n");
        }

        double min_dist = 1e14;

        PQP_CollideResult res;

        for (int i = 0; i < this->robot_models.size(); i++)
        {
            std::shared_ptr<TrPQPModel> current_robot_part = this->robot_models[i];
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

    double BurEnv::GetClosestDistance() const
    {
        if (!this->poses_are_set)
        {
            std::runtime_error("Poses are not set. Set them before getting closest distance.\n");
        }

        double min_dist = 1e14;

        PQP_DistanceResult res;

        for (int i = 0; i < this->robot_models.size(); i++)
        {
            std::shared_ptr<TrPQPModel> current_robot_part = this->robot_models[i];
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

    void BurEnv::AddRobotModel(std::shared_ptr<TrPQPModel> m)
    {
        if (this->no_more)
        {
            throw std::runtime_error("Env frozen, no more editing allowed. Initialize a new env.");
        }
        this->robot_models.push_back(m);
    }

    void BurEnv::AddForwardRt(Burs::ForwardRt forwardRt)
    {
        if (this->no_more)
        {
            throw std::runtime_error("Env frozen, no more editing allowed. Initialize a new env.");
        }
        this->forwardRt = forwardRt;
    }

    void BurEnv::AddObstacleModel(std::shared_ptr<TrPQPModel> m)
    {
        if (this->no_more)
        {
            throw std::runtime_error("Env frozen, no more editing allowed. Initialize a new env.");
        }
        this->obstacle_models.push_back(m);
    }

    void BurEnv::Freeze()
    {
        this->no_more = true;
    }

}
