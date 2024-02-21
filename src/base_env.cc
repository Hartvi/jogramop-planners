#include <memory>
#include <stdexcept>
#include "rt_model.h"
#include "PQP.h"
#include "base_env.h"
#include "bur_funcs.h"

namespace Burs
{
    using namespace Eigen;

    void
    BaseEnv::SetPoses(const RS &state)
    {
        unsigned int k = 0;
        for (int i = 0; i < state.frames.size(); ++i)
        {
            if (this->validTransforms[i])
            {
                auto [R, t] = this->robot->KDLFrameToEigen(state.frames[i]);

                this->robot_models[k]->SetRotation(R);
                this->robot_models[k]->SetTranslation(t);
                ++k;
            }
        }
        this->poses_are_set = true;
    }

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

    std::pair<int, std::vector<double>>
    BaseEnv::GetClosestDistances() const
    {
        if (!this->poses_are_set)
        {
            throw std::runtime_error("Poses are not set. Set them before getting closest distance.\n"); // Corrected to throw the exception
        }

        PQP_DistanceResult res;

        std::vector<double> segment_distances(this->robot_models.size());
        double min_total_dist = std::numeric_limits<double>::max(); // Use max double value for initial comparison
        int min_idx = -1;

        for (size_t i = 0; i < this->robot_models.size(); ++i) // Use size_t for indexing to match the size type
        {
            std::shared_ptr<RtModels::RtModel> current_robot_part = this->robot_models[i];
            double min_seg_dist = std::numeric_limits<double>::max(); // Initialize to max double value

            // Check distance to the ground if applicable
            if (i >= this->minimumColSegmentIdx)
            {
                double current_part_z = current_robot_part->getT()[2];
                double tmpDist = current_part_z - this->groundLevel;
                if (tmpDist < min_seg_dist)
                {
                    min_seg_dist = tmpDist;
                }
            }

            // Check distance to each obstacle
            for (size_t k = 0; k < this->obstacle_models.size(); ++k) // Use size_t for indexing
            {
                auto obs = this->obstacle_models[k];
                current_robot_part->CheckDistance(&res, 1e-3, 1e-3, obs.get());

                if (res.distance < min_seg_dist)
                {
                    min_seg_dist = res.distance;
                }
            }

            // Update the overall minimum distance if this segment's minimum is lower
            if (min_seg_dist < min_total_dist)
            {
                min_total_dist = min_seg_dist;
                min_idx = i;
            }

            segment_distances[i] = min_seg_dist;
        }

        // Return both the overall minimum distance and the vector of per-segment minimum distances
        return {min_idx, segment_distances};
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

            if (i >= this->minimumColSegmentIdx)
            {
                double current_part_z = current_robot_part->getT()[2];
                double tmpDist = current_part_z - this->groundLevel;
                if (tmpDist < min_dist)
                {
                    min_dist = tmpDist;
                }
            }

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

    void
    BaseEnv::SetGroundLevel(double groundLevel, int minimumColSegmentIdx)
    {
        this->groundLevel = groundLevel;
        this->minimumColSegmentIdx = minimumColSegmentIdx;
    }

    BaseEnv::BaseEnv(std::string urdf_filename)
    {
        this->robot = std::make_shared<RobotCollision>(urdf_filename);

        std::vector<std::shared_ptr<RtModels::RtModel>> mm = this->robot->GetModels();

        for (auto &m : this->robot->GetModels())
        {
            this->AddRobotModel(m);
        }
        this->validTransforms = this->robot->GetValidTransforms();
    }

    std::string
    BaseEnv::ToString()
    {
        std::stringstream ss;
        auto robot_str = this->robot->ToString();
        ss << "robot" << std::endl
           << robot_str << std::endl;
        auto obstacles = this->obstacle_models;
        for (unsigned int i = 0; i < obstacles.size(); ++i)
        {
            ss << "obstacle" << std::endl;
            auto obstacle = obstacles[i];
            obstacle->ToString();
        }

        return ss.str();
    }

}
