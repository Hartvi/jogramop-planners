#include "burs.h"

#include <memory>
#include <vector>
#include <Eigen/Dense>
#include "rt_model.h"

#ifndef BASE_ENV_H
#define BASE_ENV_H

namespace Burs
{
    using namespace Eigen;

    class BaseEnv
    {
    public:
        virtual ~BaseEnv() = default;

        /* What I need from outside:
            ForwardKinematics(i, configuration) - point-wise kinematics of the end-points
            Robot .obj files - segments of the robot that adhere to the below ForwardRt function
            ForwardRt(configuration) - the forward kinematics function that return the rotation matrix and translation of each robot segment
            r_i(configuration_1, configuration_2, t = 0..1) - the function that sets the cylinder radius around joint 'i' which is used for iteration between configurations
        */

        void
        SetPoses(VectorXd q);

        void
        AddRobotModel(std::shared_ptr<RtModels::RtModel> m);

        void
        SetForwardRt(Burs::ForwardRt forwardRt);

        /// @brief Check closest distance between robot parts and obstacles, NEED TO SET ROTATIONS AND TRANSLATIONS BEFOREHAND
        double
        GetClosestDistance() const;

        bool
        IsColliding() const;
        /*If you want to add other robots, make an environment for them and add the other robot as an obstacle to this one.*/

        int
        AddObstacle(std::string obstacle_file, Eigen::Matrix3d R = Eigen::Matrix3d::Identity(), Eigen::Vector3d t = Eigen::Vector3d::Zero());

        void
        SetObstacleRotation(int id, Eigen::Matrix3d R, Eigen::Vector3d t);

        std::vector<std::shared_ptr<RtModels::RtModel>> obstacle_models;
        std::vector<std::shared_ptr<RtModels::RtModel>> robot_models;
        std::vector<std::string> obstacle_map;

        bool poses_are_set = false;
        Burs::ForwardRt forwardRt;
    };
}

#endif // BUR_ENV_H