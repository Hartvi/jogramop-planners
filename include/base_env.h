
#ifndef BASE_ENV_H
#define BASE_ENV_H

#include <optional>
#include <memory>
#include <vector>
#include <Eigen/Dense>
#include "rt_model.h"
#include "bur_funcs.h"
#include "robot_state.h"
#include "robot_collision.h"

namespace Burs
{
    using namespace Eigen;

    class BaseEnv
    {
    public:
        std::shared_ptr<RobotCollision> robot;

        double groundLevel = -1e10;
        int minimumColSegmentIdx = 999;

        std::vector<std::shared_ptr<RtModels::RtModel>> obstacle_models;
        std::vector<std::shared_ptr<RtModels::RtModel>> robot_models;
        std::vector<std::string> obstacle_map;

        bool poses_are_set = false;
        // Full robot state might include transforms of virtual segments
        std::vector<bool> validTransforms;
        // Burs::ForwardRt forwardRt;

    public:
        BaseEnv(std::string urdf_filename);

        virtual ~BaseEnv() = default;

        /* What I need from outside:
            ForwardKinematics(i, configuration) - point-wise kinematics of the end-points
            Robot .obj files - segments of the robot that adhere to the below ForwardRt function
            ForwardRt(configuration) - the forward kinematics function that return the rotation matrix and translation of each robot segment
            r_i(configuration_1, configuration_2, t = 0..1) - the function that sets the cylinder radius around joint 'i' which is used for iteration between configurations
        */

        void
        SetPoses(const RS &state);

        void
        AddRobotModel(std::shared_ptr<RtModels::RtModel> m);

        // void
        // SetForwardRt(Burs::ForwardRt forwardRt);

        std::pair<int, std::vector<double>>
        GetClosestDistances() const;

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

        void
        SetGroundLevel(double groundLevel, int minimumColSegmentIdx);

        /*TODO: Check how the jogramop environment keeps track of the object names. Maybe pass an int as obstacle ID when creating it.*/
        // void SetObstaclePose(){}
        std::string
        ToString();
    };
}

#endif // BUR_ENV_H