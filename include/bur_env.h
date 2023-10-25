#include <memory>
#include <vector>
#include <Eigen/Dense>
#include "model.h"
#include "bur_funcs.h"

#ifndef BUR_ENV_H
#define BUR_ENV_H

namespace Burs
{
    using namespace Eigen;
    class BurEnv
    {
    public:
        /* What I need from outside:
            ForwardKinematics(i, configuration) - point-wise kinematics of the end-points
            Robot .obj files - segments of the robot that adhere to the below ForwardRt function
            ForwardRt(configuration) - the forward kinematics function that return the rotation matrix and translation of each robot segment
            r_i(configuration_1, configuration_2, t = 0..1) - the function that sets the cylinder radius around joint 'i' which is used for iteration between configurations
        */

        void SetPoses(VectorXd q);
        void AddRobotModel(std::shared_ptr<TrPQPModel> m);
        void AddForwardRt(Burs::ForwardRt forwardRt);
        void AddObstacleModel(std::shared_ptr<TrPQPModel> m);
        void Freeze();
        /// @brief Check closest distance between robot parts and obstacles, NEED TO SET ROTATIONS AND TRANSLATIONS BEFOREHAND
        double GetClosestDistance() const;
        bool IsColliding() const;

    private:
        bool poses_are_set = false;
        std::vector<std::shared_ptr<TrPQPModel>> robot_models;
        Burs::ForwardRt forwardRt;
        std::vector<std::shared_ptr<TrPQPModel>> obstacle_models;
        bool no_more = false;
    };
}

#endif // BUR_ENV_H