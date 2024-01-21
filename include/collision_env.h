
#ifndef COLLISION_ENV_H
#define COLLISION_ENV_H
#include <memory>
#include <Eigen/Dense>
#include <string>
#include "robot_collision.h"
#include "base_env.h"

namespace Burs
{
    /*The Bur environment holds the robot mesh and the obstacle meshes. I can check collisions and distances from it.*/
    class URDFEnv : public BaseEnv
    {
    public:
        std::shared_ptr<RobotCollision> myURDFRobot;

        URDFEnv(std::string urdf_filename);

        /*TODO: Check how the jogramop environment keeps track of the object names. Maybe pass an int as obstacle ID when creating it.*/
        // void SetObstaclePose(){}
        std::string
        ToString();

        std::string
        ToScenarioString(Eigen::VectorXd start_q, Eigen::VectorXd goal_q);
    };
}

#endif
