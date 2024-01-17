#include <memory>
#include <Eigen/Dense>
#include <string>
#include "robot_related/robot_collision.h"
#include "env_related/base_env.h"

#ifndef COLLISION_ENV_H
#define COLLISION_ENV_H

namespace Burs
{
    /*The Bur environment holds the robot mesh and the obstacle meshes. I can check collisions and distances from it.*/
    class CollisionEnv : public BaseEnv
    {
    public:
        std::shared_ptr<RobotCollision> myURDFRobot;

        CollisionEnv(std::string urdf_filename);

        /*Check how the jogramop environment keeps track of the object names. Maybe pass an int as obstacle ID when creating it.*/
        // void SetObstaclePose(){}
        std::string
        ToString();

        std::string
        ToScenarioString(Eigen::VectorXd start_q, Eigen::VectorXd goal_q);
    };
}

#endif
