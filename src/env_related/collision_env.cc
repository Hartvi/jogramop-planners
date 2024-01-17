
#include <sstream>
#include "env_related/collision_env.h"

namespace Burs
{
    CollisionEnv::CollisionEnv(std::string urdf_filename)
    {
        this->myURDFRobot = std::make_shared<RobotCollision>(urdf_filename);

        std::vector<std::shared_ptr<RtModels::RtModel>> mm = this->myURDFRobot->GetModels();

        // std::cout << "CollisionEnv: Number of objects: " << mm.size() << std::endl;

        for (auto &m : this->myURDFRobot->GetModels())
        {
            this->AddRobotModel(m);
        }
        this->AddForwardRt(this->myURDFRobot->GetSelectedForwardRtFunc());
    }

    std::string
    CollisionEnv::ToString()
    {
        std::stringstream ss;
        auto robot_str = this->myURDFRobot->ToString();
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

    std::string
    CollisionEnv::ToScenarioString(Eigen::VectorXd start_q, Eigen::VectorXd goal_q)
    {
        std::stringstream ss;

        ss << "start_q" << std::endl;
        size_t last_q_index = start_q.size() - 1;
        for (size_t i = 0; i < last_q_index; ++i)
        {
            ss << start_q(i) << ",";
        }
        ss << start_q(last_q_index) << std::endl;

        ss << "goal_q" << std::endl;
        for (size_t i = 0; i < last_q_index; ++i)
        {
            ss << goal_q(i) << ",";
        }
        ss << goal_q(last_q_index) << std::endl;

        return ss.str();
    }

}