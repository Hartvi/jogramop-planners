
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

}