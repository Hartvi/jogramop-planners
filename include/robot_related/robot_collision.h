#include <filesystem>

#include "bur_related/bur_funcs.h"
#include "env_related/base_env.h"
#include "robot_related/robot_base.h"
#include "model_related/rt_model.h"

#ifndef ROBOT_COLLISION_H
#define ROBOT_COLLISION_H

namespace Burs
{

    using namespace Eigen;

    //
    class RobotCollision : public RobotBase
    {
    public:
        /* let's say it has 8 joints and 9 segment.
         two joints could be in one and the same location without a mesh inbetween
          that is when the optional is false
         e.g.:
         [====] JOINT JOINT [====] JOINT [====]
         */
        std::vector<std::optional<std::shared_ptr<RtModels::RtModel>>> segmentIdToModel;

        int numberOfModels;

        std::string urdf_filename;
        std::vector<std::string> mObjs;

        // Constructor
        RobotCollision(std::string urdf_filename);

        // ForwardQ returns N rotations and translation, but we have M <= N objects, select only transforms relevant to existing meshes
        std::tuple<std::vector<Matrix3d>, std::vector<Vector3d>>
        SelectedForwardQ(const VectorXd &q_in);

        ForwardRt GetSelectedForwardRtFunc();

        std::vector<std::shared_ptr<RtModels::RtModel>>
        GetModels();
    };

    /* Accept the three functions from outside. Link the URDF to the bur-planning algorithm. */
}

#endif
