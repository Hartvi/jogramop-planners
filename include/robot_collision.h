
#ifndef ROBOT_COLLISION_H
#define ROBOT_COLLISION_H

#include <filesystem>

#include "bur_funcs.h"
// #include "base_env.h"
#include "robot_base.h"
#include "rt_model.h"

namespace Burs
{
    using namespace Eigen;

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

        std::vector<bool>
        GetValidTransforms();

        // ForwardQ returns N rotations and translation, but we have M <= N objects, select only transforms relevant to existing meshes
        // std::tuple<std::vector<Matrix3d>, std::vector<Vector3d>>
        // SelectedForwardQ(const RS &q_in);

        // ForwardRtKDL
        // GetSelectedForwardRtFunc();

        std::vector<std::shared_ptr<RtModels::RtModel>>
        GetModels();

        // std::vector<Eigen::Vector3d>
        // GetForwardPointParallel(const RS &state);

        // ForwardKinematicsParallelKDL
        // GetForwardPointParallelFunc();

        // RadiusFuncParallelKDL
        // GetRadiusFunc();

        // double
        // DistanceToGoal(const KDL::Frame &goal, const KDL::Frame &current) const;

        // double
        // GetDistToGoal(const VectorXd &q, const KDL::Frame &goal_pos) const;
        double
        EEDistance(const RS &state1, const RS &state2) const;

        KDL::Frame
        GetEEFrame(const RS &state) const;

        // double
        // MaxMovedDistance(const VectorXd &q1, const VectorXd &q2) const;
        double
        MaxDistance(const RS &state1, const RS &state2) const;

        std::pair<Matrix3d, Vector3d>
        KDLFrameToEigen(const KDL::Frame &f);
    };

    /* Accept the three functions from outside. Link the URDF to the bur-planning algorithm. */
}

#endif
