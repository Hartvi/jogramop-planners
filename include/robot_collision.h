
#ifndef ROBOT_COLLISION_H
#define ROBOT_COLLISION_H

#include <filesystem>

#include "bur_funcs.h"
// #include "base_env.h"
#include "robot_base.h"
#include "rt_model.h"

#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

namespace Burs
{
    using namespace Eigen;

    class RobotCollision : public RobotBase
    {
    public:
        // Constructor
        RobotCollision(std::string urdf_filename);

        std::vector<bool>
        GetValidTransforms();

        std::vector<std::shared_ptr<RtModels::RtModel>>
        GetModels();

        double
        EEDistance(const RS &state1, const RS &state2) const;

        KDL::Frame
        GetEEFrame(const RS &state) const;

        std::pair<int, std::vector<double>>
        MaxDistances(const RS &state1, const RS &state2) const;

        double
        MaxDistance(const RS &state1, const RS &state2) const;

        std::pair<Matrix3d, Vector3d>
        KDLFrameToEigen(const KDL::Frame &f);

        double
        MaxDistanceMeshes(const RS &state1, const RS &state2) const;

        // std::optional<VectorXd>
        // GetInverseKinematics(KDL::ChainIkSolverPos_LMA &solver, const KDL::JntArray &q_init, const KDL::Frame &tgt);

        std::optional<VectorXd>
        GetInverseKinematics(KDL::ChainIkSolverPos &solver, const KDL::JntArray &q_init, const KDL::Frame &tgt);
    };

    /* Accept the three functions from outside. Link the URDF to the bur-planning algorithm. */
}

#endif
