#include "robot_state.h"

namespace Burs
{
    using namespace Eigen;

    RS::RS(const VectorXd &config, const std::vector<KDL::Frame> &frames, const KDL::Jacobian &jac, const VectorXd &radii)
        : config(config), frames(frames), jac(jac), radii(radii)
    {
        assert(config.size() > 1);
    }

    RS::RS(const VectorXd &config, const std::vector<KDL::Frame> &frames, const KDL::Jacobian &jac)
        : config(config), frames(frames), jac(jac)
    {
        assert(config.size() > 1);
    }

}