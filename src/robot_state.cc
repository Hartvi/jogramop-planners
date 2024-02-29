#include "robot_state.h"

namespace Burs
{
    using namespace Eigen;

    // RS::RS(const VectorXd &config, const std::vector<KDL::Frame> &frames, const std::vector<KDL::Jacobian> &jacs)
    //     : config(config), frames(frames), jac(jacs[jacs.size() - 1])
    // {
    // }

    RS::RS(const VectorXd &config, const std::vector<KDL::Frame> &frames, const KDL::Jacobian &jac, const VectorXd &radii)
        : config(config), frames(frames), jac(jac), radii(radii)
    {
        // std::cout << "config: " << this->config << "\n";
        // assert(this->config.size() > 1);
        assert(config.size() > 1);
        this->has_radii = true;
    }

    RS::RS(const VectorXd &config, const std::vector<KDL::Frame> &frames)
        : config(config), frames(frames)
    {
        this->has_radii = false;
    }

    // RS::RS(const VectorXd &config, const std::vector<KDL::Frame> &frames, const KDL::Jacobian &jac)
    //     : config(config), frames(frames), jac(jac)
    // {
    //     // std::cout << "config: " << this->config << "\n";
    //     // assert(this->config.size() > 1);
    //     assert(config.size() > 1);
    // }

    // VectorXd
    // RS::GetRadii(const std::vector<KDL::Jacobian> &jacs)
    // {
    //     VectorXd r(jacs[0].columns());
    //     // Jacobian: 6xN
    //     for (unsigned int i = 0; i < jacs.size(); ++i)
    //     {
    //         KDL::Jacobian jac_i = jacs[i];
    //         // std::cout << "Jac " << i << ": \n"
    //         //           << jac_i.data << "\n";
    //         for (unsigned int k = i + 1; k < jacs.size(); ++k)
    //         {
    //             double jac_norm = jac_i.getColumn(k).vel.Norm();
    //             if (jac_norm > r(i))
    //             {
    //                 r(i) = jac_norm;
    //             }
    //         }
    //     }
    //     return r;
    // }

}