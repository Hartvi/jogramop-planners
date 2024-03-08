#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

#include <vector>
#include <Eigen/Dense>

// #include "bur_funcs.h"

namespace Burs
{
    using namespace Eigen;

    // This encapsulates a forward kinematics + full Jacobian pass for a single configuration
    class RS
    {
    public:
        VectorXd config;
        // Frame of every segment
        std::vector<KDL::Frame> frames;
        bool has_radii = false;
        // Every step in RRT requires the jacobian & frames of each segment
        KDL::Jacobian jac;
        // std::vector<KDL::Jacobian> jacs;
        VectorXd radii;

        int closest_distance_idx = -1;
        std::vector<double> closest_dists;

    public:
        // Existing constructors
        // RS(const VectorXd &config, const std::vector<KDL::Frame> &frames, const std::vector<KDL::Jacobian> &jacs);
        RS() = default;
        RS(const VectorXd &config, const std::vector<KDL::Frame> &frames, const KDL::Jacobian &jac, const VectorXd &radii);
        RS(const VectorXd &config, const std::vector<KDL::Frame> &frames);
        // RS(const VectorXd &config, const std::vector<KDL::Frame> &frames, const KDL::Jacobian &jac);

        // Copy constructor
        // RS(const RS &other)
        //     : config(other.config), frames(other.frames), jac(other.jac), radii(other.radii)
        // {
        //     std::cout << "copy config: " << this->config << "\n";
        //     assert(this->config.size() > 1);
        //     assert(config.size() > 1);
        // }

        // Optionally, consider defining a copy assignment operator if needed
        RS &operator=(const RS &other)
        {
            if (this != &other) // protect against invalid self-assignment
            {
                // Copy each field from the other instance
                config = other.config;
                frames = other.frames;
                jac = other.jac;
                radii = other.radii;
            }
            return *this;
        }

        // VectorXd
        // GetRadii(const std::vector<KDL::Jacobian> &jacs);
    };
}

#endif
