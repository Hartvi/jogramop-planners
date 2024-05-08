#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

#include <vector>
#include <Eigen/Dense>

namespace Burs
{
    using namespace Eigen;

    enum DistanceEstimateType
    {
        JacPosRot,
        JacPos,
        Projection,
        ProjectionRot,
        None
    };

    class RS
    {
    public:
        VectorXd config;
        // Frame of every segment
        std::vector<KDL::Frame> frames;

        bool hasJacobian = false;
        KDL::Jacobian jac;

        bool hasJacRadii = false;
        VectorXd radii;

        bool hasJacRigidRadii = false;
        VectorXd rigidRadii;

        bool hasClosestDists = false;
        std::vector<size_t> closest_distance_ids;
        std::vector<double> closest_dists;

        bool hasDistFromParent = false;
        double distanceFromParent;

        bool hasDistanceEstimate = false;
        DistanceEstimateType distanceEstimateType = DistanceEstimateType::None;

    public:
        RS() = default;
        RS(const VectorXd &config, const std::vector<KDL::Frame> &frames, const KDL::Jacobian &jac, const VectorXd &radii, const VectorXd &rigidRadii);
        RS(const VectorXd &config, const std::vector<KDL::Frame> &frames, const KDL::Jacobian &jac, const VectorXd &radii);
        RS(const VectorXd &config, const std::vector<KDL::Frame> &frames);

        RS &operator=(const RS &other)
        {
            if (this != &other) // protect against invalid self-assignment
            {
                // Copy each field from the other instance
                config = other.config;
                frames = other.frames;
                jac = other.jac;
                radii = other.radii;
                hasJacobian = other.hasJacobian;
                hasJacRadii = other.hasJacRadii;
                hasJacRigidRadii = other.hasJacRigidRadii;
                rigidRadii = other.rigidRadii;
                closest_distance_ids = other.closest_distance_ids;
                closest_dists = other.closest_dists;
                hasClosestDists = other.hasClosestDists;
                hasDistanceEstimate = other.hasDistanceEstimate;
                distanceEstimateType = other.distanceEstimateType;
            }
            return *this;
        }
    };
}

#endif
