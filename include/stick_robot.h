#include <Eigen/Dense>
#include <stdexcept>

#ifndef STICK_ROBOT_H
#define STICK_ROBOT_H

class StickRobot
{
public:
    const int NUM_SEGMENTS = 2;

    /// @brief Calculate rotations and translations of each robot component in euclidean space
    /// @param configuration configuration in C space
    /// @return tuple of vectors of rotations and translations
    std::tuple<std::vector<Eigen::Matrix3d>, std::vector<Eigen::Vector3d>>
    StickRt(const Eigen::VectorXd &configuration)
    {
        return {{RotationForward(configuration)}, {TranslationForward(configuration)}};
    }

    /// @brief get forward kinematics of i-th joint or end effector if argument == q_dimensionality
    Eigen::Vector3d
    ForwardJoint(const int &ith_distal_point, const Eigen::VectorXd &configuration)
    {
        if (configuration.rows() == 0)
        {
            throw std::runtime_error("StickRobot::ForwardJoint: Empty configuration!");
        }
        Eigen::Vector3d v;
        // first joint is inplae, second joint goes to the end
        v << 0, (ith_distal_point), 0;

        Eigen::Matrix3d Rx;
        Eigen::Matrix3d Rz;
        Rx = Eigen::AngleAxisd(configuration[0], Eigen::Vector3d::UnitX());
        Rz = Eigen::AngleAxisd(configuration[1], Eigen::Vector3d::UnitZ());
        return Rz * Rx * v;
    }

    /// @brief Calculates the radius in the axis of the joint that encompasses the end effector
    double
    RadiusFunc(const int &ith_distal_point, const Eigen::VectorXd &q_k)
    {
        // radius of cylinder in joint axis that encompasses end effector
        // z is up; x, y is is in the plane; this means that r is always the length of the stick-robot
        return 1.0;
    }

private:
    /// @brief Rotate the stick robot about the origin, length is 2, therefore 1 is the position where the stick position = stick center
    Eigen::Vector3d
    TranslationForward(const Eigen::VectorXd &configuration)
    {
        Eigen::Vector3d v;
        v << 0, 0, 0;

        Eigen::Matrix3d Rx;
        Eigen::Matrix3d Rz;
        Rx = Eigen::AngleAxisd(configuration[0], Eigen::Vector3d::UnitX());
        Rz = Eigen::AngleAxisd(configuration[1], Eigen::Vector3d::UnitZ());
        return Rz * Rx * v;
    }

    /// @brief Rotation matrix of the stick robot
    Eigen::Matrix3d
    RotationForward(const Eigen::VectorXd &configuration)
    {
        Eigen::Matrix3d Rx;
        Eigen::Matrix3d Rz;

        Rx = Eigen::AngleAxisd(configuration[0], Eigen::Vector3d::UnitX());
        Rz = Eigen::AngleAxisd(configuration[1], Eigen::Vector3d::UnitZ());
        return Rz * Rx;
    }
};

#endif
