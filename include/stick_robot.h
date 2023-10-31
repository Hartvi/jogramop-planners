#include <Eigen/Dense>

#ifndef STICK_ROBOT_H
#define STICK_ROBOT_H

class StickRobot
{
public:
    const int q_dimensionality = 3;

    /// @brief Calculate rotations and translations of each robot component in euclidean space
    /// @param configuration configuration in C space
    /// @return tuple of vectors of rotations and translations
    std::tuple<std::vector<Eigen::Matrix3d>, std::vector<Eigen::Vector3d>> StickRt(const Eigen::Vector3d &configuration)
    {
        return {{RotationForward(configuration)}, {TranslationForward(configuration)}};
    }

    /// @brief get forward kinematics of i-th joint or end effector if argument == q_dimensionality
    /// @param ith_distal_point index of i-th joint or end effector
    /// @param configuration configuration in C-space
    /// @return position of i-th joint or end effector in euclidean space
    Eigen::Vector3d ForwardJoint(const int &ith_distal_point, const Eigen::VectorXd &configuration)
    {
        if (ith_distal_point > 2 || ith_distal_point < 0)
        {
            throw new std::runtime_error("Invalid index for joint/end effector");
        }
        Eigen::Vector3d v;
        v << 0, 0, 2 * (ith_distal_point > 1);

        // Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d Rx;
        Eigen::Matrix3d Ry;
        Eigen::Matrix3d Rz;
        Rx = Eigen::AngleAxisd(configuration[0], Eigen::Vector3d::UnitX());
        Ry = Eigen::AngleAxisd(configuration[1], Eigen::Vector3d::UnitY());
        Rz = Eigen::AngleAxisd(configuration[2], Eigen::Vector3d::UnitZ());
        return Rz * Ry * Rx * v;
    }

    /// @brief Calculates the radius in the axis of the joint that encompasses the end effector
    /// @param ith_distal_point
    /// @param q_k
    /// @return
    double RadiusFunc(const int &ith_distal_point, const Eigen::VectorXd &q_k)
    {
        if (ith_distal_point > 2 || ith_distal_point < 0)
        {
            throw new std::runtime_error("Invalid index for joint/end effector");
        }
        // radius of cylinder in joint axis that encompasses end effector
        return 2.0 * (ith_distal_point != 2);
    }

private:
    Eigen::Vector3d TranslationForward(const Eigen::VectorXd &configuration)
    {
        Eigen::Vector3d v;
        v << 0, 0, 1;
        // Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d Rx;
        Eigen::Matrix3d Ry;
        Eigen::Matrix3d Rz;
        Rx = Eigen::AngleAxisd(configuration[0], Eigen::Vector3d::UnitX());
        Ry = Eigen::AngleAxisd(configuration[1], Eigen::Vector3d::UnitY());
        Rz = Eigen::AngleAxisd(configuration[2], Eigen::Vector3d::UnitZ());
        return Rz * Ry * Rx * v;
    }

    Eigen::Matrix3d RotationForward(const Eigen::VectorXd &configuration)
    {
        // Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d Rx;
        Eigen::Matrix3d Ry;
        Eigen::Matrix3d Rz;
        Rx = Eigen::AngleAxisd(configuration[0], Eigen::Vector3d::UnitX());
        Ry = Eigen::AngleAxisd(configuration[1], Eigen::Vector3d::UnitY());
        Rz = Eigen::AngleAxisd(configuration[2], Eigen::Vector3d::UnitZ());
        return Rz * Ry * Rx;
    }
};

#endif
