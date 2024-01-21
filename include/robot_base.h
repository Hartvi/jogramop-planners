
#ifndef ROBOT_BASE_H
#define ROBOT_BASE_H

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <random>
#include <iostream>
#include <fstream>
#include <memory>
#include <map>
#include <filesystem>

#include <Eigen/Dense>

#include "bur_funcs.h"

namespace Burs
{
    class RobotBase
    {
    public:
        std::filesystem::path urdf_file;

        urdf::ModelInterfaceSharedPtr robot_model;
        KDL::Tree kdl_tree;
        std::vector<std::string> end_links;
        KDL::Chain kdl_chain;

        std::map<std::string, std::string> segmentNameToFile;
        std::map<int, std::string> segmentIdToName;
        std::map<int, std::string> segmentIdToFile;
        std::vector<std::vector<double>> minMaxBounds;

        // std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;

        RobotBase(std::string urdf_filename);

        std::optional<urdf::ModelInterfaceSharedPtr>
        GetRobotURDF(std::string urdf_filename);

        std::map<std::string, std::string>
        GetSegmentNameToFile(urdf::ModelInterfaceSharedPtr robot_model);

        std::optional<KDL::Tree>
        GetKDLTree(urdf::ModelInterfaceSharedPtr robot_model);

        std::optional<KDL::Chain>
        GetKDLChain(urdf::ModelInterfaceSharedPtr robot_model, KDL::Tree kdl_tree, std::string end_effector_link);

        std::optional<std::vector<std::string>>
        GetEndLinks(const urdf::ModelInterfaceSharedPtr &robot_model);

        std::string
        GetLinkName(const KDL::Chain &kdl_chain, unsigned int i);

        std::map<int, std::string>
        GetSegmentIdToName(const KDL::Chain &kdl_chain);

        std::map<int, std::string>
        GetSegmentIdToFile();

        KDL::ChainFkSolverPos_recursive
        GetFKSolver(KDL::Chain kdl_chain);

        std::tuple<std::vector<Eigen::Matrix3d>, std::vector<Eigen::Vector3d>>
        ForwardQ(const Eigen::VectorXd &q_in);

        Eigen::Vector3d
        GetForwardPoint(const int &ith_distal_point, const Eigen::VectorXd &q_in);

        std::vector<Eigen::Vector3d>
        GetForwardPointParallel(const Eigen::VectorXd &q_in);

        ForwardKinematics
        GetForwardPointFunc();

        ForwardKinematicsParallel
        GetForwardPointParallelFunc();

        // pqp_handler.kdl_chain.getNrOfSegments() gets the end-effector
        double GetRadius(const int &ith_distal_point, const Eigen::VectorXd &q_in);

        RadiusFuncParallel
        GetRadiusFunc();

        Eigen::VectorXd
        GetRadii(const Eigen::VectorXd &q_in);

        std::vector<std::vector<double>>
        GetMinMaxBounds();

        std::string
        ToString();
    };
}
#endif
