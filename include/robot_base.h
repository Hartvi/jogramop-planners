
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
#include "robot_state.h"

namespace Burs
{
    using namespace Eigen;

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

        // TODO: cache like this?????
        // it has to be ideally < 6 elements
        std::map<std::vector<double>, std::vector<KDL::Frame>> fkResults;
        std::map<std::vector<double>, KDL::Jacobian> jacResults;

        // BEGIN BASE
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

        std::vector<std::vector<double>>
        GetMinMaxBounds();

        std::string
        ToString();

        // std::tuple<std::vector<Eigen::Matrix3d>, std::vector<Eigen::Vector3d>>
        // ForwardQ(const Eigen::VectorXd &q_in);

        std::vector<KDL::Frame>
        ForwardPass(const Eigen::VectorXd &q_in);

        KDL::Jacobian
        ForwardJac(const VectorXd &q_in);

        KDL::JntArray
        ForwardJPlus(const VectorXd q_in, const KDL::Twist &v_in);

        KDL::JntArray
        ForwardJPlus(const RS &state, const KDL::Twist &v_in);

        std::pair<KDL::Jacobian, VectorXd>
        ForwardJacs(const VectorXd &q_in);

        VectorXd
        GetRadii(const RS &state);

        // void
        // AddRadii(RS &state);

        RS
        FullFK(const VectorXd &q_in);

        static Eigen::VectorXd
        parseCSVToVectorXd(const std::string &path);

        static std::vector<Eigen::VectorXd>
        parseCSVToVectors(const std::string &path);
        // END BASE
    };
}
#endif
