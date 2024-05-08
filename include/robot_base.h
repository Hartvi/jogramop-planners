
#ifndef ROBOT_BASE_H
#define ROBOT_BASE_H

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolver.hpp>
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
#include "rt_model.h"

namespace Burs
{
    using namespace Eigen;

    class RobotBase
    {
    public:
        /* let's say it has 8 joints and 9 segment.
         two joints could be in one and the same location without a mesh inbetween
          that is when the optional is false
         e.g.:
         [====] JOINT JOINT [====] JOINT [====]
         */
        std::vector<std::optional<std::shared_ptr<RtModels::RtModel>>> segmentIdToModel;

        Eigen::VectorXi radialJoints;
        Eigen::VectorXd approxRadii;
        int numberOfModels;

        std::string urdf_filename;
        std::vector<std::string> mObjs;

        std::vector<VectorXd> segmentToJntCausality;

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

        MatrixXd
        JPlus(const RS &state);

        std::tuple<KDL::Jacobian, VectorXd>
        ForwardJacs(const VectorXd &q_in);

        std::tuple<KDL::Jacobian, VectorXd, VectorXd>
        ForwardJacsComplete(const VectorXd &q_in);

        VectorXd
        GetDistanceEstimates(const RS &state);

        VectorXd
        GetDistanceEstimatesWithRadii(const RS &state);

        std::vector<VectorXd>
        MovableJoints() const;

        RS
        BasicFK(const VectorXd &q_in);

        static Eigen::VectorXd
        parseCSVToVectorXd(const std::string &path);

        static std::vector<Eigen::VectorXd>
        parseCSVToVectors(const std::string &path);
        // END BASE

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

        double
        MaxDistanceMeshes(const RS &state1, const RS &state2) const;

        double
        MaxDistanceMeshSegment(const KDL::Frame &f1, const KDL::Frame &f2, const std::shared_ptr<RtModels::RtModel> rt_model) const;

        std::pair<Matrix3d, Vector3d>
        KDLFrameToEigen(const KDL::Frame &f);

        std::optional<VectorXd>
        GetInverseKinematics(KDL::ChainIkSolverPos &solver, const KDL::JntArray &q_init, const KDL::Frame &tgt);
    };
}
#endif
