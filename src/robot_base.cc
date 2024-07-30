#include <Eigen/QR>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <string>

#include "robot_base.h"
#include "bur_funcs.h"

namespace Burs
{
    using namespace Eigen;

    RobotBase::RobotBase(std::string urdf_filename)
    {
        if (!std::filesystem::exists(urdf_filename))
        {
            std::cerr << "Error: File " << urdf_filename << " does not exist." << std::endl;
            throw std::runtime_error("URDF file doesn't exist " + urdf_filename);
        }
        this->urdf_file = std::filesystem::absolute(urdf_filename);

        auto opt_robot = this->GetRobotURDF(urdf_filename);

        if (!opt_robot)
        {
            throw std::runtime_error("Failed to parse robot URDF from path " + urdf_filename);
        }

        this->robot_model = opt_robot.value();

        // Create a KDL tree from the URDF model
        auto kdl_tree_res = this->GetKDLTree(robot_model);

        if (!kdl_tree_res)
        {
            throw std::runtime_error("Failed to load URDF tree.");
        }

        this->kdl_tree = kdl_tree_res.value();
        // for (auto &l : this->kdl_tree.getSegments())
        // {
        //     std::cout << "segment: " << l.first << "\n";
        // }

        auto end_links_res = this->GetEndLinks(robot_model);

        if (!end_links_res)
        {
            throw std::runtime_error("Failed to get end links of robot (children.size() == 0).");
        }

        this->end_links = end_links_res.value();

        std::cout << "end links:" << std::endl;
        for (int i = 0; i < end_links.size(); ++i)
        {
            std::cout << "end link: " << end_links[i] << std::endl;
        }
        std::cout << "\n";
        // exit(1);

        // auto kdl_chain_res = this->GetKDLChain(robot_model, kdl_tree, "panda_hand");
        auto kdl_chain_res = this->GetKDLChain(robot_model, kdl_tree, end_links[0]);

        if (!kdl_chain_res)
        {
            throw std::runtime_error("Failed to get KDL chain.");
        }

        this->kdl_chain = kdl_chain_res.value();
        int j = 0;
        this->radialJoints = Eigen::VectorXi::Ones(this->kdl_chain.getNrOfJoints());
        for (auto &it : this->kdl_chain.segments)
        {
            auto jointType = it.getJoint().getType();
            if (jointType != KDL::Joint::JointType::None)
            {
                if (jointType == KDL::Joint::JointType::TransAxis || jointType == KDL::Joint::JointType::TransX || jointType == KDL::Joint::JointType::TransY || jointType == KDL::Joint::JointType::TransZ)
                {
                    this->radialJoints(j) = 0;
                }
                ++j;
            }
        }

        this->segmentNameToFile = this->GetSegmentNameToFile(robot_model);
        this->segmentIdToName = this->GetSegmentIdToName(this->kdl_chain);
        this->segmentIdToFile = this->GetSegmentIdToFile();

        this->minMaxBounds = this->GetMinMaxBounds();

        // KDL::ChainFkSolverPos_recursive fk_solver(this->kdl_chain);
        // this->fk_solver = KDL::ChainFkSolverPos_recursive(this->kdl_chain);
        // KDL::ChainFkSolverPos_recursive fk_solver = KDL::ChainFkSolverPos_recursive(this->kdl_chain);
        // this->fk_solver = fk_solver;

        // Get the directory of the URDF file
        std::filesystem::path urdf_path(urdf_filename);
        std::filesystem::path urdf_dir = urdf_path.parent_path();

        this->urdf_filename = urdf_dir / urdf_path;

        int numModels = 0;
        // Initialization specific to RobotBase
        // std::cout << "number of segments " << this->kdl_chain.getNrOfSegments() << std::endl;
        std::cout << "URDF dir: " << urdf_dir << "\n";
        for (int i = 0; i < this->kdl_chain.getNrOfSegments(); ++i)
        {
            const KDL::Segment &segment = kdl_chain.getSegment(i);
            std::cout << "Segment name:     " << this->kdl_chain.getSegment(i).getName() << "\n";
            if (this->segmentIdToFile.find(i) != this->segmentIdToFile.end())
            {
                // Add relative path to urdf file
                std::filesystem::path model_path = urdf_dir / this->segmentIdToFile[i];

                // for later visualization purposess
                this->mObjs.push_back(model_path);

                std::shared_ptr<RtModels::RtModel> trpqpmodel = std::make_shared<RtModels::RtModel>(model_path);
                std::cout << "Robot segment:    " << i << " \n  File:           " << this->segmentIdToFile[i] << " \n  Number of tris: " << trpqpmodel->pqpModel->num_tris << "\n";
                // std::cout << "radii: " << trpqpmodel->encompassingRadii.transpose() << "\n";
                this->segmentIdToModel.push_back(trpqpmodel);
                numModels++;
            }
            else
            {
                this->segmentIdToModel.push_back({});
            }
            std::cout << "\n";
        }
        // KDL::ChainFkSolverPos_recursive fk(this->kdl_chain);
        int nrjnts = this->kdl_chain.getNrOfJoints();
        // int nrsegs = this->kdl_chain.getNrOfSegments();
        // KDL::JntArray jntarray = KDL::JntArray(nrjnts);
        // jntarray.data = VectorXd::Zero(nrjnts);
        // std::vector<KDL::Frame> frames(nrsegs);
        // std::vector<KDL::Frame> frames2(nrsegs);
        // fk.JntToCart(jntarray, frames);
        // VectorXd approxRadii = VectorXd::Zero(nrjnts);
        // int seg_i = 0;
        // double delta_angle = 0.0001;
        // for (int j = 0; j < nrjnts; ++j)
        // {
        //     // move joint 'j'
        //     // find segment with joint
        //     auto joint = this->kdl_chain.getSegment(seg_i).getJoint();
        //     while (joint.getType() == KDL::Joint::JointType::None)
        //     {
        //         seg_i++;
        //         joint = this->kdl_chain.getSegment(seg_i).getJoint();
        //         // std::cout << "seg i: " << seg_i << "\n";
        //     }
        //     while (joint.getType() == KDL::Joint::JointType::TransAxis || joint.getType() == KDL::Joint::JointType::TransX || joint.getType() == KDL::Joint::JointType::TransY || joint.getType() == KDL::Joint::JointType::TransZ)
        //     {
        //         seg_i++;
        //         j++;
        //         joint = this->kdl_chain.getSegment(seg_i).getJoint();
        //     }
        //     jntarray.data = VectorXd::Zero(nrjnts);
        //     jntarray(j) += delta_angle;
        //     // find next segment with mesh
        //     // find translations of all meshes of that one and the ones after that do not contain a joint
        //     // std::cout << "jnt array: " << jntarray.data.transpose() << "\n";
        //     fk.JntToCart(jntarray, frames2);
        //     // while ()
        //     // {
        //     // this->MaxDistanceMeshSegment(frames[seg_i], frames2[seg_i], this->segmentIdToModel[seg_i].value());
        //     // }
        //     double tmp_dist = 1.0 / delta_angle * 0.0;
        //     while (tmp_dist <= 1e-6 || !this->segmentIdToModel[seg_i])
        //     {
        //         seg_i++;
        //         if (this->segmentIdToModel[seg_i])
        //         {
        //             tmp_dist = 1.0 / delta_angle * this->MaxDistanceMeshSegment(frames[seg_i], frames2[seg_i], this->segmentIdToModel[seg_i].value());
        //         }
        //     }
        //     double segment_dist = 0.0;
        //     // std::cout << "\nj: " << j << "\n";
        //     for (int k = 0; k < 10; ++k)
        //     {
        //         jntarray.data = M_PI * VectorXd::Random(nrjnts);
        //         // fk.JntToCart(jntarray, frames);
        //         // jntarray(j) += delta_angle;
        //         // fk.JntToCart(jntarray, frames2);

        //         auto [jac, r, rr] = this->ForwardJacsComplete(jntarray.data);
        //         tmp_dist = rr(j);
        //         // approxRadii
        //         // segment_dist = r(j);

        //         // tmp_dist = 1.0 / delta_angle * this->MaxDistanceMeshSegment(frames[seg_i], frames2[seg_i], this->segmentIdToModel[seg_i].value());
        //         // segment_dist = 1.0 / delta_angle * (frames[seg_i + 1].p - frames2[seg_i + 1].p).Norm();
        //         // tmp_dist = 1.0 / delta_angle * this->MaxDistanceMeshes(RS(jntarray.data, frames), RS(jntarray.data, frames2));
        //         // segment_dist = 1.0 / delta_angle * this->MaxDistance(RS(jntarray.data, frames), RS(jntarray.data, frames2));

        //         double d = std::max(0.0, tmp_dist - segment_dist);
        //         // std::cout << " tmp dist: " << tmp_dist << " seg dist: " << segment_dist << " delta d: " << d << "\n";
        //         if (d > approxRadii(j))
        //         {
        //             approxRadii(j) = d;
        //         }
        //     }
        // }
        this->approxRadii = Eigen::VectorXd::Zero(nrjnts);
        for (int k = 0; k < 100; ++k)
        {
            auto [jac, r, rr] = this->ForwardJacsComplete(M_PI * VectorXd::Random(nrjnts));
            this->approxRadii = this->approxRadii.cwiseMax(rr);
        }
        // std::cout << "approx radii: " << approxRadii.transpose() << "\n";
        // throw std::runtime_error("exit for debug");

        this->numberOfModels = numModels;

        auto movablejoints = this->MovableJoints();
        this->segmentToJntCausality = movablejoints;
    }

    std::vector<std::vector<double>>
    RobotBase::GetMinMaxBounds()
    {
        std::vector<std::vector<double>> minmaxs(this->kdl_chain.getNrOfJoints(), std::vector<double>(2));

        std::vector<std::string> joint_names;
        for (int i = 0; i < kdl_chain.getNrOfSegments(); ++i)
        {
            const KDL::Segment &segment = kdl_chain.getSegment(i);
            const KDL::Joint &joint = segment.getJoint();

            if (joint.getType() != KDL::Joint::None)
            {
                joint_names.push_back(joint.getName());
            }
        }

        int num_joints = 0;
        for (const std::string &name : joint_names)
        {
            auto joint = this->robot_model->getJoint(name);
            if (joint && joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
            {
                double lower_limit = joint->limits->lower;
                double upper_limit = joint->limits->upper;
                // Use the joint limits here
                minmaxs[num_joints][0] = lower_limit;
                minmaxs[num_joints][1] = upper_limit;
                // std::cout << "Joint " << name << ": Lower: " << lower_limit << " Upper: " << upper_limit << std::endl;
                num_joints++;
            }
        }
        if (num_joints != joint_names.size())
        {
            std::cout << "KDL chain joints: " << joint_names.size() << " URDF joints: " << num_joints << std::endl;
            throw std::runtime_error("Number of URDF joints != number of KDL chain joints.");
        }

        return minmaxs;
    }

    std::optional<urdf::ModelInterfaceSharedPtr>
    RobotBase::GetRobotURDF(std::string urdf_filename)
    {
        // Load the URDF file into a string
        std::ifstream urdf_file(urdf_filename);
        if (!urdf_file.good())
        {
            std::cerr << "Could not open file: " << urdf_filename << std::endl;
            return {};
        }

        std::string urdf_string((std::istreambuf_iterator<char>(urdf_file)),
                                std::istreambuf_iterator<char>());

        // Parse the string using the URDF parser
        urdf::ModelInterfaceSharedPtr robot_model = urdf::parseURDF(urdf_string);

        if (!robot_model)
        {
            std::cerr << "Failed to parse URDF file." << std::endl;
            return {};
        }
        return robot_model;
    }

    std::map<std::string, std::string>
    RobotBase::GetSegmentNameToFile(urdf::ModelInterfaceSharedPtr robot_model)
    {
        std::map<std::string, std::string> my_map;

        for (auto &l : robot_model->links_)
        {
            const auto &coll = l.second->collision;
            if (!coll)
            {
                continue;
            }
            const auto &geom = coll->geometry;
            if (!geom)
            {
                continue;
            }

            if (geom->type == urdf::Geometry::MESH)
            {
                const auto &mesh = std::static_pointer_cast<urdf::Mesh>(geom);
                std::string mesh_filename = mesh->filename;
                std::string link_name = l.second->name;
                // Do something with mesh_filename, which is the path to the .obj file
                // std::cout << "Link children: " << l.second->child_links.size() << " has mesh: " << mesh_filename << " mesh file name length: " << mesh_filename.size() << std::endl;
                // std::cout << "Link " << l.second->name << " has mesh: " << mesh_filename << std::endl;
                my_map[link_name] = mesh_filename;
                // std::cout << "Segment: " << my_map[link_name] << " File: " << mesh_filename << std::endl;
            }
        }
        // exit(1);

        return my_map;
    }

    std::optional<KDL::Tree>
    RobotBase::GetKDLTree(urdf::ModelInterfaceSharedPtr robot_model)
    {

        // Create a KDL tree from the URDF model
        KDL::Tree kdl_tree;
        if (!kdl_parser::treeFromUrdfModel(*robot_model, kdl_tree))
        {
            std::cerr << "Failed to construct KDL tree." << std::endl;
            return {};
        }

        return kdl_tree;
    }

    std::optional<KDL::Chain>
    RobotBase::GetKDLChain(urdf::ModelInterfaceSharedPtr robot_model, KDL::Tree kdl_tree, std::string end_effector_link)
    {
        KDL::Chain kdl_chain;
        if (!kdl_tree.getChain(robot_model->getRoot()->name, end_effector_link, kdl_chain))
        {
            std::cerr << "Failed to get KDL chain from tree." << std::endl;
            return {};
        }
        return kdl_chain;
    }

    std::optional<std::vector<std::string>>
    RobotBase::GetEndLinks(const urdf::ModelInterfaceSharedPtr &robot_model)
    {
        std::vector<std::string> end_links;
        for (auto &rl : robot_model->links_)
        {
            if (rl.second->child_links.size() == 0)
            {
                // std::cout << "Link : " << rl.second->name << " links: " << rl.second->child_links.size() << std::endl;
                end_links.push_back(rl.second->name);
            }
        }

        if (end_links.size() == 0)
        {
            return {};
        }
        else
        {
            return end_links;
        }
    }

    std::string
    RobotBase::GetLinkName(const KDL::Chain &kdl_chain, unsigned int i)
    {
        return kdl_chain.getSegment(i).getName();
    }

    std::map<int, std::string>
    RobotBase::GetSegmentIdToName(const KDL::Chain &kdl_chain)
    {
        std::map<int, std::string> id_to_model_name;
        for (unsigned int i = 0; i < kdl_chain.getNrOfSegments(); ++i)
        {
            std::string segment_name = kdl_chain.getSegment(i).getName();
            id_to_model_name[i] = segment_name;
        }
        return id_to_model_name;
    }

    std::map<int, std::string>
    RobotBase::GetSegmentIdToFile()
    {
        std::map<int, std::string> segmentIdToFile;

        for (const auto &entry : this->segmentIdToName)
        {
            int key = entry.first;
            // Check if 'segmentToFile' contains the 'key'
            if (this->segmentNameToFile.find(entry.second) != this->segmentNameToFile.end())
            {
                segmentIdToFile[key] = this->segmentNameToFile[entry.second];
            }
        }

        return segmentIdToFile;
    }

    KDL::ChainFkSolverPos_recursive
    RobotBase::GetFKSolver(KDL::Chain kdl_chain)
    {
        KDL::ChainFkSolverPos_recursive fk_solver = KDL::ChainFkSolverPos_recursive(kdl_chain);
        return fk_solver;
    }

    std::string
    RobotBase::ToString()
    {
        return this->urdf_file.string();
    }

    std::vector<KDL::Frame>
    RobotBase::ForwardPass(const VectorXd &q_in)
    {
        KDL::JntArray q_kdl;
        q_kdl.data = q_in;

        std::vector<KDL::Frame> p_out(this->kdl_chain.getNrOfSegments());

        KDL::ChainFkSolverPos_recursive fk_solver(this->kdl_chain);
        if (fk_solver.JntToCart(q_kdl, p_out) < 0)
        {
            throw std::runtime_error("RobotBase::ForwardPass failed.");
        }
        return p_out;
    }

    KDL::Jacobian
    RobotBase::ForwardJac(const VectorXd &q_in)
    {
        KDL::JntArray q_kdl;
        q_kdl.data = q_in;

        KDL::Jacobian jac(this->kdl_chain.getNrOfJoints());

        KDL::ChainJntToJacSolver jac_solver(this->kdl_chain);

        if (jac_solver.JntToJac(q_kdl, jac) < 0)
        {
            throw std::runtime_error("RobotBase::ForwardJac failed.");
        }
        return jac;
    }

    KDL::JntArray
    RobotBase::ForwardJPlus(const VectorXd q_in, const KDL::Twist &v_in)
    {
        KDL::JntArray q_kdl;
        q_kdl.data = q_in;

        KDL::JntArray q_dot(q_in.size());

        KDL::ChainIkSolverVel_pinv pinv_solver(this->kdl_chain);

        int res = pinv_solver.CartToJnt(q_kdl, v_in, q_dot);
        if (res < 0)
        {
            throw std::runtime_error("RobotBase::ForwardJPlus failed. error: " + std::string(pinv_solver.strError(res)));
        }
        return q_dot;
    }

    KDL::JntArray
    RobotBase::ForwardJPlus(const RS &state, const KDL::Twist &v_in)
    {
        VectorXd q_in = state.config;
        // std::cout << "TODO: reuse jacobian for multiple target calculation later on\n";
        KDL::JntArray q_kdl;
        q_kdl.data = q_in;

        KDL::JntArray q_dot(q_in.size());

        KDL::ChainIkSolverVel_pinv pinv_solver(this->kdl_chain);

        int res = pinv_solver.CartToJnt(q_kdl, v_in, q_dot);
        if (res < 0)
        {
            throw std::runtime_error("RobotBase::ForwardJPlus failed. error: " + std::string(pinv_solver.strError(res)));
        }
        return q_dot;
    }

    MatrixXd
    RobotBase::JPlus(const RS &state)
    {
        MatrixXd pinv = state.jac.data.completeOrthogonalDecomposition().pseudoInverse();
        return pinv;
    }

    std::tuple<KDL::Jacobian, VectorXd>
    RobotBase::ForwardJacs(const VectorXd &q_in)
    {
        VectorXd r(q_in.size());
        r.array() = 0;

        KDL::JntArray q_kdl(q_in.size());
        q_kdl.data = q_in;
        KDL::Jacobian jac(q_in.size());

        // Initialize Jacobian to zero since only segmentNr columns are computed
        KDL::SetToZero(jac);

        KDL::Twist t_tmp;
        KDL::Frame T_tmp;
        T_tmp = KDL::Frame::Identity();
        KDL::SetToZero(t_tmp);
        int j = 0;
        int k = 0;
        KDL::Frame total;
        for (unsigned int i = 0; i < this->kdl_chain.getNrOfSegments(); i++)
        {
            auto jointType = this->kdl_chain.getSegment(i).getJoint().getType();
            // Calculate new Frame_base_ee
            if (jointType != KDL::Joint::JointType::None)
            {
                // pose of the new end-point expressed in the base
                total = T_tmp * this->kdl_chain.getSegment(i).pose(q_in(j));
                // changing base of new segment's twist to base frame if it is not locked
                // t_tmp = T_tmp.M*chain.getSegment(i).twist(1.0);
                // if (!locked_joints_[j])
                t_tmp = T_tmp.M * this->kdl_chain.getSegment(i).twist(q_in(j), 1.0);
            }
            else
            {
                total = T_tmp * this->kdl_chain.getSegment(i).pose(0.0);
            }

            // Changing Refpoint of all columns to new ee
            changeRefPoint(jac, total.p - T_tmp.p, jac);

            // Only increase jointnr if the segment has a joint
            if (jointType != KDL::Joint::JointType::None)
            {
                // Only put the twist inside if it is not locked
                // if (!locked_joints_[j])
                jac.setColumn(k++, t_tmp);
                j++;

                if (jointType == KDL::Joint::JointType::TransAxis || jointType == KDL::Joint::JointType::TransX || jointType == KDL::Joint::JointType::TransY || jointType == KDL::Joint::JointType::TransZ)
                {
                    // ASSUMING CONFIGURATION TRANSLATES TO 1:1 Meter translation
                    // joint.scale = 1 by default
                    r(k - 1) = 1;
                }
                else
                {
                    Eigen::VectorXd tmp_rs = jac.data.topRows(3).colwise().norm();
                    r = r.cwiseMax(tmp_rs);
                }
            }

            // std::cout << "jac:\n"
            //           << jac.data << "\n";
            // std::cout << "r: " << r.transpose() << "\n";

            T_tmp = total;
        }
        return {jac, r};
    }

    std::tuple<KDL::Jacobian, VectorXd, VectorXd>
    RobotBase::ForwardJacsComplete(const VectorXd &q_in)
    {
        VectorXd r(q_in.size());
        VectorXd rigidRadii(q_in.size());
        r.array() = 0;
        rigidRadii.array() = 0;

        KDL::JntArray q_kdl(q_in.size());
        q_kdl.data = q_in;
        KDL::Jacobian jac(q_in.size());

        // Initialize Jacobian to zero since only segmentNr columns are computed
        KDL::SetToZero(jac);

        KDL::Twist t_tmp;
        KDL::Frame T_tmp;
        T_tmp = KDL::Frame::Identity();
        KDL::SetToZero(t_tmp);
        int j = 0;
        int k = 0;
        KDL::Frame total;
        for (unsigned int i = 0; i < this->kdl_chain.getNrOfSegments(); i++)
        {
            auto jointType = this->kdl_chain.getSegment(i).getJoint().getType();
            // Calculate new Frame_base_ee
            if (jointType != KDL::Joint::JointType::None)
            {
                // pose of the new end-point expressed in the base
                total = T_tmp * this->kdl_chain.getSegment(i).pose(q_in(j));
                // changing base of new segment's twist to base frame if it is not locked
                // t_tmp = T_tmp.M*chain.getSegment(i).twist(1.0);
                // if (!locked_joints_[j])
                t_tmp = T_tmp.M * this->kdl_chain.getSegment(i).twist(q_in(j), 1.0);
            }
            else
            {
                total = T_tmp * this->kdl_chain.getSegment(i).pose(0.0);
            }

            // Changing Refpoint of all columns to new ee
            changeRefPoint(jac, total.p - T_tmp.p, jac);

            // Only increase jointnr if the segment has a joint
            if (jointType != KDL::Joint::JointType::None)
            {
                // Only put the twist inside if it is not locked
                // if (!locked_joints_[j])
                jac.setColumn(k++, t_tmp);
                j++;

                if (jointType == KDL::Joint::JointType::TransAxis || jointType == KDL::Joint::JointType::TransX || jointType == KDL::Joint::JointType::TransY || jointType == KDL::Joint::JointType::TransZ)
                {
                    // ASSUMING CONFIGURATION TRANSLATES TO 1:1 Meter translation
                    // joint.scale = 1 by default
                    r(k - 1) = 1;
                }
                else
                {
                }
            }
            for (size_t l = 0; l < k; ++l)
            {
                if (radialJoints(l) == 1)
                {
                    double tmp_r = jac.data.col(l).head<3>().norm();
                    if (tmp_r > r(l))
                    {
                        r(l) = tmp_r;
                    }
                }
            }
            /*
            j = active joint
            i(j) = segment corresponding to joint j
            i >= j
            radius(j) => segment(i(j)), segment(i(j) + 1), segment(i(j) + 1)
            */
            for (size_t l = i; l < this->kdl_chain.getNrOfSegments(); ++l)
            {
                if (this->segmentIdToModel[i])
                {
                    int j_tmp = j - 1;
                    double JPhiNorm = jac.data.col(j_tmp).tail<3>().cwiseAbs().dot(this->segmentIdToModel[i].value()->encompassingRadii);
                    // double JPhiNorm = 3 * jac.data.col(l).tail<3>().cwiseAbs().maxCoeff() * this->segmentIdToModel[i].value()->encompassingRadii.maxCoeff();
                    if (JPhiNorm > rigidRadii(j_tmp))
                    {
                        rigidRadii(j_tmp) = JPhiNorm;
                    }
                    // std::cout << "radii: " << this->segmentIdToModel[i].value()->encompassingRadii.transpose() << "\n";
                    // std::cout << "rig r: " << rigidRadii.transpose() << "\n";
                }
            }

            T_tmp = total;
        }
        // throw std::runtime_error("Exiting from calculate complete radii");
        // std::cout << "r: " << r.transpose() << "\n";
        // exit(1);
        return {jac, r, rigidRadii};
    }

    VectorXd
    RobotBase::GetDistanceEstimates(const RS &state)
    {
        VectorXd dists = VectorXd::Zero(state.config.size());
        unsigned int nrSegments = this->kdl_chain.getNrOfSegments();

        unsigned int j = 0;
        // for (size_t k = 0; k < nrSegments; ++k)
        // {
        //     std::cout << "seg[" << k << "]: Joint: " << this->kdl_chain.getSegment(k).getJoint().getTypeName() << "\n";
        // }
        KDL::Frame current_pose = KDL::Frame::Identity();
        KDL::Frame last_pose;
        for (unsigned int i = 0; i < nrSegments - 1; ++i)
        {
            last_pose = current_pose;
            current_pose = state.frames[i];
            KDL::Vector pj = current_pose.p;
            // auto next_pose = state.frames[i + 1];

            // Local axis: https://docs.ros.org/en/indigo/api/orocos_kdl/html/classKDL_1_1Joint.html#a57c97b32765b0caeb84b303d66a96a1b
            auto joint = this->kdl_chain.getSegment(i).getJoint();
            // typedef enum { RotAxis,RotX,RotY,RotZ,TransAxis,TransX,TransY,TransZ,None} JointType;
            if (joint.getType() == KDL::Joint::JointType::None)
            {
                continue;
            }
            else
            {
                ++j;
            }

            if (joint.getType() == KDL::Joint::JointType::TransAxis || joint.getType() == KDL::Joint::JointType::TransX || joint.getType() == KDL::Joint::JointType::TransY || joint.getType() == KDL::Joint::JointType::TransZ)
            {
                // joint.scale is by default 1
                dists(j - 1) = 1;
            }
            else
            {
                KDL::Vector joint_axis_local = joint.JointAxis();
                KDL::Vector vj = last_pose.M * joint_axis_local;
                // std::cout << "\nj:" << j << "\njoint axis global: " << vj << "\n";
                for (unsigned int k = i + 1; k < nrSegments; ++k)
                {
                    // segment position - joint position
                    auto p = state.frames[k].p;
                    auto pv = p - pj;
                    // std::cout << "local position of next segment: " << pv << "\n";

                    // center point around the current joint
                    // project `pv` on `vj` =
                    //  (pv^T * vj)
                    //  ----------- * vj
                    //  vj^T * vj
                    auto pvT_vj = pv[0] * vj[0] + pv[1] * vj[1] + pv[2] * vj[2];
                    // std::cout << "segment to axis correlation: " << pvT_vj << "\n";
                    // project point on plane defined by joint axis
                    auto proj_vOrth_pv = pv - pvT_vj * vj;
                    // std::cout << "projection of local position on joint axis global: " << proj_vOrth_pv
                    //   << "\n";
                    double r = proj_vOrth_pv.Norm();
                    if (r > dists(j - 1))
                    {
                        dists(j - 1) = r;
                    }
                }
            }
        }
        // dists: 1 1 0.314961 0.478566 0.476605 0.48401  0.214512 0.229539 0
        // r:     1 1 0.314961 0.478566 0.476605 0.48401  0.214512 0.229539 0
        // std::cout << "dists: " << dists.transpose() << "\n";
        // throw std::runtime_error("testing projection radii");
        return dists;
    }

    VectorXd
    RobotBase::GetDistanceEstimatesWithRadii(const RS &state)
    {
        VectorXd dists = VectorXd::Zero(state.config.size());
        unsigned int nrSegments = this->kdl_chain.getNrOfSegments();
        unsigned int nrJoints = this->kdl_chain.getNrOfJoints();

        unsigned int j = 0;
        // for (size_t k = 0; k < nrSegments; ++k)
        // {
        //     std::cout << "seg[" << k << "]: Joint: " << this->kdl_chain.getSegment(k).getJoint().getTypeName() << "\n";
        // }
        KDL::Frame current_pose = KDL::Frame::Identity();
        KDL::Frame last_pose;
        for (unsigned int i = 0; i < nrSegments - 1; ++i)
        {
            last_pose = current_pose;
            current_pose = state.frames[i];
            KDL::Vector pj = current_pose.p;
            // auto next_pose = state.frames[i + 1];

            // Local axis: https://docs.ros.org/en/indigo/api/orocos_kdl/html/classKDL_1_1Joint.html#a57c97b32765b0caeb84b303d66a96a1b
            auto joint = this->kdl_chain.getSegment(i).getJoint();
            // typedef enum { RotAxis,RotX,RotY,RotZ,TransAxis,TransX,TransY,TransZ,None} JointType;
            if (joint.getType() == KDL::Joint::JointType::None)
            {
                continue;
            }
            else
            {
                ++j;
            }

            if (joint.getType() == KDL::Joint::JointType::TransAxis || joint.getType() == KDL::Joint::JointType::TransX || joint.getType() == KDL::Joint::JointType::TransY || joint.getType() == KDL::Joint::JointType::TransZ)
            {
                // joint.scale is by default 1
                dists(j - 1) = 1;
            }
            else
            {
                KDL::Vector joint_axis_local = joint.JointAxis();
                KDL::Vector vj = last_pose.M * joint_axis_local;
                // std::cout << "\nj:" << j << "\njoint axis global: " << vj << "\n";
                for (unsigned int k = i + 1; k < nrSegments; ++k)
                {
                    // segment position - joint position
                    auto p = state.frames[k].p;
                    auto pv = p - pj;
                    // std::cout << "local position of next segment: " << pv << "\n";

                    // center point around the current joint
                    // project `pv` on `vj` =
                    //  (pv^T * vj)
                    //  ----------- * vj
                    //  vj^T * vj
                    auto pvT_vj = pv[0] * vj[0] + pv[1] * vj[1] + pv[2] * vj[2];
                    // std::cout << "segment to axis correlation: " << pvT_vj << "\n";
                    // project point on plane defined by joint axis
                    auto proj_vOrth_pv = pv - pvT_vj * vj;
                    // std::cout << "projection of local position on joint axis global: " << proj_vOrth_pv
                    //   << "\n";
                    double r = proj_vOrth_pv.Norm();
                    if (r > dists(j - 1))
                    {
                        dists(j - 1) = r;
                    }
                }
            }
        }
        // std::cout << "dists: \n"
        //   << dists.transpose() << "\n\n";
        for (int i = 0; i < nrJoints; ++i)
        {
            dists(i) += this->approxRadii(i);
        }
        // std::cout << "updated dists: \n"
        //   << dists.transpose() << "\n";
        // dists: 1 1 0.314961 0.478566 0.476605 0.48401  0.214512 0.229539 0
        // r:     1 1 0.314961 0.478566 0.476605 0.48401  0.214512 0.229539 0
        // std::cout << "dists: " << dists.transpose() << "\n";
        // throw std::runtime_error("testing projection radii");
        return dists;
    }

    RS RobotBase::BasicFK(const VectorXd &q_in)
    {
        std::vector<KDL::Frame> frames = this->ForwardPass(q_in);
        RS state(q_in, frames);
        return state;
    }

    VectorXd
    RobotBase::parseCSVToVectorXd(const std::string &path)
    {
        std::ifstream file(path);
        if (!file.is_open())
        {
            throw std::runtime_error("Unable to open file: " + path);
        }

        std::string line;
        std::vector<double> values;

        while (std::getline(file, line))
        {
            std::stringstream ss(line);
            double value;
            if (ss >> value)
            {
                values.push_back(value);
            }
            else
            {
                throw std::runtime_error("Failed to parse line: " + line);
            }
        }

        VectorXd vec(values.size());
        for (size_t i = 0; i < values.size(); ++i)
        {
            vec[i] = values[i];
        }

        return vec;
    }

    std::vector<VectorXd>
    RobotBase::parseCSVToVectors(const std::string &path)
    {
        std::ifstream file(path);
        if (!file.is_open())
        {
            throw std::runtime_error("Unable to open file: " + path);
        }

        std::string line;
        std::vector<VectorXd> vectors;

        while (std::getline(file, line))
        {
            std::stringstream ss(line);
            std::vector<double> values;
            std::string value;

            while (std::getline(ss, value, ','))
            {
                try
                {
                    double num = std::stod(value);
                    values.push_back(num);
                }
                catch (const std::invalid_argument &e)
                {
                    throw std::runtime_error("Failed to parse number: " + value);
                }
            }

            VectorXd vec(values.size());
            for (size_t i = 0; i < values.size(); ++i)
            {
                vec[i] = values[i];
            }
            vectors.push_back(vec);
        }

        return vectors;
    }

    std::vector<VectorXd>
    RobotBase::MovableJoints() const
    {
        // TODO create a mask for each case so that when I generate random samples I can zero out joints that aren't supposed to be moved
        // each segment denotes joint ids that affect the segment
        // segment 1 => joints 0,1,2 affect it => 2
        std::vector<VectorXd> segmentToJointVector(this->numberOfModels, VectorXd::Ones(this->kdl_chain.getNrOfJoints()));
        std::vector<int> segmentToJointCausality;
        // It is only from the set of segments that have models:
        int joint = 0;
        /*
        lastInactiveSegment: 4/8
        i:0
         nothing
        i:1
         joint+=1
        i:2
         joint+=1
         segments+=1
        i:3
         joint+=1
         segments+=1
        i:4
         joint+=1
         segments+=1
        i:5
         joint+=1
         segments+=1
        */
        int k = 0;
        for (unsigned int i = 0; i < this->segmentIdToModel.size(); ++i)
        {
            if (this->kdl_chain.getSegment(i).getJoint().getType() != KDL::Joint::JointType::None)
            {
                ++joint;
            }
            if (this->segmentIdToModel[i])
            {
                for (int l = k; l < segmentToJointVector.size(); ++l)
                {
                    for (int m = 0; m < joint; ++m)
                    {
                        segmentToJointVector[l](m) = 0.0;
                    }
                }
                std::cout << "i: " << i << " joint: " << joint << " mask vector: " << segmentToJointVector[k].transpose() << "\n";
                segmentToJointCausality.push_back(joint);
                ++k;
            }
            else
            {
            }
        }
        return segmentToJointVector;
    }

    std::vector<bool>
    RobotBase::GetValidTransforms()
    {
        unsigned int l = this->segmentIdToModel.size();
        std::vector<bool> b(l);
        int k = 0;
        for (unsigned int i = 0; i < l; ++i)
        {
            b[i] = this->segmentIdToModel[i] ? true : false;
            if (b[i])
            {
                k++;
            }
        }
        std::cout << "num valid models: " << k << "\n";
        return b;
    }

    std::vector<std::shared_ptr<RtModels::RtModel>>
    RobotBase::GetModels()
    {
        std::vector<std::shared_ptr<RtModels::RtModel>> models(this->numberOfModels);
        int k = 0;
        for (int i = 0; i < this->segmentIdToModel.size(); ++i)
        {
            if (this->segmentIdToModel[i])
            {
                models[k] = this->segmentIdToModel[i].value();
                k++;
            }
        }
        return models;
    }

    double
    RobotBase::EEDistance(const RS &state1, const RS &state2) const
    {
        return (this->GetEEFrame(state1).p - this->GetEEFrame(state2).p).Norm();
    }

    KDL::Frame
    RobotBase::GetEEFrame(const RS &state) const
    {
        return state.frames.back();
    }

    std::pair<int, std::vector<double>>
    RobotBase::MaxDistances(const RS &state1, const RS &state2) const
    {
        auto f1 = state1.frames;
        auto f2 = state2.frames;

        double max_dist = 0;
        int max_idx = -1;
        std::vector<double> dists(f1.size());
        for (unsigned int i = 0; i < f1.size(); ++i)
        {
            // dist is in meters
            double dist = (f1[i].p - f2[i].p).Norm();
            dists[i] = dist;
            if (dist > max_dist)
            {
                max_dist = dist;
                max_idx = i;
            }
        }
        return {max_idx, dists};
    }

    double
    RobotBase::MaxDistance(const RS &state1, const RS &state2) const
    {
        auto f1 = state1.frames;
        auto f2 = state2.frames;

        double max_dist = 0;
        for (unsigned int i = 0; i < f1.size(); ++i)
        {
            // dist is in meters
            double dist = (f1[i].p - f2[i].p).Norm();
            if (dist > max_dist)
            {
                max_dist = dist;
            }
        }
        // std::cout << "Wire max dist: " << max_dist << "\n";
        // double mesh_dist = this->MaxDistanceMeshes(state1, state2);
        // std::cout << "Mesh max dist: " << mesh_dist << "\n\n";
        return max_dist;
        // return std::max(mesh_dist, max_dist);
    }

    double
    RobotBase::MaxDistanceMeshes(const RS &state1, const RS &state2) const
    {
        // TODO: for small changes this should return small values
        auto f1 = state1.frames;
        auto f2 = state2.frames;
        double max_dist = 0;
        int frame_id = 0;
        for (auto &it : this->segmentIdToModel)
        {
            if (it)
            {
                auto rtmodel = it.value();
                auto num_tris = rtmodel->pqpModel->num_tris;
                // std::cout << "Num tris: " << num_tris << "\n";
                auto tmpf2 = state2.frames[frame_id];
                auto tmpf1 = state1.frames[frame_id];
                auto deltaR2 = tmpf2.M;

                for (size_t i = 0; i < num_tris; ++i)
                {
                    auto p1 = rtmodel->pqpModel->tris[i].p1;
                    auto p1vec = KDL::Vector(p1[0], p1[1], p1[2]);
                    auto tmp_dist = (tmpf1 * p1vec - tmpf2 * p1vec).Norm();
                    if (tmp_dist > max_dist)
                    {
                        max_dist = tmp_dist;
                    }
                }
            }
            ++frame_id;
        }
        // std::cout << "\n\n\n\n";
        return max_dist;
    }

    double
    RobotBase::MaxDistanceMeshSegment(const KDL::Frame &f1, const KDL::Frame &f2, const std::shared_ptr<RtModels::RtModel> rt_model) const
    {
        // TODO: for small changes this should return small values
        double max_dist = 0;
        auto num_tris = rt_model->pqpModel->num_tris;
        // std::cout << "Num tris: " << num_tris << "\n";
        auto deltaR2 = f2.M;

        for (size_t i = 0; i < num_tris; ++i)
        {
            auto p1 = rt_model->pqpModel->tris[i].p1;
            auto p1vec = KDL::Vector(p1[0], p1[1], p1[2]);
            auto tmp_dist = (f1 * p1vec - f2 * p1vec).Norm();
            if (tmp_dist > max_dist)
            {
                max_dist = tmp_dist;
            }
        }
        return max_dist;
    }

    std::pair<Matrix3d, Vector3d>
    RobotBase::KDLFrameToEigen(const KDL::Frame &f)
    {
        Vector3d t;
        // Directly assign values
        t.x() = f.p.x();
        t.y() = f.p.y();
        t.z() = f.p.z();
        // KDL USES ROW MAJOR
        // EIGEN USES COL MAJOR => COPY INDEX BY INDEX
        Matrix3d R;
        for (unsigned int l = 0; l < 3; ++l)
        {
            for (unsigned int m = 0; m < 3; ++m)
            {
                R(l, m) = f.M(l, m);
            }
        }
        return {R, t};
    }

    std::optional<VectorXd>
    RobotBase::GetInverseKinematics(KDL::ChainIkSolverPos &solver, const KDL::JntArray &q_init, const KDL::Frame &tgt)
    {
        KDL::JntArray res(q_init.rows());
        // KDL::ChainIkSolverPos_LMA::E_NOERROR
        if (solver.CartToJnt(q_init, tgt, res) >= 0)
        {
            return res.data;
        }
        return {};
    }
}
