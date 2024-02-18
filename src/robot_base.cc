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

        this->segmentNameToFile = this->GetSegmentNameToFile(robot_model);
        this->segmentIdToName = this->GetSegmentIdToName(this->kdl_chain);
        this->segmentIdToFile = this->GetSegmentIdToFile();

        this->minMaxBounds = this->GetMinMaxBounds();
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
        std::cout << "pinv: \n"
                  << pinv << "\n";
        // exit(1);
        return pinv;
    }

    std::pair<KDL::Jacobian, VectorXd>
    RobotBase::ForwardJacs(const VectorXd &q_in)
    {
        VectorXd r(q_in.size());
        r.array() = 0;
        // std::cout << "TODO: COPY JACOBIAN FUNCTION FROM KDL REPO AND REWRITE IT TO RETURN THE VECTOR OF ALL JACOBIANS\n";
        KDL::JntArray q_kdl(q_in.size());
        q_kdl.data = q_in;
        KDL::Jacobian jac(q_in.size());

        // KDL::ChainJntToJacSolver s(this->kdl_chain);

        // int res = 0;
        // for (unsigned int i = 0; i < this->kdl_chain.getNrOfSegments(); ++i)
        // {
        //     res = s.JntToJac(q_kdl, jac, i + 1);
        //     if (res < 0)
        //     {
        //         std::cout << "failed jnt to jac: " << (i + 1) << "\n";
        //         // exit(1);
        //     }
        //     // std::cout << "jac " << i << ": \n"
        //     //           << jac.data << "\n";
        //     for (unsigned int l = 0; l < q_in.size(); ++l)
        //     {
        //         double tmp_r = jac.data.col(l).head<3>().norm();
        //         if (tmp_r > r(l))
        //         {
        //             r(l) = tmp_r;
        //         }
        //     }
        //     // std::cout << "best r: " << r.transpose() << "\n\n";
        // }
        // std::cout << "best r: " << r.transpose() << "\n\n";
        // r.array() = 0;
        // exit(1);

        //     if (locked_joints_.size() != chain.getNrOfJoints()) return (error = E_NOT_UP_TO_DATE);
        // unsigned int segmentNr;
        // if (seg_nr < 0)
        //     segmentNr = chain.getNrOfSegments();
        // else
        //     segmentNr = seg_nr;

        // Initialize Jacobian to zero since only segmentNr columns are computed
        KDL::SetToZero(jac);

        // if (q_in.rows() != chain.getNrOfJoints() || jac.columns() != chain.getNrOfJoints())
        //     return (error = E_SIZE_MISMATCH);
        // else if (segmentNr > chain.getNrOfSegments())
        //     return (error = E_OUT_OF_RANGE);

        KDL::Twist t_tmp;
        KDL::Frame T_tmp;
        T_tmp = KDL::Frame::Identity();
        KDL::SetToZero(t_tmp);
        int j = 0;
        int k = 0;
        KDL::Frame total;
        for (unsigned int i = 0; i < this->kdl_chain.getNrOfSegments(); i++)
        {
            // Calculate new Frame_base_ee
            if (this->kdl_chain.getSegment(i).getJoint().getType() != KDL::Joint::JointType::None)
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
            if (this->kdl_chain.getSegment(i).getJoint().getType() != KDL::Joint::JointType::None)
            {
                // Only put the twist inside if it is not locked
                // if (!locked_joints_[j])
                jac.setColumn(k++, t_tmp);
                j++;
            }

            // std::cout << "inside jac " << i << " jnt: " << k << " :\n"
            //           << jac.data << "\n";
            for (unsigned int l = 0; l < k; ++l)
            {
                double tmp_r = jac.data.col(l).head<3>().norm();
                if (tmp_r > r(l))
                {
                    r(l) = tmp_r;
                }
            }
            // std::cout << "best r: " << r.transpose() << "\n";

            T_tmp = total;
        }
        // std::cout << "best r: " << r.transpose() << "\n";
        // std::cout << "my jac:\n"
        //           << jac.data << "\n";
        // exit(1);
        // return (error = E_NOERROR);
        return {jac, r};
    }

    VectorXd
    RobotBase::GetRadii(const RS &state)
    {
        // auto fk_res = this->CachedForwardPass(q_in);
        VectorXd radii(state.config.size());
        unsigned int nrSegments = this->kdl_chain.getNrOfSegments();

        unsigned int j = 0;
        for (unsigned int i = 0; i < nrSegments - 1; ++i)
        {
            auto segment_pose = state.frames[i];
            Vector3d position(segment_pose.p.x(), segment_pose.p.y(), segment_pose.p.z());

            // Local axis: https://docs.ros.org/en/indigo/api/orocos_kdl/html/classKDL_1_1Joint.html#a57c97b32765b0caeb84b303d66a96a1b
            auto joint = this->kdl_chain.getSegment(i).getJoint();
            KDL::Vector joint_axis_local = joint.JointAxis();

            // typedef enum { RotAxis,RotX,RotY,RotZ,TransAxis,TransX,TransY,TransZ,None} JointType;
            // std::cout << "Joint: " << joint.getTypeName() << " Axis: " << joint.JointAxis() << std::endl;

            if (joint.getType() == KDL::Joint::JointType::None)
            {
                continue;
            }

            /*
            The expression \sum^n_{i=1} r_i |y_i − q_i| is a conservative upper bound on the displacement of any point on the manipulator
              when the configuration changes from q = (q_1 . . . q_n)T to y = (y_1 . . . y_n)^T .

            In short: it is the first order derivative of the mapping from configuration value q_i to euclidean space

            Ergo: translational joints: d(distance)/dq = 1
            rotational joints: d(phi*r)/dphi = r - the radius
            */

            double radius = 0.0;
            switch (joint.getType())
            {
            case KDL::Joint::JointType::TransAxis:
            case KDL::Joint::JointType::TransX:
            case KDL::Joint::JointType::TransY:
            case KDL::Joint::JointType::TransZ:
            {
                radius = 1;
                break;
            }
            default:
            {
                break;
            }
            }

            for (unsigned int k = i + 1; k < nrSegments; ++k)
            {
                KDL::Frame next_segment_pose = state.frames[k];
                KDL::Vector next_segment_kdl = next_segment_pose.p;
                Vector3d next_segment(next_segment_kdl.x(), next_segment_kdl.y(), next_segment_kdl.z());

                // Transform the local joint axis to the world reference frame
                KDL::Vector joint_axis_world = segment_pose.M * joint_axis_local;
                // std::cout << "GetRadius axis: " << joint_axis_world << std::endl;
                Vector3d joint_axis(joint_axis_world.x(), joint_axis_world.y(), joint_axis_world.z());

                Vector3d diff = next_segment - position;
                // Project the end effector onto the plane defined by the joint axis
                double dot_product = diff.dot(joint_axis);

                // joint_axis has norm = 1 => NO NORMALIZATION NECESSARY
                // std::cout << "joint axis: " << joint_axis.norm() << "\n";
                Vector3d projection = diff - dot_product * joint_axis;

                // Update the radius
                // std::cout << "Radius distance " << ith_distal_point << ": " << projection.norm() << std::endl;
                double tmp_radius = projection.norm();
                if (tmp_radius > radius)
                {
                    radius = tmp_radius;
                }
            }

            radii(j) = radius;
            ++j;
        }
        return radii;

        // unsigned int num_segments = this->kdl_chain.getNrOfSegments();
        // unsigned int num_joints = this->kdl_chain.getNrOfJoints();
        // // std::cout << "Number of joints: " << num_joints << "  Number of segments: " << num_segments << std::endl;

        // VectorXd radii(q_in.size());

        // KDL::ChainFkSolverPos_recursive fk_solver(this->kdl_chain);

        // assert(q_in.size() == num_joints);

        // KDL::JntArray joint_positions(num_joints);

        // for (unsigned int i = 0; i < num_joints; ++i)
        // {
        //     joint_positions(i) = q_in(i);
        // }

        // // Why does it fail when i use segment_poses instead of just a single pose
        // // CAUSE: it returns -1 for all errors, regardless of what you did wrong
        // // Solution: https://github.com/orocos/orocos_kinematics_dynamics/blob/master/orocos_kdl/src/chainfksolverpos_recursive.cpp

        // std::vector<KDL::Frame> segment_poses(num_segments);

        // if (fk_solver.JntToCart(joint_positions, segment_poses) >= 0)
        // {
        //     unsigned int j = 0;
        //     for (unsigned int i = 0; i < segment_poses.size() - 1; ++i)
        //     {
        //         auto segment_pose = segment_poses[i];
        //         Vector3d position(segment_pose.p.x(), segment_pose.p.y(), segment_pose.p.z());

        //         // Local axis: https://docs.ros.org/en/indigo/api/orocos_kdl/html/classKDL_1_1Joint.html#a57c97b32765b0caeb84b303d66a96a1b
        //         auto joint = this->kdl_chain.getSegment(i).getJoint();
        //         KDL::Vector joint_axis_local = joint.JointAxis();

        //         // typedef enum { RotAxis,RotX,RotY,RotZ,TransAxis,TransX,TransY,TransZ,None} JointType;
        //         // std::cout << "Joint: " << joint.getTypeName() << " Axis: " << joint.JointAxis() << std::endl;

        //         if (joint.getType() == KDL::Joint::JointType::None)
        //         {
        //             continue;
        //         }

        //         /*
        //         The expression \sum^n_{i=1} r_i |y_i − q_i| is a conservative upper bound on the displacement of any point on the manipulator
        //           when the configuration changes from q = (q_1 . . . q_n)T to y = (y_1 . . . y_n)^T .

        //         In short: it is the first order derivative of the mapping from configuration value q_i to euclidean space

        //         Ergo: translational joints: d(distance)/dq = 1
        //         rotational joints: d(phi*r)/dphi = r - the radius
        //         */

        //         double radius = 0.0;
        //         switch (joint.getType())
        //         {
        //         case KDL::Joint::JointType::TransAxis:
        //         case KDL::Joint::JointType::TransX:
        //         case KDL::Joint::JointType::TransY:
        //         case KDL::Joint::JointType::TransZ:
        //         {
        //             radius = 1;
        //             break;
        //         }
        //         default:
        //         {
        //             break;
        //         }
        //         }

        //         for (unsigned int k = i + 1; k < segment_poses.size(); ++k)
        //         {
        //             KDL::Frame next_segment_pose = segment_poses[k];
        //             KDL::Vector next_segment_kdl = next_segment_pose.p;
        //             Vector3d next_segment(next_segment_kdl.x(), next_segment_kdl.y(), next_segment_kdl.z());

        //             // Transform the local joint axis to the world reference frame
        //             KDL::Vector joint_axis_world = segment_pose.M * joint_axis_local;
        //             // std::cout << "GetRadius axis: " << joint_axis_world << std::endl;
        //             Vector3d joint_axis(joint_axis_world.x(), joint_axis_world.y(), joint_axis_world.z());

        //             Vector3d diff = next_segment - position;
        //             // Project the end effector onto the plane defined by the joint axis
        //             double dot_product = diff.dot(joint_axis);

        //             // joint_axis has norm = 1 => NO NORMALIZATION NECESSARY
        //             Vector3d projection = diff - dot_product * joint_axis;

        //             // Update the radius
        //             // std::cout << "Radius distance " << ith_distal_point << ": " << projection.norm() << std::endl;
        //             double tmp_radius = projection.norm();
        //             if (tmp_radius > radius)
        //             {
        //                 radius = tmp_radius;
        //             }
        //         }

        //         radii(j) = radius;
        //         ++j;
        //     }
        // }
        // else
        // {
        //     throw std::runtime_error("Forward kinematics solver failed in GetRadii");
        // }
        // return radii;
    }

    RS
    RobotBase::FullFK(const VectorXd &q_in)
    {
        // Frame of every segment
        std::vector<KDL::Frame> frames = this->ForwardPass(q_in);
        // Jacobian of every segment
        auto [jac, r] = this->ForwardJacs(q_in);
        RS state(q_in, frames, jac, r);

        // std::cout << "r: " << r.transpose() << "\n";

        // KDL::Jacobian jac = this->ForwardJac(q_in);
        // RS state(q_in, frames, jac);
        // VectorXd r_orig = this->GetRadii(state);
        // state.radii = r_orig;

        // std::cout << "r_orig: " << r_orig.transpose() << "\n";
        // exit(1);

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
}
