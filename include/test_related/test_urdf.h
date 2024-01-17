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

#include "robot_related/robot_base.h"
#include "robot_related/robot_collision.h"
#include "bur_related/urdf_planner.h"

int test_urdf(std::string urdf_filename)
{
    // Burs::RobotBase handler(urdf_filename);
    // auto num_joints = handler.kdl_chain.getNrOfJoints();
    // auto zerojoints = Eigen::VectorXd::Zero(num_joints);
    // std::cout << "Randjoints: " << zerojoints << std::endl;

    // handler.ForwardQ(zerojoints);

    Burs::RobotCollision pqp_handler(urdf_filename);
    auto num_joints = pqp_handler.kdl_chain.getNrOfJoints();
    auto zerojoints = Eigen::VectorXd::Zero(num_joints);
    std::cout << "Randjoints: " << zerojoints << std::endl;

    pqp_handler.ForwardQ(zerojoints);
    std::cout << "ForwardQ" << std::endl;
    pqp_handler.SelectedForwardQ(zerojoints);
    std::cout << "SelectedForwardQ" << std::endl;

    // burenv > pqp_urdf_handler > urdf_handler
    Burs::CollisionEnv burenv(urdf_filename);
    // 1/3 This contains the ForwardRt
    burenv.SetPoses(zerojoints);

    // pqp_handler.GetForwardPoint(const int &ith_distal_point, const Eigen::VectorXd &q_in);
    // pqp_handler.GetRadius(pqp_handler.kdl_chain.getNrOfSegments(), zerojoints);
    // 2/3 Required functions
    Burs::RadiusFuncParallel rf = pqp_handler.GetRadiusFunc();
    Burs::ForwardKinematics fk = pqp_handler.GetForwardPointFunc();

    for (int i = 0; i < pqp_handler.kdl_chain.getNrOfSegments() + 1; ++i)
    {
        Eigen::VectorXd r = rf(zerojoints);
        std::cout << "Joint: " << i << " Protective radius: " << r.transpose() << std::endl;
        Eigen::Vector3d position = fk(i, zerojoints);
        std::cout << "Joint: " << i << " 3d position: " << position.transpose() << std::endl;
    }

    // printing::WriteModelsToFile(burenv.robot_models, "test_file.txt");
    std::fstream test_file("test_file.txt");
    for (auto &model : burenv.robot_models)
    {
        // model->GetFilePath(); // string, path to the file
        // model->getR(); // 3x3 double array
        // model->getT(); // 3 double array
        test_file << model->ToString() << std::endl;
    }

    /* this should:
    - load the URDf model
    - get the 3 functions:
     - DONE: forward kinematics for each distal point
     - radius function for each joint
     - DONE: forward kinematics for each segment
*/
    int max_iters = 1;
    double d_crit = 0.1;
    double delta_q = 3.0;
    double epsilon_q = 0.2;
    int num_spikes = 7;
    Eigen::VectorXd target_pose = Eigen::VectorXd::Ones(num_joints);

    Burs::URDFPlanner urdf_planner(urdf_filename, max_iters, d_crit, delta_q, epsilon_q, num_spikes);

    auto path_opt = urdf_planner.PlanPath(zerojoints, target_pose);
    if (path_opt)
    {
        std::cout << "Path:" << std::endl;
        for (int i = 0; i < path_opt.value().size(); ++i)
        {
            std::cout << path_opt.value()[i].transpose() << std::endl;
        }
    }
    else
    {
        std::cout << "Couldn't find path for start: " << zerojoints.transpose() << " and goal " << target_pose.transpose() << std::endl;
    }
    // URDFPlanner(urdf_filename);
    // URDFPlanner(int q_dim, ForwardKinematics f, int max_iters, double d_crit, double delta_q, double epsilon_q, MatrixXd bounds, RadiusFunc radius_func, int num_spikes);

    // Burs::URDFPlanner();
    // auto opt_robot = GetRobotURDF(urdf_filename);

    // if (!opt_robot)
    // {
    //     return -1;
    // }

    // urdf::ModelInterfaceSharedPtr robot_model = opt_robot.value();

    // std::map<std::string, std::string> my_map = SegmentNameToFile(robot_model);

    // // Create a KDL tree from the URDF model
    // auto kdl_tree_res = GetKDLTree(robot_model);

    // if (!kdl_tree_res)
    // {
    //     return -1;
    // }

    // KDL::Tree kdl_tree = kdl_tree_res.value();
    // // std::string base_link = "panda_link0";

    // // std::string end_effector_link = "panda_hand";
    // auto end_links_res = GetEndLinks(robot_model);

    // if (!end_links_res)
    // {
    //     return -1;
    // }

    // std::vector<std::string> end_links = end_links_res.value();

    // // std::cout << "end links:" << std::endl;
    // // for (int i = 0; i < end_links.size(); ++i)
    // // {
    // //     std::cout << "end link: " << end_links[i] << std::endl;
    // // }

    // auto kdl_chain_res = GetKDLChain(robot_model, kdl_tree, end_links[0]);

    // if (!kdl_chain_res)
    // {
    //     return -1;
    // }

    // KDL::Chain kdl_chain = kdl_chain_res.value();

    // for (unsigned int i = 0; i < kdl_chain.getNrOfSegments(); ++i)
    // {
    //     std::string segment_name = kdl_chain.getSegment(i).getName();
    //     std::cout << "Segment " << i << ": " << segment_name << " File: " << my_map[segment_name] << std::endl;
    // }

    // Create solver based on kinematic chain
    // KDL::ChainFkSolverPos_recursive fk_solver = GetFKSolver(kdl_chain);

    // // Create random joint angle vector
    // std::default_random_engine generator;
    // std::uniform_real_distribution<double> distribution(-M_PI / 4, M_PI / 4);
    // KDL::JntArray joint_positions(kdl_chain.getNrOfJoints());

    // for (unsigned int i = 0; i < joint_positions.data.size(); ++i)
    // {
    //     joint_positions(i) = distribution(generator);
    // }

    // // Compute forward kinematics
    // KDL::Frame end_effector_pose;
    // fk_solver.JntToCart(joint_positions, end_effector_pose);

    // // Print the positions of each robot segment
    // for (unsigned int i = 0; i < kdl_chain.getNrOfSegments(); ++i)
    // {
    //     KDL::Frame segment_pose;
    //     fk_solver.JntToCart(joint_positions, segment_pose, i + 1);
    //     // std::cout << "Segment " << i << ": " << segment_pose << std::endl;
    //     // std::cout << "\nSegment " << i << ": \nPosition: \n"
    //     //           << segment_pose.p << "\nRotation: \n"
    //     //           << segment_pose.M << std::endl;
    // }

    return 0;
}
