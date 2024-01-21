
#ifndef J_PLUS_RBT_PARAMETERS_H
#define J_PLUS_RBT_PARAMETERS_H

#include "rbt_parameters.h"
#include <Eigen/Dense>
#include <kdl/frames.hpp>
#include <memory>
#include "bur_tree.h"
#include "grasps.h"

namespace Burs
{
    struct JPlusRbtParameters : RbtParameters
    {

        /*The CSV:
        max_iters,100,
        d_crit,0.1,
        delta_q,3.1415,
        epsilon_q,0.1,
        num_spikes,5,
        p_close_enough,0.01,
        probability_to_steer_to_target,0.1,
        target_poses,/path/to/poses.csv,
        */

        // int max_iters;
        // double d_crit;
        // double delta_q;
        // double epsilon_q;
        // int num_spikes;

        double p_close_enough;
        double probability_to_steer_to_target;
        std::shared_ptr<BurTree> target_poses;
        Eigen::Vector3d mean_target;

        void
        GetMeanTranslation(std::vector<KDL::Frame> &target_poses)
        {
            Eigen::Vector3d mean_vector(0, 0, 0);
            for (int i = 0; i < target_poses.size(); ++i)
            {
                Eigen::Vector3d v(target_poses[i].p.x(), target_poses[i].p.y(), target_poses[i].p.z());
                mean_vector += v;
            }
            mean_vector /= target_poses.size();
            this->mean_target = mean_vector;
        }

        void
        ConstructTreeFromTargets(std::vector<KDL::Frame> &target_poses)
        {
            // TODO: add rotation
            // Eigen::VectorXd root_node(/*load translation rotation vector*/);
            Eigen::Vector3d root_node(target_poses[0].p.x(), target_poses[0].p.y(), target_poses[0].p.z());

            std::shared_ptr<BurTree> t = std::make_shared<BurTree>(root_node, root_node.size());

            for (int i = 1; i < target_poses.size(); ++i)
            {
                Eigen::Vector3d new_node(target_poses[i].p.x(), target_poses[i].p.y(), target_poses[i].p.z());

                t->AddNode(i - 1, new_node);
            }
            this->target_poses = t;
        }

        JPlusRbtParameters(int max_iters, double d_crit, double delta_q, double epsilon_q, int num_spikes, double p_close_enough, double probability_to_steer_to_target, std::vector<KDL::Frame> target_poses_input)
            : RbtParameters(max_iters, d_crit, delta_q, epsilon_q, num_spikes), p_close_enough(p_close_enough), probability_to_steer_to_target(probability_to_steer_to_target)
        {
            if (!target_poses_input.empty())
            {
                ConstructTreeFromTargets(target_poses_input);
                GetMeanTranslation(target_poses_input);
            }
            else
            {
                throw std::runtime_error("JPlusRbtParameters: target_poses_input is empty!!!");
            }
        }

        JPlusRbtParameters(const std::string &filename, const std::string &targetPosesPath)
        {
            std::ifstream file(filename);
            std::string line;
            std::unordered_map<std::string, std::string> params;

            while (std::getline(file, line))
            {
                std::stringstream ss(line);
                std::string key, value;
                if (std::getline(ss, key, ',') && std::getline(ss, value, ','))
                {
                    params[key] = value;
                }
            }

            // Set parameters, including the base class parameters
            max_iters = std::stoi(params["max_iters"]);
            d_crit = std::stod(params["d_crit"]);
            delta_q = std::stod(params["delta_q"]);
            epsilon_q = std::stod(params["epsilon_q"]);
            num_spikes = std::stoi(params["num_spikes"]);
            p_close_enough = std::stod(params["p_close_enough"]);
            probability_to_steer_to_target = std::stod(params["probability_to_steer_to_target"]);

            // // Load target poses
            // std::string targetPosesPath = params["target_poses"];
            // // std::cout << "target poses file: " << targetPosesPath << "\n";
            std::vector<Grasp> grasps = Grasp::LoadGrasps(targetPosesPath);
            std::vector<KDL::Frame> kdlFrames = Grasp::GraspsToFrames(grasps);

            // std::cout << "target poses size: " << kdlFrames.size() << "\n";
            // std::cout << "example of first grasp: \n"
            //           << kdlFrames[0] << "\n";

            // Assuming ConstructTreeFromTargets and GetMeanTranslation are already implemented
            ConstructTreeFromTargets(kdlFrames);
            GetMeanTranslation(kdlFrames);
        }

        std::string toString() const override
        {
            std::ostringstream oss;
            // Call toString of parent class
            oss << RbtParameters::toString();
            oss << ", p_close_enough: " << p_close_enough
                << ", probability_to_steer_to_target: " << probability_to_steer_to_target
                // Include additional information as needed, for example:
                << ", mean_target: [" << mean_target.x() << ", " << mean_target.y() << ", " << mean_target.z() << "]"
                << ", poses: " << target_poses->GetNumberOfNodes();
            // If you want to include information about target_poses, you need to decide how to represent it as a string
            return oss.str();
        }
    };
}
#endif