
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
    class JPlusRbtParameters : public RbtParameters
    {
    public:
        // int preheat_type;
        // bool use_platform = true;

        int use_rotation;
        double rotation_dist_ratio;

        // int bias_calculation_type;

        // double preheat_ratio;
        double p_close_enough;
        double probability_to_steer_to_target;

        std::vector<Grasp> target_poses;

        Eigen::Vector3d mean_target;
        // double goal_bias_radius;
        // double goal_bias_probability;

        void
        GetMeanTranslation(std::vector<Grasp> &target_poses)
        {
            Eigen::Vector3d mean_vector(0, 0, 0);
            for (int i = 0; i < target_poses.size(); ++i)
            {
                Eigen::Vector3d v(target_poses[i].frame.p.x(), target_poses[i].frame.p.y(), target_poses[i].frame.p.z());
                mean_vector += v;
            }
            mean_vector /= target_poses.size();
            this->mean_target = mean_vector;
        }

        // std::shared_ptr<BurTree>
        // ConstructTreeFromTargets(std::vector<Grasp> &target_poses)
        // {
        //     // TODO: add rotation
        //     // Eigen::VectorXd root_node(/*load translation rotation vector*/);
        //     Eigen::Vector3d root_node(target_poses[0].frame.p.x(), target_poses[0].frame.p.y(), target_poses[0].frame.p.z());

        //     std::shared_ptr<BurTree> t = std::make_shared<BurTree>(root_node, root_node.size());

        //     for (int i = 1; i < target_poses.size(); ++i)
        //     {
        //         Eigen::Vector3d new_node(target_poses[i].frame.p.x(), target_poses[i].frame.p.y(), target_poses[i].frame.p.z());

        //         t->AddNode(i - 1, new_node);
        //     }
        //     // this->target_poses = t;
        //     return t;
        // }
        JPlusRbtParameters() = default;

        JPlusRbtParameters(int max_iters,
                           double d_crit,
                           double delta_q,
                           double epsilon_q,
                           int num_spikes,
                           double p_close_enough,
                           double probability_to_steer_to_target,
                           std::vector<Grasp> target_poses_input,
                           double q_resolution)

            : RbtParameters(max_iters, d_crit, delta_q, epsilon_q, num_spikes, q_resolution),
              p_close_enough(p_close_enough), probability_to_steer_to_target(probability_to_steer_to_target)
        {
            if (!target_poses_input.empty())
            {
                // ConstructTreeFromTargets(target_poses_input);
                GetMeanTranslation(target_poses_input);
                this->target_poses = target_poses_input;
            }
            else
            {
                throw std::runtime_error("JPlusRbtParameters: target_poses_input is empty!!!");
            }
        }

        std::string
        toString() const override
        {
            std::ostringstream oss;
            // Call toString of parent class
            oss << RbtParameters::toString();
            oss << ", p_close_enough: " << p_close_enough
                << ", probability_to_steer_to_target: " << probability_to_steer_to_target
                // Include additional information as needed, for example:
                << ", mean_target: [" << mean_target.x() << ", " << mean_target.y() << ", " << mean_target.z() << "]"
                << ", poses: " << target_poses.size();
            // If you want to include information about target_poses, you need to decide how to represent it as a string
            return oss.str();
        }
    };
}
#endif
