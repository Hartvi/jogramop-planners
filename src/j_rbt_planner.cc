#include <string>
#include <sstream>
#include <vector>

#include "bur_funcs.h"
#include "j_rbt_planner.h"
#include "ut.h"

namespace Burs
{
    using namespace Eigen;

    JRbtPlanner::JRbtPlanner(std::string urdf_file) : JRRTPlanner(urdf_file)
    {
        // TODO: this planner needs to:
        //  - reuse forward-kinematics results
        //  - reuse the Jacobian from a single configuration

        // Idk how to random numbers in C++:

        this->rng = std::make_shared<RandomNumberGenerator>(1, 1); // temporary seed is one, the proper seed is set at the begiging of JPlusRbt
    }

    std::optional<std::vector<VectorXd>>
    JRbtPlanner::JRbt(const VectorXd &q_start, JPlusRbtParameters &planner_parameters, PlanningResult &plan_result)
    {
        // Setup rng:
        this->rng = std::make_shared<RandomNumberGenerator>(planner_parameters.seed, planner_parameters.target_poses.size() - 1);

        auto tree = std::make_shared<BurTree>(q_start, q_start.size());

        for (unsigned int k = 0; k < planner_parameters.max_iters; ++k)
        {
        }
        return {};
    }

    std::vector<Grasp>
    JRbtPlanner::GetBestAndRandomGrasps(JPlusRbtParameters &planner_parameters) const
    {
        std::vector<Grasp> grasps(planner_parameters.num_spikes);
        unsigned int best_grasp_idx = this->GetBestGrasp(planner_parameters);
        Grasp best_grasp = planner_parameters.target_poses[best_grasp_idx];
        grasps[0] = best_grasp;

        auto non_repeating_ints = this->rng->getNonRepeatingInts();
        for (unsigned int i = 1; i < planner_parameters.num_spikes; ++i)
        {
            // doesn't matter that it indexes at i=1..N since it's random anyway
            unsigned int rand_int = *std::next(non_repeating_ints, i);
            if (rand_int == best_grasp_idx)
            {
                // rand_int == best_idx => choose index 0 because we started at 1
                grasps[i] = planner_parameters.target_poses[0];
            }
            else
            {
                grasps[i] = planner_parameters.target_poses[rand_int];
            }
        }
        return grasps;
    }

    AlgorithmState
    JRbtPlanner::ExtendToGoalRbt(std::shared_ptr<BurTree> t_a, JPlusRbtParameters &planner_parameters) const
    {

        // Copy since we will change it

        // do
        // {
        //     VectorXd q_near(best_grasp.dv.v);
        //     int prev_idx = t_a->Nearest(q_near.data());
        //     double delta_p = best_grasp.dv.d;

        //     // Get closest obstacle distance
        //     double closest_dist = this->GetClosestDistance(q_near);

        //     // Target configs to extend to
        //     MatrixXd target_configs = MatrixXd(this->q_dim, planner_parameters.num_spikes); /* target configs are given from the jacobian */

        //     for (unsigned int i = 0; i < planner_parameters.num_spikes; ++i)
        //     {
        //         // Max dist => closest_dist / dist to goal (maybe better to have distance to goal since it can be farther)
        //         KDL::Frame tgt_frame = grasps[i].frame;
        //         KDL::Frame cur_frame = best_grasp;
        //         KDL::Twist twist = this->GetTwist(tgt_frame, p_near, delta_p);

        //         // TODO: reuse the jacobian for the pseudo-inverse J+
        //         KDL::JntArray q_dot = this->myRobot->ForwardJPlus(q_near, twist);
        //         VectorXd delta_q = q_dot.data;
        //         target_configs.col(i) = q_near + delta_q;
        //     }
        //     // Get random configs based on number of spikes
        //     // Jacobian => target configs
        //     MatrixXd bur_endpoints = this->GetEndpoints(q_near, target_configs, closest_dist);

        //     for (unsigned int i = 0; i < planner_parameters.num_spikes; ++i)
        //     {
        //         VectorXd q_new = bur_endpoints.col(i);
        //         if (!this->InBounds(q_near))
        //         {
        //             // Skip the bur endpoint if it's out of bounds
        //             continue;
        //         }
        //         prev_idx = t_a->AddNode(prev_idx, q_new);
        //         this->SetGraspClosestConfigs(planner_parameters, q_near);
        //     }
        // } while (delta_p > planner_parameters.p_close_enough);

        return AlgorithmState::Reached;
    }

}