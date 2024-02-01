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
        std::cout << "RUNNING JRBT\n";
        if (planner_parameters.target_poses.size() < 1)
        {
            throw std::runtime_error("Target poses has length 0!");
        }

        // To prevent uninitialized vectors in planner_parameters
        this->InitGraspClosestConfigs(planner_parameters, q_start);

        // Random numbers
        this->rng = std::make_shared<RandomNumberGenerator>(planner_parameters.seed, planner_parameters.target_poses.size() - 1);

        auto tree = std::make_shared<BurTree>(q_start, q_start.size());

        for (unsigned int k = 0; k < planner_parameters.max_iters; ++k)
        {
            std::cout << "iter: " << k << "\r";
            // RRT STEP
            // VectorXd q_rand = this->GetRandomQ(1);
            // int idx_near = tree->Nearest(q_rand.data());

            // int step_result = this->RRTStep(tree, idx_near, q_rand, planner_parameters.epsilon_q);
            // if (step_result >= 0)
            // {
            //     // Check distance to goal
            //     VectorXd q_new = tree->GetQ(step_result);
            //     auto ee_pose = this->GetEEPose(q_new);
            //     this->SetGraspClosestConfigs(planner_parameters, q_new);
            // }
            // RRT STEP

            MatrixXd Qe = this->GetRandomQ(planner_parameters.num_spikes);

            // Random column
            int nearest_idx = tree->Nearest(Qe.col(0).data());
            VectorXd q_near = tree->GetQ(nearest_idx);

            // Slow => maybe in the future use FCL and somehow compile it because it had a ton of compilation errors and version mismatches
            double d_closest = this->GetClosestDistance(q_near);
            MatrixXd endpoints = this->GetEndpoints(q_near, Qe, d_closest);
            this->AddDenseBur(tree, nearest_idx, endpoints, planner_parameters);
            // TRAVELLED DISTANCES ARE INDEED ALWAYS SMALLER THAN D_CLOSEST

            if (this->rng->getRandomReal() < planner_parameters.probability_to_steer_to_target)
            {
                // Steer until hit the target or obstacle or joint limits
                AlgorithmState state = this->ExtendToGoalRbt(tree, planner_parameters);

                if (state == AlgorithmState::Reached)
                {
                    // Get grasp with minimal distance
                    unsigned int best_grasp_idx = this->GetBestGrasp(planner_parameters);
                    Grasp best_grasp = planner_parameters.target_poses[best_grasp_idx];
                    // Get idx in tree that leads to the best config
                    int best_idx = tree->Nearest(best_grasp.dv->v.data());
                    // Take measurements
                    plan_result.distance_to_goal = best_grasp.dv->d;
                    plan_result.num_iterations = k;
                    plan_result.tree_size = tree->GetNumberOfNodes();
                    plan_result.success = true;

                    // Return best path
                    auto path = this->ConstructPathFromTree(tree, best_idx);
                    return path;
                }
            }
        }

        // Get grasp with minimal distance
        unsigned int best_grasp_idx = this->GetBestGrasp(planner_parameters);
        Grasp best_grasp = planner_parameters.target_poses[best_grasp_idx];
        // Get idx in tree that leads to the best config
        int best_idx = tree->Nearest(best_grasp.dv->v.data());
        // Take measurements
        plan_result.distance_to_goal = best_grasp.dv->d;
        plan_result.num_iterations = planner_parameters.max_iters;
        plan_result.tree_size = tree->GetNumberOfNodes();
        plan_result.success = false;

        // Return best path
        auto path = this->ConstructPathFromTree(tree, best_idx);
        return path;
    }

    std::vector<Grasp>
    JRbtPlanner::GetBestAndRandomGrasps(JPlusRbtParameters &planner_parameters) const
    {
        std::vector<Grasp> grasps(planner_parameters.num_spikes);
        // Get best grasp IDX
        unsigned int best_grasp_idx = this->GetBestGrasp(planner_parameters);
        Grasp best_grasp = planner_parameters.target_poses[best_grasp_idx];
        grasps[0] = best_grasp;

        // Get shuffled integer vector
        auto non_repeating_ints = this->rng->getNonRepeatingInts();
        for (unsigned int i = 1; i < planner_parameters.num_spikes; ++i)
        {
            // i-th element from shuffled vector
            unsigned int rand_int = *std::next(non_repeating_ints, i);
            if (rand_int == best_grasp_idx)
            {
                // rand_int == best_idx => choose index 0 because we started at "i = 1"
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
    JRbtPlanner::ExtendToGoalRbt(std::shared_ptr<BurTree> tree, JPlusRbtParameters &planner_parameters) const
    {
        /* OUTLINE:
        1. Get best grasp's best configuration
        2. From that best configuration expand towards the grasp and some random grasps
        3. Go to 1.
        */
        double delta_p = 1e14;
        double closest_dist = 1e14;
        do
        {
            // STEP 1.
            // Best grasp is in index 0
            std::vector<Grasp> tgt_grasps = this->GetBestAndRandomGrasps(planner_parameters);
            Grasp best_grasp = tgt_grasps[0];
            VectorXd q_near = best_grasp.dv->v;

            // STEP 2.
            int idx_near = tree->Nearest(q_near.data());
            delta_p = best_grasp.dv->d;

            // Get closest obstacle distance
            closest_dist = this->GetClosestDistance(q_near);

            // Target configs to extend to gained from the jacobian
            MatrixXd target_configs = MatrixXd(this->q_dim, planner_parameters.num_spikes);

            for (unsigned int i = 0; i < planner_parameters.num_spikes; ++i)
            {
                // Max dist => closest_dist / dist to goal (maybe better to have distance to goal since it can be farther)
                KDL::Frame tgt_frame = tgt_grasps[i].frame;
                KDL::Frame cur_frame = best_grasp.best_frame;

                // From cur_frame (best config)
                // To tgt_frame (one of the randomly chosen goals)
                // Move `closest_dist` along that direction
                // Can be farther that the goal, but that's fine because we interpolate using `Densify`
                KDL::Twist twist = this->GetTwist(tgt_frame, cur_frame, closest_dist);

                // TODO: reuse the jacobian for the pseudo-inverse J+
                KDL::JntArray q_dot = this->myRobot->ForwardJPlus(q_near, twist);
                VectorXd delta_q = q_dot.data;
                target_configs.col(i) = q_near + delta_q;
            }
            // Iterate max `closest_dist` to `target_config`
            MatrixXd bur_endpoints = this->GetEndpoints(q_near, target_configs, closest_dist);

            // Checks bounds
            // Interpolates with max resolution_q distance between points
            // Updates best config for each grasp
            this->AddDenseBur(tree, idx_near, bur_endpoints, planner_parameters);

        } while (delta_p > planner_parameters.p_close_enough && closest_dist > planner_parameters.epsilon_q);

        if (delta_p <= planner_parameters.p_close_enough)
        {
            return AlgorithmState::Reached;
        }
        else
        {
            return AlgorithmState::Trapped;
        }
    }

}