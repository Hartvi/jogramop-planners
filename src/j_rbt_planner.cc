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
        RS start_state = this->NewState(q_start);

        // Random numbers
        this->rng = std::make_shared<RandomNumberGenerator>(planner_parameters.seed, planner_parameters.target_poses.size() - 1);

        auto tree = std::make_shared<BurTree>(start_state, q_start.size());

        // To prevent uninitialized vectors in planner_parameters
        this->InitGraspClosestConfigs(planner_parameters, tree, 0);

        double totalNNtime = 0;
        // double totalAddTime = 0;
        double totalRunTime = 0;
        // double totalCollisionTime = 0;
        double totalGetClosestDistTime = 0;
        double totalCollideAndAddTime = 0;
        struct rusage gt1, gt2;
        getTime(&gt1);

        for (unsigned int k = 0; k < planner_parameters.max_iters; ++k)
        {
            // LOGGING
            if (k % 1000 == 0)
            {
                getTime(&gt2);
                totalRunTime = getTime(gt1, gt2);
                auto &best_pose = planner_parameters.target_poses[this->GetBestGrasp(planner_parameters)];
                std::cout << "iter: " << k << "/" << planner_parameters.max_iters << ", tree.size: " << tree->GetNumberOfNodes() << ", distToGoal: " << best_pose.best_dist << ", ";
                std::cout << ", p_close_enough: " << planner_parameters.p_close_enough << ", totalNNtime: " << totalNNtime << ", totalCollideAndAddTime: " << totalCollideAndAddTime
                          << ", totalGetClosestDistTime: " << totalGetClosestDistTime << ", totalRunTime: " << totalRunTime << "\n";
                std::cout.flush();
            }
            // END LOGGING

            MatrixXd Qe = this->GetRandomQ(planner_parameters.num_spikes);
            std::vector<RS> Qe_states = this->NewStates(Qe);

            struct rusage tt1, tt2;
            getTime(&tt1);
            // Random column
            int nearest_idx = tree->Nearest(Qe.col(0).data());
            getTime(&tt2);
            totalNNtime += getTime(tt1, tt2);

            RS *near_state = tree->Get(nearest_idx);
            // VectorXd q_near = tree->GetQ(nearest_idx);

            getTime(&tt1);
            // Slow => maybe in the future use FCL and somehow compile it because it had a ton of compilation errors and version mismatches
            double d_closest = this->GetClosestDistance(*near_state);
            getTime(&tt2);
            totalGetClosestDistTime += getTime(tt1, tt2);

            // SWITCH TO RRT IF OBSTACLE TOO CLOSE
            bool too_close = d_closest < planner_parameters.d_crit;
            unsigned int num_extensions = too_close ? 1 : planner_parameters.num_spikes;
            double distance_to_move = too_close ? planner_parameters.epsilon_q : d_closest;

            std::vector<RS> endpoints = too_close ? this->GetEndpoints(*near_state, {Qe_states[0]}, distance_to_move)
                                                  : this->GetEndpoints(*near_state, Qe_states, distance_to_move);
            // std::vector<RS> endpoints = this->GetEndpoints(near_state, Qe_states, d_closest);
            // If closest obstacle was too close => check collisions for the RRT step

            bool rrt_colliding = false;
            if (too_close)
            {
                for (unsigned int i = 0; i < endpoints.size(); ++i)
                {
                    if (this->IsColliding(endpoints[i]))
                    {
                        std::cout << "JRBT COLLIDED IN RRT RANDOM STEP\n";
                        // continue;
                        rrt_colliding = true;
                    }
                }
            }

            if (!rrt_colliding)
            {
                this->AddDenseBur(tree, nearest_idx, endpoints, planner_parameters);
            }
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
                    int best_idx = tree->Nearest(best_grasp.best_state);
                    // int best_idx = tree->Nearest(best_grasp.dv->v.data());
                    // Take measurements
                    plan_result.distance_to_goal = best_grasp.best_dist;
                    plan_result.num_iterations = k;
                    plan_result.tree_size = tree->GetNumberOfNodes();
                    plan_result.success = true;

                    // Return best path
                    auto path = this->ConstructPathFromTree(tree, best_idx);
                    if (planner_parameters.visualize_tree > 0)
                    {
                        this->tree_csv = this->TreePoints(tree, planner_parameters.visualize_tree);
                    }
                    return path;
                }
            }
        }

        // Get grasp with minimal distance
        unsigned int best_grasp_idx = this->GetBestGrasp(planner_parameters);
        Grasp best_grasp = planner_parameters.target_poses[best_grasp_idx];

        // Get idx in tree that leads to the best config
        int best_idx = tree->Nearest(best_grasp.best_state);
        // int best_idx = tree->Nearest(best_grasp.dv->v.data());
        // Take measurements
        plan_result.distance_to_goal = best_grasp.best_dist;
        // plan_result.distance_to_goal = best_grasp.dv->d;
        plan_result.num_iterations = planner_parameters.max_iters;
        plan_result.tree_size = tree->GetNumberOfNodes();
        plan_result.success = false;

        // Return best path
        auto path = this->ConstructPathFromTree(tree, best_idx);
        if (planner_parameters.visualize_tree)
        {
            this->tree_csv = this->TreePoints(tree, planner_parameters.visualize_tree);
        }
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
            int best_state_idx = best_grasp.best_state;
            RS *best_state = tree->Get(best_state_idx);

            // VectorXd q_near = best_grasp.dv->v;

            // STEP 2.
            int idx_near = tree->Nearest(best_state_idx);
            // int idx_near = tree->Nearest(q_near.data());
            delta_p = best_grasp.best_dist;
            double best_dist = best_grasp.best_dist;
            // std::cout << "best dist: " << best_dist << "\n";
            // std::cout << "close enough: " << planner_parameters.p_close_enough << " delta_p: " << delta_p << " \n";

            // Get closest obstacle distance
            // std::cout << "best state: " << this->env->robot->GetEEFrame(*best_state).p << "\n";
            closest_dist = this->GetClosestDistance(*best_state);

            // SWITCH TO RRT IF OBSTACLE TOO CLOSE
            // bool too_close = false;
            bool too_close = closest_dist < planner_parameters.d_crit;
            unsigned int num_extensions = too_close ? 1 : planner_parameters.num_spikes;
            double distance_to_move = too_close ? planner_parameters.epsilon_q : closest_dist;
            std::cout << "closest dist: " << closest_dist << " dist to move: " << distance_to_move << "\n";

            // closest_dist = this->GetClosestDistance(q_near);

            // Target configs to extend to gained from the jacobian
            MatrixXd target_configs = MatrixXd(this->q_dim, num_extensions);

            for (unsigned int i = 0; i < num_extensions; ++i)
            {
                // Max dist => closest_dist / dist to goal (maybe better to have distance to goal since it can be farther)
                KDL::Frame tgt_frame = tgt_grasps[i].frame;
                KDL::Frame cur_frame = this->env->robot->GetEEFrame(*best_state);
                // KDL::Frame cur_frame = best_grasp.best_frame;

                // From cur_frame (best config)
                // To tgt_frame (one of the randomly chosen goals)
                // Move `closest_dist` along that direction
                // Can be farther that the goal, but that's fine because we interpolate using `Densify`
                KDL::Twist twist = this->GetTwist(tgt_frame, cur_frame, distance_to_move);

                // TODO: reuse the jacobian for the pseudo-inverse J+
                KDL::JntArray q_dot = this->env->robot->ForwardJPlus(*best_state, twist);
                VectorXd delta_q = q_dot.data;
                target_configs.col(i) = best_state->config + delta_q;
            }

            std::vector<RS> target_states = this->NewStates(target_configs);
            // DEBUG:
            auto eeframe = this->env->robot->GetEEFrame(target_states[0]);

            // END DEBUG
            // Iterate max `closest_dist` to `target_config`
            std::vector<RS> bur_endpoints = this->GetEndpoints(*best_state, target_states, distance_to_move);
            // std::vector<double> distances(bur_endpoints.size());
            for (unsigned int i = 0; i < bur_endpoints.size(); ++i)
            {
                double tmpdist = this->env->robot->MaxDistance(*best_state, bur_endpoints[i]);
                std::cout << "max dist " << i << " in extend: " << tmpdist << "\n";
            }

            // If closest obstacle was too close => check collisions for the RRT step
            if (too_close)
            {
                for (unsigned int i = 0; i < bur_endpoints.size(); ++i)
                {
                    if (this->IsColliding(bur_endpoints[i]))
                    {
                        std::cout << "JRBT COLLIDED IN RRT EXTEND\n";
                        // break;
                        return AlgorithmState::Trapped;
                    }
                }
            }

            // Checks bounds
            // Interpolates with max resolution_q distance between points
            // Updates best config for each grasp
            this->AddDenseBur(tree, idx_near, bur_endpoints, planner_parameters);

        } while (delta_p > planner_parameters.p_close_enough && closest_dist > planner_parameters.d_crit);

        // std::cout << "delta_p: " << delta_p << "\n";
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