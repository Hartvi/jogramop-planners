#include "rrt_planner.h"
#include "collision_env.h"

namespace Burs
{
    using namespace Eigen;

    RRTPlanner::RRTPlanner(std::string path_to_urdf_file)
        : BasePlanner(path_to_urdf_file)
    {
        auto my_env = this->GetEnv<URDFEnv>();
        auto my_robot = my_env->myURDFRobot;

        this->forwardKinematicsParallel = my_robot->GetForwardPointParallelFunc();
        this->radius_func = my_robot->GetRadiusFunc();
    }

    RRTPlanner::RRTPlanner() : BasePlanner()
    {
    }

    std::optional<std::vector<Eigen::VectorXd>>
    RRTPlanner::RRTConnect(const VectorXd &q_start, const VectorXd &q_goal, const RRTParameters &plan_parameters, PlanningResult &planning_result)
    {
        // start of actual algorithm
        std::shared_ptr<BurTree> t_start = std::make_shared<BurTree>(q_start, q_start.rows());
        std::shared_ptr<BurTree> t_goal = std::make_shared<BurTree>(q_goal, q_goal.rows());
        auto t_a = t_start;
        auto t_b = t_goal;
        // TODO: check collision at the beginning

        auto env = this->GetEnv<URDFEnv>();
        KDL::ChainFkSolverPos_recursive fk_solver(env->myURDFRobot->kdl_chain);
        planning_result.distance_to_goal = 1e10;
        KDL::Frame p_out;
        KDL::Vector p_goal;
        KDL::JntArray q_kdl(this->q_dim);
        Eigen::VectorXd best_q;

        q_kdl.data = q_goal;
        if (fk_solver.JntToCart(q_kdl, p_out) >= 0)
        {
            p_goal = p_out.p;
        }
        else
        {
            throw std::runtime_error("RRTConnect: couldn't perform forward kinematics for goal position");
        }
        std::cout << "goal pos: " << p_goal << "\n";

        q_kdl.data = q_start;
        std::cout << "qkdl data: " << q_kdl.data.transpose() << " p_pout: " << p_out << "\n";
        if (fk_solver.JntToCart(q_kdl, p_out) >= 0)
        {
            double new_distance = (p_out.p - p_goal).Norm();
            if (planning_result.distance_to_goal > new_distance)
            {
                planning_result.distance_to_goal = new_distance;
                best_q = q_start;
            }
        }
        else
        {
            throw std::runtime_error("RRTConnect: couldn't perform forward kinematics for start position");
        }

        for (int k = 0; k < plan_parameters.max_iters; k++)
        {
            VectorXd q_new(this->q_dim);
            /*
            for i in 1:N:
                Qe |= random_config()
            */
            Eigen::MatrixXd Qe = this->GetRandomQ(1);

            // random growth direction; can be any other among the random vectors from Qe
            VectorXd q_e_0 = Qe.col(0);
            // q_near <- NEAREST(q_{e1}, T_a)
            int nearest_index = this->NearestIndex(t_a, q_e_0);

            const VectorXd q_near = t_a->GetQ(nearest_index);

            // q_new from above, will be used as the new endpoint for BurConnect
            // small step RRT
            q_new = this->GetEndpoint(q_e_0, q_near, plan_parameters.epsilon_q);

            if (!this->IsColliding(q_new))
            {
                t_a->AddNode(nearest_index, q_new);
                Eigen::VectorXd q_rand = this->GetRandomQ(1);
                auto status_a = this->GreedyExtendRandomConfig(t_a, q_rand, plan_parameters);
                auto status_b = this->GreedyExtendRandomConfig(t_b, q_rand, plan_parameters);
                if (status_a == AlgorithmState::Reached && status_b == AlgorithmState::Reached)
                {
                    int a_closest = t_start->Nearest(q_rand.data());
                    int b_closest = t_goal->Nearest(q_rand.data());

                    planning_result.num_iterations = plan_parameters.max_iters;
                    planning_result.tree_size = t_start->GetNumberOfNodes() + t_goal->GetNumberOfNodes();
                    planning_result.success = true;
                    return this->Path(t_start, a_closest, t_goal, b_closest);
                }
            }
            else
            {
                // if small basic rrt collides, then don't continue, hence the `continue`
                continue;
            }

            q_kdl.data = q_new;
            if (fk_solver.JntToCart(q_kdl, p_out) >= 0)
            {
                double new_distance = (p_out.p - p_goal).Norm();
                if (planning_result.distance_to_goal > new_distance)
                {
                    planning_result.distance_to_goal = new_distance;
                    best_q = q_start;
                }
            }
            else
            {
                throw std::runtime_error("RRTConnect: couldn't perform forward kinematics for progress check");
            }

            std::swap(t_a, t_b);
        }

        planning_result.num_iterations = plan_parameters.max_iters;
        planning_result.tree_size = t_start->GetNumberOfNodes() + t_goal->GetNumberOfNodes();
        planning_result.success = false;

        int last_idx = t_start->Nearest(best_q.data());
        return this->ConstructPathFromTree(t_start, last_idx);
    }

    std::vector<Eigen::VectorXd>
    RRTPlanner::Path(std::shared_ptr<BurTree> t_a, int a_closest, std::shared_ptr<BurTree> t_b, int b_closest)
    {
        // std::cout << "PATH: " << std::endl;
        // std::cout << "A closest: " << t_a->GetQ(a_closest).transpose() << std::endl;
        // std::cout << "B closest: " << t_b->GetQ(b_closest).transpose() << std::endl;
        std::vector<int> res_a;
        std::vector<int> res_b;

        // connect the two path from the two trees, NODE B and NODE A to each tree's roots respectively
        int node_id_a = a_closest;
        do
        {
            res_a.push_back(node_id_a);
            node_id_a = t_a->GetParentIdx(node_id_a);
        } while (node_id_a != -1);

        // connect the two path from the two trees, NODE B and NODE A to each tree's roots respectively
        int node_id_b = b_closest;
        do
        {
            res_b.push_back(node_id_b);
            node_id_b = t_b->GetParentIdx(node_id_b);
        } while (node_id_b != -1);

        std::vector<Eigen::VectorXd> final_path(res_a.size() + res_b.size());

        std::reverse(res_a.begin(), res_a.end());

        int k = 0;
        for (int i = 0; i < res_a.size(); ++i)
        {
            // std::cout << "Qa: " << t_a->GetQ(res_a[i]) << std::endl;
            final_path[k] = t_a->GetQ(res_a[i]);
            ++k;
        }

        for (int i = 0; i < res_b.size(); ++i)
        {
            // std::cout << "Qb: " << t_b->GetQ(res_b[i]) << std::endl;
            final_path[k] = t_b->GetQ(res_b[i]);
            ++k;
        }

        // std::cout << "END PATH" << std::endl;
        return final_path;
    }

    AlgorithmState
    RRTPlanner::GreedyExtend(std::shared_ptr<BurTree> t_a, std::shared_ptr<BurTree> t_b, Eigen::VectorXd q_a, const RRTParameters &planner_parameters)
    {
        // Get closest point in tree b to point q_a from tree a
        int nearest_in_b = t_b->Nearest(q_a.data());
        Eigen::VectorXd closest_q = t_b->GetQ(nearest_in_b);

        Eigen::VectorXd new_q = this->GetEndpoint(closest_q, q_a, planner_parameters.epsilon_q);
        while (!this->IsColliding(new_q))
        {
            // Check if new point isn't too close to already existing points in the tree
            auto [n_a, d_a] = t_a->NearestIdxAndDist(new_q.data());
            // if (d_a > planner_parameters.epsilon_q)
            // {
            //     t_a->AddNode(n_a, new_q);
            // }
            // else
            // {
            //     new_q = t_a->GetQ(n_a);
            // }
            t_a->AddNode(n_a, new_q);
            // If close enough to target, finish
            if ((new_q - closest_q).norm() <= planner_parameters.epsilon_q)
            {
                return AlgorithmState::Reached;
            }
            new_q = this->GetEndpoint(closest_q, new_q, planner_parameters.epsilon_q);
        }
        return AlgorithmState::Trapped;
    }

    AlgorithmState
    RRTPlanner::GreedyExtendRandomConfig(std::shared_ptr<BurTree> t_a, Eigen::VectorXd closest_q, const RRTParameters &planner_parameters)
    {
        int nearest_in_a = t_a->Nearest(closest_q.data());
        Eigen::VectorXd q_a = t_a->GetQ(nearest_in_a);

        Eigen::VectorXd new_q = this->GetEndpoint(closest_q, q_a, planner_parameters.epsilon_q);
        double smaller_epsilon = 99 * planner_parameters.epsilon_q;
        while (!this->IsColliding(new_q))
        {

            auto [n_a, d_a] = t_a->NearestIdxAndDist(new_q.data());
            t_a->AddNode(n_a, new_q);
            // // Check if new point isn't too close to already existing points in the tree
            // auto [n_a, d_a] = t_a->NearestIdxAndDistSqr(new_q.data());

            // // if >= epsilon then it will never add it
            // if (d_a > smaller_epsilon)
            // {
            //     t_a->AddNode(n_a, new_q);
            // }
            // else // d_a <= epsilon => with step size epsilon_q it would never add it
            // {
            //     new_q = t_a->GetQ(n_a);
            // }
            // If close enough to target, finish
            if ((new_q - closest_q).norm() <= planner_parameters.epsilon_q)
            {
                return AlgorithmState::Reached;
            }
            // Do not have to check if it's out of bounds because we are going towards a point that is inbounds and we will end if we reach it
            new_q = this->GetEndpoint(closest_q, new_q, planner_parameters.epsilon_q);
            std::cout << "new_q: " << new_q.transpose() << "\n";
        }
        return AlgorithmState::Trapped;
    }
}