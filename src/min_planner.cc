#include "min_planner.h"
#include <flann/flann.hpp>
#include <Eigen/Dense>
#include <memory>
#include <iostream>
#include <fstream>
#include <vector>

namespace Burs
{
    using namespace Eigen;

    MinPlanner::MinPlanner(int q_dim, MatrixXd bounds)
        : q_dim(q_dim), bounds(bounds)
    {
    }

    MinPlanner::MinPlanner()
    {
    }

    bool
    MinPlanner::InBounds(const VectorXd &q) const
    {
        for (unsigned int i = 0; i < q.size(); ++i)
        {
            // std::cout << "bounds (rows, cols): " << this->bounds.rows() << ", " << this->bounds.cols() << "\n";
            if (this->bounds(i, 0) > q(i) || q(i) > this->bounds(i, 1))
            {
                return false;
            }
        }
        return true;
    }

    MatrixXd
    MinPlanner::GetRandomQ(const int &num_spikes) const
    {
        MatrixXd m = MatrixXd::Random(this->q_dim, num_spikes);
        m.array() += 1.0; // Using .array() allows element-wise addition
        m.array() *= 0.5;

        for (int i = 0; i < this->q_dim; i++)
        {
            double range = this->bounds(i, 1) - this->bounds(i, 0);
            m.row(i).array() *= range;
            m.row(i).array() += this->bounds(i, 0); // Element-wise addition
        }
        return m;
    }

    // VectorXd
    // MinPlanner::Nearest(std::shared_ptr<BurTree> t, VectorXd &q)
    // {
    //     return t->GetQ(t->Nearest(q.data()));
    // }

    // int
    // MinPlanner::NearestIndex(std::shared_ptr<BurTree> t, VectorXd &q)
    // {
    //     return t->Nearest(q.data());
    // }

    std::pair<int, std::vector<double>>
    MinPlanner::GetClosestDistances(const RS &state) const
    {
        this->env->SetPoses(state);
        return this->env->GetClosestDistances();
    }

    double
    MinPlanner::GetClosestDistance(const RS &state) const
    {
        this->env->SetPoses(state);
        return this->env->GetClosestDistance();
    }

    void
    MinPlanner::SetEnv(std::shared_ptr<BaseEnv> env)
    {
        this->env = env;
    }

    bool
    MinPlanner::IsColliding(const RS &state) const
    {
        this->env->SetPoses(state);
        for (unsigned int i = 4; i < state.frames.size(); ++i)
        {
            if (state.frames[i].p.z() < 0.04)
            {
                return true;
            }
        }
        return this->env->IsColliding();
    }

    std::vector<VectorXd>
    MinPlanner::ConstructPathFromTree(std::shared_ptr<BurTree> q_tree, int final_node_id)
    {
        std::vector<VectorXd> res_a;

        // connect the two path from the two trees, NODE B and NODE A to each tree's roots respectively
        int node_id_a = final_node_id;
        do
        {
            res_a.push_back(q_tree->Get(node_id_a)->config);
            node_id_a = q_tree->GetParentIdx(node_id_a);
        } while (node_id_a != -1);

        std::reverse(res_a.begin(), res_a.end());

        return res_a;
    }

    std::vector<VectorXd>
    MinPlanner::Path(std::shared_ptr<BurTree> t_a, int a_closest, std::shared_ptr<BurTree> t_b, int b_closest)
    {
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
            // std::cout << "node:\n"
            //           << t_b->GetQ(node_id_b).transpose() << "\n\n";
            node_id_b = t_b->GetParentIdx(node_id_b);
        } while (node_id_b != -1);

        std::vector<VectorXd> final_path(res_a.size() + res_b.size());

        std::reverse(res_a.begin(), res_a.end());

        int k = 0;
        for (int i = 0; i < res_a.size(); ++i)
        {
            final_path[k] = t_a->Get(res_a[i])->config;
            ++k;
        }

        for (int i = 0; i < res_b.size(); ++i)
        {
            final_path[k] = t_b->Get(res_b[i])->config;
            ++k;
        }

        return final_path;
    }

    RS
    MinPlanner::NewState(const VectorXd &q) const
    {
        return this->env->robot->FullFK(q);
    }

    std::vector<RS>
    MinPlanner::NewStates(const MatrixXd &Q) const
    {
        std::vector<RS> states;
        states.reserve(Q.cols());
        for (unsigned int i = 0; i < Q.cols(); ++i)
        {
            states.push_back(this->NewState(Q.col(i)));
        }
        return states;
    }
}
