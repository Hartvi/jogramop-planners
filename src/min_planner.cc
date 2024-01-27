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

    void
    MinPlanner::GetEndpoints(MatrixXd &Qe, const VectorXd &q_near, const double &factor) const
    {
        // MatrixXd normalized_Qe = Qe; // Create a copy of Qe to store the normalized results
        for (int j = 0; j < Qe.cols(); ++j)
        {
            Qe.col(j) = GetEndpoint(Qe.col(j), q_near, factor);
        }
    }

    VectorXd
    MinPlanner::GetEndpoint(const VectorXd &q_ei, const VectorXd &q_near, const double &factor) const
    {
        VectorXd diff = q_ei - q_near;
        double n = diff.norm();
        if (n < factor)
        {
            return q_ei;
        }
        return q_near + factor * diff / n;
    }

    VectorXd
    MinPlanner::Nearest(std::shared_ptr<BurTree> t, VectorXd &q)
    {
        return t->GetQ(t->Nearest(q.data()));
    }

    int
    MinPlanner::NearestIndex(std::shared_ptr<BurTree> t, VectorXd &q)
    {
        return t->Nearest(q.data());
    }

    double
    MinPlanner::GetClosestDistance(const VectorXd &q)
    {
        this->base_env->SetPoses(q);
        return this->base_env->GetClosestDistance();
    }

    void
    MinPlanner::SetEnv(std::shared_ptr<BaseEnv> base_env)
    {
        this->base_env = base_env;
    }

    bool
    MinPlanner::IsColliding(const VectorXd &q)
    {
        this->base_env->SetPoses(q);
        return this->base_env->IsColliding();
    }

    std::vector<VectorXd>
    MinPlanner::ConstructPathFromTree(std::shared_ptr<BurTree> q_tree, int final_node_id)
    {
        std::vector<VectorXd> res_a;

        // connect the two path from the two trees, NODE B and NODE A to each tree's roots respectively
        int node_id_a = final_node_id;
        do
        {
            res_a.push_back(q_tree->GetQ(node_id_a));
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
            node_id_b = t_b->GetParentIdx(node_id_b);
        } while (node_id_b != -1);

        std::vector<VectorXd> final_path(res_a.size() + res_b.size());

        std::reverse(res_a.begin(), res_a.end());

        int k = 0;
        for (int i = 0; i < res_a.size(); ++i)
        {
            final_path[k] = t_a->GetQ(res_a[i]);
            ++k;
        }

        for (int i = 0; i < res_b.size(); ++i)
        {
            final_path[k] = t_b->GetQ(res_b[i]);
            ++k;
        }

        return final_path;
    }

}
