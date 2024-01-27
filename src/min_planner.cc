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

    Eigen::VectorXd
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

}
