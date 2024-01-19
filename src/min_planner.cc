#include "burs.h"
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

    VectorXd
    MinPlanner::GetEndpoint(const VectorXd &q_ei, const VectorXd &q_near, double factor) const
    {
        return q_near + factor * (q_ei - q_near).normalized();
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
