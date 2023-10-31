#include "bur_env.h"
#include "bur_algorithm.h"
#include "bur_tree.h"
#include <flann/flann.hpp>
#include <Eigen/Dense>
#include <memory>
#include <iostream>
#include "bur_funcs.h"

namespace Burs
{
    using namespace Eigen;

    BurAlgorithm::BurAlgorithm(int q_dim, ForwardKinematics f, int num_distal_points, int max_iters, double d_crit, double delta_q, double epsilon_q, MatrixXd bounds, RadiusFunc radius_func, int num_spikes)
        : q_dim(q_dim), forwardKinematics(f), num_distal_points(num_distal_points), max_iters(max_iters), d_crit(d_crit), delta_q(delta_q), epsilon_q(epsilon_q), bounds(bounds), radius_func(radius_func), num_spikes(num_spikes)
    {
    }

    BurAlgorithm::~BurAlgorithm()
    {
    }

    Vector3d BurAlgorithm::ForwardEuclideanJoint(const int &ith_distal_point, const VectorXd &configuration) const
    {
        return this->forwardKinematics(ith_distal_point, configuration);
    }

    double BurAlgorithm::RhoR(const VectorXd &q1, const VectorXd &q2) const
    {
        double max_distance = 0.0;
        auto fk = this->forwardKinematics;

        for (int i = 0; i < this->num_distal_points; i++)
        {
            double tmp = (fk(i, q1) - fk(i, q2)).norm();
            std::cout << " i: " << i << " q1: " << q1.transpose() << " => " << fk(i, q1).transpose() << " q2: " << q2.transpose() << " => " << fk(i, q2).transpose() << std::endl;
            if (tmp > max_distance)
            {
                max_distance = tmp;
            }
        }
        return max_distance;
    }

    double BurAlgorithm::GetPhiFunction(const double &d_closest, const VectorXd &q, const VectorXd &q_e, const double &t) const
    {
        return d_closest - RhoR(q, q + t * (q_e - q));
    }

    // typename Burs::PhiFunc BurAlgorithm::GetPhiFunction(const double &d_closest, const VectorXd &q, const VectorXd &q_e) const
    // {
    //     PhiFunc phi_func = [=](double t) -> double
    //     {
    //         return d_closest - RhoR(q, q + t * (q_e - q));
    //     };
    //     return phi_func;
    // }

    double BurAlgorithm::GetDeltaTk(double phi_tk, double tk, const VectorXd &q_e, const VectorXd &q_k) const
    {
        double numerator_sum = 0;

        VectorXd r_vec(this->q_dim);

        for (int i = 0; i < this->q_dim; i++)
        {
            r_vec[i] = this->radius_func(i, q_k);
        }

        return phi_tk * (1 - tk) / (r_vec.transpose() * (q_e - q_k).cwiseAbs());
    }

    MatrixXd BurAlgorithm::GetRandomQ(const int &num_spikes) const
    {
        MatrixXd m = MatrixXd::Random(this->q_dim, num_spikes);
        m.array() += 1.0; // Using .array() allows element-wise addition
        m.array() /= 2.0;

        for (int i = 0; i < this->q_dim; i++)
        {
            double range = this->bounds(i, 1) - this->bounds(i, 0);
            m.row(i).array() *= range;
            m.row(i).array() += this->bounds(i, 0); // Element-wise addition
        }
        return m;
    }

    VectorXd BurAlgorithm::GetEndpoint(const VectorXd &q_ei, const VectorXd &q_near, double factor) const
    {
        return q_near + factor * (q_ei - q_near).normalized();
    }

    void BurAlgorithm::GetEndpoints(MatrixXd &Qe, const VectorXd &q_near, double factor) const
    {
        // MatrixXd normalized_Qe = Qe; // Create a copy of Qe to store the normalized results
        for (int j = 0; j < Qe.cols(); ++j)
        {
            Qe.col(j) = GetEndpoint(Qe.col(j), q_near, factor);
        }
    }

    std::vector<VectorXd> BurAlgorithm::RbtConnect(const VectorXd &q_start, const VectorXd &q_goal)
    {
        // start of actual algorithm
        std::shared_ptr<BurTree> t_start = std::make_shared<BurTree>(q_start, q_start.rows());
        std::shared_ptr<BurTree> t_goal = std::make_shared<BurTree>(q_goal, q_goal.rows());
        auto t_a = t_start;
        auto t_b = t_goal;

        for (int k = 0; k < this->max_iters; k++)
        {
            VectorXd q_new(this->q_dim);
            Eigen::MatrixXd Qe = this->GetRandomQ(num_spikes);

            // std::cout << "Qe: " << Qe << std::endl;

            // random growth direction; can be any other among the random vectors from Qe
            VectorXd q_e_0 = Qe.col(0);
            // int nearest_index = current_tree->Nearest(q_e_0.data());
            int nearest_index = this->NearestIndex(t_a, q_e_0);
            // std::cout << "nearest_index: " << nearest_index << std::endl;

            const VectorXd q_near = t_a->GetQ(nearest_index);
            // std::cout << "q_near: " << q_near.transpose() << std::endl;

            for (int i = 0; i < num_spikes; i++)
            {
                VectorXd q_e_i = Qe.col(i);
                q_e_i = this->GetEndpoint(q_e_i, q_near, this->delta_q);
                Qe.col(i).array() = q_e_i;
                // std::cout << "q_e_" << i << ": " << q_e_i.transpose() << std::endl;
            }

            double d_closest = this->GetClosestDistance(q_near);
            std::cout << "d_closest: " << d_closest << std::endl;

            if (d_closest < this->d_crit)
            {
                std::cout << "d < d_crit" << std::endl;
                // q_new from above, will be used as the new endpoint for BurConnect
                q_new = this->GetEndpoint(q_e_0, q_near, this->epsilon_q);
                std::cout << "q_new: " << q_new.transpose() << std::endl;

                if (!this->IsColliding(q_new))
                {
                    t_a->AddNode(nearest_index, q_new);
                }
                else
                {
                    continue;
                }
            }
            else
            {
                Bur b = this->GetBur(q_near, Qe, d_closest);
                std::cout << "Bur center " << b.center.transpose() << std::endl;
                std::cout << "Bur endpoints " << b.endpoints.transpose() << std::endl;

                for (int i = 0; i < Qe.cols(); ++i)
                {
                    t_a->AddNode(nearest_index, b.endpoints.col(i));
                }
                // doesn't matter which column, since they all go in random directions
                q_new = Qe.col(0);
            }

            // if small basic rrt collides, then don't go here `continue`

            // if reached, then index is the closest node in `t_b` to `q_new` in `t_a`
            auto [status, b_closest] = this->BurConnect(t_b, q_new);
            if (status == AlgorithmState::Reached)
            {
                // TODO: closest node in t_a to q_new, closest node in t_b to q_new
                int a_closest = t_a->Nearest(q_new.data());
                return this->Path(t_a, a_closest, t_b, b_closest);
            }

            std::swap(t_a, t_b);
        }
        return std::vector<VectorXd>();
    }

    std::pair<AlgorithmState, int> BurAlgorithm::BurConnect(std::shared_ptr<BurTree> t, VectorXd &q)
    {
        int nearest_index = t->Nearest(q.data());

        VectorXd q_n = t->GetQ(nearest_index);
        VectorXd q_0(q_n);

        double delta_s = 1e14;
        double threshold = 1e-2;

        while (delta_s >= this->d_crit)
        {
            double d_closest = this->GetClosestDistance(q_n);

            if (d_closest > this->d_crit)
            {
                // if q_n is within the collision free bur of q, then we finish, game over
                Bur b = this->GetBur(q_n, q, d_closest);

                VectorXd q_t = b.endpoints.col(0);

                double delta_s = (q_t - q_n).norm();

                q_n = q_t;

                if (q_n.isApprox(q, threshold))
                {
                    return {AlgorithmState::Reached, nearest_index};
                }
            }
            else
            {
                VectorXd q_t = this->GetEndpoint(q, q_n, this->epsilon_q);

                // if not colliding then proceed
                if (!this->IsColliding(q_t))
                {
                    q_n = q_t;
                }
                else
                {
                    return {AlgorithmState::Trapped, -1};
                }

                if ((q_n - q_0).norm() >= (q - q_0).norm())
                {
                    return {AlgorithmState::Reached, nearest_index};
                }
            }
        }
        return {AlgorithmState::Trapped, -1};
    }

    VectorXd BurAlgorithm::Nearest(std::shared_ptr<BurTree> t, VectorXd &q)
    {
        return t->GetQ(t->Nearest(q.data()));
    }

    int BurAlgorithm::NearestIndex(std::shared_ptr<BurTree> t, VectorXd &q)
    {
        return t->Nearest(q.data());
    }

    double BurAlgorithm::GetClosestDistance(const VectorXd &q)
    {
        this->bur_env->SetPoses(q);
        return this->bur_env->GetClosestDistance();
    }

    void BurAlgorithm::SetBurEnv(std::shared_ptr<BurEnv> bur_env)
    {
        this->bur_env = bur_env;
    }

    Bur BurAlgorithm::GetBur(const VectorXd &q_near, const MatrixXd &Q_e, double d_closest)
    {
        std::cout << "Calculating bur for " << q_near.transpose() << std::endl;
        double d_small = 0.1 * d_closest;
        MatrixXd endpoints = MatrixXd::Zero(this->q_dim, Q_e.cols());

        for (int i = 0; i < Q_e.cols(); ++i)
        {
            // Burs::PhiFunc phi_f = this->GetPhiFunction(d_closest, q_near, Q_e.col(i));
            double tk = 0;

            // always start out from the center
            VectorXd q_k(q_near);
            double phi_result = d_closest;

            // std::cout << "q_near: " << q_near.transpose() << std::endl;

            std::cout << "q_ei: " << Q_e.col(i).transpose() << std::endl;
            const VectorXd q_e = Q_e.col(i);
            while (phi_result > d_small)
            {
                std::cout << "phi_result: " << phi_result << " tk: " << tk;
                std::cout << " q_k: " << q_k.transpose() << std::endl;
                // CHECK: this is indeed PI away from q_near
                // d_closest - this->RhoR(q_near, q_k);
                // phi_result = this->GetPhiFunction(d_closest, q_near, Q_e.col(i), tk);
                // std::cout << " RhoR: " << this->RhoR(q_near, q_k) << std::endl;
                phi_result = d_closest - this->RhoR(q_near, q_k);
                double delta_tk = this->GetDeltaTk(phi_result, tk, q_e, q_k);
                tk = tk + delta_tk;
                if (tk > 1)
                {
                    std::cout << "tk exceeded 1: " << tk << std::endl;
                    q_k = q_e;
                    break;
                }
                q_k = q_near + tk * (q_e - q_near);
            }
            endpoints.col(i).array() = q_k;
        }
        Bur myBur(q_near, endpoints);
        return myBur;
    }

    bool BurAlgorithm::IsColliding(const VectorXd &q)
    {
        this->bur_env->SetPoses(q);
        return this->bur_env->IsColliding();
    }

    std::vector<VectorXd> BurAlgorithm::Path(std::shared_ptr<BurTree> t_a, int a_closest, std::shared_ptr<BurTree> t_b, int b_closest)
    {
        std::vector<int> res_a;
        std::vector<int> res_b;

        int node_id_a = a_closest;
        do
        {
            res_a.push_back(node_id_a);
            node_id_a = t_a->GetParentIdx(node_id_a);
        } while (node_id_a != -1);

        int node_id_b = b_closest;
        do
        {
            res_b.push_back(node_id_b);
            node_id_b = t_a->GetParentIdx(node_id_b);
        } while (node_id_b != -1);

        std::vector<VectorXd> final_path(res_a.size() + res_b.size());

        std::reverse(res_a.begin(), res_a.end());

        for (int i = 0; i < res_a.size(); ++i)
        {
            final_path.push_back(t_a->GetQ(res_a[i]));
        }

        for (int i = 0; i < res_b.size(); ++i)
        {
            final_path.push_back(t_b->GetQ(res_b[i]));
        }

        return final_path;
    }
}