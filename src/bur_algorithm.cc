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

    BurAlgorithm::BurAlgorithm(int q_dim, ForwardKinematics f, int num_distal_points, int max_iters, double d_crit, double delta_q, double epsilon_q, MatrixXd bounds, RadiusFunc radius_func)
        : q_dim(q_dim), forwardKinematics(f), num_distal_points(num_distal_points), max_iters(max_iters), d_crit(d_crit), delta_q(delta_q), epsilon_q(epsilon_q), bounds(bounds), radius_func(radius_func)
    {
    }

    BurAlgorithm::~BurAlgorithm()
    {
    }

    Vector3d BurAlgorithm::forward(const int &ith_distal_point, const VectorXd &configuration) const
    {
        return this->forwardKinematics(ith_distal_point, configuration);
    }

    double BurAlgorithm::Rho_r(const VectorXd &q1, const VectorXd &q2) const
    {
        double max_distance = 0.0;
        auto fk = this->forwardKinematics;

        for (int i = 0; i < this->num_distal_points; i++)
        {
            double tmp = (fk(i, q1) - fk(i, q2)).norm();
            if (tmp > max_distance)
            {
                max_distance = tmp;
            }
        }
        return max_distance;
    }

    typename Burs::PhiFunc BurAlgorithm::get_phi(const double &d_closest, const VectorXd &q, const VectorXd &q_e) const
    {
        PhiFunc phi_func = [=](double t) -> double
        {
            return d_closest - Rho_r(q, q + t * (q_e - q));
        };
        return phi_func;
    }

    double BurAlgorithm::tk_update(double phi_tk, double tk, const VectorXd &q_e, const VectorXd &q_k) const
    {
        double numerator_sum = 0;
        VectorXd r_vec(this->q_dim);
        for (int i = 0; i < this->q_dim; i++)
        {
            r_vec[i] = this->radius_func(i, q_k);
        }

        return phi_tk * (1 - tk) / (r_vec.transpose() * (q_e - q_k).cwiseAbs());
    }

    MatrixXd BurAlgorithm::get_Qe(const int &num_spikes) const
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

    VectorXd BurAlgorithm::normalize_to(const VectorXd &q_ei, const VectorXd &q_near, double factor) const
    {
        return q_near + factor * (q_ei - q_near).normalized();
    }

    void BurAlgorithm::apply_normalize_to(MatrixXd &Qe, const VectorXd &q_near, double factor) const
    {
        // MatrixXd normalized_Qe = Qe; // Create a copy of Qe to store the normalized results
        for (int j = 0; j < Qe.cols(); ++j)
        {
            Qe.col(j) = normalize_to(Qe.col(j), q_near, factor);
        }
    }

    std::vector<int> BurAlgorithm::rbt_connect(const VectorXd &q_start, const VectorXd &q_goal, const int &num_spikes, std::shared_ptr<BurEnv> bur_env)
    {
        // start of actual algorithm
        std::shared_ptr<BurTree> t_start = std::make_shared<BurTree>(q_start, q_start.rows());
        std::shared_ptr<BurTree> t_goal = std::make_shared<BurTree>(q_goal, q_goal.rows());

        auto current_tree = t_start;

        for (int k = 0; k < this->max_iters; k++)
        {
            Eigen::MatrixXd Qe = this->get_Qe(num_spikes);

            // std::cout << "Qe: " << Qe << std::endl;

            // random growth direction; can be any other among the random vectors from Qe
            VectorXd q_e_0 = Qe.col(0);
            int nearest_index = current_tree->nearest(q_e_0.data());
            // std::cout << "nearest_index: " << nearest_index << std::endl;

            const VectorXd q_near = current_tree->GetQ(nearest_index);
            // std::cout << "q_near: " << q_near.transpose() << std::endl;

            for (int i = 0; i < num_spikes; i++)
            {
                VectorXd q_e_i = Qe.col(i);
                q_e_i = this->normalize_to(q_e_i, q_near, this->delta_q);
                Qe.col(i).array() = q_e_i;
                // std::cout << "q_e_" << i << ": " << q_e_i.transpose() << std::endl;
            }

            bur_env->SetPoses(q_near);
            // TODO: get closest distance
            double d_closest = bur_env->GetClosestDistance();
            std::cout << "d_closest: " << d_closest << std::endl;

            if (d_closest < this->d_crit)
            {
                std::cout << "d < d_crit" << std::endl;
                VectorXd q_new = q_near + this->normalize_to(q_e_0, q_near, this->epsilon_q);
                std::cout << "q_new: " << q_new.transpose() << std::endl;
                bur_env->SetPoses(q_new);

                if (!bur_env->IsColliding())
                {
                    current_tree->add_node(nearest_index, q_new);
                }
            }
            else
            {
                // TODO: add bur to tree
                // Bur = endpoints that iterate phi to zero
                double d_small = 0.1 * d_closest;
                MatrixXd endpoints = MatrixXd::Zero(this->q_dim, num_spikes);

                for (int i = 0; i < num_spikes; ++i)
                {
                    Burs::PhiFunc phi_f = this->get_phi(d_closest, q_near, Qe.col(i));
                    double tk = 0;

                    // always start out from the center
                    VectorXd q_k(q_near);
                    double phi_result = d_closest;

                    std::cout << "q_near: " << q_near.transpose() << std::endl;

                    while (phi_result > d_small)
                    {
                        // CHECK: this is indeed PI away from q_near
                        VectorXd q_e = Qe.col(i);
                        // std::cout << "q_e: " << q_e.transpose() << std::endl;
                        // std::cout << "q_k: " << q_k.transpose() << std::endl;
                        std::cout << "delta q: " << (q_e - q_k).transpose() << std::endl;
                        phi_result = phi_f(tk);
                        std::cout << "phi_result: " << phi_result << std::endl;
                        std::cout << "tk: " << tk << std::endl;
                        double delta_tk = this->tk_update(phi_result, tk, q_e, q_k);
                        // std::cout << "delta_tk: " << delta_tk << std::endl;
                        tk = tk + delta_tk;
                        std::cout << "q_near: " << q_near << std::endl;
                        q_k = q_near + tk * (q_e - q_near);
                        std::cout << "after update: " << std::endl;
                        // std::cout << "q_k: " << q_k.transpose() << std::endl;
                        std::cout << "tk: " << tk << std::endl;
                        std::cout << "delta q: " << (q_e - q_k).transpose() << std::endl;
                        // std::cout << "exit" << std::endl;
                        // std::exit(1);
                    }
                    endpoints.col(i).array() = q_k;
                }
            }
        }

        return std::vector<int>({1, 1, 1});
    }
}