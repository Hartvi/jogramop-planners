#include <flann/flann.hpp>
#include <Eigen/Dense>
#include <memory>
#include "bur_env.h"
#include "bur_funcs.h"

#ifndef BUR_ALGORITHM_H
#define BUR_ALGORITHM_H

namespace Burs
{
    using namespace Eigen;

    class BurAlgorithm
    {
    public:
        BurAlgorithm(int q_dim, ForwardKinematics f, int num_distal_points, int max_iters, double d_crit, double delta_q, double epsilon_q, MatrixXd bounds, RadiusFunc radius_func);

        ~BurAlgorithm();

        /// @brief maximum distance of any segment's movement between configuration points q1 and q2
        /// \rho_R(q_1, q_2) = \max_i |f_{p_i}(q_1) - f_{p_i}(q_2)|
        double Rho_r(const VectorXd &q1, const VectorXd &q2) const;

        /// @brief distance to closest obstacle
        /// \psi(t) = d_c - \rho_R(q, q + t(q_e - q))
        PhiFunc get_phi(const double &d_closest, const VectorXd &q, const VectorXd &q_e) const;

        /// @brief parameter update to iterate to edge configuration
        // t_{k+1} = t_k + \frac{\psi(t_k)}{\sum_{i=1}^n r_i(t_k)|q_e_i - q_{k_i}|(1 - t_k)}
        double tk_update(double phi_tk, double tk, const VectorXd &q_e, const VectorXd &q_k) const;

        /// @brief Get set of random configurations
        MatrixXd get_Qe(const int &num_spikes) const;

        void apply_normalize_to(MatrixXd &Qe, const VectorXd &q_near, double factor) const;

        /// @brief normalize spine to some length
        // q_{e_i} \leftarrow q_{near} + \delta \frac{q_{e_i} - q_{near}}{\| q_{e_i} - q_{near} \|}
        VectorXd normalize_to(const VectorXd &q_ei, const VectorXd &q_near, double factor) const;

        /// @brief Plan path using two opposing trees
        std::vector<int> rbt_connect(const VectorXd &q_start, const VectorXd &q_goal, const int &num_spikes, std::shared_ptr<BurEnv> bur_env);

        double get_dc();

        Vector3d forward(const int &ith_distal_point, const VectorXd &configuration) const;

    private:
        RadiusFunc radius_func;
        ForwardKinematics forwardKinematics;
        int q_dim;
        MatrixXd bounds;
        int num_distal_points;
        int max_iters;
        double d_crit;
        double delta_q;
        double epsilon_q;
    };

}

#endif