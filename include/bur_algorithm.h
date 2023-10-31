#include <flann/flann.hpp>
#include <Eigen/Dense>
#include <memory>
#include "bur_env.h"
#include "bur_funcs.h"
#include "bur_tree.h"

#ifndef BUR_ALGORITHM_H
#define BUR_ALGORITHM_H

namespace Burs
{
    using namespace Eigen;

    struct BurAlgorithmParameters
    {
        int q_dim;
        ForwardKinematics f;
        int num_distal_points;
        int max_iters;
        double d_crit;
        double delta_q;
        double epsilon_q;
        Eigen::MatrixXd bounds;
        RadiusFunc radius_func;
        int num_spikes;

        // Constructor
        BurAlgorithmParameters(
            int q_dim_,
            ForwardKinematics f_,
            int num_distal_points_,
            int max_iters_,
            double d_crit_,
            double delta_q_,
            double epsilon_q_,
            Eigen::MatrixXd bounds_,
            RadiusFunc radius_func_,
            int num_spikes_) : q_dim(q_dim_),
                               f(f_),
                               num_distal_points(num_distal_points_),
                               max_iters(max_iters_),
                               d_crit(d_crit_),
                               delta_q(delta_q_),
                               epsilon_q(epsilon_q_),
                               bounds(bounds_),
                               radius_func(radius_func_),
                               num_spikes(num_spikes_)
        {
        }
    };

    enum AlgorithmState
    {
        Reached,
        Trapped,
        Failure
    };

    class BurAlgorithm
    {
    public:
        BurAlgorithm(int q_dim, ForwardKinematics f, int num_distal_points, int max_iters, double d_crit, double delta_q, double epsilon_q, MatrixXd bounds, RadiusFunc radius_func, int num_spikes);

        ~BurAlgorithm();

        /// @brief maximum distance of any segment's movement between configuration points q1 and q2
        /// \rho_R(q_1, q_2) = \max_i |f_{p_i}(q_1) - f_{p_i}(q_2)|
        double RhoR(const VectorXd &q1, const VectorXd &q2) const;

        /// @brief distance to closest obstacle
        /// \psi(t) = d_c - \rho_R(q, q + t(q_e - q))
        // PhiFunc GetPhiFunction(const double &d_closest, const VectorXd &q, const VectorXd &q_e) const;
        double GetPhiFunction(const double &d_closest, const VectorXd &q, const VectorXd &q_e, const double &t) const;

        /// @brief parameter update to iterate to edge configuration
        // t_{k+1} = t_k + \frac{\psi(t_k)}{\sum_{i=1}^n r_i(t_k)|q_e_i - q_{k_i}|(1 - t_k)}
        double GetDeltaTk(double phi_tk, double tk, const VectorXd &q_e, const VectorXd &q_k) const;

        /// @brief Get set of random configurations
        MatrixXd GetRandomQ(const int &num_spikes) const;

        void GetEndpoints(MatrixXd &Qe, const VectorXd &q_near, double factor) const;

        /// @brief normalize spine to some length
        // q_{e_i} \leftarrow q_{near} + \delta \frac{q_{e_i} - q_{near}}{\| q_{e_i} - q_{near} \|}
        VectorXd GetEndpoint(const VectorXd &q_ei, const VectorXd &q_near, double factor) const;

        /// @brief Plan path using two opposing trees
        std::vector<VectorXd> RbtConnect(const VectorXd &q_start, const VectorXd &q_goal);

        Vector3d ForwardEuclideanJoint(const int &ith_distal_point, const VectorXd &configuration) const;

        void SetBurEnv(std::shared_ptr<BurEnv> bur_env);

    private:
        std::vector<VectorXd> Path(std::shared_ptr<BurTree> t_a, int a_closest, std::shared_ptr<BurTree> t_b, int b_closest);

        std::pair<AlgorithmState, int> BurConnect(std::shared_ptr<BurTree> t, VectorXd &q);
        bool IsColliding(const VectorXd &q);
        double GetClosestDistance(const VectorXd &q);
        VectorXd Nearest(std::shared_ptr<BurTree> t, VectorXd &q);
        int NearestIndex(std::shared_ptr<BurTree> t, VectorXd &q);
        Bur GetBur(const VectorXd &q_near, const MatrixXd &Q_e, double d_closest);
        std::shared_ptr<BurEnv> bur_env;

        int num_spikes;
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