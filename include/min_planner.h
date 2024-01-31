
#ifndef MIN_PLANNER_H
#define MIN_PLANNER_H
#include <flann/flann.hpp>
#include <Eigen/Dense>
#include <memory>
#include <optional>
#include "base_env.h"
#include "bur_funcs.h"
#include "bur_tree.h"

namespace Burs
{
    using namespace Eigen;

    enum AlgorithmState
    {
        Reached,
        Trapped,
        Failure
    };

    class MinPlanner
    {
    public:
        MinPlanner(int q_dim,
                   MatrixXd bounds);

        MinPlanner();

        virtual ~MinPlanner() = default;

        bool
        InBounds(const VectorXd &q) const;

        /// @brief Get set of random configurations
        MatrixXd
        GetRandomQ(const int &num_spikes) const;

        template <typename T>
        std::shared_ptr<T>
        GetEnv() const;

        void
        SetEnv(std::shared_ptr<BaseEnv> bur_env);

        bool
        IsColliding(const VectorXd &q) const;

        double
        GetClosestDistance(const VectorXd &q) const;

        VectorXd
        Nearest(std::shared_ptr<BurTree> t, VectorXd &q);

        int
        NearestIndex(std::shared_ptr<BurTree> t, VectorXd &q);

        std::vector<VectorXd>
        Path(std::shared_ptr<BurTree> t_a, int a_closest, std::shared_ptr<BurTree> t_b, int b_closest);

        std::vector<VectorXd>
        ConstructPathFromTree(std::shared_ptr<BurTree> t_a, int final_node_id);

        std::shared_ptr<BaseEnv> base_env;

    protected:
        int q_dim;
        // 2 columns: min column and max column
        MatrixXd bounds;
    };

    template <typename T>
    std::shared_ptr<T>
    MinPlanner::GetEnv() const
    {
        static_assert(std::is_base_of<BaseEnv, T>::value, "T must be a derived class of MinEnv");

        std::shared_ptr<T> ret_env = std::dynamic_pointer_cast<T>(base_env);
        if (ret_env != nullptr)
        {
            return ret_env;
        }
        else
        {
            throw std::runtime_error("Could not get BurEnv. The actual type is not " + std::string(typeid(T).name()) + ".");
        }
    }
}

#endif