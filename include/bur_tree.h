
#ifndef BUR_TREE_H
#define BUR_TREE_H

#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>

#include <flann/flann.hpp>
#include <Eigen/Dense>
#include <memory>
#include "bur_funcs.h"
#include "robot_state.h"

namespace Burs
{
    using namespace Eigen;

    struct Bur
    {
        VectorXd center;
        MatrixXd endpoints;
        Bur() = default;

        Bur(VectorXd center, MatrixXd endpoints)
            : center(center),
              endpoints(endpoints)
        {
        }

        // Overload the << operator to customize output
        friend std::ostream &operator<<(std::ostream &out, const Bur &bur)
        {
            out << "Center: " << bur.center.transpose() << std::endl;
            out << "Endpoints:" << std::endl;
            out << bur.endpoints << std::endl;
            return out;
        }
    };

    struct RRTNode
    {
    public:
        /// @brief Index of the node in the array
        int parent_idx;
        // std::vector<int> child_ids;

        /// @brief Node location in configuration space
        // VectorXd q;
        RS state;

        RRTNode(int p, RS state)
            : parent_idx(p),
              state(state)
        {
        }
        // RRTNode() : parent_idx(-1), state(VectorXd(0))
        // {
        // }
    };

    class BurTree
    {
    public:
        // BurTree();
        BurTree(RS q_location, int q_dim);
        // BurTree(VectorXd q_location, int q_dim);

        // int
        // AddNode(int p, VectorXd q_location);

        int
        AddNode(int p, RS state);

        std::pair<int, double>
        NearestIdxAndDistSqr(double *new_point);

        int
        Nearest(const int &idx);

        int
        Nearest(double *new_point);

        int
        Nearest(RS &state);

        RS *
        Get(int index);

        int
        GetParentIdx(int index);

        int
        GetNumberOfNodes();

        ~BurTree();

        // friend std::ostream &operator<<(std::ostream &os, const BurTree &tree);
        // friend std::ostream &operator<<(std::ostream &os, const BurTree &tree)
        // {
        //     for (const auto &node : tree.mNodes)
        //     {
        //         os << node.parent_idx;
        //         for (int j = 0; j < node.state.config.size(); ++j)
        //         {
        //             os << ", " << node.state.config[j];
        //         }
        //         os << std::endl;
        //     }
        //     return os;
        // }

    public:
        std::vector<RRTNode> mNodes;
        int mQDim;

    private:
        flann::Matrix<double> mData;
        std::unique_ptr<flann::Index<flann::L2<double>>> mIndex;

        void RefreshIndex()
        {
            this->mIndex = std::make_unique<flann::Index<flann::L2<double>>>(this->mData, flann::KDTreeIndexParams(4));
            this->mIndex->buildIndex();
        }

        void BuildIndex()
        {
            this->RefreshDataFromNodes();
            this->RefreshIndex();
        }

        void RefreshDataFromNodes()
        {
            // TODO: only update the last node
            delete[] this->mData.ptr();

            double *_data_mtx = new double[this->mNodes.size() * this->mQDim];

            this->mData = flann::Matrix<double>(_data_mtx, this->mNodes.size(), this->mQDim);

            for (int i = 0; i < this->mNodes.size(); ++i)
            {
                for (int k = 0; k < this->mQDim; ++k)
                {
                    this->mData[i][k] = this->mNodes[i].state.config[k];
                }
            }
        }
    };

}

#endif