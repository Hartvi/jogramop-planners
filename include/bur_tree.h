
#ifndef BUR_TREE_H
#define BUR_TREE_H

#include <flann/flann.hpp>
#include <Eigen/Dense>
#include <memory>
#include "bur_funcs.h"

namespace Burs
{
    using namespace Eigen;

    struct Bur
    {
        VectorXd center;
        MatrixXd endpoints;
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

        /// @brief Node location in configuration space
        VectorXd q;

        RRTNode(int p, VectorXd q)
            : parent_idx(p),
              q(q)
        {
        }
        RRTNode() : parent_idx(-1), q(VectorXd(0))
        {
        }
    };

    class BurTree
    {
    public:
        BurTree();
        BurTree(VectorXd q_location, int q_dim);

        void
        AddNode(int p, VectorXd q_location);

        std::pair<int, double>
        NearestIdxAndDist(double *new_point);

        int
        Nearest(double *new_point);

        VectorXd
        GetQ(int index);

        int
        GetParentIdx(int index);

        int
        GetNumberOfNodes();

        ~BurTree();

        // friend std::ostream &operator<<(std::ostream &os, const BurTree &tree);
        friend std::ostream &operator<<(std::ostream &os, const BurTree &tree)
        {
            for (const auto &node : tree.mNodes)
            {
                os << node.parent_idx;
                for (int j = 0; j < node.q.size(); ++j)
                {
                    os << ", " << node.q[j];
                }
                os << std::endl;
            }
            return os;
        }

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
                    this->mData[i][k] = this->mNodes[i].q[k];
                }
            }
        }
    };
}

#endif