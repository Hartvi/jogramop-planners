#include <flann/flann.hpp>
#include <Eigen/Dense>
#include <memory>
#include "bur_funcs.h"

#ifndef BUR_TREE_H
#define BUR_TREE_H

namespace Burs
{
    using namespace Eigen;

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
    };

    struct Bur
    {
        VectorXd center;
        MatrixXd endpoints;
        Bur(VectorXd center, MatrixXd endpoints)
            : center(center),
              endpoints(endpoints)
        {
        }
    };

    class BurTree
    {
    public:
        BurTree(VectorXd q_location, int q_dim);
        void AddNode(int p, VectorXd q_location);
        int Nearest(double *new_point);
        VectorXd GetQ(int index);
        int GetParentIdx(int index);
        int GetNumberOfNodes();
        ~BurTree();

    private:
        std::vector<RRTNode> mNodes;
        int mQDim;
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