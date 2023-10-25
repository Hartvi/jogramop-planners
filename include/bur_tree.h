#include <flann/flann.hpp>
#include <Eigen/Dense>
#include <memory>
#include "bur_funcs.h"

#ifndef BUR_TREE_H
#define BUR_TREE_H

namespace Burs
{
    using namespace Eigen;

    struct BurNode
    {
    public:
        /// @brief Index of the node in the array
        int parent_idx;

        /// @brief Node location in configuration space
        VectorXd q;

        BurNode(int p, VectorXd q)
            : parent_idx(p),
              q(q)
        {
        }
    };

    class BurTree
    {
    public:
        BurTree(VectorXd q_location, int q_dim);
        void add_node(int p, VectorXd q_location);
        int nearest(double *new_point);
        VectorXd GetQ(int index);
        int GetNumberOfNodes();
        ~BurTree();

    private:
        int q_dim;
        std::vector<BurNode> nodes;
        flann::Matrix<double> data;
        std::unique_ptr<flann::Index<flann::L2<double>>> index;

        void refreshIndex()
        {
            this->index = std::make_unique<flann::Index<flann::L2<double>>>(this->data, flann::KDTreeIndexParams(4));
            this->index->buildIndex();
        }

        void buildIndex()
        {
            this->refreshDataFromNodes();
            this->refreshIndex();
        }
        void refreshDataFromNodes()
        {
            // TODO: only update the last node
            delete[] this->data.ptr();

            double *_data_mtx = new double[this->nodes.size() * this->q_dim];

            this->data = flann::Matrix<double>(_data_mtx, this->nodes.size(), this->q_dim);

            for (int i = 0; i < this->nodes.size(); ++i)
            {
                for (int k = 0; k < q_dim; ++k)
                {
                    this->data[i][k] = this->nodes[i].q[k];
                }
            }
        }
    };
}

#endif