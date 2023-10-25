#include "bur_tree.h"
#include <Eigen/Dense>

namespace Burs
{
    using namespace Eigen;

    BurTree::BurTree(VectorXd q_location, int q_dim) : q_dim(q_dim)
    {
        this->add_node(-1, q_location);
    }

    void BurTree::add_node(int p, VectorXd q_location)
    {
        nodes.emplace_back(p, q_location);

        // have to build index to register the new node
        this->buildIndex();
    }

    int BurTree::nearest(double *new_point)
    {
        flann::Matrix<double> query(new_point, 1, this->q_dim); // Single row matrix for the new point
        flann::Matrix<int> indices(new int[1], 1, 1);           // Single row matrix for the index
        flann::Matrix<double> dists(new double[1], 1, 1);       // Single row matrix for the distance

        // Search for the closest point. We're only interested in the nearest one.
        this->index.get()->knnSearch(query, indices, dists, 1, flann::SearchParams(128));

        int closestIndex = indices[0][0];

        delete[] indices.ptr();
        delete[] dists.ptr();

        return closestIndex; // Return the index of the nearest node
    }

    VectorXd BurTree::GetQ(int index)
    {
        return this->nodes[index].q;
    }

    int BurTree::GetNumberOfNodes()
    {
        return this->nodes.size();
    }

    BurTree::~BurTree()
    {
        // delete index;
        delete[] this->data.ptr();
    }
}