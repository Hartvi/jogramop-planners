#include "bur_tree.h"
#include <Eigen/Dense>

namespace Burs
{
    using namespace Eigen;

    BurTree::BurTree()
    {
        // ye
    }

    BurTree::BurTree(VectorXd q_location, int q_dim) : mQDim(q_dim)
    {
        this->AddNode(-1, q_location);
    }

    void
    BurTree::AddNode(int p, VectorXd q_location)
    {
        this->mNodes.emplace_back(p, q_location);

        // have to build index to register the new node
        this->BuildIndex();
    }

    std::pair<int, double>
    BurTree::NearestIdxAndDist(double *new_point)
    {
        flann::Matrix<double> query(new_point, 1, this->mQDim); // Single row matrix for the new point
        flann::Matrix<int> indices(new int[1], 1, 1);           // Single row matrix for the index
        flann::Matrix<double> dists(new double[1], 1, 1);       // Single row matrix for the distance

        // Search for the closest point. We're only interested in the Nearest one.
        this->mIndex.get()->knnSearch(query, indices, dists, 1, flann::SearchParams(128));

        int closestIndex = indices[0][0];
        double closestDist = dists[0][0];

        delete[] indices.ptr();
        delete[] dists.ptr();

        return std::make_pair(closestIndex, closestDist); // Return the index and distance of the Nearest node
    }

    int
    BurTree::Nearest(double *new_point)
    {
        flann::Matrix<double> query(new_point, 1, this->mQDim); // Single row matrix for the new point
        flann::Matrix<int> indices(new int[1], 1, 1);           // Single row matrix for the index
        flann::Matrix<double> dists(new double[1], 1, 1);       // Single row matrix for the distance

        // Search for the closest point. We're only interested in the Nearest one.
        this->mIndex.get()->knnSearch(query, indices, dists, 1, flann::SearchParams(128));

        int closestIndex = indices[0][0];

        delete[] indices.ptr();
        delete[] dists.ptr();

        return closestIndex; // Return the index of the Nearest node
    }

    VectorXd
    BurTree::GetQ(int index)
    {
        return this->mNodes[index].q;
    }

    int
    BurTree::GetParentIdx(int index)
    {
        return this->mNodes[index].parent_idx;
    }

    int
    BurTree::GetNumberOfNodes()
    {
        return this->mNodes.size();
    }

    BurTree::~BurTree()
    {
        // delete index;
        delete[] this->mData.ptr();
    }
}