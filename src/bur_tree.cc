#include "bur_tree.h"
#include <Eigen/Dense>

namespace Burs
{
    using namespace Eigen;

    BurTree::BurTree(RS state, int q_dim) : mQDim(q_dim)
    {
        this->AddNode(-1, state);
    }

    int
    BurTree::AddNode(int p, RS state)
    {
        int ret_idx = this->mNodes.size();
        this->mNodes.emplace_back(p, state);

        // HACK
        if (mNodes.size() == 1)
        {
            BuildIndex();
        }
        else
        {
            const int dimension = this->mQDim;
            flann::Matrix<double> add_point_matrix(new double[1 * dimension], 1, dimension);
            for (int i = 0; i < this->mQDim; i++)
            {
                add_point_matrix[0][i] = state.config[i];
            }
            mIndex->addPoints(add_point_matrix, 2.0);
        }
        // delete[] pointData; // Clean up the allocated memory
        return ret_idx;
    }

    std::pair<int, double>
    BurTree::NearestIdxAndDistSqr(double *new_point)
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
    BurTree::Nearest(const int &idx)
    {
        return this->Nearest(this->mNodes[idx].state.config.data());
    }

    int
    BurTree::Nearest(RS &state)
    {
        return this->Nearest(state.config.data());
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

    RS *
    BurTree::Get(int index)
    {
        return &this->mNodes[index].state;
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
