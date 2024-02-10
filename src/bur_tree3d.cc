#include "bur_tree3d.h"
#include <Eigen/Dense>

namespace Burs
{
    using namespace Eigen;

    BurTree3D::BurTree3D(RS state)
    {
        this->mQDim = 3;
        this->AddNode(-1, state);
    }

    int
    BurTree3D::AddNode(int p, RS state)
    {
        int ret_idx = this->mNodes.size();
        this->mNodes.emplace_back(p, state);
        // have to build index to register the new node
        //        this->BuildIndex();

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
                add_point_matrix[0][i] = state.frames.back().p[i];
            }
            mIndex->addPoints(add_point_matrix, 2.0);
        }
        // delete[] pointData; // Clean up the allocated memory
        return ret_idx;
    }

    std::pair<int, double>
    BurTree3D::NearestIdxAndDistSqr(double *new_point)
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
    BurTree3D::Nearest(const int &idx)
    {
        return this->Nearest(this->mNodes[idx].state.frames.back().p.data);
    }

    int
    BurTree3D::Nearest(RS &state)
    {
        return this->Nearest(state.frames.back().p.data);
    }

    int
    BurTree3D::Nearest(double *new_point)
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
    BurTree3D::Get(int index)
    {
        return &this->mNodes[index].state;
    }

    int
    BurTree3D::GetParentIdx(int index)
    {
        return this->mNodes[index].parent_idx;
    }

    int
    BurTree3D::GetNumberOfNodes()
    {
        return this->mNodes.size();
    }

    BurTree3D::~BurTree3D()
    {
        // delete index;
        delete[] this->mData.ptr();
    }
}
