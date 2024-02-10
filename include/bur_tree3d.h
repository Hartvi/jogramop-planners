
#ifndef BUR_TREE3D_H
#define BUR_TREE3D_H

#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>

#include <flann/flann.hpp>
#include <Eigen/Dense>
#include <memory>
#include "bur_funcs.h"
#include "robot_state.h"
#include "bur_tree.h"

namespace Burs
{
    using namespace Eigen;

    class BurTree3D
    {
    public:
        BurTree3D(RS q_location);

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

        ~BurTree3D();

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