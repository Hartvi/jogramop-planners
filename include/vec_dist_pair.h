
#ifndef VEC_DIST_PAIR_H
#define VEC_DIST_PAIR_H

#include <Eigen/Dense>

namespace Burs
{
    using namespace Eigen;

    class VecDistPair
    {
    public:
        VecDistPair(VectorXd vec, double val);
        VecDistPair();

        double d;
        VectorXd v;
    };
}
#endif
