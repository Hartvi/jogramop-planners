#include "vec_dist_pair.h"

namespace Burs
{
    VecDistPair::VecDistPair() : v(), d() {}

    /// @brief vector distance pair that often goes together
    /// @param vec vector
    /// @param val best/worst distance
    VecDistPair::VecDistPair(VectorXd vec, double val)
        : v(vec), d(val)
    {
    }
}