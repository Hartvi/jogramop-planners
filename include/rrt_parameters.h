
#ifndef RRT_PARAMETERS_H
#define RRT_PARAMETERS_H

#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <iostream>
// #include "planning_result.h"

namespace Burs
{

    class RRTParameters
    {
    public:
        int visualize_tree = 0;
        int max_iters;
        double epsilon_q;
        double q_resolution;
        double collision_resolution;

        RRTParameters(int max_iters, double epsilon_q)
            : max_iters(max_iters), epsilon_q(epsilon_q), q_resolution(0)
        {
        }

        RRTParameters(int max_iters, double epsilon_q, double q_resolution)
            : max_iters(max_iters), epsilon_q(epsilon_q), q_resolution(q_resolution)
        {
        }

        RRTParameters()
            : max_iters(0), epsilon_q(0), q_resolution(0) {}

        virtual ~RRTParameters() = default;

        virtual std::string
        toString() const
        {
            std::ostringstream oss;
            oss << "max_iters: " << max_iters
                << ", epsilon_q: " << epsilon_q
                << ", q_resolution: " << q_resolution
                << ", collision_resolution: " << collision_resolution;
            return oss.str();
        }
    };

    // std::ostream &operator<<(std::ostream &os, const RRTParameters &params)
    // {
    //     os << "max_iters: " << params.max_iters
    //        << ", d_crit: " << params.d_crit
    //        << ", delta_q: " << params.delta_q
    //        << ", epsilon_q: " << params.epsilon_q
    //        << ", num_spikes: " << params.num_spikes;
    //     return os;
    // }

}

#endif
