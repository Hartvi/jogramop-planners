
#ifndef RBT_PARAMETERS_H
#define RBT_PARAMETERS_H

#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <iostream>
#include "planning_result.h"
#include "rrt_planner.h"

namespace Burs
{

    class RbtParameters : public RRTParameters
    {
    public:
        // int max_iters;
        double d_crit;
        double delta_q;
        // double epsilon_q;
        int num_spikes;
        double q_resolution;

        RbtParameters(int max_iters, double d_crit, double delta_q, double epsilon_q, int num_spikes, double q_resolution)
            : RRTParameters(max_iters, epsilon_q),
              d_crit(d_crit), delta_q(delta_q), num_spikes(num_spikes), q_resolution(q_resolution)
        {
        }

        RbtParameters()
            : RRTParameters(0, 0),
              d_crit(0), delta_q(0), num_spikes(0), q_resolution(0) {}

        std::string
        toString() const override
        {
            std::ostringstream oss;
            oss << RRTParameters::toString();
            oss << ", d_crit: " << d_crit
                << ", delta_q: " << delta_q
                << ", num_spikes: " << num_spikes
                << ", q_resolution: " << q_resolution;
            return oss.str();
        }
    };

}

#endif
