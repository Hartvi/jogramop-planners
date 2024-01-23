
#ifndef RBT_PARAMETERS_H
#define RBT_PARAMETERS_H

#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <iostream>
#include "planning_result.h"

namespace Burs
{

    struct RbtParameters
    {
        int max_iters;
        double d_crit;
        double delta_q;
        double epsilon_q;
        int num_spikes;
        double q_resolution;

        RbtParameters(int max_iters, double d_crit, double delta_q, double epsilon_q, int num_spikes, double q_resolution)
            : max_iters(max_iters), d_crit(d_crit), delta_q(delta_q), epsilon_q(epsilon_q), num_spikes(num_spikes), q_resolution(q_resolution)
        {
        }

        RbtParameters()
            : max_iters(0), d_crit(0), delta_q(0), epsilon_q(0), num_spikes(0), q_resolution(0) {}

        RbtParameters(const std::string &filename)
        {
            std::ifstream file(filename);
            std::string line;
            std::unordered_map<std::string, std::string> params;

            while (std::getline(file, line))
            {
                std::stringstream ss(line);
                std::string key, value;
                if (std::getline(ss, key, ',') && std::getline(ss, value, ','))
                {
                    params[key] = value;
                }
            }

            this->max_iters = std::stoi(params["max_iters"]);
            this->d_crit = std::stod(params["d_crit"]);
            this->delta_q = std::stod(params["delta_q"]);
            this->epsilon_q = std::stod(params["epsilon_q"]);
            this->num_spikes = std::stoi(params["num_spikes"]);
        }

        virtual std::string
        toString() const
        {
            std::ostringstream oss;
            oss << "max_iters: " << max_iters
                << ", d_crit: " << d_crit
                << ", delta_q: " << delta_q
                << ", epsilon_q: " << epsilon_q
                << ", num_spikes: " << num_spikes
                << ", q_resolution: " << q_resolution;
            return oss.str();
        }
    };

    // std::ostream &operator<<(std::ostream &os, const RbtParameters &params)
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
