
#ifndef RAND_NUMS_H
#define RAND_NUMS_H

#include <random>

class RandomNumberGenerator
{
public:
    RandomNumberGenerator(int maxNodes)
        : rng(rd()), realDistr(0.0, 1.0), intDistr(0, maxNodes) {}

    double getRandomReal()
    {
        return realDistr(rng);
    }

    int getRandomInt()
    {
        return intDistr(rng);
    }

private:
    std::random_device rd;
    std::mt19937 rng;
    std::uniform_real_distribution<> realDistr;
    std::uniform_int_distribution<int> intDistr;
};

#endif
