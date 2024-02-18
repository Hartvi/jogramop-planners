
#ifndef RAND_NUMS_H
#define RAND_NUMS_H

#include <random>
#include <algorithm>

class RandomNumberGenerator
{
public:
    RandomNumberGenerator(const int seed, const int maxNodes)
        : rng(rd()), realDistr(0.0, 1.0), intDistr(0, maxNodes - 1), intVector(maxNodes)
    {
        rng.seed(seed);
        std::iota(this->intVector.begin(), this->intVector.end(), 0);
        std::cout << "max rand: " << this->intVector.size() << "\n";
    }

    double
    getRandomReal()
    {
        return realDistr(rng);
    }

    int
    getRandomInt()
    {
        return intDistr(rng);
    }

    std::vector<unsigned int>::iterator
    getNonRepeatingInts()
    {
        std::shuffle(this->intVector.begin(), this->intVector.end(), this->rd);
        return this->intVector.begin();
    }

private:
public:
    std::random_device rd;
    std::mt19937 rng;
    std::uniform_real_distribution<> realDistr;
    std::uniform_int_distribution<int> intDistr;
    // TODO: CREATE VECTOR FOR RANDOM NON-REPEATING INTEGERS IN RANGE 0 - N-1
    std::vector<unsigned int> intVector;
};

#endif
