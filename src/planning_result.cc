#include "planning_result.h"
#include <sstream>
#include <fstream>
#include <stdexcept>

namespace Burs
{

    PlanningResult::PlanningResult() : success(false), distance_to_goal(0), time_taken(0), num_iterations(0), tree_size(0) {}

    PlanningResult::PlanningResult(bool _success, double _distance_to_goal, double _time_taken, int _num_iterations, int _tree_size)
        : success(_success), distance_to_goal(_distance_to_goal), time_taken(_time_taken), num_iterations(_num_iterations), tree_size(_tree_size) {}

    std::string PlanningResult::toCSVString(const PlanningResult &result)
    {
        std::stringstream ss;
        ss << result.success << ','
           << result.distance_to_goal << ','
           << result.time_taken << ','
           << result.num_iterations << ','
           << result.tree_size;
        return ss.str();
    }

    PlanningResult PlanningResult::fromCSVString(const std::string &csvString)
    {
        std::stringstream ss(csvString);
        int successAsInt;
        double distance_to_goal, time_taken;
        int num_iterations, tree_size;
        char delimiter;

        ss >> successAsInt >> delimiter;
        bool success = successAsInt != 0;
        ss >> distance_to_goal >> delimiter;
        ss >> time_taken >> delimiter;
        ss >> num_iterations >> delimiter;
        ss >> tree_size;

        if (ss.fail())
        {
            throw std::runtime_error("Failed to extract PlanningResult from " + csvString);
        }

        return PlanningResult(success, distance_to_goal, time_taken, num_iterations, tree_size);
    }

    std::vector<PlanningResult> PlanningResult::loadFromCSVFile(const std::string &filePath)
    {
        std::vector<PlanningResult> results;
        std::ifstream file(filePath);
        std::string line;

        while (std::getline(file, line))
        {
            results.push_back(fromCSVString(line));
        }

        return results;
    }

}
