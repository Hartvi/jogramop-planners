#include "planning_result.h"
#include <sstream>
#include <fstream>
#include <stdexcept>

namespace Burs
{

    PlanningResult::PlanningResult() : success(false), distance_to_goal(0), time_taken(0), num_iterations(0), tree_size(0), max_runtime(0), finished_in_time(0) {}

    PlanningResult::PlanningResult(bool _success, double _distance_to_goal, double _time_taken, int _num_iterations, int _tree_size)
        : success(_success), distance_to_goal(_distance_to_goal), time_taken(_time_taken), num_iterations(_num_iterations), tree_size(_tree_size) {}

    std::string
    PlanningResult::toCSVString()
    {
        std::stringstream ss;
        ss << this->success << ","
           << this->distance_to_goal << ","
           << this->time_taken << ","
           << this->num_iterations << ","
           << this->tree_size;
        return ss.str();
    }

    std::string
    PlanningResult::toJSON()
    {
        std::stringstream ss;
        ss << "{ ";
        ss << "\"sr\":" << this->success << ","
           << "\"dtg\":" << this->distance_to_goal << ","
           << "\"time\":" << this->time_taken << ","
           << "\"iters\":" << this->num_iterations << ","
           << "\"treesize\": " << this->tree_size << ","
           << "\"maxRuntime\": " << this->max_runtime << ","
           << "\"finishedInTime\": " << this->finished_in_time;
        ss << "}\n";
        return ss.str();
    }




    PlanningResult
    PlanningResult::fromCSVString(const std::string &csvString)
    {
        throw std::runtime_error("PlanningResult::fromCSVString: TO BE IMPLEMENTED");

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

    std::vector<PlanningResult>
    PlanningResult::loadFromCSVFile(const std::string &filePath)
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
