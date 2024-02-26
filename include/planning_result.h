#ifndef PLANNING_RESULT_H
#define PLANNING_RESULT_H

#include <string>
#include <vector>

namespace Burs
{

    class PlanningResult
    {
    public:
        bool success;
        double distance_to_goal;
        double time_taken;
        int num_iterations;
        int tree_size;
        double max_runtime;
        double finished_in_time;

        PlanningResult();
        PlanningResult(bool _success, double _distance_to_goal, double _time_taken, int _num_iterations, int _tree_size);

        std::string toCSVString();
        std::string toJSON();
        static PlanningResult fromCSVString(const std::string &csvString);
        static std::vector<PlanningResult> loadFromCSVFile(const std::string &filePath);
    };

}

#endif // PLANNING_RESULT_H
