
#ifndef PLANNER_SWITCHER_H
#define PLANNER_SWITCHER_H

enum PlannerId
{
    Rbt,
    JPlusRbt,
    RRT
};

void RunPlanner(PlannerId planner_id);

#endif