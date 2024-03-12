# main todos
- implement rrt - DONE
- implement rbt - DONE
- implement rrt (in config space with intermediate collision checks) - DONE
- implement  ik-rrt (in config space with intermediate collision checks) - DONE
- implement concave extended rbt - WIP
- statistically test algorithms and compare speeds
- write thesis related works
- write thesis methods & materials
  - short section on burs, ik-rrt, j-rrt, extended burs, etc
  - short section on how to extend burs for linear joints
  - visualize using pictures from python
- write thesis experiments & results
- write thesis conclusion & future work
  - maybe extend the burs with collision-free capsules?

## THESIS TASK
1. Get familiar with the motion planning problem and sampling-based planning techniques like Rapidly-exploring Random Tree and its variants [1]. Study problem of path planning for robotic manipulators [6].
2. Implement a variant of RRT for mobile manipulators using inverse kinematics, e.g., RRT-IK [4], and adapt it to mobile manipulator robots. Use the BURG toolkit as a model/simulation of the mobile manipulator and the environment [5].
3. Implement Burs-RRT [2,3] and adapt it for mobile manipulator robots.
4. Extend a selected planner from tasks 2) or 3) for grasping in constrained environments. An external library gives grasping positions. The task is to move the robot as close as possible to the grasping position. Multiple grasping positions are available for each object. Assume that final positions are given only in the task space (i.e., without inverse kinematics). Grasping is not part of this task and does not need to be solved.
5. Compare methods from tasks 2) and 3) in a set of scenarios for grasping in constrained environments. Consider „easy“ scenarios (robot motion is not blocked by obstacles), as well as „challenging “ scenarios (robot or object to be grasped is behind an obstacle). Statistically evaluate the performance of the planners.

# Literature
1. S. M. LaValle, Planning algorithms, 2006, Cambridge press
2. Lacevic, Bakir, and Dinko Osmankovic. “Path Planning for Rigid Bodies Using Burs of Free C-Space.” IFAC-
PapersOnLine 51, no. 22 (2018): 280–85. https://doi.org/10.1016/j.ifacol.2018.11.555.
3. Lacevic, Bakir, Dinko Osmankovic, and Adnan Ademovic. “Burs of Free C-Space: A Novel Structure for Path
Planning.” In 2016 IEEE International Conference on Robotics and Automation (ICRA), 70–76. Stockholm,
Sweden: IEEE, 2016. https://doi.org/10.1109/ICRA.2016.7487117.
4. Vahrenkamp, Nikolaus, Dmitry Berenson, Tamim Asfour, James Kuffner, and Rudiger Dillmann. “Humanoid
Motion Planning for Dual-Arm Manipulation and Re-Grasping Tasks.” In 2009 IEEE/RSJ International
Conference on Intelligent Robots and Systems, 2464–70. St. Louis, MO: IEEE, 2009.
https://doi.org/10.1109/IROS.2009.5354625.
5. Martin Rudorfer, Markus Suchi, Mohan Sridharan, Markus Vincze, Aleš Leonardis, BURG-Toolkit: Robot
Grasping Experiments in Simulation and the Real World,
6. Kevin M. Lynch and Frank C. Park: Modern Robotics: Mechanics, Planning, and Control", Cambridge University
Press, 2017, ISBN 9781107156302