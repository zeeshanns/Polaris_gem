#ifndef PLANNER_HPP
#define PLANNER_HPP

#include "geometry_msgs/PoseArray.h"
#include "polaris_decision_maker/state_manager.hpp"
#include "ros/ros.h"
#include <string>
#include <vector>

struct Waypoint {
  double x, y, z;
};

class Planner {
private:
    StateManager* state_manager;
    std::vector<Waypoint> waypoints;
    ros::Publisher st_wp_pub;
    ros::Publisher pp_wp_pub;
    std::string active_controller; // "stanley" or "pure_pursuit"
    size_t idx;
public:
    Planner(StateManager* sm, ros::NodeHandle& nh);
    void addWaypoint(const Waypoint& wp);
    void setController(const std::string& controller);
    void publishNextWaypoint();
    void firstWaypoint();
    void reset();
};

#endif
