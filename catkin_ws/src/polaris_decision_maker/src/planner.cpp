#include "polaris_decision_maker/planner.hpp"
#include "ros/ros.h"

// Planner::Planner(StateManager* sm, ros::NodeHandle& nh, const std::string&
// topic)
Planner::Planner(StateManager *sm, ros::NodeHandle &nh)
    : state_manager(sm), idx(0), active_controller("stanley") {
  st_wp_pub = nh.advertise<geometry_msgs::Point>("/st/waypoints", 10);
  pp_wp_pub = nh.advertise<geometry_msgs::Point>("/pp/waypoints", 10);
}

void Planner::addWaypoint(const Waypoint &wp) { waypoints.push_back(wp); }

void Planner::setController(const std::string &controller) {
  active_controller = controller;
}

void Planner::firstWaypoint() {
  for (int i = 0; i < 2; i++) {
    geometry_msgs::Point pt;
    pt.x = waypoints[idx].x;
    pt.y = waypoints[idx].y;
    pt.z = waypoints[idx].z;
    if (active_controller == "stanley") {
      st_wp_pub.publish(pt);
    } else {
      pp_wp_pub.publish(pt);
    }
    ros::Duration(2.0).sleep();
    idx++;
  }
}

void Planner::publishNextWaypoint() {
  if (idx < waypoints.size()) {
    geometry_msgs::Point pt;
    pt.x = waypoints[idx].x;
    pt.y = waypoints[idx].y;
    pt.z = waypoints[idx].z;
    if (active_controller == "stanley") {
      st_wp_pub.publish(pt);
    } else {
      pp_wp_pub.publish(pt);
    }
    idx++;
  }
}

void Planner::reset() { idx = 0; }
