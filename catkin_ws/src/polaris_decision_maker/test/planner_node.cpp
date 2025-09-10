#include "polaris_decision_maker/planner.hpp"
#include "polaris_decision_maker/state_manager.hpp"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "planner_node");
  ros::NodeHandle nh;

  StateManager sm;
  ros::Publisher wp_pub = nh.advertise<geometry_msgs::Point>("/waypoints", 10);
  // Planner planner(&sm, nh, controller_topic);
  Planner planner(&sm, nh);

  ros::Rate rate(1);
  std::vector<std::pair<float, float>> waypoints = {
      // {10.0, -2.0},
      {30.0, -2.0},
      // {50.0, 0.0},
      // {70.0, 1.0}
  };
  size_t idx = 0;
  while (ros::ok()) {

    geometry_msgs::Point pt;
    pt.x = waypoints[idx].first;
    pt.y = waypoints[idx].second;
    pt.z = 0.0;
    wp_pub.publish(pt);
    // ROS_INFO("Published waypoint: (%.2f, %.2f)", pt.x, pt.y);
    // idx++;
    rate.sleep();
  }
  ros::spin();
  return 0;
}
