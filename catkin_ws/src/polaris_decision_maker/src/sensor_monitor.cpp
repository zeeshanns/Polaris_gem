#include "polaris_decision_maker/planner.hpp"
#include "polaris_decision_maker/state_manager.hpp"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
bool st_goal_reached_flag = false;
bool pp_goal_reached_flag = false;
StateManager sm;

void batteryCallback(const std_msgs::Float32::ConstPtr &msg) {
  sm.updateBattery(msg->data);
}

void tempCallback(const std_msgs::Float32::ConstPtr &msg) {
  sm.updateTemperature(msg->data);
}

void gpsCallback(const std_msgs::Float32::ConstPtr &msg) {
  sm.updateGPS(msg->data); // horizontal accuracy
}

void estopCallback(const std_msgs::Bool::ConstPtr &msg) {
  sm.updateEStop(msg->data);
}

void networkCallback(const std_msgs::Int32::ConstPtr &msg) {
  sm.updateNetwork(msg->data);
}

void startMissionCallback(const std_msgs::Bool::ConstPtr &msg) {
  sm.setStartMission(msg->data);
}

void goalReachedCallback(const std_msgs::Bool::ConstPtr &msg) {
  st_goal_reached_flag = msg->data;
}
void ppgoalReachedCallback(const std_msgs::Bool::ConstPtr &msg) {
  pp_goal_reached_flag = msg->data;
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "sensor_monitor_node");
  ros::NodeHandle nh;
  ros::Subscriber battery_sub =
      nh.subscribe("/mock/battery_level", 10, batteryCallback);
  ros::Subscriber temp_sub =
      nh.subscribe("/mock/temperature", 10, tempCallback);
  ros::Subscriber gps_sub = nh.subscribe("/mock/gps_accuracy", 10, gpsCallback);
  ros::Subscriber estop_sub =
      nh.subscribe("/mock/emergency_stop", 10, estopCallback);
  ros::Subscriber network_sub =
      nh.subscribe("/mock/network_signal", 10, networkCallback);

  ros::Subscriber start_mission_sub =
      nh.subscribe("/start_mission", 1, startMissionCallback);

  ////////rostopic responsible to control & communicate the stanley
  ros::Publisher st_state_pub =
      nh.advertise<std_msgs::String>("/st/robot_state", 10);
  ros::Publisher st_wp_pub =
      nh.advertise<geometry_msgs::Point>("/st/waypoints", 10);
  ros::Subscriber st_goal_reached_sub =
      nh.subscribe("/st/goal_reached", 1, goalReachedCallback);

  ////////rostopic responsible to control & communicate the pure pursuit
  ros::Publisher pp_state_pub =
      nh.advertise<std_msgs::String>("/pp/robot_state", 10);
  ros::Publisher pp_wp_pub =
      nh.advertise<geometry_msgs::Point>("/pp/waypoints", 10);
  ros::Subscriber pp_goal_reached_sub =
      nh.subscribe("/pp/goal_reached", 1, ppgoalReachedCallback);

  // StateManager sm;
  // Planner planner(&sm, nh, controller_topic);
  Planner planner(&sm, nh);
  std::string controller_type;
  nh.getParam("controller_type", controller_type);
  planner.setController(controller_type);
  
  planner.addWaypoint({10.0, -2.0, 0.0});
  planner.addWaypoint({150.0, 100.0, 0.0});
  planner.addWaypoint({0.0, 197.0, 0.0});
  planner.addWaypoint({-130.0, 157.0, 0.0});
  planner.addWaypoint({-100.0, 11.0, 0.0});
  bool first_sent = false;

  ros::Rate rate(10);
  while (ros::ok()) {
    sm.tick();
    const char *state_str = "UNKNOWN";
    switch (sm.getState()) {
    case 0:
      state_str = "IDLE";
      break;
    case 1:
      state_str = "RUNNING";
      break;
    case 2:
      state_str = "ERROR";
      break;
    }

    std_msgs::String st_state_msg;
    st_state_msg.data = state_str;
    st_state_pub.publish(st_state_msg);

    std_msgs::String pp_state_msg;
    pp_state_msg.data = state_str;
    pp_state_pub.publish(pp_state_msg);

    ROS_INFO("Current State: %s", state_str);

    if (!first_sent) {
      planner.firstWaypoint();
      first_sent = true;
    } else if (st_goal_reached_flag) {
      planner.setController("stanley");
      planner.publishNextWaypoint();
      st_goal_reached_flag = false;
    } else if (pp_goal_reached_flag) {
      planner.setController("pure_pursuit");
      planner.publishNextWaypoint();
      pp_goal_reached_flag = false;
    }
    ros::spinOnce();
    rate.sleep();
  }
}
