#ifndef SENSOR_MONITOR_HPP
#define SENSOR_MONITOR_HPP

#include "ros/ros.h"
#include "sensor_msgs/BatteryState.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Bool.h"
#include "polaris_decision_maker/state_manager.hpp"

class SensorMonitor {
private:
    ros::NodeHandle nh;

    ros::Subscriber battery_sub;
    ros::Subscriber temp_sub;
    ros::Subscriber gps_sub;
    ros::Subscriber estop_sub;
    ros::Subscriber network_sub; // optional if network signal topic exists

    StateManager* state_manager;
public:
    SensorMonitor(StateManager* sm);

    void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg);
    void tempCallback(const sensor_msgs::Temperature::ConstPtr& msg);
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void estopCallback(const std_msgs::Bool::ConstPtr& msg);
    void networkCallback(const std_msgs::Bool::ConstPtr& msg); // or int type for signal strength

    void spin();
};

#endif
