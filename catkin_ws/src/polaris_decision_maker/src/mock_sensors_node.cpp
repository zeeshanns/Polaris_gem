/*
 * Mock sensor publisher for assignment scenario testing.
 * Publishes battery, temperature, GPS accuracy, network signal, and emergency
 * stop status. Use this node in rostest scenarios to simulate sensor failures
 * and recovery.
 *
 * Topics:
 *   /mock/battery_level      (std_msgs/Float32)
 *   /mock/temperature        (std_msgs/Float32)
 *   /mock/gps_accuracy       (std_msgs/Float32)
 *   /mock/network_signal     (std_msgs/Bool)
 *   /mock/emergency_stop     (std_msgs/Bool)
 *
 * This script cycles through each failure scenario for demonstration.
 */

#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Temperature.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

enum Scenario { BATTERY, TEMPERATURE, GPS, NETWORK, ESTOP, RECOVERY, DONE };

int main(int argc, char **argv) {
  ros::init(argc, argv, "mock_sensors_node");
  ros::NodeHandle nh;

  ros::Publisher battery_pub =
      nh.advertise<std_msgs::Float32>("/mock/battery_level", 1);
  ros::Publisher temp_pub =
      nh.advertise<std_msgs::Float32>("/mock/temperature", 1);
  ros::Publisher gps_acc_pub =
      nh.advertise<std_msgs::Float32>("/mock/gps_accuracy", 1);
  ros::Publisher net_signal_pub =
      nh.advertise<std_msgs::Int32>("/mock/network_signal", 1);
  ros::Publisher estop_pub =
      nh.advertise<std_msgs::Bool>("/mock/emergency_stop", 1);

  ros::Rate rate(10);

  Scenario scenario = BATTERY;
  ros::Time scenario_start = ros::Time::now();
  int network_phase = 0;
  bool error_phase = true;

  float battery = 100.0;
  float temp = 30.0;
  float gps_acc = 1000.0;
  int net_signal = 1;
  bool estop = false;

  while (ros::ok() && scenario != DONE) {
    ros::Duration elapsed = ros::Time::now() - scenario_start;

    switch (scenario) {
    case BATTERY:
      if (error_phase) {
        battery = 50.0;
        temp = 30.0;
        gps_acc = 1000.0;
        net_signal = 1;
        estop = false;
        if (elapsed.toSec() > 6.0) {
          error_phase = false;
          scenario_start = ros::Time::now();
        }
      } else {
        battery = 100.0;
        temp = 30.0;
        gps_acc = 1000.0;
        net_signal = 1;
        estop = false;
        if (elapsed.toSec() > 5.0) {
          scenario = TEMPERATURE;
          scenario_start = ros::Time::now();
          error_phase = true;
        }
      }
      break;
    case TEMPERATURE:
      if (error_phase) {
        temp = 55.0;
        battery = 100.0;
        gps_acc = 1000.0;
        net_signal = 1;
        estop = false;
        if (elapsed.toSec() > 6.0) {
          error_phase = false;
          scenario_start = ros::Time::now();
        }
      } else {
        temp = 30.0;
        battery = 100.0;
        gps_acc = 1000.0;
        net_signal = 1;
        estop = false;
        if (elapsed.toSec() > 5.0) {
          scenario = GPS;
          scenario_start = ros::Time::now();
          error_phase = true;
        }
      }
      break;
    case GPS:
      if (error_phase) {
        gps_acc = 0.15; // 150mm in meters
        battery = 100.0;
        temp = 30.0;
        net_signal = 1;
        estop = false;
        if (elapsed.toSec() > 16.0) {
          error_phase = false;
          scenario_start = ros::Time::now();
        }
      } else {
        gps_acc = 1.0; // 1 meter
        battery = 100.0;
        temp = 30.0;
        net_signal = 1;
        estop = false;
        if (elapsed.toSec() > 5.0) {
          scenario = NETWORK;
          scenario_start = ros::Time::now();
          error_phase = true;
          network_phase = 0;
        }
      }
      break;
    case NETWORK:
      if (error_phase) {
        if (network_phase == 0) {
          net_signal = 0;
          battery = 100.0;
          temp = 30.0;
          gps_acc = 1000.0;
          estop = false;
          if (elapsed.toSec() > 11.0) {
            network_phase = 1;
            scenario_start = ros::Time::now();
          }
        } else if (network_phase == 1) {
          net_signal = 2;
          battery = 100.0;
          temp = 30.0;
          gps_acc = 1000.0;
          estop = false;
          if (elapsed.toSec() > 21.0) {
            error_phase = false;
            scenario_start = ros::Time::now();
          }
        }
      } else {
        net_signal = 1;
        battery = 100.0;
        temp = 30.0;
        gps_acc = 1000.0;
        estop = false;
        if (elapsed.toSec() > 5.0) {
          scenario = ESTOP;
          scenario_start = ros::Time::now();
          error_phase = true;
        }
      }
      break;
    case ESTOP:
      if (error_phase) {
        estop = true;
        battery = 100.0;
        temp = 30.0;
        gps_acc = 1000.0;
        net_signal = 1;
        if (elapsed.toSec() > 6.0) {
          error_phase = false;
          scenario_start = ros::Time::now();
        }
      } else {
        estop = false;
        battery = 100.0;
        temp = 30.0;
        gps_acc = 1000.0;
        net_signal = 1;
        if (elapsed.toSec() > 5.0) {
          scenario = RECOVERY;
          scenario_start = ros::Time::now();
          error_phase = true;
        }
      }
      break;
    case RECOVERY:
      battery = 100.0;
      temp = 30.0;
      gps_acc = 1000.0;
      net_signal = 1;
      estop = false;
      if (elapsed.toSec() > 5.0) {
        scenario = DONE;
      }
      break;
    default:
      break;
    }

    std_msgs::Float32 battery_msg;
    std_msgs::Float32 temp_msg;
    std_msgs::Float32 gps_acc_msg;
    std_msgs::Int32 net_signal_msg;
    std_msgs::Bool estop_msg;

    battery_msg.data = battery;
    temp_msg.data = temp;
    gps_acc_msg.data = gps_acc;
    net_signal_msg.data = net_signal;
    estop_msg.data = estop;

    battery_pub.publish(battery_msg);
    temp_pub.publish(temp_msg);
    gps_acc_pub.publish(gps_acc_msg);
    net_signal_pub.publish(net_signal_msg);
    estop_pub.publish(estop_msg);

    // Print scenario and sensor values for visualization
    const char *scenario_str = "";
    switch (scenario) {
    case BATTERY:
      scenario_str = "BATTERY";
      break;
    case TEMPERATURE:
      scenario_str = "TEMPERATURE";
      break;
    case GPS:
      scenario_str = "GPS";
      break;
    case NETWORK:
      scenario_str = "NETWORK";
      break;
    case ESTOP:
      scenario_str = "ESTOP";
      break;
    case RECOVERY:
      scenario_str = "RECOVERY";
      break;
    case DONE:
      scenario_str = "DONE";
      break;
    }
    ROS_INFO("Scenario: %s | Battery: %.1f | Temp: %.1f | GPS Acc: %.1f | Net: "
             "%d | EStop: %d",
             scenario_str, battery, temp, gps_acc, net_signal, estop);

    rate.sleep();
  }
  return 0;
}
