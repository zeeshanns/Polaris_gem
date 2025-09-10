#ifndef STATE_MANAGER_HPP
#define STATE_MANAGER_HPP

#include <ros/ros.h>
#include <chrono>

enum RobotState { IDLE, RUNNING, ERROR };

class StateManager {
private:
    RobotState current_state;
    bool battery_error, temp_error, gps_error, network_error, estop_error;
    ros::Time gps_error_start, network_error_start;
    int network_signal; // 0=not connected, 1=connected, 2=low
public:
    StateManager();
    void updateBattery(float level);
    void updateTemperature(float temp);
    void updateGPS(float accuracy);
    void updateNetwork(int signal);
    void updateEStop(bool pressed);
    RobotState getState();
    void setStartMission(bool flag);

    void tick(); // call periodically for timeout checks
// private:
    void evaluateState();
private:
    bool startMission = false;


};

#endif
