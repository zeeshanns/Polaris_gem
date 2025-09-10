#include "polaris_decision_maker/state_manager.hpp"

StateManager::StateManager()
    : current_state(IDLE), battery_error(false), temp_error(false),
      gps_error(false), network_error(false), estop_error(false),
      network_signal(1) {}

void StateManager::updateBattery(float level) {
  battery_error = (level <= 50);
  evaluateState();
}

void StateManager::updateTemperature(float temp) {
  temp_error = (temp >= 55);
  evaluateState();
}

void StateManager::updateGPS(float accuracy) {
  if (accuracy <= 0.2) { // <=200mm
    if (gps_error_start.isZero())
      gps_error_start = ros::Time::now();
    double elapsed = (ros::Time::now() - gps_error_start).toSec();
    gps_error = (elapsed >= 15.0);
  } else if (accuracy > 0.2) {
    gps_error = false;
    gps_error_start = ros::Time(0);
  }
  evaluateState();
}

void StateManager::updateNetwork(int signal) {
  network_signal = signal;
  if (signal == 0) { // not connected
    if (network_error_start.isZero())
      network_error_start = ros::Time::now();
    double elapsed = (ros::Time::now() - network_error_start).toSec();
    network_error = (elapsed >= 10.0);
  } else if (signal == 2) { // low signal
    if (network_error_start.isZero())
      network_error_start = ros::Time::now();
    double elapsed = (ros::Time::now() - network_error_start).toSec();
    network_error = (elapsed >= 20.0);
  } else { // connected
    network_error = false;
    network_error_start = ros::Time(0);
  }
  evaluateState();
}

void StateManager::updateEStop(bool pressed) {
  estop_error = pressed;
  evaluateState();
}

RobotState StateManager::getState() { return current_state; }

void StateManager::setStartMission(bool flag) {
  startMission = flag;
  evaluateState();
}
void StateManager::tick() {
  // For timeouts, call this periodically
  if (network_signal == 0 && !network_error && !network_error_start.isZero()) {
    if ((ros::Time::now() - network_error_start).toSec() >= 10.0)
      network_error = true;
  }
  if (network_signal == 2 && !network_error && !network_error_start.isZero()) {
    if ((ros::Time::now() - network_error_start).toSec() >= 20.0)
      network_error = true;
  }
  evaluateState();
}

void StateManager::evaluateState() {
  if (battery_error || temp_error || gps_error || network_error || estop_error)
    current_state = ERROR;
  else
    current_state = IDLE;
  if (startMission && current_state == IDLE)
    current_state = RUNNING;
}
