# Polaris GEM e2 Decision-Making System

## Docker Setup & Quickstart
A Docker environment is provided for easy setup and reproducibility.

### 1. Build the Docker Image
```bash
docker build -t assignment:latest .
```

### 2. Verify the Docker Image
```bash
docker image ls
```

### 3. Launch the Container with Docker Compose
```bash
docker compose up -d
```

### 4. Enter the Running Container
```bash
docker exec -it assignment-assignment-1 bash
```

### 5. Initialize the ROS Workspace
```bash
cd catkin_ws
source devel/setup.bash
```

---

## Overview
This repository implements a modular, scalable decision-making and planning system for the Polaris GEM e2 autonomous vehicle simulator. It manages operational states (IDLE, RUNNING, ERROR), integrates sensor data, and interfaces with path-tracking controllers (Stanley or Pure Pursuit).

---

### Codebase Architecture
![Codebase Architecture](docs/uml/code_structure.png)
[View PlantUML Source](docs/uml/code_structure.puml)

---

### Pure Pursuit Controller Interface
![Pure Pursuit Controller UML](docs/uml/pure_pursuit_intyerface.png)
[View UML Source](docs/uml/pure_pursuit_intyerface.uml)

---

### Stanley Controller Interface
![Stanley Controller UML](docs/uml/stanley_interface.png)
[View UML Source](docs/uml/stanley_interface.uml)

---

## Installation & Simulation Environment
1. **Install the POLARIS_GEM_e2 simulator**  
   Follow instructions at [GEMillins / POLARIS_GEM_e2 · GitLab](https://gitlab.com/GEMillins/POLARIS_GEM_e2).

2. **Build the workspace**  
   ```
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

3. **Launch the simulator**  
   ```
   roslaunch gem_gazebo gem_gazebo_rviz.launch velodyne_points:="true"
   ```

## Launching the Decision-Making System

### 1. Choose and launch the controller
You can select either Stanley or Pure Pursuit controller using the `controller_type` argument.

**Stanley Controller:**
```
roslaunch polaris_decision_maker state_manager.launch controller_type:=stanley
roslaunch polaris_decision_maker controller.launch controller_type:=stanley
```

**Pure Pursuit Controller:**
```
roslaunch polaris_decision_maker state_manager.launch controller_type:=pure_pursuit
roslaunch polaris_decision_maker controller.launch controller_type:=pure_pursuit
```

### 2. Launch the mock sensor simulation
This node simulates sensor failures for scenario testing:
```
roslaunch polaris_decision_maker mock_sensor_sim.launch
```

## Monitoring Topics
- **Goal Reached:**
  ```
  rostopic echo /st/goal_reached
  rostopic echo /pp/goal_reached
  ```
- **Waypoints Sent:**
  ```
  rostopic echo /st/waypoints
  rostopic echo /pp/waypoints
  ```

## Assignment Features
- Modular state management (IDLE, RUNNING, ERROR)
- Sensor integration and automated error handling
- High-level planner sends waypoints only under safe conditions
- Flexible controller selection (Stanley or Pure Pursuit)
- Scenario-based testing with mock sensors

## Scenario Testing
The system supports the following scenarios via mock_sensors_node:
- Battery Failure
- Temperature Spike
- GPS Fluctuation
- Network Signal Fluctuation
- Emergency Stop

Integration tests verify that navigation tasks are sent only under safe conditions.

## Deliverables
- **Repository:** Modular, well-documented code
- **README:** This file
- **Dockerfile:** All dependencies for simulation and system
- **Demonstration Video:** Shows launch, navigation, and scenario testing
- **Performance Report:** Analysis of response times, robustness, and recovery

---

## Codebase Structure
The codebase is organized into modular components for clarity and scalability:

- **StateManager**: Handles operational states (IDLE, RUNNING, ERROR). Manages transitions based on sensor data and supports easy extension for new states or porting to ROS2.
- **SensorManager**: Monitors battery, temperature, GPS accuracy, network signal, and emergency stop. Triggers ERROR state and recovery based on sensor thresholds.
- **Planner**: High-level navigation planner. Interfaces with StateManager to send waypoints to the controller only when the robot is in a safe state.
- **Controller**: Path-tracking controller (Stanley or Pure Pursuit). Receives waypoints from Planner and executes navigation.
- **mock_sensors_node**: Simulates sensor failures and scenario events for testing. Used to verify system robustness and error handling.
- **TestScenarios**: Integration and scenario tests to ensure navigation tasks are sent only under safe conditions.

### Component Interactions
- SensorManager updates StateManager with sensor status.
- Planner queries StateManager before sending waypoints to Controller.
- mock_sensors_node simulates sensor events for scenario testing.
- TestScenarios verify correct system behavior and safe navigation.

Refer to the PlantUML diagram (`codebase_architecture.puml`) for a visual overview of these relationships.

---

## More Details: Video Demonstration
- The system currently does not have a dynamic global planner; instead, the controller uses waypoints from the existing `wps.csv` file.
- In `sensor_monitor.cpp`, four demonstration waypoints are added to the planner:
  - `{150.0, 100.0, 0.0}`
  - `{0.0, 197.0, 0.0}`
  - `{-130.0, 157.0, 0.0}`
  - `{-100.0, 11.0, 0.0}`
- For ease of demonstration, these waypoints are sent sequentially to the controller. The logic in `state_manager.cpp` sends the next waypoint once the previous goal is reached.
- This logic can be extended to accept waypoints from the user interactively; the system will wait for user input before proceeding to the next goal.
- The mission only starts when a manual trigger is set using the `/start_mission` topic.
- The controller stops for 5 seconds at each waypoint before executing the next, if available.
- Whenever a new waypoint is received, the system processes and sends it to the controller.

### Improvements / Future Work
- Implement a dynamic global planner to generate waypoints instead of relying on static `wps.csv` files.
- Ideally, the architecture should include a global planner, local planner, and controller for full autonomy.


