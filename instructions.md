# TurtleBot3 Launch Instructions

## Prerequisites

Ensure the following are available on your system:

- ROS 2 Jazzy installed at `/opt/ros/jazzy/`
- TurtleBot3 workspace installed at `/opt/turtlebot3_ws/`
- `colcon` build tool
- Python 3


## Step 1 — Navigate to the Bundle Root

```bash
cd /path/to/your/bundle
```

All subsequent commands must be run from this directory.


## Step 2 — Source the ROS 2 and TurtleBot3 Environments

```bash
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
```


## Step 3 — Build the Workspace

```bash
colcon build
source install/setup.bash
```

Wait for the build to complete before continuing.


## Step 4 — Set the TurtleBot3 Model

```bash
export TURTLEBOT3_MODEL=burger
```


## Step 5 — (Optional) Launch Gazebo

If you are using the simulation instead of a physical robot, run:

```bash
ros2 launch my_tb3_world new_world.launch.py &
```

If you are using a physical TurtleBot3, skip this step and ensure the robot is powered on and connected to ROS2.


## Step 6 — Launch the Navigation Node

```bash
python3 NavNode.py &
```


## Step 7 — Launch Obstacle Avoidance

```bash
ros2 launch tb3_twin_avoidance twin_avoidance.launch.py &
```


## Step 8 — Launch the Battery Monitor and Return-to-Base

```bash
python3 ./BatteryMonitor.py &
python3 ./ReturnToBase.py &
```


## Step 9 — Launch the Dashboard

```bash
cd ./dashboard/
./run.sh &
cd ..
```

Once the dashboard is running, open it in Firefox:

```bash
firefox http://localhost:8080
```

Or manually navigate to **http://localhost:8080** in your Firefox browser.

