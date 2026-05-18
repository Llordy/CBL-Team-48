
# Preparation
## On the turtlebot/lab laptop

### On the turtlebot (SSH):
Run: 
```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02
ros2 launch turtlebot3_bringup robot.launch.py
```

### On the laptop:
For gazebo:
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

For teleop:
```bash
ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r /cmd_vel:=cmd_vel
```

## On docker


# GPS

### Phone:
Install the `GPSPublisher(http(s) relay).apk` and launch it.
In the app enter `puchalski.xyz` as the destination and click start publishing.

### On the laptop/turtlebot:
Clone this github repo and run `GPSPollingPublisher.py`. You need to specify 
the URL and token for the program for example:
```bash
python3 GPSPollingPublisher.py \
    --relay https://puchalski.xyz \
    --token super_secret_token_shared_on_whatsapp
```
You should now see a readout of the received gps values. Other nodes should be 
able to access the gps position at `/phone/gps/fix`



