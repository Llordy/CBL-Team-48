
# TurtleBot3 Launch Instructions

## Prerequisites

Ensure to have the following available:

- An Android phone (Should be compatible with Android 7+, tested on Android 15) 
with the ability to install APKs (The APK requires GooglePlay services for location)
- A Turtlebot3 (or Gazebo equivalent) set up in accordance with 
[instructions.pdf](instructions.pdf)

## Step 1 --- Choosing an http relay:
There are two supported solutions. If possible connecting the phone to the same 
network and hosting a private relay is preferable, alternately it's possible to use 
[https://gps.puchalski.xyz](https://gps.puchalski.xyz) as a relay.

### Option 1 --- Hosting the relay:
This step requires a host machine on the same network as the ROS laptop and the 
GPS phone (can be the same as the laptop). It uses TCP port 5080 so the user must 
ensure it's not blocked by firewall and not in use by other programs. This step 
requires `cargo`. Instructions to install cargo can be found [here](https://doc.rust-lang.org/cargo/getting-started/installation.html). 
Once cargo is installed, from the bundle root, run:
```bash
cd gps-api
cargo r -r 
```
Once the compilation is done, you'll have a relay running on port 5080.
To check that the relay is working you can open http://\<relayip\>:5080/gps in 
your browser of choice. 

> Note: \<relayip\> is the IP address of your relay. On Linux you can check it 
> using `ip a`, on Windows `ipconfig`.

In the rest of the document please replace `<relay>` with `<relayip>:5080`.

### Option 2 --- Using the provided relay:
There is a publicly available relay at https://gps.puchalski.xyz
for the rest of the document please replace `<relay>` with `gps.puchalski.xyz`.
You can check the relay is working by opening `https://gps.puchalski.xyz/gps` in 
your browser fo choice

> Note: Due to network configuration at the TUE, the one-way communication delay 
> between the phone and the robot is upwards of three seconds. 

## Step 2 --- Publishing GPS to relay:
On your Android phone install `GPSapp.apk`. Once the app is installed start it, 
In the text field labelled `Laptop IP address` enter `<relay>` and press `start 
publishing`. When prompted allow location access.

## Step 3 --- Receiving GPS into ROS:
On the ROS laptop, from the bundle directory, run:
```bash
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
python3 ./GPSPollingPublisher.py \
    --relay http://<relay> \
    --token weloveah
```
You should be good to go. To check whether everything works smoothly you can run:
```bash
ros2 topic echo /phone/gps/fix
```
You should see the phone's position in the terminal.




