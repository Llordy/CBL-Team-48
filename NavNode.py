#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32


#-----------------
#    Constants
#-----------------
CONTROL_HZ=10
ARRIVAL_RADIUS = 0.5

# topics
CONTROL_TOPIC = "/cmd_vel"
GOAL_TOPIC =  '/nav/goal'

# proportional control
MAX_LINEAR = 0.22
LINEAR_GAIN = 1
MAX_ANGULAR = 1
ANGULAR_GAIN = 0.5
HEADING_THRESHOLD = 15

# Haversine constans
RADIUS = 6371008.7714 #mean
CIRCUM = 2*math.pi*RADIUS
METERS_PER_DEG = CIRCUM/360

def offset_position(lat, lon, offset_north, offset_east):
    delta_lat = offset_north / METERS_PER_DEG
    delta_lon = offset_east / (METERS_PER_DEG * math.cos(math.radians(lat)))

    return lat+delta_lat, lon+delta_lon

def heading_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return (720 - math.degrees(math.atan2(siny_cosp, cosy_cosp))) % 360

def rotate(angle: float, x, y):
    #angle should be in radians
    cos = math.cos(angle)
    sin = math.sin(angle)
    return (cos*x - sin*y,
            sin*x + cos*y)
def bearing_to(lat1,lon1, lat2, lon2):
    rlat1, rlon1, rlat2, rlon2 = list(map(math.radians, (lat1,lon1,lat2,lon2)))
    x= math.cos(rlat2) * math.sin(rlon2 - rlon1)
    y= (math.cos(rlat1)*math.sin(rlat2) - 
        math.sin(rlat1)*math.cos(rlat2)*math.cos(rlon2-rlon1))
    #range [0;360]
    return (720+math.degrees(math.atan2(x,y)))%360

def gps_distance(lat1,lon1,lat2,lon2):
    phi1, lam1, phi2, lam2 = list(map(math.radians, (lat1,lon1,lat2,lon2)))
    dphi = phi2-phi1
    dlam = lam2-lam1

    hav = 0.5*(1- math.cos(dphi) + math.cos(phi1) * math.cos(phi2) *(1 - math.cos(dlam)))

    theta = 2* math.asin(math.sqrt(hav))
    return theta * RADIUS

clamp = lambda _min,_max, x: min(_max,max(_min,x))


class NavNode(Node):
    def __init__(self):
        super().__init__('gps_subscriber')
        self.log = self.get_logger()
        self.create_subscription(NavSatFix, '/phone/gps/fix', self.gps_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Float32, '/heading', self.heading_callback, 10)
        self.create_subscription(Point, GOAL_TOPIC, self.new_goal, 10)
        self._cmd_pub = self.create_publisher(TwistStamped, CONTROL_TOPIC, 10)
        self._timer =self.create_timer(1/CONTROL_HZ, self.loop_callback)

        self.last_heading = 0;
        self.last_odom_heaidng =0;
        self._odom_heading=0;
        self._last_gps = (0,0)
        self._last_gps_pos = (0,0)

        self.last_pos = (0,0)

        self.goal =None;


    def new_goal(self, msg: Point):
        self.log.warn(f"New goal for: {msg.x}, {msg.y}")
        self.goal = (msg.x, msg.y)

    def gps_callback(self, msg:NavSatFix):
        if msg.status.status < 0: return  
        self._last_gps = (msg.latitude, msg.longitude)
        self._last_gps_pos = self.last_pos    

    def heading_callback(self, msg: float):
        self.last_heading = msg
        self.last_odom_heaidng = self._odom_heading

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        self.last_pos = pos.x, pos.y 
        self._odom_heading = heading_from_quaternion(msg.pose.pose.orientation)

    def publish_cmd(self, forward: float, turn: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x  = float(forward)
        msg.twist.angular.z = float(turn)
        self._cmd_pub.publish(msg)

    def loop_callback(self):
        if self.goal == None: 
            self.log.info("No goal")
            return
        start = time.monotonic()
        #calculate(guess) current position and heading
        #make the position offest in (north, east) coordinates
        #x- north, y - west
        position_offset = (self.last_pos[0] - self._last_gps_pos[0], 
                           -(self.last_pos[1] - self._last_gps_pos[1]))
        
        odom_heading_error = self.last_heading - self.last_odom_heaidng 
        ne_position_offset = rotate(math.radians(odom_heading_error), *position_offset)
        guesstimated_gps_pos = offset_position(*self._last_gps, *ne_position_offset)
        guesstimated_heading = (720 + self._odom_heading - odom_heading_error) % 360

        distance = gps_distance(*guesstimated_gps_pos, *self.goal)

        if distance < ARRIVAL_RADIUS: 
            self.goal = None
            self.publish_cmd(0,0)
            return 

        
        self.log.info(f"currently at: {guesstimated_gps_pos}, facing: {guesstimated_heading}")
        #calculate target bearing and the bearing offset
        target_bearing = bearing_to(*guesstimated_gps_pos, *self.goal)
        
        # range [-360;360]
        bearing_error = (target_bearing - guesstimated_heading)
        #normalize results to [-180;180]
        if bearing_error > 180: bearing_error -= 360
        elif bearing_error < -180: bearing_error += 360
        self.log.info(f"must face: {target_bearing}, error: {bearing_error}")
        #proportional drive
        angular_proportional = -ANGULAR_GAIN * bearing_error
        angular_proportional = clamp(-MAX_ANGULAR, MAX_ANGULAR, angular_proportional)

        linear_proportional = LINEAR_GAIN * distance
        linear_proportional = clamp(-MAX_LINEAR, MAX_LINEAR, linear_proportional)

        #dont go forward if facing away
        if abs(bearing_error) > HEADING_THRESHOLD:
            self.log.info("no forward")
            linear_proportional = 0

        self.log.info(f"forward speed: {linear_proportional}, turn rate: {angular_proportional}")

        #publish directions
        self.publish_cmd(linear_proportional, angular_proportional)
        self.log.info(f"loop time: {time.monotonic() - start}")


def main():
    rclpy.init()
    node = NavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



