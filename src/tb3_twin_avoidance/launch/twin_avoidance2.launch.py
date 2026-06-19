from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   return LaunchDescription([
       Node(
           package='tb3_twin_avoidance',
           executable='twin_avoidance_node',
           output='screen',
           parameters=[
                {'real_scan_topic': '/scan'},
                {'sim_scan_topic': '/scan'},
                {'input_cmd_topic': '/cmd_vel_raw'},
                {'real_cmd_topic': '/cmd_vel'},
                {'sim_cmd_topic': '/cmd_vel'},

                {'use_nav_services': True},

                {'stop_distance': 0.35},
                {'front_clear_distance': 0.60},
                {'side_clear_distance': 0.45},

                {'front_angle_deg': 30.0},
                {'side_sector_center_deg': 90.0},
                {'side_sector_half_width_deg': 35.0},

                {'turn_speed': 0.7},
                {'pass_speed': 0.12},

                {'takeover_only_on_forward_motion': True},
                {'use_min_for_side_score': True},
           ]
       )
   ])
