#!/usr/bin/env python
import rospy
from math import pi, cos, sin, atan2, sqrt

#import ros msgs
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

right_wheel_cmd = rospy.Publisher('/rhodent/right_wheel_velocity_controller/command', Float64, queue_size=10)
left_wheel_cmd = rospy.Publisher('/rhodent/left_wheel_velocity_controller/command', Float64, queue_size=10)

rospy.init_node('diff_kinematics')

# def polar_to_cartesian(r, th):
#     y = r*sin(th)
#     x = r*cos(th)
#     return x, y

CENTER_TO_WHEEL = 0.101
WHEEL_RADIUS = 0.03225

def twist_to_wheel_vel(twist):
    heading_vel = twist.linear.x
    ang_vel = twist.angular.z

    r_cmd_vel = (heading_vel+ang_vel*CENTER_TO_WHEEL/2)/WHEEL_RADIUS
    l_cmd_vel = (heading_vel-ang_vel*CENTER_TO_WHEEL/2)/WHEEL_RADIUS

    print (r_cmd_vel, l_cmd_vel)

    right_wheel_cmd.publish(r_cmd_vel)
    left_wheel_cmd.publish(l_cmd_vel)
    

if __name__ == '__main__':
    try:
        rospy.Subscriber('rhodent/cmd_vel', Twist, twist_to_wheel_vel)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass