#!/usr/bin/env python
import rospy
import tf
from math import pi, cos, sin, atan2, sqrt

#import ros msgs
from geometry_msgs.msg import Twist, Point, Pose, Quaternion, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

right_wheel_cmd = rospy.Publisher('/rhodent/right_wheel_velocity_controller/command', Float64, queue_size=10)
left_wheel_cmd = rospy.Publisher('/rhodent/left_wheel_velocity_controller/command', Float64, queue_size=10)
odom_broadcaster = tf.TransformBroadcaster()
odom_publisher = rospy.Publisher('rhodent/odom', Odometry, queue_size=20)

rospy.init_node('diff_kinematics')

CENTER_TO_WHEEL = 0.101
WHEEL_RADIUS = 0.03225

# dt = 1/100.0
sum_theta = 0
sum_x = 0
sum_y = 0
prev_time = 0



def twist_to_wheel_vel(twist):
    heading_vel = twist.linear.x
    ang_vel = twist.angular.z

    r_cmd_vel = (heading_vel+ang_vel*CENTER_TO_WHEEL)/WHEEL_RADIUS
    l_cmd_vel = (heading_vel-ang_vel*CENTER_TO_WHEEL)/WHEEL_RADIUS

    # print (r_cmd_vel, l_cmd_vel)

    right_wheel_cmd.publish(r_cmd_vel)
    left_wheel_cmd.publish(l_cmd_vel)
    
def odom_callback(joint_states):
    global sum_theta
    global sum_x
    global sum_y
    global prev_time

    lwh = joint_states.velocity[0]
    rwh = joint_states.velocity[1]

    vl = lwh * WHEEL_RADIUS
    vr = rwh * WHEEL_RADIUS

    v = (vr+vl)/2.0
    theta_dot = (vr-vl)/(2.0*CENTER_TO_WHEEL)

    time_now = joint_states.header.stamp.secs+joint_states.header.stamp.nsecs*1e-9
    dt = time_now-prev_time
    sum_theta = sum_theta + theta_dot*dt 
    x_dot = v*cos(sum_theta)
    y_dot = v*sin(sum_theta)

    sum_x = sum_x + x_dot*dt
    sum_y = sum_y + y_dot*dt

    q = tf.transformations.quaternion_from_euler(0, 0, sum_theta)
    odom_broadcaster.sendTransform((sum_x, sum_y, 0), q, rospy.Time.now(), "base_footprint", "odom")

    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "odom"
    odom.pose.pose.position = Point(sum_x, sum_y, 0)
    odom.pose.pose.orientation.x = q[0]
    odom.pose.pose.orientation.y = q[1]
    odom.pose.pose.orientation.z = q[2]
    odom.pose.pose.orientation.w = q[3]
    odom.child_frame_id = "base_footprint"
    odom.twist.twist = Twist(Vector3(x_dot,y_dot, 0), Vector3(0, 0, theta_dot))
    odom_publisher.publish(odom)

    prev_time = time_now

if __name__ == '__main__':
    try:
        rospy.Subscriber('rhodent/cmd_vel', Twist, twist_to_wheel_vel)
        rospy.Subscriber('rhodent/joint_states', JointState, odom_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass