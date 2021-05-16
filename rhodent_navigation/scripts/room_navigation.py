#!/usr/bin/env python
import rospy
import tf
from math import pi, cos, sin, atan2, sqrt
from anytree import Node, RenderTree, Walker

#import ros msgs
from geometry_msgs.msg import Twist, Point, Pose, Quaternion, Vector3
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseActionGoal

goal_publisher = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)

room_list = ["corridor", "storage", "dining", "kitchen", "living_room", "bed_room", "office"]

corridor = Node("corridor")
storage = Node("storage", parent=corridor)
dining = Node("dining", parent=corridor)
kitchen = Node("kitchen", parent=dining)
living_room = Node("living_room", parent=corridor)
bed_room = Node("bed_room", parent=living_room)
office = Node("office", parent=corridor)

via_point_tol = 2
goal_tol = 0.2

# corridor_area = [[4, -4],[-1, 2]] #tr bl
# living_room_area = [[-5, -1],[-7.5, 2.7]]
# dining_area = [[4, 5.45],[-1.5, 9]]

corridor_point = [1.3, -0.85]
storage_point = [2.9, -8.5]
dining_point = [-0.5, 7]
kitchen_point = [1, 13]
living_room_point = [-6.3, 0.8]
bed_room_point = [-9, 9]
office_point = [-4.4, -9]
room_points = []
for i in room_list:
    exec("room_points.append({}_point)".format(i))

# print(room_points)

rospy.init_node("room_navigation")

listener = tf.TransformListener()

def get_path(here, togo):
    path_here = str(here).split("\'")[1].split("/")[1:]
    path_togo = str(togo).split("\'")[1].split("/")[1:]
    if togo.name in path_here: #reverse in tree
        idx = path_here.index(here.name)
        idx_here = path_here.index(togo.name)
        path = path_here[idx_here:idx]
    elif here.name not in path_togo:
        path_here = [path_here[1:][::-1]] if type(path_here[1:][::-1]) == str else path_here[1:][::-1]
        path = (path_here + path_togo)[1:]
    else:
        path = [room for room in path_togo if room not in path_here]
    rospy.loginfo("Generated room path: %s", str(path))
    return path

def get_dist_to_go(goal_point):
    here,_ = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
    dist_to_goal = sqrt((here[0]-goal_point[0])**2 + (here[1]-goal_point[1])**2)
    return here,dist_to_goal

def room_nav_callback(togo):
    goal = MoveBaseActionGoal()
    here,_ = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
    
    if togo.data in room_list:
        distsq = []
        for i in room_points:
            distsq.append((here[0]-i[0])**2 + (here[1]-i[1])**2)
        val, idx = min((val, idx) for (idx, val) in enumerate(distsq))
        here_room = room_list[idx]
        exec("path = get_path({}, {})".format(here_room, togo.data))
        for viapoint in path:
            exec('goal_point ='+viapoint+'_point')
            print (viapoint, goal_point)
            goal.goal.target_pose.header.stamp = rospy.Time.now()
            goal.goal.target_pose.header.frame_id = 'map'
            goal.goal.target_pose.pose.position.x = goal_point[0]
            goal.goal.target_pose.pose.position.y = goal_point[1]
            goal.goal.target_pose.pose.orientation.w = 1
            goal.goal.target_pose.pose.orientation.x = 0
            goal.goal.target_pose.pose.orientation.y = 0
            goal.goal.target_pose.pose.orientation.z = 0
            goal_publisher.publish(goal)
            here, dist_to_goal = get_dist_to_go(goal_point)
            while(dist_to_goal > via_point_tol): 
                rospy.loginfo("Driving to way point... Distance left: {dist:.2f} meters.".format(dist=dist_to_goal))
                here, dist_to_goal = get_dist_to_go(goal_point) 
                if togo.data == viapoint:
                    break
                rospy.sleep(0.5)
        exec('goal_point ='+togo.data+'_point')
        here, dist_to_goal = get_dist_to_go(goal_point)
        while(dist_to_goal > goal_tol): 
            here, dist_to_goal = get_dist_to_go(goal_point)
            rospy.loginfo("On its way to destination... Distance left: {dist:.2f} meters.".format(dist=dist_to_goal))
            rospy.sleep(0.25)
        here,rot = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        goal.goal.target_pose.header.stamp = rospy.Time.now()
        goal.goal.target_pose.header.frame_id = 'map'
        goal.goal.target_pose.pose.position.x = here[0]
        goal.goal.target_pose.pose.position.y = here[1]
        goal.goal.target_pose.pose.orientation.x = rot[0]
        goal.goal.target_pose.pose.orientation.y = rot[1]
        goal.goal.target_pose.pose.orientation.z = rot[2]
        goal.goal.target_pose.pose.orientation.w = rot[3]
        goal_publisher.publish(goal)
        rospy.loginfo("Destination reached!")
    else:
        rospy.logwarn("\"{}\" room does not exist.".format(togo.data))


if __name__ == '__main__':
    try:
        rospy.Subscriber('rhodent/room_cmd', String, room_nav_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass