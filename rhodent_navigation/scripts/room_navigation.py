#!/usr/bin/env python
import rospy
import tf
from math import pi, cos, sin, atan2, sqrt
from anytree import Node, RenderTree, Walker

#import ros msgs
from geometry_msgs.msg import Twist, Point, Pose, Quaternion, Vector3
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from move_base_msgs.msg import MoveBaseActionGoal

goal_publisher = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
marker_publisher = rospy.Publisher('/rhodent/via_points', MarkerArray, queue_size=10)

room_list = ["corridor", "storage", "dining", "kitchen", "living_room", "bed_room", "office"]

#Create the tree for rooms with the corridor as the root
corridor = Node("corridor")
storage = Node("storage", parent=corridor)
dining = Node("dining", parent=corridor)
kitchen = Node("kitchen", parent=dining)
living_room = Node("living_room", parent=corridor)
bed_room = Node("bed_room", parent=living_room)
office = Node("office", parent=corridor)

via_point_tol = 2 #goto the next viapoint after reaching the current viapoint within this distance (m)
goal_tol = 0.2 #final position x,y tolerance

# corridor_area = [[4, -4],[-1, 2]] #tr bl
# living_room_area = [[-5, -1],[-7.5, 2.7]]
# dining_area = [[4, 5.45],[-1.5, 9]]

#define room positions (x,y)
corridor_point = [1.3, -0.85]
storage_point = [2.9, -8.5]
dining_point = [-0.5, 7]
kitchen_point = [1, 13]
living_room_point = [-6.3, 0.8]
bed_room_point = [-9, 9]
office_point = [-4.4, -9]

#add room positions to the room_points list according to the original list
room_points = []
for i in room_list:
    exec("room_points.append({}_point)".format(i))

rospy.init_node("room_navigation") #initialize this node

listener = tf.TransformListener()

def get_path(here, togo):
    path_here = str(here).split("\'")[1].split("/")[1:]
    path_togo = str(togo).split("\'")[1].split("/")[1:]
    if togo.name in path_here: #initial point lies between the goal and the root (reverse in tree)
        idx = path_here.index(here.name)
        idx_here = path_here.index(togo.name)
        path = path_here[idx_here:idx]
    elif here.name not in path_togo: #initial point lies between the root and the goal
        path_here = [path_here[1:][::-1]] if type(path_here[1:][::-1]) == str else path_here[1:][::-1]
        path = (path_here + path_togo)[1:]
    else: #the initial point and the goal is in different branch
        path = [room for room in path_togo if room not in path_here]
    rospy.loginfo("Generated room path: %s", str(path))
    return path

def get_dist_to_go(goal_point):
    here,_ = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0)) #get the base_link position
    dist_to_goal = sqrt((here[0]-goal_point[0])**2 + (here[1]-goal_point[1])**2) #cartesian distance x,y
    return here,dist_to_goal #return both current position and the distance-to-go

def room_nav_callback(togo):
    goal = MoveBaseActionGoal() #goal place holder
    here,_ = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0)) 
    
    if togo.data in room_list:
        distsq = []
        for i in room_points:
            distsq.append((here[0]-i[0])**2 + (here[1]-i[1])**2)
        val, idx = min((val, idx) for (idx, val) in enumerate(distsq))
        here_room = room_list[idx]
        exec("path = get_path({}, {})".format(here_room, togo.data))
        via_points_marker = MarkerArray()
        for num,viapoint in enumerate(path):
            msize = via_point_tol if num != len(path)-1 else goal_tol #visualize the tolerance radius
            mbl = 1 if num != len(path)-1 else 0 #select the color
            mgr = 0 if num != len(path)-1 else 1
            exec('marker_point ='+viapoint+'_point')
            marker = Marker()
            marker.id = num
            marker.header.frame_id = "/map"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = msize*2
            marker.scale.y = msize*2
            marker.scale.z = msize*2
            marker.color.a = 0.2
            marker.color.r = 0.0
            marker.color.g = mgr
            marker.color.b = mbl
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = marker_point[0]
            marker.pose.position.y = marker_point[1]
            marker.pose.position.z = 0
            via_points_marker.markers.append(marker) #send markers
        marker_publisher.publish(via_points_marker)
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
            goal_publisher.publish(goal) #publish current viapoint
            here, dist_to_goal = get_dist_to_go(goal_point) 
            while(dist_to_goal > via_point_tol): #while not entering the current viapoint position
                rospy.loginfo("Driving to way point... Distance left: {dist:.2f} meters.".format(dist=dist_to_goal))
                here, dist_to_goal = get_dist_to_go(goal_point) 
                if togo.data == viapoint:
                    break
                rospy.sleep(0.5)
        exec('goal_point ='+togo.data+'_point') #Navigation to the final goal
        here, dist_to_goal = get_dist_to_go(goal_point)
        while(dist_to_goal > goal_tol): 
            here, dist_to_goal = get_dist_to_go(goal_point)
            rospy.loginfo("On its way to destination... Distance left: {dist:.2f} meters.".format(dist=dist_to_goal))
            rospy.sleep(0.25)
        # This part keeps the final orientation of the robot by publishing the pose after entering the final goal radius
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