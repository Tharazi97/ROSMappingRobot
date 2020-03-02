#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from nav_msgs.srv import GetMap
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped

import roslib
import tf
import geometry_msgs.msg
import math

def call_map():
        rospy.wait_for_service('dynamic_map')
        try:
                get_map = rospy.ServiceProxy('dynamic_map', GetMap)
                resp1 = get_map()
                return resp1
        except rospy.ServiceException, e:
                print "Service call failed: %s"%e

def call_plan(start_pose, goal_pose):
	rospy.wait_for_service('move_base/make_plan')
	try:
		get_plan = rospy.ServiceProxy('move_base/make_plan', GetPlan)
		req = GetPlan()
		req.start = start_pose
		req.goal = goal_pose
		req.tolerance = 0.25
		resp1 = get_plan(req.start, req.goal, req.tolerance)
		return resp1
        except rospy.ServiceException, e:
                print "Service call failed: %s"%e


if __name__ == '__main__':
        rospy.init_node('autonomus_path_requester', anonymous=False)

	listener = tf.TransformListener()

        map_pub = rospy.Publisher("auto_map", OccupancyGrid, queue_size=10)
	path_pub = rospy.Publisher("auto_path", Path, queue_size=10)
	goal_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)

        rate = rospy.Rate(0.01) # 0.15hz

	robot_pos_in_map = 0
	robot_pos_in_map_x = 0
	robot_pos_in_map_y = 0
	rospy.sleep(2.0)

        while not rospy.is_shutdown():

                full_map = call_map()
                created_map = OccupancyGrid()
                created_map.header = full_map.map.header
                created_map.info = full_map.map.info
		start_pose = PoseStamped()

		try:
                        trans_base_link_map = listener.lookupTransform('/base_link', '/map', rospy.Time(0))
			robot_pos_in_map_x = round((trans_base_link_map[0][0]-full_map.map.info.origin.position.x)/full_map.map.info.resolution)
			robot_pos_in_map_y = round((trans_base_link_map[0][1]-full_map.map.info.origin.position.y)/full_map.map.info.resolution)
			robot_pos_in_map = (robot_pos_in_map_y*full_map.map.info.width + robot_pos_in_map_x)
			#print robot_pos_in_map

			start_pose.pose.position.x = trans_base_link_map[0][0]
			start_pose.pose.position.y = trans_base_link_map[0][1]
			start_pose.pose.position.z = trans_base_link_map[0][2]
			start_pose.pose.orientation.x = trans_base_link_map[1][0]
			start_pose.pose.orientation.y = trans_base_link_map[1][1]
			start_pose.pose.orientation.z = trans_base_link_map[1][2]
			start_pose.pose.orientation.w = trans_base_link_map[1][3]

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        print 'fail'

                lowest_dist_index = 0
		lowest_dist = 99999
		found_any = False

		#for i in range(full_map.map.info.width * full_map.map.info.height):
		for i in range(len(full_map.map.data)):
			if i == round(robot_pos_in_map):
				created_map.data.append(-2)
			else:
		                if full_map.map.data[i] == 0:
		                        if full_map.map.data[i-1] == -1 or full_map.map.data[i+1] == -1 or full_map.map.data[i-full_map.map.info.width] == -1 or full_map.map.data[i+full_map.map.info.width] == -1:
		                                created_map.data.append(120)
						x = i % full_map.map.info.width
						y = i / full_map.map.info.width
						distance = math.sqrt((robot_pos_in_map_x-x)*(robot_pos_in_map_x-x)+(robot_pos_in_map_y-y)*(robot_pos_in_map_y-y))
						if distance < lowest_dist:
							lowest_dist_index = i
							lowest_dist = distance
							found_any = True
		                        else:
		                                created_map.data.append(full_map.map.data[i])
		                else:
		                        created_map.data.append(full_map.map.data[i])
                map_pub.publish(created_map)


		#print lowest_dist

		if found_any:
			goal_pose = PoseStamped()
			goal_pose.header.stamp = rospy.Time.now()
			goal_pose.header.frame_id = "map"
			goal_pose.pose.position.x = (lowest_dist_index % full_map.map.info.width)*full_map.map.info.resolution + full_map.map.info.origin.position.x
			goal_pose.pose.position.y = (lowest_dist_index / full_map.map.info.width)*full_map.map.info.resolution + full_map.map.info.origin.position.y
			goal_pose.pose.position.z = 0
			goal_pose.pose.orientation.x = 0
			goal_pose.pose.orientation.y = 0
			goal_pose.pose.orientation.z = 0
			goal_pose.pose.orientation.w = 1
			#print goal_pose

			start_pose.header.stamp = rospy.Time.now()
			start_pose.header.frame_id = "map"

			ret = call_plan(start_pose, goal_pose)
			ret.plan.header.frame_id = "map"
			path_pub.publish(ret.plan)
			thirds_path = ret.plan.poses[len(ret.plan.poses)/3]
			goal_pub.publish(thirds_path)


                rate.sleep()







