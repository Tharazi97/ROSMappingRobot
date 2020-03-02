#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from nav_msgs.srv import GetMap
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Bool

import roslib
import tf
import geometry_msgs.msg
import math

statusPlan = 0
full_map = OccupancyGrid()
isProcesing = False
start_auto_mapping = False

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
		req.tolerance = 0.19
		resp1 = get_plan(req.start, req.goal, req.tolerance)
		return resp1
        except rospy.ServiceException, e:
                print "Service call failed: %s"%e


def callback_plan(data):
	global statusPlan
	if len(data.status_list) > 0:
		statusPlan = data.status_list[len(data.status_list)-1].status

def callback_map(data):
	global full_map
	global isProcesing
	if not isProcesing:
		full_map = data

def callback_auto(data):
	global start_auto_mapping
	start_auto_mapping = data.data

if __name__ == '__main__':
	global statusPlan
	global full_map
	global isProcesing
	global start_auto_mapping
        rospy.init_node('autonomus_path_requester', anonymous=False)

	listener = tf.TransformListener()

        map_pub = rospy.Publisher("auto_map", OccupancyGrid, queue_size=10)
	path_pub = rospy.Publisher("auto_path", Path, queue_size=10)
	goal_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)

	rospy.Subscriber("/move_base/status", GoalStatusArray, callback_plan);
	rospy.Subscriber("/map", OccupancyGrid, callback_map);
	rospy.Subscriber("/auto_map_start", Bool, callback_auto);

        rate = rospy.Rate(1)

	robot_pos_in_map = 0
	robot_pos_in_map_x = 0
	robot_pos_in_map_y = 0
	rospy.sleep(20.0)
	start_pose = PoseStamped()

        while not rospy.is_shutdown():
		if start_auto_mapping == True:
			print("call map")
			isProcesing = True
	                #full_map = call_map()
	                created_map = OccupancyGrid()
	                created_map.header = full_map.header
	                created_map.info = full_map.info
			#start_pose = PoseStamped()

			try:
	                        trans_base_link_map = listener.lookupTransform("/map", "/base_link", listener.getLatestCommonTime("/base_link","/map"))
				robot_pos_in_map_x = round((trans_base_link_map[0][0]-full_map.info.origin.position.x)/full_map.info.resolution)
				robot_pos_in_map_y = round((trans_base_link_map[0][1]-full_map.info.origin.position.y)/full_map.info.resolution)
				robot_pos_in_map = round(robot_pos_in_map_y*full_map.info.width + robot_pos_in_map_x)
				#print robot_pos_in_map

				start_pose.pose.position.x = trans_base_link_map[0][0]
				start_pose.pose.position.y = trans_base_link_map[0][1]
				start_pose.pose.position.z = trans_base_link_map[0][2]
				start_pose.pose.orientation.x = trans_base_link_map[1][0]
				start_pose.pose.orientation.y = trans_base_link_map[1][1]
				start_pose.pose.orientation.z = trans_base_link_map[1][2]
				start_pose.pose.orientation.w = trans_base_link_map[1][3]

	                #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
	                except:
			        print 'fail'
			print start_pose
			found_any = False
			distances = []
			indexes = []
			print("processing map")
			#for i in range(full_map.map.info.width * full_map.map.info.height):
			for i in range(len(full_map.data)):
				if i == robot_pos_in_map:
					created_map.data.append(-2)
				else:
			                if full_map.data[i] == 0:
			                        if full_map.data[i-1] == -1 or full_map.data[i+1] == -1 or full_map.data[i-full_map.info.width] == -1 or full_map.data[i+full_map.info.width] == -1:
			                                created_map.data.append(120)
							x = i % full_map.info.width
							y = i / full_map.info.width
							distance = math.sqrt((robot_pos_in_map_x-x)*(robot_pos_in_map_x-x)+(robot_pos_in_map_y-y)*(robot_pos_in_map_y-y))
							distances.append(distance)
							indexes.append(i)
							found_any = True
			                        else:
			                                created_map.data.append(full_map.data[i])
			                else:
			                        created_map.data.append(full_map.data[i])
	                map_pub.publish(created_map)


			#print lowest_dist

			if found_any:
				sortedDist = sorted(zip(distances,indexes))
				print("trying plans")
				for i in sortedDist:
					goal_pose = PoseStamped()
					goal_pose.header.stamp = rospy.Time.now()
					goal_pose.header.frame_id = "map"
					goal_pose.pose.position.x = (i[1] % full_map.info.width)*full_map.info.resolution + full_map.info.origin.position.x
					goal_pose.pose.position.y = (i[1] / full_map.info.width)*full_map.info.resolution + full_map.info.origin.position.y
					goal_pose.pose.position.z = 0
					goal_pose.pose.orientation.x = 0
					goal_pose.pose.orientation.y = 0
					goal_pose.pose.orientation.z = 0
					goal_pose.pose.orientation.w = 1
					#print goal_pose

					start_pose.header.stamp = rospy.Time.now()
					start_pose.header.frame_id = "map"

					print("call plan")
					ret = call_plan(start_pose, goal_pose)
					if ret:
						ret.plan.header.frame_id = "map"
						#print ret.plan
						if len(ret.plan.poses) == 0: # jesli nie udalo sie wyznaczyc planu, sprobuj kolejny punkt
							continue
						path_pub.publish(ret.plan)
						#if (i[0]*full_map.map.info.resolution < 1):
						j = 1
						stop = False
						while (j < len(ret.plan.poses)) and (not rospy.is_shutdown()):
							x = ret.plan.poses[len(ret.plan.poses)-j].pose.position.x
							y = ret.plan.poses[len(ret.plan.poses)-j].pose.position.y
							lenghtFromRobot = math.sqrt((trans_base_link_map[0][0]-x)*(trans_base_link_map[0][0]-x)+(trans_base_link_map[0][1]-y)*(trans_base_link_map[0][1]-y))
							if lenghtFromRobot < 0.2:
								break
							lenghtFromPoint = math.sqrt((goal_pose.pose.position.x-x)*(goal_pose.pose.position.x-x)+(goal_pose.pose.position.y-y)*(goal_pose.pose.position.y-y))
							if lenghtFromPoint > 0.35:
								goal_pub.publish(ret.plan.poses[len(ret.plan.poses)-j])
								rospy.sleep(1.0)
								#print(len(ret.plan.poses))
								while ((statusPlan == 1) or (statusPlan == 0)) and (not rospy.is_shutdown()):
									rospy.sleep(0.1)
									print("sleep")
								if statusPlan == 3:
									stop = True
									break
							j = j+1
							print("derp")
						if stop == True:
							break
						#else:
						#	j = 1
						#	stop = False
						#	while j < (len(ret.plan.poses)-1):
						#		x = ret.plan.poses[j].pose.position.x
	                                        #        	y = ret.plan.poses[j].pose.position.y
						#		lenght = math.sqrt((trans_base_link_map[0][0]-x)*(trans_base_link_map[0][0]-x)+(trans_base_link_map[0][1]-y)*(trans_base_link_map[0][1]-y))
						#		print(lenght)
						#		if lenght > 1.0:
						#			goal_pub.publish(ret.plan.poses[j])
		                                #                        rospy.sleep(1.0)
		                                #                        while (statusPlan == 1) or (statusPlan == 0):
						#				rospy.sleep(0.1)
						#			if statusPlan == 3:
						#				stop = True
						#				break
						#		j=j+5
						#		print("derp")
						#	if stop == True:
						#		break
						#	else:
						#		goal_pub.publish(ret.plan.poses[len(ret.plan.poses)-1])
	                                        #                rospy.sleep(1.0)
	                                        #                while (statusPlan == 1) or (statusPlan == 0):
	                                        #                        rospy.sleep(0.1)
	                                        #                if statusPlan == 3:
	                                        #                        break
					rospy.sleep(1.0)
	                isProcesing = False
			rospy.sleep(45.0)
		rate.sleep()






