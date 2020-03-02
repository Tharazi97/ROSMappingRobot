#!/usr/bin/env python

import rospy
import os
import subprocess
from std_msgs.msg import Bool
from std_msgs.msg import Int16

global progress

def callbackProgress(msg):
        global progress
        progress = msg.data

if __name__ == '__main__':
	global progress
	progress = 0
	rospy.init_node('user_input', anonymous=False)
	scan3D_pub = rospy.Publisher("make_3DScan", Int16, queue_size=10)
	auto_map_pub = rospy.Publisher("auto_map_start", Bool, queue_size=10)
	rospy.Subscriber("progress_map3D", Int16, callbackProgress)
	os.system('clear')

	while not rospy.is_shutdown():
		print("Dostepne komendy:\n")
		print("skan3D       - wykonuje skan3D pomieszczenia. Kat nachylenia podaje sie jako kolejna komenda.\n")
		print("wlacz        - wlacza automatyczne mapowanie 2D pomieszczenia.\n")
		print("wylacz       - wylacza automatyczne mapowanie 2D pomieszczenia.\n")
		print("zapiszMape   - zapisuje mape 2D pomieszczenia.\n")
		print("zapiszSkan3D - zapisuje ostatnia chmure punktow 3D.\n")
		print("teleop       - wlacza program do recznego porusznia robotem.\n")
		print("wyjdz        - wylacza ten program.\n")
		fnc = raw_input("Podaj komende:\n")
		if fnc == "skan3D":
			value = input("Podaj do jakiego stopnia nachylenia wykonac skan:\n")
			if value > 90:
				value = 90
			doScan3D = Int16()
			doScan3D.data = value
			scan3D_pub.publish(doScan3D)
			os.system('clear')
			while progress < value:
				print("wykonano: " + str(progress) + "/" + str(value) + " skanow.")
				rospy.sleep(1)
				os.system('clear')
			progress = 0

		elif fnc == "wlacz":
			startAuto = Bool()
			startAuto.data = True
			auto_map_pub.publish(startAuto)

		elif fnc == "wylacz":
			stopAuto = Bool()
                        stopAuto.data = False
                        auto_map_pub.publish(stopAuto)

		elif fnc == "zapiszMape":
			os.system("rosrun map_server map_saver -f /home/poli/github/ROS-mapping-robot/map")

		elif fnc == "zapiszSkan3D":
			os.system('''timeout 5s bash -c "rosrun pcl_ros pointcloud_to_pcd input:=/points2_out _prefix:=/home/poli/github/ROS-mapping-robot/"''')

		elif fnc == "teleop":
			os.system("rosrun mapping_robot teleop.py")

		elif fnc == "wyjdz":
			break
		rospy.sleep(2)
		os.system('clear')

