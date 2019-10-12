#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <mapping_robot/Point.h>

#define DEBUG

// We are using this subscriber publiher class to ease using publish and subscribe at the same time
class SubPub {
	public:	
	void callback(const mapping_robot::Point msg) {
	#if defined(DEBUG)
		ROS_INFO("I heard: [%d]", msg.distance);
	#endif
	}
	
	ros::NodeHandle n;
	ros::Publisher pub;
	ros::Subscriber sub;
	
	SubPub() {
		// Inform master that we will be publishing Bool msgs on the create_map toppic
		pub = n.advertise<std_msgs::Bool>("create_map", 10);

		// Inform master that we will be subscribing msgs on the measured_points toppic.
		// Everytime msg occurs on measured_points toppic callback method of this object will be invoked 
		sub = n.subscribe("measured_points", 1000, &SubPub::callback, this);
		
		// Sleep for a little bit just not to publish something just after calling advertise function (first messages could not be published)
		ros::Duration(0.5).sleep();
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mapper");
	SubPub subPub;

	// Create start msg to invoke lidar_wrapper to start scanning
	std_msgs::Bool msg;
	msg.data = 1;

	subPub.pub.publish(msg);

#if defined(DEBUG)
ROS_INFO("I :");
#endif

	// Keep the program running waiting for msgs
	ros::spin();

	return 0;

}
