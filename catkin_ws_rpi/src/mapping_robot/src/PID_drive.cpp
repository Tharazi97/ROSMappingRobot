#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>
#include "mapping_robot/Speeds.h"
#include "mapping_robot/ChangeDir.h"

#define motor_max 255
#define motor_min (-255)

// We are using this subscriber publiher class to ease using publish and subscribe at the same time
class SubPub {
	public:	
	void callbackWanted(const geometry_msgs::Twist msg) {
        wantedDFiL = (msg.linear.x / wheelR) + ((shaft * msg.angular.z) / (2 * wheelR));
        wantedDFiR = (msg.linear.x / wheelR) - ((shaft * msg.angular.z) / (2 * wheelR));
	}

    void callbackSpeeds(const mapping_robot::Speeds speeds) {
        currentDFiL = speeds.left;
        currentDFiR = speeds.right;
	}
	
	ros::NodeHandle n;
	ros::Publisher pubL;
    ros::Publisher pubR;
	ros::Subscriber subWanted;
    ros::Subscriber subSpeeds;
    ros::ServiceClient clientLeft;
    ros::ServiceClient clientRight;

    double wantedDFiL = 0, wantedDFiR = 0;
    double currentDFiL = 0, currentDFiR = 0;
    double wheelR = 0.065;
    double shaft = 0.202;
	
	SubPub() {
		pubL = n.advertise<std_msgs::UInt8>("left_wheel", 10);
        pubR = n.advertise<std_msgs::UInt8>("right_wheel", 10);

		subWanted = n.subscribe("cmd_vel", 10, &SubPub::callbackWanted, this);
        subSpeeds = n.subscribe("speeds", 10, &SubPub::callbackSpeeds, this);

        clientLeft = n.serviceClient<mapping_robot::ChangeDir>("ChangeDirL");
        clientRight = n.serviceClient<mapping_robot::ChangeDir>("ChangeDirR");
	}
};

class PIDWheel {

    float kp, ki, kd;
    double lastDFi = 0;
    double err = 0, lastErr = 0;
    double integral = 0;
    double derivative = 0;
    ros::Time currentTime, lastTime;

    public:
    PIDWheel(float p = 10, float i = 10, float d = 0.001): kp(p), ki(i), kd(d)
    {
        currentTime = ros::Time::now();
        lastTime = ros::Time::now();
    }

    int doPID(double wanted, double current) {
        if (wanted == 0) {
            return 0;
        }

        currentTime = ros::Time::now();
        double dt = (currentTime - lastTime).toSec();
        lastTime = currentTime;
        
        err = wanted - current;

        integral += err * dt;
        derivative = (err - lastErr)/dt;
        
        lastErr = err;

        int motor = (kp * err) + (ki * integral) + (kd * derivative);

        if (motor > motor_max) {
            motor = motor_max;
            integral -= err * dt;
        }
        else if (motor < motor_min) {
            motor = motor_min;
            integral -= err * dt;
        }
    }

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "PID_drive");
	SubPub subPub;

    PIDWheel leftWheel(10, 10, 0.001); //need to be calibrated
    PIDWheel rightWheel(10, 10, 0.001);

    uint8_t lastLeftOut = 0, lastRightOut = 0;

    ros::Rate r(10.0);
    while(subPub.n.ok()) {
        ros::spinOnce();
        
        uint8_t leftOut = leftWheel.doPID(subPub.wantedDFiL, subPub.currentDFiL);
        uint8_t rightOut = rightWheel.doPID(subPub.wantedDFiR, subPub.currentDFiR);
        
        std_msgs::UInt8 leftMsg;
        std_msgs::UInt8 rightMsg;

        leftMsg.data = leftOut;
        leftMsg.data = rightOut;

        subPub.pubL.publish(leftMsg);
        subPub.pubR.publish(rightMsg);

        if(lastLeftOut < 0 && leftOut > 0) {
            mapping_robot::ChangeDir srvLeft;
            srvLeft.request.data = true;
            if (!subPub.clientLeft.call(srvLeft)) {
                ROS_INFO("fail");
            }
        } else if (lastLeftOut > 0 && leftOut < 0) {
            mapping_robot::ChangeDir srvLeft;
            srvLeft.request.data = false;
            if (!subPub.clientLeft.call(srvLeft)) {
                ROS_INFO("fail");
            }
        }

        if(lastRightOut < 0 && rightOut > 0) {
            mapping_robot::ChangeDir srvRight;
            srvRight.request.data = true;
            if (!subPub.clientRight.call(srvRight)) {
                ROS_INFO("fail");
            }
        } else if (lastRightOut > 0 && rightOut < 0) {
            mapping_robot::ChangeDir srvRight;
            srvRight.request.data = false;
            if (!subPub.clientRight.call(srvRight)) {
                ROS_INFO("fail");
            }
        }
        
        lastLeftOut = leftOut;
        lastRightOut = rightOut;
        r.sleep();
    }
	return 0;

}
