#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "geometry_msgs/Twist.h"
#include "mapping_robot/Speeds.h"
#include "mapping_robot/ChangeDir.h"
#include "mapping_robot/MotorPowers.h"

#define motor_max 400
#define motor_min (-400)

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
	ros::Publisher pub;
	ros::Subscriber subWanted;
    ros::Subscriber subSpeeds;
    ros::ServiceClient clientLeft;
    ros::ServiceClient clientRight;

    double wantedDFiL = 0, wantedDFiR = 0;
    double currentDFiL = 0, currentDFiR = 0;
    double wheelR = 0.065;
    double shaft = 0.202;
	
	SubPub() {
		pub = n.advertise<mapping_robot::MotorPowers>("motor_powers", 10);

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

    int16_t doPID(double wanted, double current) {
        currentTime = ros::Time::now();
        double dt = (currentTime - lastTime).toSec();
        lastTime = currentTime;
        
        err = wanted - current;

        integral += err * dt;
        derivative = (err - lastErr)/dt;
        
        lastErr = err;

        // might worse the breaking process
        if (wanted == 0) {
            return 0;
        }

        int16_t motor = (kp * err) + (ki * integral) + (kd * derivative);

        if (motor > motor_max) {
            motor = motor_max;
            integral -= err * dt;
        } else if (motor < motor_min) {
            motor = motor_min;
            integral -= err * dt;
        }
        return motor;
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
        
        int16_t leftOut = leftWheel.doPID(subPub.wantedDFiL, subPub.currentDFiL);
        int16_t rightOut = rightWheel.doPID(subPub.wantedDFiR, subPub.currentDFiR);
        
        mapping_robot::MotorPowers powers;

        powers.left = leftOut;
        powers.right = rightOut;

        subPub.pub.publish(powers);

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
