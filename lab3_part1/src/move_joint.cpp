#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "math.h"

float x;

void jointCallback(const std_msgs::Float64 msg){
	if(msg.data < x) {
		ROS_INFO("You entered lower data");
	} else {
		x = msg.data;
		ROS_INFO("X: %f", x);
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "testing");
	x = 0;
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<std_msgs::Float64>("/robot/joint5_position_controller/command", 10);

	ros::Subscriber sub = nh.subscribe("move_joint", 1000, jointCallback);

	ros::Rate loop_rate(1);

	while(ros::ok()) {
		std_msgs::Float64 msg_to_send;
		msg_to_send.data = x;
		pub.publish(msg_to_send);
		ROS_INFO("moving joint 5");
		ros::spinOnce();
		loop_rate.sleep();
	}
}
