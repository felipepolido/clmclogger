#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <sstream>

int main(int argc, char **argv)
{

ros::init(argc, argv, "number_publisher");

ros::NodeHandle n;

ros::Publisher inc_pub = n.advertise<std_msgs::Float64>("chatter", 1000);

ros::Rate loop_rate(100);

double data = 0;

std::cout << "Publishing..." << std::endl;

while (ros::ok())
{
	std_msgs::Float64 msg;
	data += 0.03;
	msg.data = data;

	inc_pub.publish(msg);

	ros::spinOnce();

	loop_rate.sleep();
}


return 0;
}