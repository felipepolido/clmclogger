#include "ros/ros.h"
#include "std_msgs/Float64.h"

//Logger
#include <mrdplot/Logger.h>

//Example of logging data from a ros topic. This behavior is very similar to what a rosbag would do
//The BatchLogger can also just as easily be used in the loop, using the same principles as below.

BatchLogger logger;
double double_data = 0.0;
double local_time = 0.0;
bool stop = false;

void chatterCallback(const std_msgs::Float64::ConstPtr& msg)
{
    //Update variables:
    double_data =  msg->data;
    local_time = ros::Time::now().toNSec();
    //Save the data
    logger.saveData();

    static int count = 0;
    count ++;
    std::cout << "Logged " << double_data << " in position "
        << count << std::endl;
    if (count == 300)
    { //Save X number of elements
      //If we reached X number of points, save the data into file
      logger.writeToMRDPLOT2("ctrl");
      stop = true;
    }

} 

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  double period = 0.01;//Assume the topic is being published at 100 Hz
  if (!logger.hasInited())
  {
    logger.init(period);
  }

  // Add variables to be logged:
  logger.add_datapoint("double_data","s",&double_data);
  logger.add_datapoint("local_time","s",&local_time);

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  while(1)
  {
    ros::spinOnce();
    if(stop) break;
  }

  return 0;
}

