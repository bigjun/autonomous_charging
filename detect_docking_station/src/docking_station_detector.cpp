#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "DockingStationFinder.h"

void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO("Laser scan received... %d", (int)(msg->ranges.size()));
  ROS_INFO("min angle: %f max_angle: %f", msg->angle_min, msg->angle_max);
  ROS_INFO("min range: %f max range: %f", msg->range_min, msg->range_max);
  std::vector<float> x_scan, y_scan;
  x_scan.resize(msg->ranges.size());
  y_scan.resize(msg->ranges.size());

  for(size_t i=0; i < msg->ranges.size(); i++)
  {
    x_scan[i] = msg->ranges[i] * cos(msg->angle_increment * i);
    y_scan[i] = msg->ranges[i] * sin(msg->angle_increment * i);
  }

  DockingStationFinder dsf;
  std::vector<float> out = dsf.getMostLikelyLocation(y_scan, x_scan);
  std::cout << "best score:" << " " << out[0] << " " << out[1] << " " << out[2] << " " << out[3] << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n("~");
  std::string laserTN;
  n.getParam("laser", laserTN);
  ros::Subscriber sub = n.subscribe(laserTN, 1, chatterCallback);
  ros::spin();
  return 0;
}
