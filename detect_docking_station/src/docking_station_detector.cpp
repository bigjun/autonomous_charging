#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "DockingStationFinder.h"
#include <visualization_msgs/Marker.h>

class
DockingStationDetector
{
  private:
    ros::Subscriber sub_;
    ros::Publisher marker_pub_;
  public:

    DockingStationDetector(int argc, char ** argv)
    {

      ros::init(argc, argv, "listener");
      std::string laserTN;
      ros::NodeHandle node_handle("~");
      node_handle.getParam("laser", laserTN);
      std::cout << laserTN << std::endl;
      sub_ = node_handle.subscribe(laserTN, 1, &DockingStationDetector::laserScan_callback, this);
      marker_pub_ = node_handle.advertise<visualization_msgs::Marker>("dockingStationMarker", 1);
      ros::spin();
    }

    void laserScan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
      ROS_INFO("Laser scan received... %d", (int)(msg->ranges.size()));
      ROS_INFO("min angle: %f max_angle: %f", msg->angle_min, msg->angle_max);
      ROS_INFO("min range: %f max range: %f", msg->range_min, msg->range_max);
      std::vector<float> x_scan, y_scan;
      x_scan.resize(msg->ranges.size());
      y_scan.resize(msg->ranges.size());

      for(size_t i=0; i < msg->ranges.size(); i++)
      {
        x_scan[i] = msg->ranges[i] * cos(msg->angle_min + msg->angle_increment * i);
        y_scan[i] = msg->ranges[i] * sin(msg->angle_min + msg->angle_increment * i);
      }

      DockingStationFinder dsf;
      std::vector<float> out = dsf.getMostLikelyLocation(x_scan, y_scan);
      std::cout << "best score:" << " " << out[0] << " " << out[1] << " " << out[2] << " " << out[3] << std::endl;

      //publish a marker with the position
      uint32_t shape = visualization_msgs::Marker::CUBE;
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/base_laser_link";
      marker.header.stamp = ros::Time::now();

      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      marker.ns = "basic_shapes";
      marker.id = 0;

      // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
      marker.type = shape;

      // Set the marker action.  Options are ADD and DELETE
      marker.action = visualization_msgs::Marker::ADD;

      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      float angle = out[2] - 3.141692;
      float y = out[1];
      float x = out[0];
      marker.pose.position.x = x; //* cos(angle) - y * sin(angle);
      marker.pose.position.y = y; // * sin(angle) + y * cos(angle);

      /*marker.pose.position.x = x_scan[0];
      marker.pose.position.y = y_scan[0];*/
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;

      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;

      marker.lifetime = ros::Duration();

      // Publish the marker
      marker_pub_.publish(marker);
    }
};

int main(int argc, char **argv)
{
  DockingStationDetector dsd(argc, argv);
  return 0;
}
