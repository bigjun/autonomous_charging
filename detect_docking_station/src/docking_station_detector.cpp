#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "DockingStationFinder.h"
#include <visualization_msgs/Marker.h>
#include "DockingStationFinder.h"
#include "DockingStationFinderSegmented.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include "detect_docking_station/nav_to_docking_station.h"

class
DockingStationDetector
{
  private:
    ros::Subscriber sub_;
    ros::Publisher marker_pub_;
    DockingStationFinder * dsf_;

    float object_found_threshold_;
    tf::Vector3 base_goal_;
    ros::ServiceServer nav_to_goal_service_;
  public:

    DockingStationDetector(int argc, char ** argv)
    {

      ros::init(argc, argv, "dsd");
      std::string laserTN;
      ros::NodeHandle node_handle("~");
      node_handle.getParam("laser", laserTN);
      std::cout << laserTN << std::endl;
      sub_ = node_handle.subscribe(laserTN, 1, &DockingStationDetector::laserScan_callback, this);
      marker_pub_ = node_handle.advertise<visualization_msgs::Marker>("dockingStationMarker", 1);
      object_found_threshold_ = 10.f;
      nav_to_goal_service_ = node_handle.advertiseService("nav_to_docking_station", &DockingStationDetector::nav_to_ds, this);

      dsf_ = new DockingStationFinderSegmented();

      ros::spin();
    }

    bool nav_to_ds(detect_docking_station::nav_to_docking_station::Request  &req,
                     detect_docking_station::nav_to_docking_station::Response  &res)
    {
      std::cout << "Service called, navigating to:" << base_goal_[0] << " " << base_goal_[1] << std::endl;
      //use action lib to move the robot there
      typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
      MoveBaseClient ac("move_base", true);
      while(!ac.waitForServer(ros::Duration(5.0))){
          ROS_INFO("Waiting for the move_base action server to come up");
      }

      tf::Vector3 base_goal_map_cs(base_goal_[0], base_goal_[1], 0);

      /*tf::TransformListener listener;
      tf::StampedTransform transform;
      try{
        listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("/map", "/base_link",
                                 ros::Time(0), transform);
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
      }

      tf::Vector3 base_goal_map_cs(base_goal_[0], base_goal_[1], 0);
      base_goal_map_cs = transform * base_goal_map_cs;*/

      std::cout << "Service called, navigating to:" << base_goal_map_cs[0] << " " << base_goal_map_cs[1] << " in MAP CS" << std::endl;

      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose.header.frame_id = "/base_link";
      //goal.target_pose.header.frame_id = "/map";
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose.position.x = base_goal_map_cs[0];
      goal.target_pose.pose.position.y = base_goal_map_cs[1];
      goal.target_pose.pose.orientation.x = 0.0;
      goal.target_pose.pose.orientation.y = 0.0;
      goal.target_pose.pose.orientation.z = 0.0;
      goal.target_pose.pose.orientation.w = 1.0;
      ac.sendGoal(goal);
      ac.waitForResult(ros::Duration(50.0));
      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("Hooray, the base moved to the goal");
      }
      else
      {
        ROS_INFO("The base failed to move to the goal");
      }

      return true;
    }

    void publishMarker(float x, float y, float angle, std::string link, bool add=true)
    {
      uint32_t shape = visualization_msgs::Marker::CUBE;
      visualization_msgs::Marker marker;
      marker.header.frame_id = link;
      marker.header.stamp = ros::Time::now();

      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      marker.ns = "basic_shapes";
      marker.id = 0;

      // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
      marker.type = shape;

      // Set the marker action.  Options are ADD and DELETE
      marker.action = visualization_msgs::Marker::ADD;
      if(!add)
        marker.action = visualization_msgs::Marker::DELETE;

      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      angle = angle - 3.141692;
      //y = y - 0.3f;
      //x = x - 0.476f/2;
      //x -= 0.7f;
      marker.pose.position.x = x;
      marker.pose.position.y = y;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = marker.scale.y = marker.scale.z = 0.1;

      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.70;

      marker.lifetime = ros::Duration();

      // Publish the marker
      marker_pub_.publish(marker);
    }

    void laserScan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
      //transform laser scan to dsf format
      std::vector<float> x_scan, y_scan;
      x_scan.resize(msg->ranges.size());
      y_scan.resize(msg->ranges.size());
      for(size_t i=0; i < msg->ranges.size(); i++)
      {
        x_scan[i] = msg->ranges[i] * cos(msg->angle_min + msg->angle_increment * i);
        y_scan[i] = msg->ranges[i] * sin(msg->angle_min + msg->angle_increment * i);
      }
      std::vector<float> out = dsf_->getMostLikelyLocation(x_scan, y_scan);

      //get transform from laser to base_link
      float goal_x_laser = out[0] - 0.3f;
      float goal_y_laser = out[1];
      tf::TransformListener listener;
      tf::StampedTransform transform;
      try{
        listener.waitForTransform("/base_link", "/base_laser_link", ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("/base_link", "/base_laser_link",
                                 ros::Time(0), transform);
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
      }

      tf::Vector3 origin = transform.getOrigin();
      //transform goal to base link coordinates
      tf::Vector3 goal_laser(goal_x_laser, goal_y_laser, 0);
      float goal_x_base = goal_laser[0] + origin[0];
      float goal_y_base = goal_laser[1];

      publishMarker(goal_x_base, goal_y_base,out[2], "/base_link");
      std::cout << "Relative to the robot (base_link):" << goal_x_base << " " << goal_y_base << std::endl;
      //transform goal to /map coordinates
      /*{
        tf::TransformListener listener;
        tf::StampedTransform transform;
        try{
          listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(10.0) );
          listener.lookupTransform("/map", "/base_link",
                                   ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
        }

        tf::Vector3 base_goal_map_cs(goal_x_base, goal_y_base, 0);
        base_goal_map_cs = transform * base_goal_map_cs;
        publishMarker(base_goal_map_cs[0], base_goal_map_cs[1],out[2], "/map");

      }*/

      if(out[3] < object_found_threshold_)
      {
        return;
      }
      else
      {
        //if threshold is ok, then save goal!
        base_goal_ = tf::Vector3(goal_x_base, goal_y_base, out[2]);
      }
    }
};

int main(int argc, char **argv)
{
  DockingStationDetector dsd(argc, argv);
  return 0;
}
