#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <cmath>

double pickUpPos[2]  = {0, 1}; // the first point robot go to pick up
double dropOffPos[2] = {-2.6, -4.86}; // the last point robot drop off

double pose[2] = {0, 0};  // current pose

enum State {
    PICKUP = 1,  // going to pick up zon
    DROP,    // already drop
    DoNothing = 0
  } state = PICKUP;


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  uint32_t shape = visualization_msgs::Marker::CUBE;
  state = PICKUP;

  while (ros::ok())
  {
    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = shape;

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    ros::spinOnce();
	
    switch (state)
    {
      // Check state equal pickup, add new maker with position set by user
      // Check if robot reach maker, waiting 5s and change state to carry
      case PICKUP:
        ROS_INFO("Add new Maker ... ");
	marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = pickUpPos[0];
        marker.pose.position.y = pickUpPos[1];
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
        marker_pub.publish(marker);
	sleep(4);
	state = DROP;
        break;

      // add new maker to visualize to make sure the maker is drop
      case DROP: 
	// delete first maker and create new maker
	marker.action = visualization_msgs::Marker::DELETE;
	ROS_INFO("Swap Maker in another location ... ");
	sleep(1);
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = dropOffPos[0];
        marker.pose.position.y = dropOffPos[1];
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
        marker_pub.publish(marker);
	state = DoNothing;
        break;
    }
  }
}
