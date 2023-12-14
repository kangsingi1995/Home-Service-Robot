#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <cmath>

double pickUpPos[2]  = {0, 1}; // the first point robot go to pick up
double dropOffPos[2] = {-2.6, -4.86}; // the last point robot drop off

double pose[2] = {0, 0};  // current pose

enum State {
    PICKUP,  // going to pick up zon
    CARRY,   // carry to drop zone
    DROP,    // already drop
  } state = PICKUP;

void get_current_pose(const nav_msgs::Odometry::ConstPtr& msg)
{
  pose[0] = msg->pose.pose.position.x;
  pose[1] = msg->pose.pose.position.y;
}

// Function to calculate Manhattan Distance between two points
double manhattanDistance(double x1, double y1, double x2, double y2)
{
  return std::abs(x1 - x2) + std::abs(y1 - y2);
}

bool CheckState_OfRobot()
{
	State stateRobot;
	
	if(state == PICKUP && manhattanDistance(pose[0], pose[1], pickUpPos[0], pickUpPos[1])< 0.2)
	{
		stateRobot = CARRY;
	}
	else if(state == CARRY && manhattanDistance(pose[0], pose[1], dropOffPos[0], dropOffPos[1])< 0.2)
	{
		stateRobot = DROP;
	}
	else
		stateRobot = PICKUP;
	return stateRobot;
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  ros::Subscriber pose_sub = n.subscribe("odom", 10, get_current_pose);

  uint32_t shape = visualization_msgs::Marker::CUBE;
  state = PICKUP;

  ROS_INFO("Going to pick up zone ... ");
  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "odom";
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
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = pickUpPos[0];
        marker.pose.position.y = pickUpPos[1];
        marker_pub.publish(marker);
		state = CheckState_OfRobot();
        break;
    // Check state equal carry, remove the first maker
    // after that check robot reach drop zone or not, if yes change state to drop
      case CARRY:
        marker.action = visualization_msgs::Marker::DELETE;
        marker.pose.position.x = dropOffPos[0];
        marker.pose.position.y = dropOffPos[1];
        marker_pub.publish(marker);
        state = CheckState_OfRobot();
        break;
	  // add new maker to visualize to make sure the maker is drop
      case DROP: 
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = dropOffPos[0];
        marker.pose.position.y = dropOffPos[1];
        marker_pub.publish(marker);
        break;
    }
  }
}
