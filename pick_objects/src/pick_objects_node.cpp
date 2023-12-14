#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <visualization_msgs/Marker.h>
#include <actionlib/client/simple_action_client.h>

double pickUpPos[2]  = {0, 1};
double dropOffPos[2] = {-2.6, -4.86};

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");
  move_base_msgs::MoveBaseGoal robot_move_to_goal;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // set up the frame parameters
  robot_move_to_goal.target_pose.header.frame_id = "odom";
  robot_move_to_goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  robot_move_to_goal.target_pose.pose.position.x = pickUpPos[0];
  robot_move_to_goal.target_pose.pose.position.y = pickUpPos[1];
  robot_move_to_goal.target_pose.pose.position.z = 0.0;
  robot_move_to_goal.target_pose.pose.orientation.x = 0.0;
  robot_move_to_goal.target_pose.pose.orientation.y = 0.0;
  robot_move_to_goal.target_pose.pose.orientation.z = 0.0;
  robot_move_to_goal.target_pose.pose.orientation.w = 1.0;

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending packge to goal....");
  ac.sendGoal(robot_move_to_goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Finish, reached the goal =v=");
  else
    ROS_INFO("The base failed to reach the goal for some reason.");

  sleep(5); // simulate the picking up

  // ============= Next step - reaches drop-off zone ===========
  // Define a position and orientation for the robot to reach
  robot_move_to_goal.target_pose.pose.position.x = dropOffPos[0];
  robot_move_to_goal.target_pose.pose.position.y = dropOffPos[1];
  robot_move_to_goal.target_pose.pose.position.z = 0.0;
  robot_move_to_goal.target_pose.pose.orientation.x = 0.0;
  robot_move_to_goal.target_pose.pose.orientation.y = 0.0;
  robot_move_to_goal.target_pose.pose.orientation.z = 0.0;
  robot_move_to_goal.target_pose.pose.orientation.w = 1.0;

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending packge to drop off position...");
  ac.sendGoal(robot_move_to_goal);
		
  // Wait an infinite time for the results
  ac.waitForResult();
		
  // Check the robot reaches drop-off zone
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Reached the drop off zone");
  else
      ROS_INFO("Robot has failed to move to the drop-off zone");

  return 0;
}
