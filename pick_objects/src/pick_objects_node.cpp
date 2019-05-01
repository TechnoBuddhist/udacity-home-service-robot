#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "ros/time.h"

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool moveToPickupPoint(MoveBaseClient &ac, move_base_msgs::MoveBaseGoal &goal);
bool moveToDropOffPoint(MoveBaseClient &ac, move_base_msgs::MoveBaseGoal &goal);
bool moveToGoal(MoveBaseClient &ac, move_base_msgs::MoveBaseGoal &goal);

int main(int argc, char** argv){
  ros::init(argc, argv, "pick_objects");
  ros::Time::init();

  // Tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true); 

  // Wait 5 sec for move_base action server to come up
  while ( !ac.waitForServer(ros::Duration(5.0)) ) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";   // Set up the goal frame id

  auto returnStatus{1};

  if ( moveToPickupPoint(ac, goal) ) {
    ros::Duration(5, 0).sleep();
    if ( moveToDropOffPoint(ac, goal) ) {
      returnStatus = 0;
    }
  }

  return returnStatus;
}



bool moveToPickupPoint(MoveBaseClient &ac, move_base_msgs::MoveBaseGoal &goal){
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 4.0;
  goal.target_pose.pose.position.y = 3.0;
  goal.target_pose.pose.orientation.w = 1.0;

  bool succeeded{false};
  if ( succeeded = moveToGoal(ac, goal) ) {
    ROS_INFO("Reached pickup destination");
  } else {
    ROS_INFO("Failed to reach pickup destination");
  }

  return succeeded;
}

bool moveToDropOffPoint(MoveBaseClient &ac, move_base_msgs::MoveBaseGoal &goal){
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 8.0;
  goal.target_pose.pose.position.y = -4.0;
  goal.target_pose.pose.orientation.w = 1.0;

  bool succeeded{false};
  if ( succeeded = moveToGoal(ac, goal) ) {
    ROS_INFO("Reached dropoff destination");
  } else {
    ROS_INFO("Failed to reach dropoff destination");
  }

  return succeeded;
}

bool moveToGoal(MoveBaseClient &ac, move_base_msgs::MoveBaseGoal &goal){
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  bool succeeded{false};
  if ( ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ) {
    succeeded = true;
  } else {
    succeeded =  false;
  }

  return succeeded;
}