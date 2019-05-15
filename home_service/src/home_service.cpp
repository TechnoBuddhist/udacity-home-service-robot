#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Point.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "marker_manager/action.h"

// Define a client to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void init(const MoveBaseClient &ac);
void showMarker(const float x, const float y);
void hideMarker();
bool moveToPickupPoint(MoveBaseClient& ac, move_base_msgs::MoveBaseGoal& goal);
bool moveToDropOffPoint(MoveBaseClient& ac, move_base_msgs::MoveBaseGoal& goal);
bool moveToGoal(MoveBaseClient& ac, move_base_msgs::MoveBaseGoal& goal);

const float PICK_UP_POINT_X = 1.0f; //4.0f;
const float PICK_UP_POINT_Y = 0.0f;  //3.0f;
const float DROP_OFF_POINT_X = 0.0f; //8.0f;
const float DROP_OFF_POINT_Y = 0.0f; //-4.0f;

ros::Publisher addMarkerPub;
move_base_msgs::MoveBaseGoal goal;
marker_manager::action action;

int main(int argc, char** argv){
  ros::init(argc, argv, "home_service");
  ros::Time::init();

  // Tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true); 
  init(ac);
  showMarker(PICK_UP_POINT_X, PICK_UP_POINT_Y);

  auto returnStatus{1};
  if ( moveToPickupPoint(ac, goal) ) {
    ros::Duration(5, 0).sleep();
    if ( moveToDropOffPoint(ac, goal) ) {
      returnStatus = 0;
    }
  }

  return returnStatus;
}

/**
 * @brief Initialise global variables and wait for related action server and relevant subscribers
 **/
void init(const MoveBaseClient& ac){
  ROS_INFO("init");
  goal.target_pose.header.frame_id = "map";   // Set up the goal frame id

  // Wait 5 sec for move_base action server to come up
  while ( !ac.waitForServer(ros::Duration(5.0)) ) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  ros::NodeHandle nodeHandle;
  
  addMarkerPub = nodeHandle.advertise<marker_manager::action>("/marker_manager_action_topic", 1);
  ROS_INFO("addMarkerPub.getNumSubscribers() = %d", addMarkerPub.getNumSubscribers());
  while ( addMarkerPub.getNumSubscribers() <= 0 ){
    ROS_INFO("Waiting for subscribers...");
    ros::Duration(1.0f, 0).sleep();
  }
}

/**
 * @brief Move to the pickup point at the specified Point(2D only) and return true/false if successful
 **/
bool moveToPickupPoint(MoveBaseClient& ac, move_base_msgs::MoveBaseGoal& goal){
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = PICK_UP_POINT_X;
  goal.target_pose.pose.position.y = PICK_UP_POINT_Y;
  goal.target_pose.pose.orientation.w = 1.0;
  
  bool succeeded{false};
  if ( succeeded = moveToGoal(ac, goal) ) {
    ROS_INFO("Reached pickup destination");
    hideMarker();
  } else {
    ROS_INFO("Failed to reach pickup destination");
  }

  return succeeded;
}

/**
 * @brief Move to the drop off point at the specified Point(2D only) and return true/false if successful
 **/
bool moveToDropOffPoint(MoveBaseClient& ac, move_base_msgs::MoveBaseGoal& goal){
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = DROP_OFF_POINT_X;
  goal.target_pose.pose.position.y = DROP_OFF_POINT_Y;
  goal.target_pose.pose.orientation.w = 1.0;

  bool succeeded{false};
  if ( succeeded = moveToGoal(ac, goal) ) {
    ROS_INFO("Reached dropoff destination");
    showMarker(DROP_OFF_POINT_X, DROP_OFF_POINT_Y);
  } else {
    ROS_INFO("Failed to reach dropoff destination");
  }

  return succeeded;
}

/**
 * @brief Move to the specified goal and return true/false if successfully got to goal
 **/
bool moveToGoal(MoveBaseClient& ac, move_base_msgs::MoveBaseGoal& goal){
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

/**
 * @brief Show a visualization marker at the specified Point(only 2D point)
 **/
void showMarker(const float x, const float y){
  action.point.x = x;
  action.point.y = y;
  action.point.z = 0.0f;
  action.type = "SHOW";
  addMarkerPub.publish(action);
  ros::spinOnce();

  return;
}

/**
 * @brief Hide a visualization marker
 **/
void hideMarker(){
  action.type = "DELETE";
  addMarkerPub.publish(action);
  ros::spinOnce();

  return;
}