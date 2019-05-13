#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include "marker_manager/action.h"

void moveToPickupPoint();
void moveToDropOffPoint();
void hideMarker();
  
ros::Publisher addMarkerPub;
marker_manager::action action;

int main(int argc, char** argv){
  ros::init(argc, argv, "add_markers");
  ros::Time::init();
  ros::NodeHandle nodeHandle;

  addMarkerPub = nodeHandle.advertise<marker_manager::action>("/marker_manager_action_topic", 1);

  ROS_INFO("add_marker_node - moveToPickupPoint");
  moveToPickupPoint();
  ros::Duration(5, 0).sleep();
  ROS_INFO("add_marker_node - hideMArker");
  hideMarker();
  ros::Duration(5, 0).sleep();
  ROS_INFO("add_marker_node - moveToDropOffPoint");
  moveToDropOffPoint();
 
  return 0;
}

void hideMarker(){
  action.type = "DELETE";
  ROS_INFO("HIDE marker request");
  addMarkerPub.publish(action);
  ros::spinOnce();

  return;
}

void moveToPickupPoint(){
  action.point.x = 4.0f;
  action.point.y = 3.0f;
  action.point.z = 0.0f;
  action.type = "SHOW";
  ROS_INFO("pickup marker requested");
  addMarkerPub.publish(action);
  ros::spinOnce();

  return;
}

void moveToDropOffPoint(){
  action.point.x = 8.0f;
  action.point.y = -4.0f;
  action.point.z = 0.0f;
  action.type = "SHOW";
  ROS_INFO("dropoff marker requested");
  addMarkerPub.publish(action);
  ros::spinOnce();

  return;
}