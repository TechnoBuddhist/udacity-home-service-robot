#include <ros/ros.h>
#include "AddMarker.h"


int main( int argc, char** argv ){
  ros::init(argc, argv, "add_markers");
  ros::Time::init();
  ros::NodeHandle nodeHandle;
  ros::Rate rate(1);

  addMarkerPub = nodeHandle.advertise<add_markers::Action>("action", 1);

  while ( !ros::ok() ) {
    sleep(1);
  }

  moveToPickupPoint() ) {
  ros::Duration(5, 0).sleep();
  hideMarker();
  ros::Duration(5, 0).sleep();
  moveToDropOffPoint() 
 
  return 0;
}

void hideMarker(){
  action.type = "DELETE";
  ROS_INFO("HIDE marker request");
  addMarkerPub.publish(action);

  return;
}
}

void moveToPickupPoint(){
  action.point.x = 4.0f;
  action.point.y = 3.0f;
  action.point.z = 0.0f;
  action.type = "SHOW";
  ROS_INFO("pickup marker requested");
  addMarkerPub.publish(action);
  
  return;
}

void moveToDropOffPoint(){
  action.point.x = 8.0f;
  action.point.y = -4.0f;
  action.point.z = 0.0f;
  action.type = "SHOW";
  ROS_INFO("dropoff marker requested");
  addMarkerPub.publish(action);

  return;
}