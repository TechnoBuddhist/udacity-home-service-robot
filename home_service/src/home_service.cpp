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
  while ( addMarkerPub.getNumSubscribers() <= 0 ){
    ros::Duration(0.5f, 0).sleep();
  }

  moveToPickupPoint();
  ros::Duration(5, 0).sleep();
  hideMarker();
  ros::Duration(5, 0).sleep();
  moveToDropOffPoint();
 
  return 0;
}

void hideMarker(){
  action.type = "DELETE";
  addMarkerPub.publish(action);
  ros::spinOnce();

  return;
}

void moveToPickupPoint(){
  action.point.x = 4.0f;
  action.point.y = 3.0f;
  action.point.z = 0.0f;
  action.type = "SHOW";
  addMarkerPub.publish(action);
  ros::spinOnce();

  return;
}

void moveToDropOffPoint(){
  action.point.x = 8.0f;
  action.point.y = -4.0f;
  action.point.z = 0.0f;
  action.type = "SHOW";
  addMarkerPub.publish(action);
  ros::spinOnce();

  return;
}