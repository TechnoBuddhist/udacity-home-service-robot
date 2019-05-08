#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include "add_markers/Action.h"

#ifndef ADD_MARKERS_NODE_H
#define ADD_MARKERS_NODE_H

private:
  void moveToPickupPoint();
  void moveToDropOffPoint();
  void hideMarker();
  
  ros::Publisher addMarkerPub;
  add_markers::Action action;
#endif