#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include "add_markers/Action.h"

#ifndef ADD_MARKER_H
#define ADD_MARKER_H

class AddMarker {
  public:
    AddMarker();

  private:
    void initMarker();
    void showMarker();
    void hideMarker();
    void action(const add_markers::Action& input);
    
    ros::NodeHandle nodeHandle; 
    ros::Publisher  markerPublisher;
    ros::Subscriber markerSubscriber;
    visualization_msgs::Marker marker;
};

#endif