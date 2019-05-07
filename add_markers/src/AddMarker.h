#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>

#ifndef ADD_MARKER_PUBLISHER_H
#define ADD_MARKER_PUBLISHER_H

class AddMarkerPublisher {
  public:
    AddMarkerPublisher() {}
    void showMarker();
    void hideMarker();

  private:
    void initMarker();
    bool moveToPickup();
    bool moveToDropOff();
    bool moveTo(geometry_msgs::Pose goal);
    
    ros::NodeHandle nodeHandle; 
    ros::Publisher markerPublisher;
    visualization_msgs::Marker marker;
};

#endif