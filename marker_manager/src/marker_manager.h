#ifndef MARKER_MANAGER_H
#define MARKER_MANAGER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include "marker_manager/action.h"

class MarkerManager {
  public:
    MarkerManager(const ros::NodeHandle* handle);

  private:
    void initMarker();
    void showMarker();
    void hideMarker();
    void publish();
    void action(const marker_manager::action& input);

    ros::Publisher  markerPublisher;
    ros::Subscriber markerSubscriber;
    visualization_msgs::Marker marker;
    ros::NodeHandle nodeHandle;
};

#endif