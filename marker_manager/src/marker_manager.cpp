#include "marker_manager.h"

MarkerManager::MarkerManager(const ros::NodeHandle* handle):nodeHandle(*handle) {
  markerPublisher  = nodeHandle.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  markerSubscriber = nodeHandle.subscribe("/marker_manager_action_topic", 1, &MarkerManager::action, this);
  visualization_msgs::Marker marker;
  initMarker();
}

void MarkerManager::publish(){
  ROS_INFO("marker_manager - publish");
  if ( ros::ok() ) {
    markerPublisher.publish(marker);
    ros::spinOnce();
  }
}

void MarkerManager::showMarker(){
  ROS_INFO("marker_manager - show");
  marker.action = visualization_msgs::Marker::ADD;
  publish();
}

void MarkerManager::hideMarker(){
  ROS_INFO("marker_manager - hide");
  marker.action = visualization_msgs::Marker::DELETE;
  publish();
}

void MarkerManager::action(const marker_manager::action& input){
  if ( input.type == "SHOW" ) {
    ROS_INFO("SHOWing marker");
    marker.pose.position.x = input.point.x;
    marker.pose.position.y = input.point.y;
    marker.pose.position.z = input.point.z;
    showMarker();
  } else {
    ROS_INFO("HIDing marker");
    hideMarker();
  }
}

void MarkerManager::initMarker(){
  ROS_INFO("marker_manager - init");
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0.0f;
    marker.pose.position.y = 0.0f;
    marker.pose.position.z = 0.0f;
    marker.pose.orientation.x = 0.0f;
    marker.pose.orientation.y = 0.0f;
    marker.pose.orientation.z = 0.0f;
    marker.pose.orientation.w = 1.0f;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.5f;
    marker.scale.y = 0.5f;
    marker.scale.z = 0.5f;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    // Init the marker as hidden
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();
}

int main(int argc, char** argv){
  ROS_INFO("marker_manager - MAIN");
  ros::init(argc, argv, "marker_manager");
  ros::NodeHandle nodeHandle;

  MarkerManager marker(&nodeHandle);
  ros::spin();

  return 0;
}