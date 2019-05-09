#include "AddMArker.h"

AddMarker::AddMarker(ros::NodeHandle* handle):nodeHandle(*handle) {
  markerPublisher  = nodeHandle.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  markerSubscriber = nodeHandle.subscribe("/add_markers/action", 1, &AddMarker::Action, this);
  visualization_msgs::Marker marker;
  initMarker(marker);

  ros::Rate rate(1);

  while ( ros::ok() )  {
    markerPublisher.publish(marker);
    rate.sleep();
  }
}

void AddMarker::showMarker(){
  marker.action = visualization_msgs::Marker::ADD;
}

void AddMarker::hideMarker(){
  marker.action = visualization_msgs::Marker::DELETE;
}

void action(const add_markers::action& input){
  if ( input.type == "SHOW" ) {
    marker.pose.position.x = input.point.x;
    marker.pose.position.y = input.point.y;
    marker.pose.position.z = input.point.z;
    showMarker();
  } else {
    hideMarker();
  }
}

void initMarker(){
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
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;


    // Init the marker as hidden
    marker.action = visualization_msgs::Marker::DELETE;
  
    marker.lifetime = ros::Duration();
}

void main(){
  ros::init(argc, argv, "AddMarker");
  ros::NodeHandle nodeHandle;

  AddMarker marker(&nh);
  ros::spin();

  return 0;
}