AddMarkerPublisher::AddMarkerPublisher() {
  //Topic you want to publish
  markerPublisher = nodeHandle.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  markerSubscriber = nodeHandle.subscribe("/subscribed_topic", 1, &SubscribeAndPublish::action, this);

  ros::Rate rate(1);
  while ( ros::ok() )  {
    visualization_msgs::Marker marker;

    initMarker(marker);
    markerPublisher.publish(marker);

    rate.sleep();
  }
}

bool AddMarkerPublisher::moveTo(geometry_msgs::Pose goal){
  ROS_INFO("Marker moved");
  
  // Check if the robot reached its goal
  bool succeeded{false};
  if ( ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ) {
    succeeded = true;
  } else {
    succeeded =  false;
  }

  return succeeded;
}

bool AddMarkerPublisher::moveToPickup(){

}

bool AddMarkerPublisher::moveToDropOff(){

}

void AddMarkerPublisher::showMarker(){
  marker.action = visualization_msgs::Marker::ADD;
}

void AddMarkerPublisher::hideMarker(){
  marker.action = visualization_msgs::Marker::DELETE;
}

void action(const add_markers::action& input){
  
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
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 0.65;

    marker.lifetime = ros::Duration();
}