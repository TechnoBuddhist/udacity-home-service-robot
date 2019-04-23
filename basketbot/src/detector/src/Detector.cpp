#include "Detector.h"
#include "geometry_msgs/Twist.h"

Detector::Detector() {
  moving = true;  // Default to keep robot moving until an object is found

  // Advertise we publish Twist(cmd_vel)topic
  pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  // Subscribe to LaserScan topic
  sub = node.subscribe("/scan", 1, &Detector::detect, this);
}

// Publish a Twist message to move robot
// void Detector::move() {
//   geometry_msgs::Twist msg;
//   msg.linear.x = FORWARD_SPEED;
//   pub.publish(msg);
// }

void Detector::detect(const sensor_msgs::LaserScan::ConstPtr& laserScan){
  // From the array of laser scans find the closest
  int minScanIndex = ceil((MIN_SCAN_ANGLE - laserScan->angle_min) / laserScan->angle_increment);
  int maxScanIndex = floor((MAX_SCAN_ANGLE - laserScan->angle_min) / laserScan->angle_increment);

  float closest = laserScan->ranges[minScanIndex];
  for (int currScanIndex = minScanIndex+1; currScanIndex <= maxScanIndex; currScanIndex++){
    if (laserScan->ranges[currScanIndex] < closest){
      closest = laserScan->ranges[currScanIndex];
    }
  }
  ROS_INFO("Closest object is %fm", closest);

  // Is closest scan closer than the limit set?
  if (closest < CLOSEST_OBJECT_DIST){
    ROS_INFO("Too close to object... stopping!");
    moving = false;
  }
}

void Detector::startDetecting(){
  ros::Rate rate(10);

  while (ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }
}