#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class Detector {
  public:
    const static double MIN_SCAN_ANGLE = -0.01; // radians
    const static double MAX_SCAN_ANGLE = 0.01;  // radians
    const static float CLOSEST_OBJECT_DIST = 0.5;

    Detector();
    void startDetecting();

  private:
    ros::NodeHandle node;
    ros::Publisher pub;
    ros::Subscriber sub;
    bool moving;

    void detect(const sensor_msgs::LaserScan::ConstPtr &laserScan);
};