#include "Detector.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "detector");

  Detector detector;
  detector.startDetecting();

  return 0;
}