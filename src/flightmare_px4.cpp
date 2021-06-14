#include <ros/ros.h>
#include "flightmare_px4/FlightmarePX4.hpp"

using namespace flightmare_px4;


int main(int argc, char** argv) {
  ros::init(argc, argv, "flightmare_px4");
  flightmare_px4::FlightmarePX4 flightmare_px4(ros::NodeHandle(), ros::NodeHandle("~"));
  
  // spin the ros
  ros::spin();
  
  return 0;
}
