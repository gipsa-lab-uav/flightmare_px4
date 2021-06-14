
#pragma once

#include <memory>

// ros
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_broadcaster.h>

// flightlib
#include "flightlib/bridges/unity_bridge.hpp"
#include "flightlib/common/quad_state.hpp"
#include "flightlib/common/types.hpp"
#include "flightlib/objects/quadrotor.hpp"
#include "flightlib/sensors/rgb_camera.hpp"


using namespace flightlib;

namespace flightmare_px4 {

class FlightmarePX4 {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FlightmarePX4(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  ~FlightmarePX4();

  // callbacks
  void mainLoopCallback(const ros::TimerEvent& event);
  void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);

  bool setUnity(const bool render);
  bool connectUnity(void);
  bool loadParams(void);

 private:
  // ros nodes
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  sensor_msgs::CameraInfo camera_info_msg_; 
  
  // publisher
  image_transport::Publisher rgb_pub_;
  image_transport::Publisher depth_pub_;
  image_transport::Publisher segmentation_pub_;
  image_transport::Publisher opticalflow_pub_;

  ros::Publisher camera_info_pub_;
  
  std::shared_ptr< image_transport::ImageTransport> it_;
  
  // subscriber
  ros::Subscriber sub_state_est_;
  
  // service client
    ros::ServiceClient pause_srv_, unpause_srv_;
  
  //tf broadcaster
  tf::TransformBroadcaster br_;

  // main loop timer
  ros::Timer timer_main_loop_;

  // unity quadrotor
  std::shared_ptr<Quadrotor> quad_ptr_;
  std::shared_ptr<RGBCamera> rgb_camera_;
  QuadState quad_state_;
  
  // Flightmare(Unity3D)
  std::shared_ptr<UnityBridge> unity_bridge_ptr_;
  SceneID scene_id_{UnityScene::WAREHOUSE};
  bool unity_ready_{false};
  bool unity_render_{false};
  RenderMessage_t unity_output_;
  uint16_t receive_id_{0};  

  // auxiliary variables
  double main_loop_freq_;
  tf::Transform camera_transform_;
  std::string frame_id_;
  
  bool enable_depth_, enable_segmentation_, enable_opt_flow_;
};
}  // namespace flightmare_px4
