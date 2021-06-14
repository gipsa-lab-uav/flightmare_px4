#include "flightmare_px4/FlightmarePX4.hpp"
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_datatypes.h>
#include <std_srvs/Empty.h>

namespace flightmare_px4 {

FlightmarePX4::FlightmarePX4(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
  : nh_(nh),
    pnh_(pnh),
    scene_id_(UnityScene::WAREHOUSE),
    unity_ready_(false),
    unity_render_(false),
    receive_id_(0),
    main_loop_freq_(50.0) {
  // load parameters
  if (!loadParams()) {
    ROS_WARN("[%s] Could not load all parameters.",
             pnh_.getNamespace().c_str());
  } else {
    ROS_INFO("[%s] Loaded all parameters.", pnh_.getNamespace().c_str());
  }

  // quad initialization
  quad_ptr_ = std::make_shared<Quadrotor>();

  // add mono camera
  double cam_x, cam_y, cam_z, cam_qx, cam_qy, cam_qz, cam_qw; 
  pnh_.param("cam_x", cam_x, 0.3);
  pnh_.param("cam_y", cam_y, 0.);
  pnh_.param("cam_z", cam_z, 0.3);
  pnh_.param("cam_qx", cam_qx, 0.5);
  pnh_.param("cam_qy", cam_qy, -0.5);
  pnh_.param("cam_qz", cam_qz, 0.5);
  pnh_.param("cam_qw", cam_qw, -0.5);
  tf::Vector3 cam_origin(cam_x, cam_y, cam_z);
  tf::Quaternion cam_rot(cam_qx, cam_qy, cam_qz, cam_qw); //(0.4196665, -0.4196665, 0.5691046, -0.5691046); //(0.5, -0.5, 0.5, -0.5);  
  
  tf::Transform flu2rfu(tf::Quaternion(0,0,0.7071068,0.7071068));
  tf::Transform rotTransfo(tf::Quaternion(0.5, -0.5, 0.5, 0.5));
  
  tf::Quaternion cam_rot_rfu = /*flu2rfu**/rotTransfo*cam_rot;
  tf::Vector3 cam_origin_rfu = flu2rfu*cam_origin;
  
  camera_transform_ = tf::Transform(cam_rot, cam_origin);

  rgb_camera_ = std::make_shared<RGBCamera>();
  Vector<3> B_r_BC(cam_origin_rfu.x(), cam_origin_rfu.y(), cam_origin_rfu.z());
  Matrix<3, 3> R_BC = Quaternion(cam_rot_rfu.w(), cam_rot_rfu.x(), cam_rot_rfu.y(), cam_rot_rfu.z()).toRotationMatrix();
  std::cout << R_BC << std::endl;
  rgb_camera_->setFOV(90);
  rgb_camera_->setWidth(640);
  rgb_camera_->setHeight(480);
  rgb_camera_->setRelPose(B_r_BC, R_BC);
  rgb_camera_->setDepthScale(0.1);
  rgb_camera_->setPostProcesscing(
    std::vector<bool>{enable_depth_, enable_segmentation_, enable_opt_flow_});  // depth, segmentation, optical flow
  quad_ptr_->addRGBCamera(rgb_camera_);
  
  camera_info_msg_.header.frame_id = "camera";
  
  camera_info_msg_.height = rgb_camera_->getHeight();
  camera_info_msg_.height = rgb_camera_->getWidth();
  camera_info_msg_.distortion_model = "plumb_bob";
  double f = ( rgb_camera_->getHeight() / 2.0 ) / tan( (M_PI * rgb_camera_->getFOV() /180.0 )/2.0 );
  
  camera_info_msg_.K = {f, 0., rgb_camera_->getWidth()/2,
			0., f, rgb_camera_->getHeight()/2,
			0., 0., 1.};
  camera_info_msg_.R={1., 0., 0.,
		      0., 1., 0.,
		      0., 0., 1.};
  camera_info_msg_.P={f, 0., rgb_camera_->getWidth()/2, 0.,
		      0., f, rgb_camera_->getHeight()/2, 0.,
		      0., 0., 1., 0.};
  

  // initialization
  quad_state_.setZero();
  quad_ptr_->reset(quad_state_);
  
  camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("camera/camera_info", 1);
  
  it_.reset(new image_transport::ImageTransport(nh_));
  rgb_pub_ = it_->advertise("camera/rgb", 1);
  if(enable_depth_)
      depth_pub_ = it_->advertise("camera/depth", 1);
  if(enable_segmentation_)
      segmentation_pub_ = it_->advertise("camera/segmentation", 1);
  if(enable_opt_flow_)
      opticalflow_pub_ = it_->advertise("camera/opticalflow", 1);
  
  // initialize subscriber call backs
  sub_state_est_ = nh_.subscribe("ground_truth/odometry", 1,
                                 &FlightmarePX4::poseCallback, this);
  
  pause_srv_ = nh_.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
  unpause_srv_ = nh_.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
  
  timer_main_loop_ = nh_.createTimer(ros::Rate(main_loop_freq_),
                                     &FlightmarePX4::mainLoopCallback, this);


  // wait until the gazebo and unity are loaded
  ros::Duration(5.0).sleep();

  // connect unity
  setUnity(unity_render_);
  connectUnity();
}

FlightmarePX4::~FlightmarePX4() {}

void FlightmarePX4::poseCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  
    //ROS_INFO("pose callback");
  tf::Quaternion rot;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, rot);
  tf::Quaternion rfu2flu(0,0,-0.7071068,0.7071068);
  
  tf::Quaternion att = rot*rfu2flu;
  
  
  quad_state_.x[QS::POSX] = (Scalar)msg->pose.pose.position.x;
  quad_state_.x[QS::POSY] = (Scalar)msg->pose.pose.position.y;
  quad_state_.x[QS::POSZ] = (Scalar)msg->pose.pose.position.z;
  //quad_state_.x[QS::ATTW] = (Scalar)msg->pose.pose.orientation.w;
  //quad_state_.x[QS::ATTX] = (Scalar)msg->pose.pose.orientation.x;
  //quad_state_.x[QS::ATTY] = (Scalar)msg->pose.pose.orientation.y;
  //quad_state_.x[QS::ATTZ] = (Scalar)msg->pose.pose.orientation.z;
  //
  quad_state_.x[QS::ATTW] = (Scalar) att.w();
  quad_state_.x[QS::ATTX] = (Scalar) att.x();
  quad_state_.x[QS::ATTY] = (Scalar) att.y();
  quad_state_.x[QS::ATTZ] = (Scalar) att.z();

  
  quad_ptr_->setState(quad_state_);

}

void FlightmarePX4::mainLoopCallback(const ros::TimerEvent &event) {
  // empty

  cv::Mat img;

  ros::Time timestamp = ros::Time::now();
  
  //std_srvs::Empty srv;
  //pause_srv_.call(srv);
  
  if (unity_render_ && unity_ready_) {
      unity_bridge_ptr_->getRender(0);
      unity_bridge_ptr_->handleOutput();
  }
   
  br_.sendTransform(tf::StampedTransform(camera_transform_, timestamp, frame_id_, "camera"));
  
  camera_info_msg_.header.stamp = timestamp;
  camera_info_pub_.publish(camera_info_msg_);
  
  if(rgb_pub_.getNumSubscribers()>0)
  {
      rgb_camera_->getRGBImage(img);
      sensor_msgs::ImagePtr rgb_msg =
	  cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
      rgb_msg->header.stamp = timestamp;
      rgb_msg->header.frame_id = "camera";
      rgb_pub_.publish(rgb_msg);
  }
  
  if(enable_depth_ && depth_pub_.getNumSubscribers()>0)
  {
      rgb_camera_->getDepthMap(img);
      sensor_msgs::ImagePtr depth_msg =
	  cv_bridge::CvImage(std_msgs::Header(), "32FC1", img).toImageMsg();
      depth_msg->header.stamp = timestamp;
      depth_msg->header.frame_id = "camera";
      depth_pub_.publish(depth_msg);
  }

  if(enable_segmentation_ && segmentation_pub_.getNumSubscribers()>0)
  {
      rgb_camera_->getSegmentation(img);
      sensor_msgs::ImagePtr segmentation_msg =
	  cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
      segmentation_msg->header.stamp = timestamp;
      segmentation_msg->header.frame_id = "camera";
      segmentation_pub_.publish(segmentation_msg);
  }

  if(enable_opt_flow_ && opticalflow_pub_.getNumSubscribers()>0)
  {
      // The current optical flow is not correct.
      rgb_camera_->getOpticalFlow(img);
      sensor_msgs::ImagePtr opticflow_msg =
	  cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
      opticflow_msg->header.stamp = timestamp;
      opticflow_msg->header.frame_id = "camera";
      opticalflow_pub_.publish(opticflow_msg);
  }
  
  if(quad_ptr_->getCollision())
  {
      ROS_INFO("Collision !!");
  }

  //unpause_srv_.call(srv);

}

bool FlightmarePX4::setUnity(const bool render) {
  unity_render_ = render;
  if (unity_render_ && unity_bridge_ptr_ == nullptr) {
    // create unity bridge
    unity_bridge_ptr_ = UnityBridge::getInstance();
    unity_bridge_ptr_->addQuadrotor(quad_ptr_);
    ROS_INFO("[%s] Unity Bridge is created.", pnh_.getNamespace().c_str());
  }
  return true;
}

bool FlightmarePX4::connectUnity() {
  if (!unity_render_ || unity_bridge_ptr_ == nullptr) return false;
  unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  return unity_ready_;
}

bool FlightmarePX4::loadParams(void) {
  // load parameters 
  int id;
  pnh_.param("scene_id", id, 0);
  scene_id_ = id;
  
  pnh_.param("main_loop_freq", main_loop_freq_, 50.);
  pnh_.param("unity_render", unity_render_, false);
  
  
  pnh_.param("frame_id", frame_id_, std::string("base_link"));
  
  pnh_.param("enable_depth", enable_depth_, true);
  pnh_.param("enable_segmentation", enable_segmentation_, false);
  pnh_.param("enable_opt_flow", enable_opt_flow_, false);
  
  
  return true;
}

}  // namespace flightmare_px4
