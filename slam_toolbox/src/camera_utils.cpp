#include "slam_toolbox/camera_utils.hpp"
#include <cmath>

namespace camera_utils
{
  CameraAssistant::CameraAssistant(ros::NodeHandle &nh,
                                   tf2_ros::Buffer *tf,
                                   const std::string &base_frame,
                                   const std::string &camera_frame)
      : nh_(nh), tf_(tf), base_frame_(base_frame), camera_frame_(camera_frame){};

  CameraAssistant::~CameraAssistant(){};

  karto::Camera *CameraAssistant::makeCamera()
  {
    double mountingYaw = setCameraPose();
    
    camera_ = karto::Camera::CreateCamera(karto::Name("Custom Described Camera"));
    karto::Vector3<kt_double> position(camera_pose_.transform.translation.x,
                                       camera_pose_.transform.translation.y,
                                       camera_pose_.transform.translation.z);
    karto::Quaternion orientation;
    orientation.FromEulerAngles(mountingYaw, 0.0, 0.0);
    camera_->SetOffsetPose(karto::Pose3(position, orientation));
    return camera_;
  };

  double CameraAssistant::setCameraPose()
  {
    geometry_msgs::TransformStamped camera_ident;
    camera_ident.header.stamp = ros::Time::now();
    camera_ident.header.frame_id = camera_frame_;
    camera_ident.transform.rotation.w = 1.0;

    camera_pose_ = tf_->transform(camera_ident, base_frame_);
    double mountingYaw = tf2::getYaw(camera_pose_.transform.rotation);

    ROS_DEBUG("camera %s's pose wrt base:\n Translation: %.3f %.3f %.3f\n Heading: %.3f",
              camera_frame_.c_str(),
              camera_pose_.transform.translation.x,
              camera_pose_.transform.translation.y,
              camera_pose_.transform.translation.z,
              mountingYaw);

    return mountingYaw;
  };

  karto::Camera *CameraAssistant::getCamera()
  {
    return camera_;
  };

} // end namespace
