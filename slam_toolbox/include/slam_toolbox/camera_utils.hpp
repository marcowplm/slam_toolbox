#ifndef SLAM_TOOLBOX_CAMERA_UTILS_H_
#define SLAM_TOOLBOX_CAMERA_UTILS_H_

#include <string>

#include "ros/ros.h"
#include "slam_toolbox/toolbox_types.hpp"
#include "tf2/utils.h"
#include "karto_sdk/Mapper.h"

namespace camera_utils
{

  /**
   * Help take an image from a camera and create a camera object
   */
  class CameraAssistant
  {
  public:
    CameraAssistant(ros::NodeHandle &nh, tf2_ros::Buffer *tf, 
      const std::string &base_frame, const std::string &camera_frame);
    ~CameraAssistant();
    karto::Camera *getCamera();
    karto::Camera *makeCamera();

  private:
    double setCameraPose();

    ros::NodeHandle nh_;
    tf2_ros::Buffer *tf_;
    karto::Camera *camera_;
    std::string camera_frame_, base_frame_;
    geometry_msgs::TransformStamped camera_pose_;
  };

} // end namespace

#endif //SLAM_TOOLBOX_CAMERA_UTILS_H_
