#ifndef SLAM_TOOLBOX_CAMERA_UTILS_H_
#define SLAM_TOOLBOX_CAMERA_UTILS_H_

#include <string>

#include "ros/ros.h"
#include "slam_toolbox/toolbox_types.hpp"
#include "tf2/utils.h"
#include "karto_sdk/Mapper.h"

namespace camera_utils
{

  // Store camera information
  class CameraMetadata
  {
  public:
    CameraMetadata();
    ~CameraMetadata();
    CameraMetadata(karto::Camera *cam);
    karto::Camera *getCamera();

  private:
    karto::Camera *camera;
  };

  // Help take a scan from a laser and create a laser object
  class LaserAssistant
  {
  public:
    LaserAssistant(ros::NodeHandle &nh, tf2_ros::Buffer *tf, const std::string &base_frame);
    ~LaserAssistant();
    CameraMetadata toLaserMetadata(sensor_msgs::LaserScan scan);

  private:
    karto::Camera *makeLaser(const double &mountingYaw);
    bool isInverted(double &mountingYaw);

    ros::NodeHandle nh_;
    tf2_ros::Buffer *tf_;
    sensor_msgs::LaserScan scan_;
    std::string frame_, base_frame_;
    geometry_msgs::TransformStamped laser_pose_;
  };

  // Hold some scans and utilities around them
  class ScanHolder
  {
  public:
    ScanHolder(std::map<std::string, camera_utils::CameraMetadata> &lasers);
    ~ScanHolder();
    sensor_msgs::LaserScan getCorrectedScan(const int &id);
    void addScan(const sensor_msgs::LaserScan scan);

  private:
    std::unique_ptr<std::vector<sensor_msgs::LaserScan>> current_scans_;
    std::map<std::string, camera_utils::CameraMetadata> &lasers_;
  };

} // end namespace

#endif //SLAM_TOOLBOX_CAMERA_UTILS_H_
