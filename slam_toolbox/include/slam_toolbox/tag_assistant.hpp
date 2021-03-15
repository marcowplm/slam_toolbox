#ifndef SLAM_TOOLBOX_TAG_ASSISTANT_H_
#define SLAM_TOOLBOX_TAG_ASSISTANT_H_

#include <string>
#include <utility>
#include <Eigen/Geometry>

#include "ros/ros.h"
#include "slam_toolbox/toolbox_types.hpp"
#include "slam_toolbox/visualization_utils.hpp"
#include "tf2/utils.h"
#include "tf2_ros/transform_broadcaster.h"
#include "karto_sdk/Mapper.h"
#include "apriltag_ros/AprilTagDetectionArray.h"

namespace tag_assistant
{
  using namespace ::toolbox_types;

  /**
   * Converts a geometry_msgs Pose in a karto Pose3
   */
  inline karto::Pose3 poseGeometryToKarto(const geometry_msgs::Pose &pose_in)
  {
    karto::Vector3 position_out(pose_in.position.x, pose_in.position.y, pose_in.position.z);
    karto::Quaternion orientation_out(pose_in.orientation.x, pose_in.orientation.y, pose_in.orientation.z, pose_in.orientation.w);

    karto::Pose3 pose_out(position_out, orientation_out);
    return pose_out;
  };

  /**
   * Converts a karto Pose3 in a geometry_msgs Pose
   */
  inline geometry_msgs::Pose poseKartoToGeometry(const karto::Pose3 &pose_in)
  {
    geometry_msgs::Point position_out;
    position_out.x = pose_in.GetPosition().GetX();
    position_out.y = pose_in.GetPosition().GetY();
    position_out.z = pose_in.GetPosition().GetZ();

    geometry_msgs::Quaternion orientation_out;
    orientation_out.x = pose_in.GetOrientation().GetX();
    orientation_out.y = pose_in.GetOrientation().GetY();
    orientation_out.z = pose_in.GetOrientation().GetZ();
    orientation_out.w = pose_in.GetOrientation().GetW();

    geometry_msgs::Pose pose_out;
    pose_out.position = position_out;
    pose_out.orientation = orientation_out;
    return pose_out;
  };

  /**
   * Converts a karto Quaternion in a Eigen Quaternion (normalized)
   */
  inline Eigen::Quaterniond quaternionKartoToEigen(const karto::Quaternion &quat_in)
  {
    Eigen::Quaterniond quat_out(quat_in.GetW(), quat_in.GetX(), quat_in.GetY(), quat_in.GetZ());
    quat_out.normalize();
    return quat_out;
  };

  /**
     * Converts a Eigen Quaternion in a karto Quaternion
     */
  inline karto::Quaternion quaternionEigenToKarto(const Eigen::Quaterniond &quat_in)
  {
    karto::Quaternion quat_out;
    quat_out.SetW(quat_in.w());
    quat_out.SetX(quat_in.x());
    quat_out.SetY(quat_in.y());
    quat_out.SetZ(quat_in.z());
    return quat_out;
  };

  /**
   * Converts a Eigen Isometry3d (Transform) in a karto Pose3
   */
  inline karto::Pose3 isometryEigenToPoseKarto(const Eigen::Isometry3d &isometry_in)
  {
    Eigen::Vector3d translation = Eigen::Vector3d(isometry_in.translation());
    Eigen::Quaterniond rotation(isometry_in.rotation());
    rotation.normalize();

    karto::Vector3<double> translation_out(translation.x(), translation.y(), translation.z());
    karto::Quaternion rotation_out = quaternionEigenToKarto(rotation);

    karto::Pose3 pose_out(translation_out, rotation_out);

    return pose_out;
  };

  /**
   * Converts a karto Pose3 in a Eigen Isometry3d (Transform)
   */
  inline Eigen::Isometry3d poseKartoToEigenIsometry(const karto::Pose3 &pose_in)
  {
    Eigen::Isometry3d isometry_out;
    Eigen::Quaterniond quat_out = quaternionKartoToEigen(pose_in.GetOrientation()); 
    isometry_out.linear() = quat_out.toRotationMatrix();
    isometry_out.translation() = Eigen::Vector3d(
        pose_in.GetPosition().GetX(),
        pose_in.GetPosition().GetY(),
        pose_in.GetPosition().GetZ());

    return isometry_out;
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
     * Class ApriltagAssistant
     */
  class ApriltagAssistant
  {
  public:
    ApriltagAssistant(ros::NodeHandle &nh, tf2_ros::Buffer *tf, karto::Mapper *mapper, karto::Dataset *dataset);

    karto::Camera *makeCamera();
    karto::Camera *getCamera();
    void setCamera(karto::Camera *cam);

    void publishMarkerGraph();
    void publishLinks();
    bool processDetection(const karto::Vertex<karto::LocalizedRangeScan> *last_vertex,
                          const apriltag_ros::AprilTagDetectionArrayConstPtr &detection_array);

  private:
    karto::LocalizedMarker *createLocalizedMarker(karto::Camera *camera, int tag_id, karto::Pose3 tag_pose);
    bool getTagPose(karto::Pose3 &karto_pose, const apriltag_ros::AprilTagDetection &detection);
    bool addLocalizedMarker(karto::LocalizedRangeScan *pScan,
                            const apriltag_ros::AprilTagDetection &detection);

  private:
    geometry_msgs::TransformStamped camera_pose_;

    tf2_ros::Buffer *tf_;
    ros::NodeHandle &nh_;
    ros::Publisher tag_publisher_;
    ros::Publisher link_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tfB_;

    karto::Mapper *mapper_;
    karto::Dataset *dataset_;
    karto::ScanSolver *solver_;
    karto::Camera *camera_;

    std::string map_frame_, odom_frame_, camera_frame_;
    double m_cov_;

    std::map<int, std::set<int>> tags_; // Maps each tag id (int key) to many vertices id (set<int>)
    std::map<int, std::set<int>>::iterator tags_Iter;

    std::map<int, int> ids_; // Maps markers StateId (= tag id) against markers UniqueId

    std::vector<apriltag_ros::AprilTagDetection>::const_iterator detectionConstIter;
  };

} // namespace tag_assistant

#endif //SLAM_TOOLBOX_TAG_ASSISTANT_H_
