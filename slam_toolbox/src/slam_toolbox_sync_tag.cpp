#include <iostream>

#include "slam_toolbox/slam_toolbox_sync_tag.hpp"

namespace slam_toolbox
{

  /*****************************************************************************/
  SynchronousSlamToolboxAT::SynchronousSlamToolboxAT(ros::NodeHandle &nh)
      : SlamToolbox(nh)
  /*****************************************************************************/
  {
    ssClear_ = nh.advertiseService("clear_queue",
                                   &SynchronousSlamToolboxAT::clearQueueCallback, this);

    // TODO: sta roba deve essere spostata in SlamToolbox::setROSInterfaces! (probabilmente con un if(use_markers))
    tag_detection_filter_sub_ = std::make_unique<message_filters::Subscriber<apriltag_ros::AprilTagDetectionArray>>(nh, tag_topic_, 5);
    tag_detection_filter_ = std::make_unique<tf2_ros::MessageFilter<apriltag_ros::AprilTagDetectionArray>>(*tag_detection_filter_sub_, *tf_, camera_frame_, 5, nh);
    tag_detection_filter_->registerCallback(boost::bind(&SynchronousSlamToolboxAT::tagCallback, this, _1));

    threads_.push_back(std::make_unique<boost::thread>(
        boost::bind(&SynchronousSlamToolboxAT::run, this)));

    loadPoseGraphByParams(nh);
  }

  /*****************************************************************************/
  void SynchronousSlamToolboxAT::run()
  /*****************************************************************************/
  {
    ros::Rate r(100);
    while (ros::ok())
    {
      if (!q_.empty() && !isPaused(PROCESSING))
      {
        PosedScan scan_w_pose = q_.front();
        q_.pop();

        if (q_.size() > 10)
        {
          ROS_WARN_THROTTLE(10., "Queue size has grown to: %i. "
                                 "Recommend stopping until message is gone if online mapping.",
                            (int)q_.size());
        }

        addScan(getLaser(scan_w_pose.scan), scan_w_pose);
        continue;
      }

      r.sleep();
    }
  }

  /*****************************************************************************/
  void SynchronousSlamToolboxAT::laserCallback(
      const sensor_msgs::LaserScan::ConstPtr &scan)
  /*****************************************************************************/
  {
    // no odom info
    karto::Pose2 pose;
    if (!pose_helper_->getOdomPose(pose, scan->header.stamp))
    {
      return;
    }

    // ensure the laser can be used
    karto::LaserRangeFinder *laser = getLaser(scan);
    if (!laser)
    {
      ROS_WARN_THROTTLE(5., "SynchronousSlamToolboxAT: Failed to create laser"
                            " device for %s; discarding scan",
                        scan->header.frame_id.c_str());
      return;
    }

    // if sync and valid, add to queue
    if (shouldProcessScan(scan, pose))
    {
      q_.push(PosedScan(scan, pose));
    }

    return;
  }

  /*****************************************************************************/
  void SynchronousSlamToolboxAT::tagCallback(
      const apriltag_ros::AprilTagDetectionArrayConstPtr &detection_array)
  /*****************************************************************************/
  {
    // ensure the camera can be used
    karto::Camera *camera = getCamera();
    if (!camera)
    {
      ROS_WARN_THROTTLE(5., "SynchronousSlamToolboxAT: Failed to create camera");
      return;
    }

    if (detection_array->detections.size() == 0)
    {
      return;
    }

    VerticeMap mapper_vertices = smapper_->getMapper()->GetGraph()->GetVertices();
    ScanMap scan_vertices = mapper_vertices.find(karto::Name("Custom Described Lidar"))->second;
    tag_assistant_->processDetection(scan_vertices.rbegin()->second, detection_array);

    return;
  }

  /*****************************************************************************/
  bool SynchronousSlamToolboxAT::clearQueueCallback(
      slam_toolbox_msgs::ClearQueue::Request &req,
      slam_toolbox_msgs::ClearQueue::Response &resp)
  /*****************************************************************************/
  {
    ROS_INFO("SynchronousSlamToolboxAT: Clearing all queued scans to add to map.");
    while (!q_.empty())
    {
      q_.pop();
    }
    resp.status = true;
    return true;
  }

  /*****************************************************************************/
  bool SynchronousSlamToolboxAT::deserializePoseGraphCallback(
      slam_toolbox_msgs::DeserializePoseGraph::Request &req,
      slam_toolbox_msgs::DeserializePoseGraph::Response &resp)
  /*****************************************************************************/
  {
    if (req.match_type == procType::LOCALIZE_AT_POSE)
    {
      ROS_ERROR("Requested a localization deserialization "
                "in non-localization mode.");
      return false;
    }
    return SlamToolbox::deserializePoseGraphCallback(req, resp);
  }

} // namespace slam_toolbox
