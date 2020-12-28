#ifndef SLAM_TOOLBOX_SLAM_TOOLBOX_SYNC_TAG_H_
#define SLAM_TOOLBOX_SLAM_TOOLBOX_SYNC_TAG_H_

#include "slam_toolbox/slam_toolbox_common.hpp"

namespace slam_toolbox
{

  class SynchronousSlamToolboxAT : public SlamToolbox
  {
  public:
    SynchronousSlamToolboxAT(ros::NodeHandle &nh);
    ~SynchronousSlamToolboxAT(){};
    void run();

  protected:
    virtual void laserCallback(const sensor_msgs::LaserScan::ConstPtr &scan) override final;
    bool clearQueueCallback(slam_toolbox_msgs::ClearQueue::Request &req, slam_toolbox_msgs::ClearQueue::Response &resp);
    virtual bool deserializePoseGraphCallback(slam_toolbox_msgs::DeserializePoseGraph::Request &req,
                                              slam_toolbox_msgs::DeserializePoseGraph::Response &resp) override final;
    void tagCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr &detection_array);

    std::unique_ptr<message_filters::Subscriber<apriltag_ros::AprilTagDetectionArray>> tag_detection_filter_sub_;
    std::unique_ptr<tf2_ros::MessageFilter<apriltag_ros::AprilTagDetectionArray>> tag_detection_filter_;

    std::queue<PosedScan> q_;
    ros::ServiceServer ssClear_;
  };

} // namespace slam_toolbox

#endif //SLAM_TOOLBOX_SLAM_TOOLBOX_SYNC_TAG_NODE_H_
