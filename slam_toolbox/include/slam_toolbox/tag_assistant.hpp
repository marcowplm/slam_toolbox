#ifndef SLAM_TOOLBOX_TAG_ASSISTANT_H_
#define SLAM_TOOLBOX_TAG_ASSISTANT_H_

#include <string>
#include <utility>
#include <Eigen/Geometry>

#include "ros/ros.h"
#include "slam_toolbox/toolbox_types.hpp"
#include "slam_toolbox/visualization_utils.hpp"
#include "tf2/utils.h"
#include "karto_sdk/Mapper.h"
#include "apriltag_ros/AprilTagDetectionArray.h"

namespace tag_assistant
{
    using namespace ::toolbox_types;

    /**
     * Object containing a tag ID, a vertex ID and a link between them
     */
    struct Link
    {
        Link(int tag_id, int vertex_id, karto::Pose3 transform)
            : tag_id_(tag_id), vertex_id_(vertex_id), transform_(transform)
        {
        }

        int GetTagId()
        {
            return tag_id_;
        }

        int GetVertexId()
        {
            return vertex_id_;
        }

        karto::Pose3 GetTransform()
        {
            return transform_;
        }

        // Serve solo per poter inserire correttamente oggetti di tipo Link in un set
        bool operator<(const Link &li) const
        {
            if (tag_id_ < li.tag_id_)
                return true;
            else if (tag_id_ > li.tag_id_)
                return false;
            else if (vertex_id_ < li.vertex_id_)
                return true;
            else if (vertex_id_ > li.vertex_id_)
                return false;
            else
                return (transform_.GetPosition() < li.transform_.GetPosition());
        }

        karto::Pose3 transform_;
        int tag_id_;
        int vertex_id_;
    };


    /**
     * Converts a geometry_msgs Pose in a karto Pose3
     */
    inline karto::Pose3 geometryToKarto(const geometry_msgs::Pose &pose_in)
    {
        karto::Vector3 position_out(pose_in.position.x, pose_in.position.y, pose_in.position.z);
        karto::Quaternion orientation_out(pose_in.orientation.x, pose_in.orientation.y, pose_in.orientation.z, pose_in.orientation.w);

        karto::Pose3 pose_out(position_out, orientation_out);
        return pose_out;
    };

    /**
     * Converts a karto Pose3 in a geometry_msgs Pose
     */
    inline geometry_msgs::Pose kartoToGeometry(const karto::Pose3 &pose_in)
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
     * Converts a karto Quaternion in a Eigen Quaternion
     */
    inline Eigen::Quaterniond kartoToEigenQuaternion(const karto::Quaternion &quat_in)
    {
        Eigen::Quaterniond quat_out(quat_in.GetW(), quat_in.GetX(), quat_in.GetY(), quat_in.GetZ());
        quat_out.normalize();
        return quat_out;
    };

    /**
     * Converts a Eigen Quaternion in a karto Quaternion
     */
    inline karto::Quaternion EigenTokartoQuaternion(const Eigen::Quaterniond &quat_in)
    {
        karto::Quaternion quat_out;
        quat_out.SetW(quat_in.w());
        quat_out.SetX(quat_in.x());
        quat_out.SetY(quat_in.y());
        quat_out.SetZ(quat_in.z());
        return quat_out;
    };

    /**
     * Computes the inverse of a 3D transform
     */
    inline karto::Pose3 invertTransform3(const karto::Pose3 &pose_in)
    {
        karto::Quaternion orientation = pose_in.GetOrientation();
        karto::Vector3 position = pose_in.GetPosition();
        Eigen::Quaterniond inv(orientation.GetW(), orientation.GetX(), orientation.GetY(), orientation.GetZ());
        inv.inverse();
        orientation = EigenTokartoQuaternion(inv);
        position.SetX(-position.GetX());
        position.SetY(-position.GetY());
        position.SetZ(-position.GetZ());

        karto::Pose3 pose_out(position, orientation);
        return pose_out;
    };

    /**
     * Class ApriltagAssistant
     */
    class ApriltagAssistant
    {
    public:
        ApriltagAssistant(ros::NodeHandle &nh, tf2_ros::Buffer *tf, karto::Mapper *mapper);

        karto::Camera *makeCamera();
        karto::Camera *getCamera();

        void publishMarkerGraph();
        void publishLinks();
        bool processDetection(const int &last_vertex_id, const karto::Pose2 last_vertex_pose, const apriltag_ros::AprilTagDetectionArrayConstPtr &detection_array);

    private:
        karto::LocalizedMarker *getLocalizedTag(karto::Camera *camera, int unique_id, karto::Pose3 karto_pose);
        bool getTagPose(karto::Pose3 &karto_pose, const apriltag_ros::AprilTagDetection &detection);
        bool addLink(const int &vertex_id, const karto::Pose2 vertex_pose, const apriltag_ros::AprilTagDetection &detection, const bool isFirstLink);
        bool addTag(const int &vertex_id, const karto::Pose2 vertex_pose, const apriltag_ros::AprilTagDetection &detection);

    private:
        geometry_msgs::TransformStamped camera_pose_;

        tf2_ros::Buffer *tf_;
        ros::NodeHandle &nh_;
        ros::Publisher tag_publisher_;

        karto::Mapper *mapper_;
        karto::ScanSolver *solver_;
        karto::Camera *camera_;

        std::string map_frame_, odom_frame_, camera_frame_;

        std::map<int, std::set<Link>> links_;
        std::map<int, std::set<Link>>::iterator links_Iter;
        std::map<int, std::set<int>> tags_; // Maps a tag id to many vertices id
        std::map<int, std::set<int>>::iterator tags_Iter;
        std::vector<apriltag_ros::AprilTagDetection>::const_iterator detectionConstIter;
    };

} // namespace tag_assistant

#endif //SLAM_TOOLBOX_TAG_ASSISTANT_H_
