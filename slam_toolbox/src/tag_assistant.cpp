#include <iostream>

#include "slam_toolbox/tag_assistant.hpp"
#include <eigen_conversions/eigen_msg.h>

namespace tag_assistant
{

  /*****************************************************************************/
  ApriltagAssistant::ApriltagAssistant(
      ros::NodeHandle &nh, tf2_ros::Buffer *tf, karto::Mapper *mapper)
      : nh_(nh), tf_(tf), mapper_(mapper)
  /*****************************************************************************/
  {
    nh_.getParam("map_frame", map_frame_);
    nh_.getParam("camera_frame", camera_frame_);
    tag_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("tag_visualization", 1);
    solver_ = mapper_->GetScanSolver();
  }

  // Ottiene la posizione del tag nel riferimento mondo (map_frame_), nel momento in cui il tag viene creato
  /*****************************************************************************/
  bool ApriltagAssistant::getTagPose(karto::Pose3 &tag_pose_karto,
                                     const apriltag_ros::AprilTagDetection &detection)
  /*****************************************************************************/
  {
    // camera_ident è un "contenitore" che tf_->transform usa per tirare fuori il suo source frame e il timestamp
    geometry_msgs::TransformStamped camera_ident;
    camera_ident.header.stamp = detection.pose.header.stamp;
    camera_ident.header.frame_id = camera_frame_;
    camera_ident.transform.rotation.w = 1.0;

    try
    {
      // Ottiene la posizione della camera -> Target frame: map / Source frame: camera_frame (all'interno di camera_ident)
      camera_pose_ = tf_->transform(camera_ident, map_frame_); //NB: NON usare odom_frame_ !!!
    }
    catch (tf2::TransformException e)
    {
      // TODO: capisci perchè alcuni tf non sono sincronizzati...
      // ROS_ERROR("Failed to compute camera pose, skipping tag (%s)", e.what());
      return false;
    }
    geometry_msgs::Pose tag_pose_;
    // Input pose: detection_pose / Output pose: tag_pose_ / Transformed by camera_pose_
    tf2::doTransform(detection.pose.pose.pose, tag_pose_, camera_pose_);
    tag_pose_karto = poseGeometryToKarto(tag_pose_);

    //- TODO: da eliminare  --  Questo pezzo pubblica la tf di tag_pose rispetto a map_frame
    /* geometry_msgs::TransformStamped tag_pose_stamped_;
       tag_pose_stamped_.transform.rotation = tag_pose_.orientation;
       tag_pose_stamped_.transform.translation.x = tag_pose_.position.x;
       tag_pose_stamped_.transform.translation.y = tag_pose_.position.y;
       tag_pose_stamped_.transform.translation.z = tag_pose_.position.z;
       tag_pose_stamped_.header.frame_id = map_frame_;
       tag_pose_stamped_.header.stamp = camera_ident.header.stamp;
       tag_pose_stamped_.child_frame_id = "TAG" + std::to_string(detection.id[0]);
       tfB_->sendTransform(tag_pose_stamped_); */

    return true;
  }

  // Qui prendo i Marker e i Vertex del grafo e visualizzo gli edge tra le loro Pose
  /*****************************************************************************/
  void ApriltagAssistant::publishMarkerGraph()
  /*****************************************************************************/
  {
    std::unordered_map<int, karto::Pose3> *graph = solver_->getGraph();
    std::unordered_map<int, karto::Pose3> *markers = solver_->getMarkers();
    std::map<int, std::list<int>> constraints = solver_->getConstraints();

    if (markers->size() == 0)
    {
      return;
    }

    ROS_DEBUG("MarkerGraph size: %i", (int)markers->size());

    visualization_msgs::MarkerArray marray;
    visualization_msgs::Marker m = vis_utils::toTagMarker(map_frame_, nh_.getNamespace());
    visualization_msgs::Marker e = vis_utils::toEdgeMarker(map_frame_, nh_.getNamespace(), 0.01);

    for (ConstGraphIterator it = markers->begin(); it != markers->end(); ++it)
    {
      geometry_msgs::Pose mPose = poseKartoToGeometry(it->second);
      m.id = it->first + 1;
      m.pose = mPose;
      //-
      /* std::cout << "Marker pose: " << m.id << "\tYaw: " << tf::getYaw(mPose.orientation) << std::endl;
      std::cout << m.pose << std::endl; */

      marray.markers.push_back(m);
      //-
      /* std::cout << "\nMarker: " << it->first << "\t| Constraints: "; */
      std::list<int>::const_iterator listit = constraints[it->first].begin();
      for (listit; listit != constraints[it->first].end(); ++listit)
      {
        e.points.clear();

        geometry_msgs::Point p = m.pose.position; // Posizione del Marker
        e.points.push_back(p);
        /* std::cout << *listit << ", "; */
        p = poseKartoToGeometry(graph->find(*listit)->second).position; // Posizione dello Scan
        e.points.push_back(p);

        e.id = ((std::hash<double>()(m.id) ^ (std::hash<double>()((int)*listit) << 1)) >> 1);
        e.color.r = 1.0;
        e.color.g = 0.5;
        e.color.b = 0.0;

        marray.markers.push_back(e);
      }
    }
    /* std::cout << "\n"; */

    tag_publisher_.publish(marray);

    return;
  }

} // end namespace tag_assistant