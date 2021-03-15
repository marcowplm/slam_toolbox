#include <iostream>

#include "slam_toolbox/tag_assistant.hpp"
#include <eigen_conversions/eigen_msg.h>

namespace tag_assistant
{

  /*****************************************************************************/
  ApriltagAssistant::ApriltagAssistant(
      ros::NodeHandle &nh, tf2_ros::Buffer *tf, karto::Mapper *mapper, karto::Dataset *dataset)
      : nh_(nh), tf_(tf), mapper_(mapper), dataset_(dataset)
  /*****************************************************************************/
  {
    nh_.getParam("map_frame", map_frame_);
    nh_.getParam("odom_frame", odom_frame_);
    nh_.getParam("camera_frame", camera_frame_);
    nh_.getParam("marker_link_covariance", m_cov_);
    tag_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("tag_visualization", 1);
    link_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("link_visualization", 1);
    //tfB_ = std::make_unique<tf2_ros::TransformBroadcaster>();
    solver_ = mapper_->getScanSolver();
    camera_ = makeCamera();
    dataset_->Add(camera_, true);
  }

  /*****************************************************************************/
  bool ApriltagAssistant::processDetection(const karto::Vertex<karto::LocalizedRangeScan> *last_vertex,
                                           const apriltag_ros::AprilTagDetectionArrayConstPtr &detection_array)
  /*****************************************************************************/
  {
    bool processed = false;
    karto::LocalizedRangeScan *pScan = last_vertex->GetObject();
    int scan_id = pScan->GetUniqueId();

    for (detectionConstIter = detection_array->detections.begin(); detectionConstIter != detection_array->detections.end(); ++detectionConstIter)
    {
      tags_Iter = tags_.find(detectionConstIter->id[0]);
      if (tags_Iter != tags_.end()) // il tag esiste nella struttura dati, quindi l'ho già visto e ho creato il LocalizedMarker
      {
        if (tags_Iter->second.find(scan_id) == tags_Iter->second.end()) // il tag esiste ma non è associato allo scan
        {
          // Recupero il marker con lo UniqueId associato al tag_id di questo tag
          int marker_unique_id = ids_[tags_Iter->first];
          karto::LocalizedMarker *pMarker = mapper_->GetMarkerManager()->GetMarker(marker_unique_id);
          
          Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Identity();
          cov(0,0) = cov(1,1) = cov(2,2) = m_cov_;
          mapper_->GetMarkerGraph()->LinkMarkerToScan(pMarker, pScan, cov);
          
          tags_Iter->second.insert(scan_id); // aggiungo l'id dello scan alla struttura dati, in corrispondenza dell'id del tag
          // mapper_->CorrectPoses(); // così sparo l'ottimizzazione ad OGNI nuovo MarkerEdge (un po' troppo...)
          processed = true;
        } // else: il tag è già associato allo scan -> non fare niente
      }
      else // il tag non esiste nella map (è la prima volta che lo vedo)
      {
        if (!addLocalizedMarker(pScan, *detectionConstIter))
        {
          processed = false;
          return processed;
        }
        tags_.insert(std::pair<int, std::set<int>>(detectionConstIter->id[0], {scan_id})); // inserisco l'id del nuovo tag con relativo ide dello scan nella struttura
        processed = true;
      }
    }
    return processed;
  }

  /** 
   * Recupera la posizione del tag nel mondo, crea il Vertex associato al tag da aggiungere 
   * e poi crea l'Edge tra il nuovo vertice e il vertice dello scan
   */
  /*****************************************************************************/
  bool ApriltagAssistant::addLocalizedMarker(karto::LocalizedRangeScan *pScan,
                                             const apriltag_ros::AprilTagDetection &detection)
  /*****************************************************************************/
  {
    karto::Pose3 tag_pose; // è la posizione del tag nel mondo
    if (!getTagPose(tag_pose, detection))
    {
      return false;
    }

    karto::LocalizedMarker *tag = createLocalizedMarker(getCamera(), detection.id[0], tag_pose);
    int unique_id;
    if (!mapper_->ProcessMarker(tag, pScan, unique_id))
    {
      std::cout << "ProcessMarker FALSE!" << std::endl;
      return false;
    }
    ids_.insert(std::pair<int, int>(detection.id[0], unique_id));

    /* std::cout << "\nContenuto di ids_:\n tagID\t|  UniqueId" << std::endl;
    std::map<int, int>::const_iterator ids_Iter = ids_.begin();
    for (ids_Iter; ids_Iter != ids_.end(); ++ids_Iter)
    {
      std::cout << "   " << ids_Iter->first << "\t|    " << ids_Iter->second << std::endl;
    } */

    return true;
  }

  // Chiama il costruttore di karto::LocalizedMarker
  /*****************************************************************************/
  karto::LocalizedMarker *ApriltagAssistant::createLocalizedMarker(karto::Camera *camera, int tag_id,
                                                                   karto::Pose3 tag_pose)
  /*****************************************************************************/
  {
    karto::LocalizedMarker *tag = new karto::LocalizedMarker(camera->GetName(), tag_id, tag_pose);
    tag->SetCorrectedPose(tag_pose);
    return tag;
  }
  
  // Ottiene la posizione del tag nel riferimento mondo (map_frame_), nel momento in cui il tag viene creato
  /*****************************************************************************/
  bool ApriltagAssistant::getTagPose(karto::Pose3 &tag_pose_karto,
                                     const apriltag_ros::AprilTagDetection &detection)
  /*****************************************************************************/
  {
    // camera_ident è un "contenitore" che tf_->transform userà per tirare fuori il suo source frame,
    // camera_rgb_optical, che è in camera_ident.header.frame_id
    geometry_msgs::TransformStamped camera_ident;
    camera_ident.header.stamp = detection.pose.header.stamp;
    camera_ident.header.frame_id = detection.pose.header.frame_id;
    camera_ident.transform.rotation.w = 1.0;

    try
    {
      // Ottiene la posizione della camera -> Target frame: map / Source frame: camera_rgb_optical (all'interno di camera_ident)
      camera_pose_ = tf_->transform(camera_ident, map_frame_);
    }
    catch (tf2::TransformException e)
    {
      // ROS_ERROR("Failed to compute pose, aborting continue mapping (%s)", e.what());
      return false;
    }
    geometry_msgs::Pose tag_pose_;
    // Input pose: detection_pose / Output pose: tag_pose_ / Transformed by camera_pose_
    tf2::doTransform(detection.pose.pose.pose, tag_pose_, camera_pose_);
    tag_pose_karto = poseGeometryToKarto(tag_pose_);

    /* Questo pezzo pubblica la tf di tag_pose rispetto a map_frame
        geometry_msgs::TransformStamped tag_pose_stamped_;
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

  // Qui prendo semplicemente i Marker e i Vertex del grafo e visualizzo gli edge tra le loro Pose
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

    visualization_msgs::MarkerArray marray;
    visualization_msgs::Marker m = vis_utils::toTagMarker(map_frame_, nh_.getNamespace());
    visualization_msgs::Marker e = vis_utils::toEdgeMarker(map_frame_, nh_.getNamespace(), 0.02);

    for (ConstGraphIterator it = markers->begin(); it != markers->end(); ++it)
    {
      geometry_msgs::Pose mPose = poseKartoToGeometry(it->second);
      m.id = it->first + 1;
      m.pose = mPose;
      /* std::cout << "Marker pose: " << m.id << "\tYaw: " << tf::getYaw(mPose.orientation) << std::endl;
      std::cout << m.pose << std::endl; */

      marray.markers.push_back(m);

      std::list<int>::const_iterator listit = constraints[it->first].begin();
      for (listit; listit != constraints[it->first].end(); ++listit)
      {
        e.points.clear();

        geometry_msgs::Point p = m.pose.position; // Posizione del Marker
        e.points.push_back(p);

        p = poseKartoToGeometry(graph->find(*listit)->second).position; // Posizione dello Scan
        e.points.push_back(p);

        e.id = ((std::hash<double>()(m.id) ^ (std::hash<double>()((int)*listit) << 1)) >> 1);
        e.color.r = 1.0;
        e.color.g = 0.5;
        e.color.b = 0.0;

        marray.markers.push_back(e);
      }
    }

    tag_publisher_.publish(marray);
    // publishLinks(); // FIXME: non funziona con la serializzazione!
    
    return;
  }

  /*****************************************************************************/
  karto::Camera *ApriltagAssistant::makeCamera()
  /*****************************************************************************/
  {
    karto::Camera *camera = karto::Camera::CreateCamera(karto::Name("Custom Camera"));
    return camera;
  }

  /*****************************************************************************/
  karto::Camera *ApriltagAssistant::getCamera()
  /*****************************************************************************/
  {
    return camera_;
  }

  // TODO: è qui solo per TEST - poi deve essere eliminato!
  /*****************************************************************************/
  void ApriltagAssistant::publishLinks()
  /*****************************************************************************/
  {
    std::vector<karto::MarkerEdge *> marker_edges = mapper_->GetMarkerGraph()->GetMarkerEdges();

    if (marker_edges.size() == 0)
    {
      return;
    }

    visualization_msgs::MarkerArray marray;
    visualization_msgs::Marker e = vis_utils::toEdgeMarker(map_frame_, nh_.getNamespace(), 0.02);

    int count = 0;
    //std::cout << std::endl;
    std::vector<karto::MarkerEdge *>::const_iterator edgesIter = marker_edges.cbegin();
    for (edgesIter; edgesIter != marker_edges.end(); ++edgesIter)
    {
      count++;
      karto::Pose3 mPose_karto = (*edgesIter)->GetSource()->GetLocalizedMarker()->GetCorrectedPose();
      geometry_msgs::Pose mPose = poseKartoToGeometry(mPose_karto);
      Eigen::Isometry3d mPose_eigen = poseKartoToEigenIsometry(mPose_karto);
      
      karto::MarkerLinkInfo *pMarkerLinkInfo = (karto::MarkerLinkInfo *)((*edgesIter)->GetLabel());
      karto::Pose3 linkPose_karto(pMarkerLinkInfo->GetPoseDifference());
      Eigen::Isometry3d linkPose_eigen = poseKartoToEigenIsometry(linkPose_karto);
      
      // Applico la transform rappresentata dal link alla posizione del marker per ottenere la posizione dello scan!
      Eigen::Isometry3d scanPose_eigen = mPose_eigen * linkPose_eigen;
      geometry_msgs::Pose scanPose;
      tf::poseEigenToMsg(scanPose_eigen, scanPose);

      e.points.clear();

      geometry_msgs::Point p = scanPose.position; // Posizione dello scan calcolata tramite transform
      e.points.push_back(p);

      p = mPose.position; // Posizione del marker
      e.points.push_back(p);

      e.id = count;
      e.color.r = 0.25;
      e.color.g = 0.85;
      e.color.b = 0.75;

      marray.markers.push_back(e);
    }

    link_publisher_.publish(marray);
    return;
  }

  // TODO: sta roba deve sparire
  void ApriltagAssistant::setCamera(karto::Camera *cam)
  {
    camera_ = cam;
    return;
  }

} // end namespace tag_assistant