#include <iostream>

#include "slam_toolbox/tag_assistant.hpp"

namespace tag_assistant
{

    /*****************************************************************************/
    ApriltagAssistant::ApriltagAssistant(
        ros::NodeHandle &nh, tf2_ros::Buffer *tf, karto::Mapper *mapper)
        : nh_(nh), tf_(tf), mapper_(mapper)
    /*****************************************************************************/
    {
        nh_.getParam("map_frame", map_frame_);
        nh_.getParam("odom_frame", odom_frame_);
        nh_.getParam("camera_frame", camera_frame_);
        tag_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("tag_visualization", 1);
        solver_ = mapper_->getScanSolver();
        camera_ = makeCamera();
    }

    /*****************************************************************************/
    bool ApriltagAssistant::processDetection(const int &last_vertex_id,
                                             const karto::Pose2 last_vertex_pose,
                                             const apriltag_ros::AprilTagDetectionArrayConstPtr &detection_array)
    /*****************************************************************************/
    {
        // std::cout << ">>>>>>>>>> processDetection!" << std::endl;
        bool processed = false;
        for (detectionConstIter = detection_array->detections.begin(); detectionConstIter != detection_array->detections.end(); ++detectionConstIter)
        {
            geometry_msgs::Pose detection_pose_(detectionConstIter->pose.pose.pose);
            double dist = Eigen::Vector3d(detection_pose_.position.x, detection_pose_.position.y, detection_pose_.position.z).norm();
            if (dist < 3.0)
            {
                tags_Iter = tags_.find(detectionConstIter->id[0]);
                if (tags_Iter != tags_.end()) // il tag esiste nella struttura dati
                {
                    if (tags_Iter->second.find(last_vertex_id) == tags_Iter->second.end()) // il tag esiste ma non è associato al vertex
                    {
                        if (!addLink(last_vertex_id, last_vertex_pose, *detectionConstIter, false))
                        {
                            processed = false;
                            return processed;
                        }
                        tags_Iter->second.insert(last_vertex_id); // aggiungo il vertex alla map, in corrispondenza dell'id del tag
                        processed = true;
                    } // else se il tag è già associato al vertex -> non fare niente
                }
                else // il tag non esiste nella map (è la prima volta che lo vedo)
                {
                    if (!addTag(last_vertex_id, last_vertex_pose, *detectionConstIter))
                    {
                        processed = false;
                        return processed;
                    }
                    std::set<int> temp_set{last_vertex_id};
                    tags_.insert(std::pair<int, std::set<int>>(detectionConstIter->id[0], temp_set)); // inserisco il nuovo tag con relativo vertex nella struttura
                    processed = true;
                }
            }
        }
        // std::cout << ">>>>>>>>>> processDetection >>>> " << processed << std::endl;
        return processed;
    }

    /** 
     * Prende l'ID del tag e dello scan. Cerca il Vertex del tag e dello scan, 
     * recupera entrambe le posizioni e ne calcola la transform
     */
    /*****************************************************************************/
    bool ApriltagAssistant::addLink(const int &vertex_id, const karto::Pose2 vertex_pose, const apriltag_ros::AprilTagDetection &detection, const bool isFirstLink)
    /*****************************************************************************/
    {
        // std::cout << ">>>>>>>>>> addLink!" << std::endl;
        karto::Pose3 link_transform;
        karto::Pose3 vertex_pose3(vertex_pose);
        karto::Pose3 tag_pose; // è la posizione del tag nel mondo
        if (!getTagPose(tag_pose, detection))
        {
            return false;
        }
        vertex_pose3 = invertTransform3(vertex_pose3);
        link_transform.SetPosition(vertex_pose3.GetPosition() + tag_pose.GetPosition());

        Eigen::Quaterniond l_v_quat = kartoToEigenQuaternion(vertex_pose3.GetOrientation());
        Eigen::Quaterniond t_p_quat = kartoToEigenQuaternion(tag_pose.GetOrientation());
        Eigen::Quaterniond l_t_quat = l_v_quat * t_p_quat;
        l_t_quat.normalize();
        link_transform.SetOrientation(EigenTokartoQuaternion(l_t_quat));

        tag_assistant::Link link_(detection.id[0], vertex_id, link_transform);
        std::set<Link> temp_set ({link_});
        if (isFirstLink)
        {
            links_.insert(std::pair <int, std::set<Link> > (detection.id[0], temp_set));
        }
        else
        {
            links_[detection.id[0]].insert(link_);
        }
        
        return true;
    }

    /** 
     * Recupera la posizione del tag nel mondo, crea il Vertex associato al tag da aggiungere 
     * e poi crea l'Edge tra il nuovo vertice e il vertice dello scan
     */
    /*****************************************************************************/
    bool ApriltagAssistant::addTag(const int &vertex_id, const karto::Pose2 vertex_pose, const apriltag_ros::AprilTagDetection &detection)
    /*****************************************************************************/
    {
        // std::cout << ">>>>>>>>> addTag!" << std::endl;
        karto::Pose3 tag_pose; // è la posizione del tag nel mondo
        if (!getTagPose(tag_pose, detection))
        {
            return false;
        }
        karto::LocalizedMarker* tag = getLocalizedTag(getCamera(), detection.id[0], tag_pose);
        if (!mapper_->ProcessMarker(tag))
        {
            std::cout << "ProcessMarker FALSE!" << std::endl;
            return false;
        }

        if (!addLink(vertex_id, vertex_pose, detection, true))
        {
            return false;
        }
        return true;
    }

    // Chiama il costruttore di karto::LocalizedMarker
    /*****************************************************************************/
    karto::LocalizedMarker* ApriltagAssistant::getLocalizedTag(karto::Camera* camera, int unique_id, karto::Pose3 karto_pose)
    /*****************************************************************************/
    {
        // std::cout << ">>>>>>> getLocalizedTag!" << std::endl;
        karto::LocalizedMarker* tag = new karto::LocalizedMarker(camera->GetName(), unique_id, karto_pose);
        tag->SetCorrectedPose(karto_pose);
        return tag;
    }

    // Ottiene la posizione del tag nel mondo
    /*****************************************************************************/
    bool ApriltagAssistant::getTagPose(karto::Pose3 &karto_pose,
                                       const apriltag_ros::AprilTagDetection &detection)
    /*****************************************************************************/
    {
        geometry_msgs::TransformStamped base_ident;
        base_ident.header.stamp = detection.pose.header.stamp;
        base_ident.header.frame_id = detection.pose.header.frame_id;
        base_ident.transform.rotation.w = 1.0;

        try
        {
            // Ottiene la posizione della camera -> Target frame: odom / Source frame: camera_rgb_optical
            camera_pose_ = tf_->transform(base_ident, map_frame_);
        }
        catch (tf2::TransformException e)
        {
            // ROS_ERROR("Failed to compute laser pose, aborting continue mapping (%s)", e.what());
            return false;
        }
        geometry_msgs::Pose tag_pose_;
        // Input frame: detection_pose_ / Output_frame: tag_pose_ / Transformed by camera_pose_
        tf2::doTransform(detection.pose.pose.pose, tag_pose_, camera_pose_);
        karto_pose = geometryToKarto(tag_pose_);
        return true;
    }

    // inutile
    /*****************************************************************************/
    karto::Camera *ApriltagAssistant::makeCamera()
    /*****************************************************************************/
    {
        karto::Camera *camera = karto::Camera::CreateCamera(karto::Name("Custom Camera"));
        karto::SensorManager::GetInstance()->RegisterSensor(camera);
        return camera;
    }

    // inutile
    /*****************************************************************************/
    karto::Camera *ApriltagAssistant::getCamera()
    /*****************************************************************************/
    {
        return camera_;
    }

    /*****************************************************************************/
    void ApriltagAssistant::publishMarkerGraph()
    /*****************************************************************************/
    {
        karto::LocalizedMarkerMap markers = mapper_->GetMarkerGraph()->GetLocalizedMarkers();
        std::unordered_map<int, Eigen::Vector3d>* graph = solver_->getGraph();

        if (markers.size() == 0)
        {
            return;
        }

        visualization_msgs::MarkerArray marray;
        visualization_msgs::Marker m = vis_utils::toTagMarker(
            map_frame_, nh_.getNamespace(), 0.1);
        visualization_msgs::Marker e = vis_utils::toEdgeMarker(
            map_frame_, nh_.getNamespace(), 0.03);

        for (tags_Iter = tags_.begin(); tags_Iter != tags_.end(); ++tags_Iter)
        {     
            geometry_msgs::Pose mPose = kartoToGeometry(markers.find(tags_Iter->first)->second->GetOdometricPose());

            m.id = tags_Iter->first;
            m.pose.orientation = mPose.orientation;
            m.pose.position = mPose.position;

            marray.markers.push_back(m);

            std::set<int>::const_iterator set_Iter = tags_Iter->second.begin();
            for (set_Iter; set_Iter != tags_Iter->second.end(); ++set_Iter)
            {
                e.points.clear();

                geometry_msgs::Point p = m.pose.position; // Posizione del Marker
                e.points.push_back(p);

                p.x = graph->find(*set_Iter)->second(0); // Posizione dello Scan
                p.y = graph->find(*set_Iter)->second(1);
                p.z = 0.0;
                e.points.push_back(p);

                e.id = ((std::hash<double>()(m.id) ^ (std::hash<double>()((int)*set_Iter) << 1)) >> 1);
                e.color.r = 1.0;
                e.color.g = 0.5;
                e.color.b = 0.0;

                marray.markers.push_back(e);
            }
        }

        tag_publisher_.publish(marray);
        return;
    }

    /*****************************************************************************/
    void ApriltagAssistant::publishLinks()
    /*****************************************************************************/
    {
        std::cout << "\n================= publish links_ =================" << std::endl;
        for (links_Iter = links_.begin(); links_Iter != links_.end(); ++links_Iter)
        {
            std::set<Link>::iterator setIter = links_Iter->second.begin();
            for (setIter; setIter != links_Iter->second.end(); ++setIter)
            {
                std::cout << "TagID:\t\t" << setIter->tag_id_ << std::endl
                          << "VertexID:\t" << setIter->vertex_id_ << std::endl
                          << "Link (transform between tag and vertex):" << std::endl 
                          << " Position:\t" << setIter->transform_.GetPosition() << std::endl 
                          << " Orientation:\t" << setIter->transform_.GetOrientation() << std::endl
                          << "-------------" << std::endl;
            }        
        }
    }
    
} // end namespace tag_assistant