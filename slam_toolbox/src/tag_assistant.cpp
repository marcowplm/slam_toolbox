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
        nh_.getParam("odom_frame", odom_frame_);
        nh_.getParam("camera_frame", camera_frame_);
        tag_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("tag_visualization", 1);
        link_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("link_visualization", 1);
        tfB_ = std::make_unique<tf2_ros::TransformBroadcaster>();
        solver_ = mapper_->getScanSolver();
        markerGraph_ = mapper_->GetMarkerGraph();   // inutile!
        camera_ = makeCamera();
    }

    /*****************************************************************************/
    bool ApriltagAssistant::processDetection(const int last_vertex_id,
                                             const karto::Pose2 last_vertex_pose,
                                             const apriltag_ros::AprilTagDetectionArrayConstPtr &detection_array)
    /*****************************************************************************/
    {
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
                        karto::LocalizedMarkerMap::const_iterator markers_it = mapper_->GetMarkers().find(tags_Iter->first);
                        karto::Pose3 tag_pose_ = markers_it->second->GetOdometricPose();
                        if (!addLink(last_vertex_id, last_vertex_pose, tags_Iter->first, tag_pose_, false))
                        {
                            processed = false;
                            return processed;
                        }
                        tags_Iter->second.insert(last_vertex_id); // aggiungo il vertex alla map, in corrispondenza dell'id del tag
                        processed = true;
                    } // else: il tag è già associato al vertex -> non fare niente
                }
                else // il tag non esiste nella map (è la prima volta che lo vedo)
                {
                    if (!addLocalizedMarker(last_vertex_id, last_vertex_pose, *detectionConstIter))
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
    bool ApriltagAssistant::addLink(const int vertex_id, 
                                    const karto::Pose2 vertex_pose, 
                                    const int tag_id, 
                                    const karto::Pose3 tag_pose, 
                                    const bool isFirstLink)
    /*****************************************************************************/
    {
        karto::Pose3 link_pose;                 // qui ci metto la tansform
        karto::Pose3 vertex_pose3(vertex_pose); // questa è la posizione dello scan vertex

        Eigen::Isometry3d vertex_transform = poseKartoToEigenIsometry(vertex_pose3);
        Eigen::Isometry3d tag_transform = poseKartoToEigenIsometry(tag_pose);
        Eigen::Isometry3d link_transform = vertex_transform.inverse() * tag_transform; // link_transform ha origine in Vertex e punta in Tag

        link_pose = isometryEigenToPoseKarto(link_transform);

        tag_assistant::Link link_(tag_id, vertex_id, link_transform);
        //std::set<Link> temp_set ({link_});
        if (isFirstLink)
        {
            links_.insert(std::pair <int, std::set<Link> > (tag_id, std::set<Link>({link_})));
        }
        else
        {
            links_[tag_id].insert(link_);
        } 

        return true;
    }

    /** 
     * Recupera la posizione del tag nel mondo, crea il Vertex associato al tag da aggiungere 
     * e poi crea l'Edge tra il nuovo vertice e il vertice dello scan
     */
    /*****************************************************************************/
    bool ApriltagAssistant::addLocalizedMarker(const int vertex_id, 
                                               const karto::Pose2 vertex_pose, 
                                               const apriltag_ros::AprilTagDetection &detection)
    /*****************************************************************************/
    {
        karto::Pose3 tag_pose_; // è la posizione del tag nel mondo
        if (!getTagPose(tag_pose_, detection))
        {
            return false;
        }
        karto::LocalizedMarker *tag = createLocalizedMarker(getCamera(), detection.id[0], tag_pose_);
        if (!mapper_->ProcessMarker(tag))
        {
            std::cout << "ProcessMarker FALSE!" << std::endl;
            return false;
        }

        if (!addLink(vertex_id, vertex_pose, detection.id[0], tag_pose_, true))
        {
            return false;
        }
        return true;
    }

    // Chiama il costruttore di karto::LocalizedMarker
    /*****************************************************************************/
    karto::LocalizedMarker *ApriltagAssistant::createLocalizedMarker(karto::Camera *camera, 
                                                                     int unique_id, 
                                                                     karto::Pose3 tag_pose_karto)
    /*****************************************************************************/
    {
        karto::LocalizedMarker *tag = new karto::LocalizedMarker(camera->GetName(), unique_id, tag_pose_karto);
        tag->SetCorrectedPose(tag_pose_karto);
        return tag;
    }

    // Ottiene la posizione del tag nel riferimento mondo (map_frame_), nel momento in cui il tag viene creato
    /*****************************************************************************/
    bool ApriltagAssistant::getTagPose(karto::Pose3 &tag_pose_karto,
                                       const apriltag_ros::AprilTagDetection &detection)
    /*****************************************************************************/
    {
        // base_ident è un "contenitore" che tf_->transform userà per tirare fuori il suo source frame,
        // camera_rgb_optical, che è in base_ident.header.frame_id
        geometry_msgs::TransformStamped base_ident;
        base_ident.header.stamp = detection.pose.header.stamp;
        base_ident.header.frame_id = detection.pose.header.frame_id;
        base_ident.transform.rotation.w = 1.0;

        try
        {
            // Ottiene la posizione della camera -> Target frame: map / Source frame: camera_rgb_optical (all'interno di base_ident)
            camera_pose_ = tf_->transform(base_ident, map_frame_);
        }
        catch (tf2::TransformException e)
        {
            // ROS_ERROR("Failed to compute laser pose, aborting continue mapping (%s)", e.what());
            return false;
        }
        geometry_msgs::Pose tag_pose_;
        // Input pose: detection_pose / Output pose: tag_pose_ / Transformed by camera_pose_
        tf2::doTransform(detection.pose.pose.pose, tag_pose_, camera_pose_);
        tag_pose_karto = poseGeometryToKarto(tag_pose_);

        // Adesso provo a pubblicare la tf di tag_pose rispetto a map_frame
        geometry_msgs::TransformStamped tag_pose_stamped_;
        tag_pose_stamped_.transform.rotation = tag_pose_.orientation;
        tag_pose_stamped_.transform.translation.x = tag_pose_.position.x;
        tag_pose_stamped_.transform.translation.y = tag_pose_.position.y;
        tag_pose_stamped_.transform.translation.z = tag_pose_.position.z;
        tag_pose_stamped_.header.frame_id = map_frame_;
        tag_pose_stamped_.header.stamp = base_ident.header.stamp;
        tag_pose_stamped_.child_frame_id = "TAG" + std::to_string(detection.id[0]);
        tfB_->sendTransform(tag_pose_stamped_);
        return true;
    }

    // Qui prendo semplicemente i Marker e i Vertex del grafo e visualizzo gli edge tra queste due posiioni 
    /*****************************************************************************/
    void ApriltagAssistant::publishMarkerGraph()
    /*****************************************************************************/
    {   
        std::unordered_map<int, Eigen::Vector3d> *graph = solver_->getGraph();

        visualization_msgs::MarkerArray marray;
        visualization_msgs::Marker m = vis_utils::toTagMarker(map_frame_, nh_.getNamespace());
        visualization_msgs::Marker e = vis_utils::toEdgeMarker(map_frame_, nh_.getNamespace(), 0.03);

        for (tags_Iter = tags_.begin(); tags_Iter != tags_.end(); ++tags_Iter)
        {
            karto::LocalizedMarkerMap::iterator markers_it = mapper_->GetMarkers().find(tags_Iter->first);
            geometry_msgs::Pose mPose = poseKartoToGeometry(markers_it->second->GetOdometricPose());

            m.id = tags_Iter->first;
            m.pose.orientation = mPose.orientation;
            m.pose.position = mPose.position;

            marray.markers.push_back(m);

            std::set<int>::const_iterator set_Iter = tags_Iter->second.begin();
            for (set_Iter; set_Iter != tags_Iter->second.end(); ++set_Iter)
            {
                e.points.clear();

                geometry_msgs::Point p = mPose.position; // Posizione del Marker presa da MarkerGraph
                e.points.push_back(p);

                p.x = graph->find(*set_Iter)->second(0); // Posizione dello Scan presa da MapperGraph
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

    /**
     * Qui prendo i Marker e le transform salvate e calcolo di conseguenza la posizione dei Vertex.
     * L'edge visualizzato, quindi, corrsiponde proprio alla transform calcolata
     * TODO: questa funzione sarà eliminata, insieme alla struct Link (e a links_)
     */  
    /*****************************************************************************/
    void ApriltagAssistant::publishLinks()
    /*****************************************************************************/
    {
        visualization_msgs::MarkerArray marray;
        visualization_msgs::Marker m = vis_utils::toTagMarker(map_frame_, nh_.getNamespace());
        visualization_msgs::Marker e = vis_utils::toEdgeMarker(map_frame_, nh_.getNamespace(), 0.02);

        std::cout << "\n================= publish & visualize links_ - size of links_: " << links_.size() << " =================" << std::endl;
        for (links_Iter = links_.begin(); links_Iter != links_.end(); ++links_Iter)
        {
            karto::LocalizedMarkerMap::const_iterator markers_it = mapper_->GetMarkers().find(links_Iter->first);
            karto::Pose3 mPose_karto = markers_it->second->GetOdometricPose();
            geometry_msgs::Pose mPose = poseKartoToGeometry(mPose_karto);
            Eigen::Isometry3d mPose_eigen = poseKartoToEigenIsometry(mPose_karto);

            m.id = links_Iter->first;
            m.pose = mPose;
            marray.markers.push_back(m);

            std::set<Link>::const_iterator set_Iter = links_Iter->second.begin();
            for (set_Iter; set_Iter != links_Iter->second.end(); ++set_Iter)
            {
                std::cout << "\nTagID:\t\t" << set_Iter->tag_id_ << std::endl
                          << "VertexID:\t" << set_Iter->vertex_id_ << std::endl
                          << "Link (transform between tag and vertex):" << std::endl
                          << " Norm (length of translation):\t" << set_Iter->transform_.translation().norm() << std::endl
                          << "------------------------" << std::endl;

                // Applico la transform rappresentata dal link alla posizione del marker per ottenere la posizione del vertex!
                Eigen::Isometry3d vertexPose_eigen = mPose_eigen * (set_Iter->transform_).inverse();
                geometry_msgs::Pose vertexPose;
                tf::poseEigenToMsg(vertexPose_eigen, vertexPose);

                e.points.clear();

                geometry_msgs::Point p = vertexPose.position; // posizione del Vertex calcolata tramite transform
                e.points.push_back(p);

                p = m.pose.position; // Posizione del Marker
                e.points.push_back(p);

                e.id = ((std::hash<double>()(m.id) ^ (std::hash<double>()(set_Iter->vertex_id_) << 1)) >> 1);
                e.color.r = 0.25;
                e.color.g = 0.85;
                e.color.b = 0.75; 

                marray.markers.push_back(e);
            }
        }

        link_publisher_.publish(marray);
        return;
    }

    /*****************************************************************************/
    karto::Camera *ApriltagAssistant::makeCamera()
    /*****************************************************************************/
    {
        karto::Camera *camera = karto::Camera::CreateCamera(karto::Name("Custom Camera"));
        karto::SensorManager::GetInstance()->RegisterSensor(camera);
        return camera;
    }

    /*****************************************************************************/
    karto::Camera *ApriltagAssistant::getCamera()
    /*****************************************************************************/
    {
        return camera_;
    }

} // end namespace tag_assistant