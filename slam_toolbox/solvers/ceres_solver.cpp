/*
 * Copyright 2018 Simbe Robotics, Inc.
 * Author: Steve Macenski (stevenmacenski@gmail.com)
 */

#include "ceres_solver.hpp"
#include <karto_sdk/Karto.h>

#include "ros/console.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(solver_plugins::CeresSolver, karto::ScanSolver)

namespace solver_plugins
{

  /*****************************************************************************/
  CeresSolver::CeresSolver() : nodes3d_(new std::unordered_map<int, CeresPose3d>()),
                               blocks_(new std::unordered_map<std::size_t, ceres::ResidualBlockId>()),
                               problem_(NULL), was_constant_set_(false)
  /*****************************************************************************/
  {
    ros::NodeHandle nh("~");
    std::string solver_type, preconditioner_type, dogleg_type,
        trust_strategy, loss_fn, mode;
    nh.getParam("ceres_linear_solver", solver_type);
    nh.getParam("ceres_preconditioner", preconditioner_type);
    nh.getParam("ceres_dogleg_type", dogleg_type);
    nh.getParam("ceres_trust_strategy", trust_strategy);
    nh.getParam("ceres_loss_function", loss_fn);
    nh.getParam("mode", mode);
    nh.getParam("debug_logging", debug_logging_);

    corrections_.clear();
    first_node3d_ = nodes3d_->end();

    // formulate problem
    quaternion_local_parameterization_ = new ceres::EigenQuaternionParameterization;
    subset_local_parameterization_ = new ceres::SubsetParameterization(3, {2});

    // choose loss function default squared loss (NULL)
    loss_function_ = NULL;
    if (loss_fn == "HuberLoss")
    {
      ROS_INFO("CeresSolver: Using HuberLoss loss function.");
      loss_function_ = new ceres::HuberLoss(0.7);
    }
    else if (loss_fn == "CauchyLoss")
    {
      ROS_INFO("CeresSolver: Using CauchyLoss loss function.");
      loss_function_ = new ceres::CauchyLoss(0.7);
    }

    // choose linear solver default CHOL
    options_.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    if (solver_type == "SPARSE_SCHUR")
    {
      ROS_INFO("CeresSolver: Using SPARSE_SCHUR solver.");
      options_.linear_solver_type = ceres::SPARSE_SCHUR;
    }
    else if (solver_type == "ITERATIVE_SCHUR")
    {
      ROS_INFO("CeresSolver: Using ITERATIVE_SCHUR solver.");
      options_.linear_solver_type = ceres::ITERATIVE_SCHUR;
    }
    else if (solver_type == "CGNR")
    {
      ROS_INFO("CeresSolver: Using CGNR solver.");
      options_.linear_solver_type = ceres::CGNR;
    }

    // choose preconditioner default Jacobi
    options_.preconditioner_type = ceres::JACOBI;
    if (preconditioner_type == "IDENTITY")
    {
      ROS_INFO("CeresSolver: Using IDENTITY preconditioner.");
      options_.preconditioner_type = ceres::IDENTITY;
    }
    else if (preconditioner_type == "SCHUR_JACOBI")
    {
      ROS_INFO("CeresSolver: Using SCHUR_JACOBI preconditioner.");
      options_.preconditioner_type = ceres::SCHUR_JACOBI;
    }

    if (options_.preconditioner_type == ceres::CLUSTER_JACOBI ||
        options_.preconditioner_type == ceres::CLUSTER_TRIDIAGONAL)
    {
      //default canonical view is O(n^2) which is unacceptable for
      // problems of this size
      options_.visibility_clustering_type = ceres::SINGLE_LINKAGE;
    }

    // choose trust region strategy default LM
    options_.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    if (trust_strategy == "DOGLEG")
    {
      ROS_INFO("CeresSolver: Using DOGLEG trust region strategy.");
      options_.trust_region_strategy_type = ceres::DOGLEG;
    }

    // choose dogleg type default traditional
    if (options_.trust_region_strategy_type == ceres::DOGLEG)
    {
      options_.dogleg_type = ceres::TRADITIONAL_DOGLEG;
      if (dogleg_type == "SUBSPACE_DOGLEG")
      {
        ROS_INFO("CeresSolver: Using SUBSPACE_DOGLEG dogleg type.");
        options_.dogleg_type = ceres::SUBSPACE_DOGLEG;
      }
    }

    // a typical ros map is 5cm, this is 0.001, 50x the resolution
    options_.function_tolerance = 1e-3;
    options_.gradient_tolerance = 1e-6;
    options_.parameter_tolerance = 1e-3;

    options_.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
    options_.max_num_consecutive_invalid_steps = 3;
    options_.max_consecutive_nonmonotonic_steps =
        options_.max_num_consecutive_invalid_steps;
    options_.num_threads = 50;
    options_.use_nonmonotonic_steps = true;
    options_.jacobi_scaling = true;

    options_.min_relative_decrease = 1e-3;

    options_.initial_trust_region_radius = 1e4;
    options_.max_trust_region_radius = 1e8;
    options_.min_trust_region_radius = 1e-16;

    options_.min_lm_diagonal = 1e-6;
    options_.max_lm_diagonal = 1e32;

    if (options_.linear_solver_type == ceres::SPARSE_NORMAL_CHOLESKY)
    {
      options_.dynamic_sparsity = true;
    }

    if (mode == std::string("localization"))
    {
      // doubles the memory footprint, but lets us remove contraints faster
      options_problem_.enable_fast_removal = true;
    }

    problem_ = new ceres::Problem(options_problem_);

    return;
  }

  /*****************************************************************************/
  CeresSolver::~CeresSolver()
  /*****************************************************************************/
  {
    if (loss_function_ != NULL)
    {
      delete loss_function_;
    }
    if (nodes3d_ != NULL)
    {
      delete nodes3d_;
    }
    if (problem_ != NULL)
    {
      delete problem_;
    }
    constraints_.clear();
    marker_ids_.clear();
  }

  /*****************************************************************************/
  void CeresSolver::Compute()
  /*****************************************************************************/
  {
    boost::mutex::scoped_lock lock(nodes_mutex_);
    std::cout << "\n\e[1;35mCOMPUTE!  =============================================================\e[0m\n";
    if (nodes3d_->size() == 0)
    {
      ROS_ERROR("CeresSolver: Ceres was called when there are no nodes."
                " This shouldn't happen.");
      return;
    }

    // populate constraint for static initial pose
    if (!was_constant_set_ && first_node3d_ != nodes3d_->end())
    {
      ROS_DEBUG("CeresSolver: Setting first node as a constant pose:"
                "%0.2f, %0.2f, %0.2f.",
                first_node3d_->second.p.x(),
                first_node3d_->second.p.y(), first_node3d_->second.GetEulerHeading());

      problem_->SetParameterBlockConstant(first_node3d_->second.p.data());
      problem_->SetParameterBlockConstant(first_node3d_->second.q.coeffs().data());
      was_constant_set_ = !was_constant_set_;
    }

    const ros::Time start_time = ros::Time::now();
    ceres::Solver::Summary summary;
    ceres::Solve(options_, problem_, &summary);
    if (debug_logging_)
    {
      std::cout << summary.FullReport() << '\n';
    }

    if (!summary.IsSolutionUsable())
    {
      ROS_WARN("CeresSolver: "
               "Ceres could not find a usable solution to optimize.");
      return;
    }

    // store corrected poses
    if (!corrections_.empty())
    {
      corrections_.clear();
    }
    corrections_.reserve(nodes3d_->size());
    karto::Pose3 pose;
    ConstGraphIterator3d iter = nodes3d_->begin();
    /* std::cout << "\nPrint 3D nodes after optimization: ---------------------------\n"; */
    for (iter; iter != nodes3d_->end(); ++iter)
    {
      pose = (iter->second).ToKartoPose3();

      corrections_.push_back(std::make_pair(iter->first, pose));
      //- /* std::cout << "\nCorrected pose of node with ID: " << iter->first << "\n" << pose << "\n"; */
    }
    std::cout << "\n\e[1;35m=======================================================================\e[0m\n";
    return;
  }

  /*****************************************************************************/
  const karto::ScanSolver::IdPoseVector &CeresSolver::GetCorrections() const
  /*****************************************************************************/
  {
    return corrections_;
  }

  /*****************************************************************************/
  void CeresSolver::Clear()
  /*****************************************************************************/
  {
    corrections_.clear();
  }

  /*****************************************************************************/
  void CeresSolver::Reset()
  /*****************************************************************************/
  {
    boost::mutex::scoped_lock lock(nodes_mutex_);

    corrections_.clear();
    constraints_.clear();
    marker_ids_.clear();

    was_constant_set_ = false;

    if (problem_)
    {
      delete problem_;
    }

    if (nodes3d_)
    {
      delete nodes3d_;
    }

    if (blocks_)
    {
      delete blocks_;
    }

    nodes3d_ = new std::unordered_map<int, CeresPose3d>();
    blocks_ = new std::unordered_map<std::size_t, ceres::ResidualBlockId>();
    problem_ = new ceres::Problem(options_problem_);
    first_node3d_ = nodes3d_->end();

    quaternion_local_parameterization_ = new ceres::EigenQuaternionParameterization;
  }

  /*****************************************************************************/
  void CeresSolver::AddNode(karto::Vertex<karto::LocalizedRangeScan> *pVertex)
  /*****************************************************************************/
  {
    // store LocalizedRangeScan nodes
    if (!pVertex)
    {
      return;
    }

    CeresPose3d pose3d(pVertex->GetObject()->GetCorrectedPose());
    const int id = pVertex->GetObject()->GetUniqueId();
    boost::mutex::scoped_lock lock(nodes_mutex_);
    nodes3d_->insert(std::pair<int, CeresPose3d>(id, pose3d));
    std::cout << "\n[Ceres] AddNode SCAN: " << id
              << /*"\n" << pose3d <<*/ "\n";

    // TODO: questo pezzo serve solo per controllare la coerenza delle trasformazioni tra quat e RPY -> da eliminare
    /* double yaw, pitch, roll;    
    karto::Pose3 pose_karto(pVertex->GetObject()->GetCorrectedPose());
    std::cout << "\n[karto::Pose3] AddNode SCAN:\t" << id << "\n" << pose_karto << "\n";
    
    pose3d.ToEulerAngles(yaw, pitch, roll);
    Eigen::Vector3d vect(yaw, pitch, roll);
    std::cout << "\n\n[CeresPose3d]  YPR: " << vect.transpose() << "\n";
    
    pose_karto.GetOrientation().ToEulerAngles(yaw, pitch, roll);
    Eigen::Vector3d vect2(yaw, pitch, roll);
    std::cout << "[karto::Pose3] YPR: " << vect2.transpose() << "\n";
    std::cout << "[Original Pose2] Y: " << pVertex->GetObject()->GetCorrectedPose().GetHeading() << "\n";
 */
    if (nodes3d_->size() == 1)
    {
      first_node3d_ = nodes3d_->find(id);
      /* std::cout << "\nSet first_node3d_->position\t" << first_node3d_->second.p.transpose() << "\n";
    std::cout << "Set first_node3d_->orientation\t" << first_node3d_->second.q.coeffs().transpose() << "\n"; */
    }
  }

  /*****************************************************************************/
  void CeresSolver::AddNode(karto::MarkerVertex *pMarkerVertex)
  /*****************************************************************************/
  {
    // store LocalizedMarker nodes
    if (!pMarkerVertex)
    {
      return;
    }

    CeresPose3d pose3d(pMarkerVertex->GetLocalizedMarker()->GetMarkerPose());
    const int id = pMarkerVertex->GetLocalizedMarker()->GetUniqueId();
    std::cout << "\n[Ceres] AddNode MARKER: " << id << /*"\n" << pose3d <<*/ "\n";

    // TODO: questo pezzo serve solo per controllare la coerenza delle trasformazioni tra quat e RPY -> da eliminare
    /* double yaw, pitch, roll;
    std::cout << "\n\n[CeresPose3d] AddNode MARKER:\t" << id << "\n" << pose3d << "\n";
    karto::Pose3 pose_karto = pMarkerVertex->GetLocalizedMarker()->GetMarkerPose();
    std::cout << "\n[karto::Pose3] AddNode MARKER:\t" << id << "\n" << pose_karto << "\n";

    pose3d.ToEulerAngles(yaw, pitch, roll);
    Eigen::Vector3d vect(yaw, pitch, roll);
    std::cout << "\n\n[CeresPose3d]  YPR: " << vect.transpose() << "\n";
    
    pose_karto.GetOrientation().ToEulerAngles(yaw, pitch, roll);
    Eigen::Vector3d vect2(yaw, pitch, roll);
    std::cout << "[karto::Pose3] YPR: " << vect2.transpose() << "\n"; */

    boost::mutex::scoped_lock lock(nodes_mutex_);
    nodes3d_->insert(std::pair<int, CeresPose3d>(id, pose3d));
    marker_ids_.insert(id);
  }

  /*****************************************************************************/
  void CeresSolver::AddConstraint(karto::Edge<karto::LocalizedRangeScan> *pEdge)
  /*****************************************************************************/
  {
    std::cout << "\n[Ceres] AddConstraint SCAN\t---------------------------------------\n";
    // get IDs in graph for this edge
    boost::mutex::scoped_lock lock(nodes_mutex_);

    if (!pEdge)
    {
      return;
    }

    const int node1 = pEdge->GetSource()->GetObject()->GetUniqueId();
    GraphIterator3d node1it = nodes3d_->find(node1);
    const int node2 = pEdge->GetTarget()->GetObject()->GetUniqueId();
    GraphIterator3d node2it = nodes3d_->find(node2);

    // TODO: solo per test  -> da eliminare
    /* std::cout << "\nNode1 ID: " << (int)node1it->first << "\n";
  std::cout << "Position   \t" << node1it->second.p.transpose() << "\n";
  std::cout << "Orientation\t" << node1it->second.q.coeffs().transpose() << "\n";
  std::cout << "\nNode2 ID: " << (int)node2it->first << "\n";
  std::cout << "Position   \t" << node2it->second.p.transpose() << "\n";
  std::cout << "Orientation\t" << node2it->second.q.coeffs().transpose() << "\n"; */

    if (node1it == nodes3d_->end() ||
        node2it == nodes3d_->end() ||
        node1it == node2it)
    {
      ROS_WARN("CeresSolver: Failed to add constraint, could not find nodes.");
      return;
    }

    // extract transformation
    karto::LinkInfo *pLinkInfo = (karto::LinkInfo *)(pEdge->GetLabel());
    CeresPose3d pose3d(pLinkInfo->GetPoseDifference());

    karto::Matrix3 precisionMatrix = pLinkInfo->GetCovariance().Inverse();

    // Extend precision matrix to 3D formulation:
    Eigen::Matrix<double, 6, 6> precisionMatrix3 = Eigen::Matrix<double, 6, 6>::Identity();
    precisionMatrix3(0, 0) = precisionMatrix(0, 0); // xx
    precisionMatrix3(0, 1) = precisionMatrix(0, 1); // xy
    precisionMatrix3(1, 0) = precisionMatrix(1, 0); // yx
    precisionMatrix3(1, 1) = precisionMatrix(1, 1); // yy
    //precisionMatrix3(2,2) = 0.0; // zz
    precisionMatrix3(5, 5) = precisionMatrix(2, 2); // yaw
    Eigen::Matrix<double, 6, 6> sqrt_information = precisionMatrix3.llt().matrixL();

    // TODO: solo per test  -> da eliminare
    /* std::cout << "\n\ncovariance_matrix 2D (x, y, heading)\n"
              << pLinkInfo->GetCovariance() << "\n";
    std::cout << "\nsqrt_information (square root of precision matrix) (x, y, z, roll, pitch, yaw)\n"
              << sqrt_information << "\n";
    std::cout << "\n\npose3d for cost function (t_ab_measured = diff btw node1 and node2):\n"
              << "Position:\t\t" << pose3d.p.transpose() << "\n"
              << "Orientation:\t" << pose3d.q.coeffs().transpose() << "\n"; */

    // populate residual and parameterization for heading normalization
    ceres::CostFunction *cost_function = PoseGraph3dErrorTerm::Create(pose3d, sqrt_information);

    // aggiunge le informazioni al problema, specificando CostFunction, LossFunction e parameter blocks
    ceres::ResidualBlockId block = problem_->AddResidualBlock(cost_function, loss_function_,
                                                              node1it->second.p.data(), node1it->second.q.coeffs().data(),
                                                              node2it->second.p.data(), node2it->second.q.coeffs().data());

    // imposta la LocalParameterization per i parameter block relativi all'orientamento dei due nodi
    problem_->SetParameterization(node1it->second.q.coeffs().data(), quaternion_local_parameterization_);
    problem_->SetParameterization(node2it->second.q.coeffs().data(), quaternion_local_parameterization_);

    // imposta la LocalParameterization per i parameter block relativi alla traslazione dei due nodi
    problem_->SetParameterization(node1it->second.p.data(), subset_local_parameterization_);
    problem_->SetParameterization(node2it->second.p.data(), subset_local_parameterization_);

    blocks_->insert(std::pair<std::size_t, ceres::ResidualBlockId>(
        GetHash(node1, node2), block));

    constraints_[node1].push_back(node2);
    /* std::cout << "--------------------------------------------------------------------------\n";*/

    return;
  }

  /*****************************************************************************/
  void CeresSolver::AddConstraint(karto::MarkerEdge *pMarkerEdge)
  /*****************************************************************************/
  {
    std::cout << "\n[Ceres] AddConstraint MARKER\t---------------------------------------\n";
    // get IDs in graph for this edge
    boost::mutex::scoped_lock lock(nodes_mutex_);

    if (!pMarkerEdge)
    {
      return;
    }

    const int node1 = pMarkerEdge->GetSource()->GetLocalizedMarker()->GetUniqueId();
    GraphIterator3d node1it = nodes3d_->find(node1);
    const int node2 = pMarkerEdge->GetTarget()->GetObject()->GetUniqueId();
    GraphIterator3d node2it = nodes3d_->find(node2);

    // TODO: solo per test  -> da eliminare
    /* std::cout << "\nNode1 ID: " << (int)node1it->first << "\n"
            << "Position   \t" << node1it->second.p.transpose() << "\n"
            << "Orientation\t" << node1it->second.q.coeffs().transpose() << "\n"
            << "\nNode2 ID: " << (int)node2it->first << "\n"
            << "Position   \t" << node2it->second.p.transpose() << "\n"
            << "Orientation\t" << node2it->second.q.coeffs().transpose() << "\n"; */

    if (node1it == nodes3d_->end() ||
        node2it == nodes3d_->end() ||
        node1it == node2it)
    {
      ROS_WARN("CeresSolver: Failed to add constraint, could not find nodes.");
      return;
    }

    // extract transformation
    karto::MarkerLinkInfo *pMarkerLinkInfo = (karto::MarkerLinkInfo *)(pMarkerEdge->GetLabel());
    CeresPose3d pose3d(pMarkerLinkInfo->GetPoseDifference());

    Eigen::Matrix<double, 6, 6> precisionMatrix3 = pMarkerLinkInfo->GetCovariance().inverse();
    Eigen::Matrix<double, 6, 6> sqrt_information = precisionMatrix3.llt().matrixL();

    // TODO: solo per test  -> da eliminare
    /* std::cout << "\n\ncovariance_matrix 3D (x, y, z, roll, pitch, yaw)\n"
              << pMarkerLinkInfo->GetCovariance() << "\n";
    std::cout << "\nsqrt_information (square root of precision matrix) (x, y, z, roll, pitch, yaw)\n"
              << sqrt_information << "\n";
    std::cout << "\n\npose3d for cost function (t_ab_measured = diff btw node1 and node2):\n"
              << "Position:\t\t" << pose3d.p.transpose() << "\n"
              << "Orientation:\t" << pose3d.q.coeffs().transpose() << "\n"; */

    // populate residual and parameterization for heading normalization
    ceres::CostFunction *cost_function = PoseGraph3dErrorTerm::Create(pose3d, sqrt_information);

    // aggiunge le informazioni al problema, specificando CostFunction, LossFunction e parameter blocks
    ceres::ResidualBlockId block = problem_->AddResidualBlock(cost_function, loss_function_,
                                                              node1it->second.p.data(), node1it->second.q.coeffs().data(),
                                                              node2it->second.p.data(), node2it->second.q.coeffs().data());

    // imposta la LocalParameterization per i parameter block relativi all'orientamento
    problem_->SetParameterization(node1it->second.q.coeffs().data(), quaternion_local_parameterization_);
    problem_->SetParameterization(node2it->second.q.coeffs().data(), quaternion_local_parameterization_);

    // imposta la LocalParameterization per il parameter block relativo alla traslazione del nodo 2 (blocca la z!)
    problem_->SetParameterization(node2it->second.p.data(), subset_local_parameterization_);

    blocks_->insert(std::pair<std::size_t, ceres::ResidualBlockId>(
        GetHash(node1, node2), block));

    constraints_[node1].push_back(node2);

    /* std::cout << "--------------------------------------------------------------------------\n"; */

    return;
  }

  /*****************************************************************************/
  void CeresSolver::RemoveNode(kt_int32s id)
  /*****************************************************************************/
  {
    boost::mutex::scoped_lock lock(nodes_mutex_);
    GraphIterator3d nodeit = nodes3d_->find(id);
    if (nodeit != nodes3d_->end())
    {
      // if removing a marker node, clean the set of ids
      if (marker_ids_.find(id) != marker_ids_.end())
      {
        marker_ids_.erase(id);
        //-
        std::cout << "\n>> CeresSolver::RemoveNode, node with id: "
                  << id << " removed from marker_ids_\n";
      }

      problem_->RemoveParameterBlock(nodeit->second.p.data());
      problem_->RemoveParameterBlock(nodeit->second.q.coeffs().data());
      nodes3d_->erase(nodeit);
    }
    else
    {
      ROS_ERROR("RemoveNode: Failed to find node matching id %i", (int)id);
    }
  }

  /*****************************************************************************/
  void CeresSolver::RemoveConstraint(kt_int32s sourceId, kt_int32s targetId)
  /*****************************************************************************/
  {
    // Remove constraint from list of constraints
    boost::mutex::scoped_lock l(constraints_mutex_);
    std::map<int, std::list<int>>::iterator constraints_it = constraints_.find(sourceId);
    if (constraints_it != constraints_.end())
    {
      constraints_it->second.remove(targetId);
      if (constraints_it->second.empty())
      {
        constraints_.erase(sourceId);
      }
    }
    else
    {
      ROS_ERROR("RemoveConstraint: Failed to find constraint in list for %i %i",
                (int)sourceId, (int)targetId);
    }

    // Remove constraint from the problem
    boost::mutex::scoped_lock lock(nodes_mutex_);
    std::unordered_map<std::size_t, ceres::ResidualBlockId>::iterator it_a =
        blocks_->find(GetHash(sourceId, targetId));
    std::unordered_map<std::size_t, ceres::ResidualBlockId>::iterator it_b =
        blocks_->find(GetHash(targetId, sourceId));
    if (it_a != blocks_->end())
    {
      problem_->RemoveResidualBlock(it_a->second);
      blocks_->erase(it_a);
    }
    else if (it_b != blocks_->end())
    {
      problem_->RemoveResidualBlock(it_b->second);
      blocks_->erase(it_b);
    }
    else
    {
      ROS_ERROR("RemoveConstraint: Failed to find residual block for %i %i",
                (int)sourceId, (int)targetId);
    }
    //-
    std::cout << "\n>> CeresSolver::RemoveConstraint (" << (int)sourceId << " -> "
              << (int)targetId << ")\n";
  }

  /*****************************************************************************/
  void CeresSolver::ModifyNode(const int &unique_id, Eigen::Vector3d pose)
  /*****************************************************************************/
  {
    boost::mutex::scoped_lock lock(nodes_mutex_);
    GraphIterator3d it = nodes3d_->find(unique_id);
    if (it != nodes3d_->end())
    {
      double yaw_updated = it->second.GetEulerHeading() + pose[2];
      CeresPose3d tmp;
      tmp.FromEulerAngles(yaw_updated, 0.0, 0.0);
      tmp.p << (pose[0], pose[1], 0.0);
      it->second = tmp;
    }
  }

  /*****************************************************************************/
  void CeresSolver::GetNodeOrientation(const int &unique_id, double &pose)
  /*****************************************************************************/
  {
    // NB: c'è scritto pose ma è yaw...
    boost::mutex::scoped_lock lock(nodes_mutex_);
    GraphIterator3d it = nodes3d_->find(unique_id);
    if (it != nodes3d_->end())
    {
      pose = it->second.GetEulerHeading();
    }
  }

  // NB: ritorna solo il grafo degli scan node, NON ritorna i marker node!
  /*****************************************************************************/
  std::unordered_map<int, karto::Pose3> *CeresSolver::getGraph()
  /*****************************************************************************/
  {
    boost::mutex::scoped_lock lock(nodes_mutex_);

    nodes_ = new std::unordered_map<int, karto::Pose3>();
    karto::Pose3 node;
    ConstGraphIterator3d iter = nodes3d_->begin();
    for (iter; iter != nodes3d_->end(); ++iter)
    {
      // add node only if it's NOT in the set of markers ids
      if (marker_ids_.find(iter->first) == marker_ids_.end())
      {
        node = (iter->second).ToKartoPose3();
        nodes_->insert(std::pair<int, karto::Pose3>(iter->first, node));
      }
    }
    return nodes_;
  }

  // NB: ritorna solo il grafo dei marker node, NON ritorna gli scan node!
  /*****************************************************************************/
  std::unordered_map<int, karto::Pose3> *CeresSolver::getMarkers()
  /*****************************************************************************/
  {
    boost::mutex::scoped_lock lock(nodes_mutex_);

    nodes_ = new std::unordered_map<int, karto::Pose3>();
    karto::Pose3 node;
    ConstGraphIterator3d iter = nodes3d_->begin();
    for (iter; iter != nodes3d_->end(); ++iter)
    {
      // add node only if IT'S PRESENT in the set of markers ids
      if (marker_ids_.find(iter->first) != marker_ids_.end())
      {
        node = (iter->second).ToKartoPose3();
        nodes_->insert(std::pair<int, karto::Pose3>(iter->first, node));
      }
    }
    return nodes_;
  }

  /*****************************************************************************/
  std::map<int, std::list<int>> CeresSolver::getConstraints()
  /*****************************************************************************/
  {
    boost::mutex::scoped_lock l(constraints_mutex_);

    // NB: Questo pezzo di codice serve solo a stampare il vettore di liste constraints_
    /* if (constraints_.size() != 0)
    {
        std::cout << "\n>>>>>>>>>>> Constraints:\n";
        std::map<int, std::list<int>>::const_iterator constrIter = constraints_.cbegin();
        for (constrIter; constrIter != constraints_.end(); ++constrIter)
        {
          std::cout << "Constraints of node: " << constrIter->first << " -> ";
          std::list<int>::const_iterator listIter = constrIter->second.cbegin();
          for (listIter; listIter != constrIter->second.end(); ++listIter)
          {
            std::cout << *listIter << ", ";
          }
          std::cout << "\n";
        }
    } */
    return constraints_;
  }

} // namespace solver_plugins
