/*
 * Copyright 2018 Simbe Robotics, Inc.
 * Author: Steve Macenski (stevenmacenski@gmail.com)
 */

#ifndef KARTO_CERESSOLVER_H
#define KARTO_CERESSOLVER_H

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <karto_sdk/Mapper.h>
#include <ceres/ceres.h>
#include <ceres/local_parameterization.h>
#include <cmath>
#include <math.h>

#include "../include/slam_toolbox/toolbox_types.hpp"
#include "ceres_utils.h"

namespace solver_plugins
{

  using namespace ::toolbox_types;

  class CeresSolver : public karto::ScanSolver
  {
  public:
    CeresSolver();
    virtual ~CeresSolver();

  public:
    virtual const karto::ScanSolver::IdPoseVector &GetCorrections() const;
    virtual void Compute();
    virtual void Clear();
    virtual void Reset();

    virtual void AddNode(karto::Vertex<karto::LocalizedRangeScan> *pVertex);
    virtual void AddNode(karto::MarkerVertex *pMarkerVertex);
    virtual void AddConstraint(karto::Edge<karto::LocalizedRangeScan> *pEdge);
    virtual void AddConstraint(karto::MarkerEdge *pMarkerEdge);
    virtual std::unordered_map<int, karto::Pose3> *getGraph();
    virtual std::unordered_map<int, karto::Pose3> *getMarkers();
    virtual std::map<int, std::list<int>> getConstraints();
    virtual void RemoveNode(kt_int32s id);
    virtual void RemoveConstraint(kt_int32s sourceId, kt_int32s targetId);

    virtual void ModifyNode(const int &unique_id, Eigen::Vector3d pose);
    virtual void GetNodeOrientation(const int &unique_id, double &pose);

  private:
    // karto
    karto::ScanSolver::IdPoseVector corrections_;

    // ceres
    ceres::Solver::Options options_;
    ceres::Problem::Options options_problem_;
    ceres::LossFunction *loss_function_;
    ceres::Problem *problem_;
    ceres::LocalParameterization *quaternion_local_parameterization_;
    ceres::LocalParameterization *subset_local_parameterization_;
    bool was_constant_set_, debug_logging_;

    // graph
    std::unordered_map<int, karto::Pose3> *nodes_;
    std::unordered_map<size_t, ceres::ResidualBlockId> *blocks_;
    std::unordered_map<int, Eigen::Vector3d>::iterator first_node_;
    boost::mutex nodes_mutex_;
    boost::mutex constraints_mutex_;

    std::unordered_map<int, CeresPose3d> *nodes3d_;
    std::unordered_map<int, CeresPose3d>::iterator first_node3d_;
    std::map<int, std::list<int>> constraints_;
    std::unordered_set<int> marker_ids_;
  };

}

#endif
