//
// Created by ksatyaki on 2019-10-30.
//

#pragma once
#include <Eigen/Dense>
#include <array>
#include <functional>

#include <ompl/mod/objectives/MoDOptimizationObjective.h>
#include <whytemap_ros/whytemap.hpp>

namespace ompl {
namespace mod {

class DTWOptimizationObjective : public MoDOptimizationObjective {

  /// Maximum vehicle speed used in the computation of Down-The-CLiFF cost.
  double max_vehicle_speed;

  /// A std smart pointer to the CLiFFMap.
  whytemap_ros::WHyTeMap whytemap;

  /**
   * @brief This is the time at which WHyTe-map costs will be evaluated.
   * There is no mechanism to use OMPL's functionality to query the cost as a
   * function of time. This is how we get around it. The timestamp must be set
   * before the planning is started.
   */
  double timestamp;

public:
  /**
   * Constructor
   * @param si SpaceInformationPtr that we get from the problem setup.
   */
  DTWOptimizationObjective(const ompl::base::SpaceInformationPtr &si,
                           const whytemap_ros::WHyTeMap &whytemap, double wd,
                           double wq, double wc, double maxvs);

  virtual ~DTWOptimizationObjective() = default;

  virtual bool isSymmetric() const override { return false; }

  ompl::base::Cost stateCost(const ompl::base::State *s) const override;

  ompl::base::Cost motionCost2(const ompl::base::State *s1,
                               const ompl::base::State *s2,
                               bool print = false) const override;

  ompl::base::Cost
  motionCostHeuristic(const ompl::base::State *s1,
                      const ompl::base::State *s2) const override;

  inline void setTimeStamp(double timestamp) { this->timestamp = timestamp; }

  inline double getTimeStamp() { return this->timestamp; }
};

} // namespace mod
} // namespace ompl
