/*
 *   Copyright (c) 2019 Chittaranjan Srinivas Swaminathan
 *   This file is part of ompl_mod_objectives.
 *
 *   ompl_mod_objectives is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   ompl_mod_objectives is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with ompl_planners_ros.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <Eigen/Dense>
#include <array>
#include <functional>

#include <cliffmap_ros/cliffmap.hpp>

#include "ompl/base/OptimizationObjective.h"

namespace ompl {

namespace mod {

/**
 * The optimization objective class for DownTheCLiFF cost.
 * This is a multi-optimization objective but doens't derive from the
 * corresponding OMPL class.
 */
class DTCOptimizationObjective : public ompl::base::OptimizationObjective {
  /// The weight associated with Euclidean distance cost.
  double weight_d;

  /// The weight associated with quaternion distance cost.
  double weight_q;

  /// The weight associated with Down-The-CLiFF cost.
  double weight_c;

  /// Maximum vehicle speed used in the computation of Down-The-CLiFF cost.
  double max_vehicle_speed;

  /// A std smart pointer to the CLiFFMap.
  cliffmap_ros::CLiFFMap cliffmap;

 public:
  /**
   * Constructor
   * @param si SpaceInformationPtr that we get from the problem setup.
   */
  DTCOptimizationObjective(const ompl::base::SpaceInformationPtr &si,
                           const cliffmap_ros::CLiFFMap &cliffmap, double wd,
                           double wq, double wc, double maxvs);

  virtual ~DTCOptimizationObjective() {}

  virtual bool isSymmetric() const override { return false; }

  ompl::base::Cost stateCost(const ompl::base::State *s) const override;

  ompl::base::Cost motionCost(const ompl::base::State *s1,
                              const ompl::base::State *s2) const override;

  ompl::base::Cost motionCostHeuristic(
      const ompl::base::State *s1, const ompl::base::State *s2) const override;
};
}
}
