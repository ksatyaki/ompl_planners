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

#pragma once

#include <stefmap_ros/stefmap.hpp>
#include "ompl/base/OptimizationObjective.h"

namespace ompl {
namespace mod {

class UpstreamCriterionOptimizationObjective
    : public ompl::base::OptimizationObjective {
  double weight_d;

  double weight_q;

  double weight_c;

  stefmap_ros::STeFMap stefmap;

 public:
  UpstreamCriterionOptimizationObjective(
      const ompl::base::SpaceInformationPtr &si,
      const stefmap_ros::STeFMap &stefmap, float wd, float wq, float wc);

  virtual inline bool isSymmetric() const override { return false; }

  ompl::base::Cost stateCost(const ompl::base::State *s) const override;

  ompl::base::Cost motionCost(const ompl::base::State *s1,
                              const ompl::base::State *s2) const override;

  ompl::base::Cost motionCostHeuristic(
      const ompl::base::State *s1, const ompl::base::State *s2) const override;

  virtual ~UpstreamCriterionOptimizationObjective() {}
};

} /* namespace mod */
} /* namespace ompl */
