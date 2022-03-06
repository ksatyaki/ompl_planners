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
 *   along with ompl_mod_objectives. If not, see <https://www.gnu.org/licenses/>
 */

#pragma once

#include <ompl/mod/objectives/MoDOptimizationObjective.h>

#include <cliffmap_ros/cliffmap.hpp>
#include <gmmtmap_ros/gmmtmap.hpp>
#include <stefmap_ros/stefmap.hpp>

namespace ompl {
namespace mod {

class UpstreamCriterionOptimizationObjective
    : public ompl::mod::MoDOptimizationObjective {
  stefmap_ros::STeFMapPtr stefmap;

  gmmtmap_ros::GMMTMapPtr gmmtmap;

  cliffmap_ros::CLiFFMapPtr cliffmap;

  cliffmap_ros::IntensityMap intensity_map;

  bool use_intensity{false};

public:
  UpstreamCriterionOptimizationObjective(
      const ompl::base::SpaceInformationPtr &si,
      const stefmap_ros::STeFMap &stefmap, float wd, float wq, float wc);

  UpstreamCriterionOptimizationObjective(
      const ompl::base::SpaceInformationPtr &si,
      const gmmtmap_ros::GMMTMap &gmmtmap, float wd, float wq, float wc);

  UpstreamCriterionOptimizationObjective(
      const ompl::base::SpaceInformationPtr &si,
      const cliffmap_ros::CLiFFMap &cliffmap, double wd, double wq, double wc);

  UpstreamCriterionOptimizationObjective(
      const ompl::base::SpaceInformationPtr &si,
      const cliffmap_ros::CLiFFMap &cliffmap,
      const std::string &intensity_map_file_name, double wd, double wq,
      double wc);

  UpstreamCriterionOptimizationObjective(
      const ompl::base::SpaceInformationPtr &si,
      const ompl::mod::MapType &map_type, const std::string &map_file_name,
      float wd, float wq, float wc);

  inline bool isSymmetric() const override { return false; }

  ompl::base::Cost stateCost(const ompl::base::State *s) const override;

  ompl::base::Cost motionCost(const ompl::base::State *s1,
                              const ompl::base::State *s2) const override;

  double getSTeFMapCost(double x, double y, double alpha) const;

  double getGMMTMapCost(double x, double y, double alpha) const;

  double getCLiFFMapCost(double x, double y, double alpha) const;

  ompl::base::Cost
  motionCostHeuristic(const ompl::base::State *s1,
                      const ompl::base::State *s2) const override;

  ~UpstreamCriterionOptimizationObjective() override {}
};

typedef std::shared_ptr<UpstreamCriterionOptimizationObjective>
    UpstreamCriterionOptimizationObjectivePtr;

} /* namespace mod */
} /* namespace ompl */
