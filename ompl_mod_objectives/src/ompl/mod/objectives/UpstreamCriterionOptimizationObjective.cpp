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

#include <boost/geometry.hpp>
#include <ompl/mod/objectives/UpstreamCriterionOptimizationObjective.h>

ompl::mod::UpstreamCriterionOptimizationObjective::
    UpstreamCriterionOptimizationObjective(
        const ompl::base::SpaceInformationPtr &si,
        const stefmap_ros::STeFMap &stefmap, float wd, float wq, float wc)
    : ompl::mod::MoDOptimizationObjective(si, wd, wq, wc, MapType::STeFMap),
      stefmap(new stefmap_ros::STeFMap(stefmap)) {
  description_ = "Upstream Cost over STeF-map";

  // Setup a default cost-to-go heuristics:
  setCostToGoHeuristic(ompl::base::goalRegionCostToGo);
}

ompl::mod::UpstreamCriterionOptimizationObjective::
    UpstreamCriterionOptimizationObjective(
        const ompl::base::SpaceInformationPtr &si,
        const gmmtmap_ros::GMMTMap &gmmtmap, float wd, float wq, float wc)
    : ompl::mod::MoDOptimizationObjective(si, wd, wq, wc, MapType::GMMTMap),
      gmmtmap(new gmmtmap_ros::GMMTMap(gmmtmap)) {
  description_ = "Upstream Cost over GMMT-map";

  // Setup a default cost-to-go heuristics:
  setCostToGoHeuristic(ompl::base::goalRegionCostToGo);
}

ompl::mod::UpstreamCriterionOptimizationObjective::
    UpstreamCriterionOptimizationObjective(
        const ompl::base::SpaceInformationPtr &si,
        const cliffmap_ros::CLiFFMap &cliffmap, double wd, double wq, double wc)
    : ompl::mod::MoDOptimizationObjective(si, wd, wq, wc, MapType::CLiFFMap),
      cliffmap(new cliffmap_ros::CLiFFMap(cliffmap)) {
  description_ = "Upstream Cost over CLiFF-map";

  // Setup a default cost-to-go heuristic:
  setCostToGoHeuristic(ompl::base::goalRegionCostToGo);
}

ompl::base::Cost ompl::mod::UpstreamCriterionOptimizationObjective::stateCost(
    const ompl::base::State *s) const {
  return ompl::base::Cost(0.0);
}

ompl::base::Cost
ompl::mod::UpstreamCriterionOptimizationObjective::motionCostHeuristic(
    const ompl::base::State *s1, const ompl::base::State *s2) const {
  return motionCost(s1, s2);
}

ompl::base::Cost ompl::mod::UpstreamCriterionOptimizationObjective::motionCost(
    const ompl::base::State *s1, const ompl::base::State *s2) const {
  auto space = si_->getStateSpace();
  // 1. Declare the intermediate states.
  std::vector<ompl::base::State *> intermediate_states;

  // 2. How many segments do we want. Each segment should be approximately the
  // size of resolution.
  unsigned int numSegments = space->validSegmentCount(s1, s2);

  // 3. Get intermediate states.
  si_->getMotionStates(s1, s2, intermediate_states, numSegments - 1, true,
                       true);

  double total_cost = 0.0;
  this->last_cost_.cost_d_ = 0.0;
  this->last_cost_.cost_q_ = 0.0;
  this->last_cost_.cost_c_ = 0.0;

  for (unsigned int i = 0; i < intermediate_states.size() - 1; i++) {
    std::array<double, 3> state_a{
        *space->getValueAddressAtIndex(intermediate_states[i], 0),
        *space->getValueAddressAtIndex(intermediate_states[i], 1),
        *space->getValueAddressAtIndex(intermediate_states[i], 2)};
    std::array<double, 3> state_b{
        *space->getValueAddressAtIndex(intermediate_states[i + 1], 0),
        *space->getValueAddressAtIndex(intermediate_states[i + 1], 1),
        *space->getValueAddressAtIndex(intermediate_states[i + 1], 2)};

    double dot = cos((state_b[2] - state_a[2]) / 2.0);

    // 4a. Compute Euclidean distance.
    double cost_d =
        si_->distance(intermediate_states[i], intermediate_states[i + 1]);

    // 4b. Compute the quaternion distance.
    double cost_q = (1.0 - dot * dot);

    double alpha = atan2(state_b[1] - state_a[1], state_b[0] - state_a[0]);

    double x = state_b[0];
    double y = state_b[1];

    double cost_c = 0.0;
    switch (map_type_) {
    case MapType::GMMTMap:
      cost_c = getGMMTMapCost(x, y, alpha);
      break;
    case MapType::STeFMap:
      cost_c = getSTeFMapCost(x, y, alpha);
      break;
    case MapType::CLiFFMap:
      cost_c = getCLiFFMapCost(x, y, alpha);
      break;
    default:
      ROS_WARN_THROTTLE(2,
                        "Warning: motionCost() called with MapType: %s. "
                        "Returning identity cost.",
                        getMapTypeStr().c_str());
      cost_c = this->identityCost().value();
    }

    total_cost +=
        (weight_d_ * cost_d) + (weight_q_ * cost_q) + (weight_c_ * cost_c);
    this->last_cost_.cost_c_ += cost_c;
    this->last_cost_.cost_d_ += cost_d;
    this->last_cost_.cost_q_ += cost_q;
    si_->freeState(intermediate_states[i]);
  }
  si_->freeState(intermediate_states[intermediate_states.size() - 1]);
  return ompl::base::Cost(total_cost);
}

double ompl::mod::UpstreamCriterionOptimizationObjective::getSTeFMapCost(
    double x, double y, double alpha) const {
  double mod_cost = 0.0;

  const stefmap_ros::STeFMapCell &cell = (*stefmap)(x, y);
  for (int i = 0; i < 8; i++) {
    mod_cost +=
        (cell.probabilities[i] * 0.01) * (1 - cos(alpha - (i * M_PI / 4)));
  }
  return mod_cost;
}

double ompl::mod::UpstreamCriterionOptimizationObjective::getGMMTMapCost(
    double x, double y, double alpha) const {
  double mod_cost = 0.0;
  auto dists = (*gmmtmap)(x, y);

  for (const auto &dist : dists) {
    double mixing_factor = gmmtmap->getMixingFactorByClusterID(dist.second[0]);
    double dist_heading = gmmtmap->getHeadingAtDist(dist.second[0], dist.second[1]);
    
    double distance_between_gmmtmap_mean_and_current_state_xy =
        boost::geometry::distance(dist.first, gmmtmap_ros::Point2D(x, y));
    mod_cost += mixing_factor *
                (1 - distance_between_gmmtmap_mean_and_current_state_xy /
                         gmmtmap->getStdDev()) *
                (1 - cos(alpha - dist_heading));
  }

  return mod_cost;
}

double ompl::mod::UpstreamCriterionOptimizationObjective::getCLiFFMapCost(
    double x, double y, double alpha) const {
  double mod_cost = 0.0;
  const cliffmap_ros::CLiFFMapLocation &cl = (*cliffmap)(x, y);

  for (const auto &dist : cl.distributions) {
    mod_cost +=
        dist.getMixingFactor() * (1 - cos(dist.getMeanHeading() - alpha));
  }
  return mod_cost;
}
