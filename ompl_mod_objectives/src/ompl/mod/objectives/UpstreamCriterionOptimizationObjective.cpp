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

ompl::base::Cost ompl::mod::UpstreamCriterionOptimizationObjective::stateCost(
    const ompl::base::State *s) const {
  return this->identityCost();
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
    double this_distance =
        si_->distance(intermediate_states[i], intermediate_states[i + 1]);

    // 4b. Compute the quaternion distance.
    double q_dist = (1.0 - dot * dot);

    double alpha = atan2(state_b[1] - state_a[1], state_b[0] - state_a[0]);
    double o_change = atan2(sin(state_b[2]) - sin(state_a[2]),
                            cos(state_b[2]) - cos(state_a[2]));

    double reverse_cost =
        std::abs(atan2(sin(alpha - o_change), cos(alpha - o_change))) < 0.5
            ? 0.0
            : 1.0;

    double x = state_b[0];
    double y = state_b[1];

    double mod_cost = 0.0;
    switch (map_type_) {
    case MapType::GMMTMap:
      mod_cost = getGMMTMapCost(x, y, alpha);
      break;
    case MapType::STeFMap:
      mod_cost = getSTeFMapCost(x, y, alpha);
      break;
    default:
      ROS_WARN_THROTTLE(2,
                        "Warning: motionCost() called with MapType: %s. "
                        "Returning identity cost.",
                        getMapTypeStr().c_str());
      mod_cost = this->identityCost().value();
    }

    total_cost += (weight_d_ * this_distance) + (weight_q_ * q_dist) +
                  (mod_cost * weight_c_) +
                  (reverse_cost * weight_d_ * this_distance);
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
    mod_cost += cell.probabilities[i] * (1 - cos(alpha - (i * M_PI / 4)));
  }
  return mod_cost;
}

double ompl::mod::UpstreamCriterionOptimizationObjective::getGMMTMapCost(
    double x, double y, double alpha) const {
  double mod_cost = 0.0;
  auto dists = (*gmmtmap)(x, y);

  for (const auto &dist : dists) {
    double beta = atan2(dist.first.get<1>(), dist.first.get<0>());
    double distance_between_gmmtmap_mean_and_current_state_xy =
        boost::geometry::distance(dist.first, gmmtmap_ros::Point2D(x, y));
    mod_cost += (1 - distance_between_gmmtmap_mean_and_current_state_xy /
                         gmmtmap->getStdDev()) *
                (1 - cos(alpha - beta));
  }

  return mod_cost;
}