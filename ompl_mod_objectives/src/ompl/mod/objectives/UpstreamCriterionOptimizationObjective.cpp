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

#include <ompl/mod/objectives/UpstreamCriterionOptimizationObjective.h>

ompl::mod::UpstreamCriterionOptimizationObjective::
    UpstreamCriterionOptimizationObjective(
        const ompl::base::SpaceInformationPtr &si,
        const stefmap_ros::STeFMap &stefmap, float wd, float wq, float wc)
    : ompl::base::OptimizationObjective(si),
      stefmap(stefmap),
      weight_d(wq),
      weight_q(wq),
      weight_c(wc) {
  description_ = "STeF-up Cost";

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

    double dot = cos(state_b[2] / 2.0) * cos(state_a[2] / 2.0) +
                 sin(state_b[2] / 2.0) * sin(state_a[2] / 2.0);

    // 4a. Compute Euclidean distance.
    double this_distance =
        si_->distance(intermediate_states[i], intermediate_states[i + 1]);

    // 4b. Compute the quaternion distance.
    double q_dist = (1.0 - dot * dot);

    double stefmap_cost = 0.0;
    double alpha = atan2(state_b[1] - state_a[1], state_b[0] - state_a[0]);

    double x = state_b[0];
    double y = state_b[1];

    const stefmap_ros::STeFMapCell &cell = stefmap(x, y);

    for (int i = 0; i < 8; i++) {
      stefmap_cost += cell.probabilities[i] * (1 - cos(alpha - (i * M_PI / 4)));
    }

    total_cost += (weight_d * this_distance) + (weight_q * q_dist) +
                  (stefmap_cost * weight_c);
    si_->freeState(intermediate_states[i]);
  }
  si_->freeState(intermediate_states[intermediate_states.size() - 1]);
  return ompl::base::Cost(total_cost);
}