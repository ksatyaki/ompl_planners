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

#include "ompl/mod/objectives/DTCOptimizationObjective.h"

#include <ros/console.h>

#include <memory>

ompl::mod::DTCOptimizationObjective::DTCOptimizationObjective(
    const ompl::base::SpaceInformationPtr &si,
    const cliffmap_ros::CLiFFMap &cliffmap, double wd, double wq, double wc,
    double maxvs, double mahalanobis_distance_threshold, bool use_mixing_factor)
    : ompl::mod::MoDOptimizationObjective(si, wd, wq, wc, MapType::CLiFFMap),
      max_vehicle_speed(maxvs),
      cliffmap(cliffmap),
      mahalanobis_distance_threshold(mahalanobis_distance_threshold),
      use_mixing_factor(use_mixing_factor) {
  description_ = "DownTheCLiFF Cost";
  // Setup a default cost-to-go heuristic:
  setCostToGoHeuristic(ompl::base::goalRegionCostToGo);
}

ompl::base::Cost ompl::mod::DTCOptimizationObjective::stateCost(
    const ompl::base::State *s) const {
  return ompl::base::Cost(0.0);
}

ompl::base::Cost ompl::mod::DTCOptimizationObjective::motionCost(
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

    double dot = cos(state_b[2] / 2.0) * cos(state_a[2] / 2.0) +
                 sin(state_b[2] / 2.0) * sin(state_a[2] / 2.0);

    // 4a. Compute Euclidean distance.
    double cost_d =
        si_->distance(intermediate_states[i], intermediate_states[i + 1]);

    // 4b. Compute the quaternion distance.
    double cost_q = (1.0 - dot * dot);

    double alpha = atan2(state_b[1] - state_a[1], state_b[0] - state_a[0]);

    double cost_c = 0.0;
    Eigen::Vector2d V;
    V[0] = alpha;
    V[1] = max_vehicle_speed;

    double x = state_b[0];
    double y = state_b[1];
    const cliffmap_ros::CLiFFMapLocation &cl = cliffmap(x, y);
    double trust = cl.p * cl.q;

    for (const auto &dist : cl.distributions) {
      Eigen::Matrix2d Sigma;
      std::array<double, 4> sigma_array = dist.getCovariance();
      Sigma(0, 0) = sigma_array[0];
      Sigma(0, 1) = sigma_array[1];
      Sigma(1, 0) = sigma_array[2];
      Sigma(1, 1) = sigma_array[3];
      Eigen::Vector2d myu;
      myu[0] = atan2(sin(dist.getMeanHeading()), cos(dist.getMeanHeading()));
      myu[1] = dist.getMeanSpeed();

      double inc_cost = 0.0;
      if (Sigma.determinant() < 1e-8 && Sigma.determinant() > -1e-8)
        inc_cost = mahalanobis_distance_threshold * trust;
      else {
        double mahalanobis =
            sqrt((V - myu).transpose() * Sigma.inverse() * (V - myu));
        if (mahalanobis > mahalanobis_distance_threshold)
          mahalanobis = mahalanobis_distance_threshold;

        inc_cost = mahalanobis * trust;
      }

      if (inc_cost < 0.0) {
        printf("WHAT THE HOLY?!");
      }

      if (std::isnan(inc_cost)) {
        inc_cost = mahalanobis_distance_threshold * trust;
      }

      if (use_mixing_factor) inc_cost = inc_cost * dist.getMixingFactor();

      cost_c += inc_cost;
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

ompl::base::Cost ompl::mod::DTCOptimizationObjective::motionCostHeuristic(
    const ompl::base::State *s1, const ompl::base::State *s2) const {
  return motionCost(s1, s2);
}
