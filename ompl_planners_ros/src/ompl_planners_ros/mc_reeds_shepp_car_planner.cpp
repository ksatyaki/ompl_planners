/*
 *   Copyright (c) Chittaranjan Srinivas Swaminathan
 *   This file is part of ompl_planners_ros.
 *
 *   ompl_planners_ros is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   ompl_planners_ros is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with ompl_planners_ros.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <ompl_planners_ros/mc_reeds_shepp_car_planner.hpp>

namespace ompl_planners_ros {

MultipleCirclesReedsSheppCarPlanner::MultipleCirclesReedsSheppCarPlanner(
    const PlannerParameters &planner_params,
    const VehicleParameters &vehicle_params,
    const nav_msgs::OccupancyGridConstPtr &occ_map_ptr)
    : no_map_(false), vehicle_params_(vehicle_params),
      planner_params_(planner_params) {
  grid_map_.setSize(
      occ_map_ptr->info.origin.position.x,
      occ_map_ptr->info.origin.position.x +
          (occ_map_ptr->info.width * occ_map_ptr->info.resolution),
      occ_map_ptr->info.origin.position.y,
      occ_map_ptr->info.origin.position.y +
          (occ_map_ptr->info.height * occ_map_ptr->info.resolution),
      occ_map_ptr->info.resolution);

  for (int h = 0; h < occ_map_ptr->info.height; h++) {
    for (int w = 0; w < occ_map_ptr->info.width; w++) {
      float value = -1.0f;
      const int8_t &occ_map_value =
          occ_map_ptr->data[w + h * occ_map_ptr->info.width];
      if (occ_map_value <= 100) {
        value = 1.0f - (float)occ_map_value / 100.0f;
      } else if (occ_map_value == -1) {
        value = -1.0f;
      } else {
        std::cerr << "[ERROR]: Converting nav_msgs::OccupancyGrid to "
                     "mrpt::maps::COccupancyGridMap2D. Saw an unknown value in "
                     "data: "
                  << occ_map_value << "\n";
      }
      grid_map_.setCell(w, h, value);
    }
  }

  // ************************* //
  // STATE SPACE SETUP         //
  // ************************* //
  ob::StateSpacePtr space(
      new ob::CarStateSpace(vehicle_params_.turning_radius));
  ss = boost::make_shared<og::SimpleSetup>(space);

  ob::RealVectorBounds bounds(2);
  if (!no_map_) {
    bounds.low[0] = grid_map_.getXMin();
    bounds.low[1] = grid_map_.getYMin();
    bounds.high[0] = grid_map_.getXMax();
    bounds.high[1] = grid_map_.getYMax();
  } else {
    bounds.low[0] = -10000;
    bounds.low[1] = -10000;
    bounds.high[0] = 10000;
    bounds.high[1] = 10000;
  }
  space->as<ob::SE2StateSpace>()->setBounds(bounds);

  ob::SpaceInformationPtr si(ss->getSpaceInformation());

  // ************************* //
  // PLANNER                   //
  // ************************* //
  auto planner = std::make_shared<og::RRTstar>(si);
  planner->setKNearest(false);
  planner->setRange(space->getMaximumExtent());

  // ************************* //
  // STATE VALIDITY CHECKER    //
  // ************************* //

  // Get the vertices as a vector of Xs and Ys.
  std::vector<double> x_coords_, y_coords_;
  vehicle_params_.footprint.getAllVertices(x_coords_, y_coords_);

  si->setStateValidityChecker(ob::StateValidityCheckerPtr(
      new MultipleCircleStateValidityChecker(si, grid_map_,
                                             vehicle_params_.inflation_radius,
                                             x_coords_, y_coords_)));

  ss->getSpaceInformation()->setStateValidityCheckingResolution(
      this->planner_params_.path_resolution / space->getMaximumExtent());

  ss->setPlanner(planner);
  ss->setup();
  ss->print();

  ROS_INFO_STREAM(
      "State validity checking resolution: "
      << ss->getSpaceInformation()->getStateValidityCheckingResolution());
  ROS_INFO_STREAM("Longest valid segment length is "
                  << ss->getStateSpace()->getLongestValidSegmentLength());
  ROS_INFO_STREAM(
      "Planner range is: "
      << std::dynamic_pointer_cast<og::RRTstar>(ss->getPlanner())->getRange());
  ROS_INFO_STREAM("MCRSCP is ready to take goals.");
}

boost::shared_ptr<og::SimpleSetup>
MultipleCirclesReedsSheppCarPlanner::generateSimpleSetup(
    const PlannerParameters &planner_params,
    const VehicleParameters &vehicle_params,
    const nav_msgs::OccupancyGridConstPtr &occ_map_ptr) {
  mm::COccupancyGridMap2D grid_map_temp;
  grid_map_temp.setSize(
      occ_map_ptr->info.origin.position.x,
      occ_map_ptr->info.origin.position.x +
          (occ_map_ptr->info.width * occ_map_ptr->info.resolution),
      occ_map_ptr->info.origin.position.y,
      occ_map_ptr->info.origin.position.y +
          (occ_map_ptr->info.height * occ_map_ptr->info.resolution),
      occ_map_ptr->info.resolution);

  for (int h = 0; h < occ_map_ptr->info.height; h++) {
    for (int w = 0; w < occ_map_ptr->info.width; w++) {
      float value = -1.0f;
      const int8_t &occ_map_value =
          occ_map_ptr->data[w + h * occ_map_ptr->info.width];
      if (occ_map_value <= 100) {
        value = 1.0f - (float)occ_map_value / 100.0f;
      } else if (occ_map_value == -1) {
        value = -1.0f;
      } else {
        std::cerr << "[ERROR]: Converting nav_msgs::OccupancyGrid to "
                     "mrpt::maps::COccupancyGridMap2D. Saw an unknown value in "
                     "data: "
                  << occ_map_value << "\n";
      }
      grid_map_temp.setCell(w, h, value);
    }
  }

  // ************************* //
  // STATE SPACE SETUP         //
  // ************************* //
  ob::StateSpacePtr space(new ob::CarStateSpace(vehicle_params.turning_radius));
  boost::shared_ptr<og::SimpleSetup> ss_t =
      boost::make_shared<og::SimpleSetup>(space);

  ob::RealVectorBounds bounds(2);
  bounds.low[0] = grid_map_temp.getXMin();
  bounds.low[1] = grid_map_temp.getYMin();
  bounds.high[0] = grid_map_temp.getXMax();
  bounds.high[1] = grid_map_temp.getYMax();

  space->as<ob::SE2StateSpace>()->setBounds(bounds);

  ob::SpaceInformationPtr si(ss_t->getSpaceInformation());

  // ************************* //
  // STATE VALIDITY CHECKER    //
  // ************************* //

  // Get the vertices as a vector of Xs and Ys.
  std::vector<double> x_coords_, y_coords_;
  vehicle_params.footprint.getAllVertices(x_coords_, y_coords_);

  si->setStateValidityChecker(
      ob::StateValidityCheckerPtr(new MultipleCircleStateValidityChecker(
          si, grid_map_temp, vehicle_params.inflation_radius, x_coords_,
          y_coords_)));

  ss_t->getSpaceInformation()->setStateValidityCheckingResolution(
      planner_params.path_resolution / space->getMaximumExtent());

  return ss_t;
}

bool MultipleCirclesReedsSheppCarPlanner::plan(
    const geometry_msgs::Pose2D &startState,
    const geometry_msgs::Pose2D &goalState,
    std::vector<geometry_msgs::Pose2D> *path) {
  ss->clearStartStates();
  ss->clear();

  double pLen = 0.0;

  ob::ScopedState<> start(ss->getSpaceInformation()->getStateSpace()),
      goal(ss->getSpaceInformation()->getStateSpace());

  // set the start and goal states
  start[0] = startState.x;
  start[1] = startState.y;
  start[2] = startState.theta;

  goal[0] = goalState.x;
  goal[1] = goalState.y;
  goal[2] = goalState.theta;
  ss->setStartAndGoalStates(start, goal);

  ROS_INFO_STREAM(std::fixed << std::setprecision(2) << "Start state: \x1b[34m("
                             << start[0] << ", " << start[1] << ", " << start[2]
                             << ")");
  ROS_INFO_STREAM(std::fixed << std::setprecision(2) << "Goal  state: \x1b[34m("
                             << goal[0] << ", " << goal[1] << ", " << goal[2]
                             << ")");

  ob::PlannerStatus solved =
      ss->getPlanner()->solve(planner_params_.planning_time);

  if (solved) {
    // ss->simplifySolution();
    og::PathGeometric pth = ss->getSolutionPath();
    pth.interpolate();

    std::vector<ob::State *> states = pth.getStates();
    std::vector<double> reals;

    path->clear();
    path->resize(states.size());

    for (unsigned i = 0; i < states.size(); i++) {
      ss->getStateSpace()->copyToReals(reals, states[i]);
      (*path)[i].x = reals[0];
      (*path)[i].y = reals[1];
      (*path)[i].theta = reals[2];
    }

    // Save last solution cost path-point-wise
    auto space_info = ss->getSpaceInformation();
    solution_cost.clear();

    for (size_t i = 0; i < (states.size() - 1); i++) {
      auto this_cost =
          std::dynamic_pointer_cast<ompl::mod::MoDOptimizationObjective>(
              ss->getOptimizationObjective())
              ->motionCost(states[i], states[i + 1]);
      solution_cost.push_back(
          std::dynamic_pointer_cast<ompl::mod::MoDOptimizationObjective>(
              ss->getOptimizationObjective())
              ->getLastCost());
    }
    ROS_INFO("Solution cost size. %ld", solution_cost.size());
    return 1;
  }
  ROS_WARN_STREAM("\x1b[93mNo solution found");
  return 0;
}
} // namespace ompl_planners_ros
