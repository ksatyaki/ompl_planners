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

#pragma once

#include <mrpt/math/CPolygon.h>

#include "ompl_planners_ros/mc_state_validity_checker.hpp"
#include "ompl_planners_ros/visualization.hpp"

#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/mod/objectives/MoDOptimizationObjective.h>
#include "ompl_planners_ros/car_state_space.hpp"

namespace mm = mrpt::maps;
namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl_planners_ros {

/**
 * This struct defines configurable planner parameters.
 */
struct PlannerParameters {

  /**
   * Should the planner publish visualization markers to RViz?
   */
  bool publish_viz_markers{false};

  /**
   * Weight associated with the Euclidean distance between two state positions.
   */
  double weight_d{1.0};

  /**
   * Weight associated with the quaternion distance between two state
   * orientations.
   */
  double weight_q{1.0};

  /**
   * Weight associated with the cost due to CLiFF-map or Dynamics Map or MoD
   */
  double weight_c{0.0};

  /**
   * Maximum planning time.
   */
  double planning_time{5.0};

  /**
   * Resolution that determines how many path points are returned when planning
   * is completed.
   */
  double path_resolution{0.25};
};

/**
 * This struct defines the configurable vehicle parameters for the Reeds-Shepp
 * car.
 */
struct VehicleParameters {
  /**
   * Vehicle footprint.
   */
  mrpt::math::CPolygon footprint;

  /**
   * Turning radius of the Reeds Shepp vehicle.
   */
  double turning_radius{0.5};

  /**
   * Maximum vehicle speed used in the DownTheCLiFF planning to evaluate
   * Down-The-CLiFF costs.
   */
  double max_vehicle_speed{1.0};

  /**
   * Inflation radius used in the multiple-circle collission checking procedure.
   */
  double inflation_radius{0.25};
};

/**
 * The planner class that uses the multiple circle collision checking procedure.
 */
class MultipleCirclesReedsSheppCarPlanner {

  ompl_planners_ros::Visualization viz;

  /**
   * Occupancy gridmap used for collision checking.
   */
  mm::COccupancyGridMap2D grid_map_;

  /**
   * Is there a map?
   */
  bool no_map_;

  /**
   * Set of planner parameters.
   */
  PlannerParameters planner_params_;

  /**
   * Set of vehicle parameters.
   */
  VehicleParameters vehicle_params_;

  std::vector<ompl::mod::Cost> solution_cost;

public:
  /**
   * OMPL simple setup.
   */
  boost::shared_ptr<og::SimpleSetup> ss;

  inline std::vector<ompl::mod::Cost> getSolutionCost() {
    return solution_cost;
  }

  /**
   * Constructor
   * @param planner_params The planning parameters
   * @param vehicle_params The vehicle parameters
   * @param occ_map_ptr A ConstPtr to the occupancy map
   */
  MultipleCirclesReedsSheppCarPlanner(
      const PlannerParameters &planner_params,
      const VehicleParameters &vehicle_params,
      const nav_msgs::OccupancyGridConstPtr &occ_map_ptr);

  /**
   * Destructor
   */
  ~MultipleCirclesReedsSheppCarPlanner() {}

  /**
   * The planning method.
   * @param startState The start state for planning.
   * @param goalState The goal state for planning.
   * @param path A pointer to where the solution should be stored.
   * @return True if success. False otherwise.
   */
  bool plan(const geometry_msgs::Pose2D &startState,
            const geometry_msgs::Pose2D &goalState,
            std::vector<geometry_msgs::Pose2D> *path);
};
} // namespace ompl_planners_ros
