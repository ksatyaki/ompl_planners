#include "ompl/mod/objectives/DTCOptimizationObjective.h"
#include "ompl_planners_ros/mc_state_validity_checker.hpp"

#include <boost/math/constants/constants.hpp>

#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <mrpt/math/CPolygon.h>

#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>

namespace mm = mrpt::maps;
namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl_planners_ros {

/**
 * This struct defines configurable planner parameters.
 */
struct PlannerParameters {
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

  /**
   * File-name of the CLiFF-map xml.
   */
  std::string cliffmap_filename;
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

 public:

  /**
   * OMPL simple setup.
   */
  boost::shared_ptr<og::SimpleSetup> ss;

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
  bool plan(const geometry_msgs::Pose &startState,
            const geometry_msgs::Pose &goalState,
            geometry_msgs::PoseArray *path);
};
}
