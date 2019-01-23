#include "ompl_planners_ros/mc_state_validity_checker.hpp"
#include "ompl/mod/objectives/DTCOptimizationObjective.h"

#include <boost/math/constants/constants.hpp>

#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <mrpt/math/CPolygon.h>

#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>

namespace mm = mrpt::maps;
namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl_planners_ros {

struct PlannerParameters {
  double weight_d{1.0};
  double weight_q{1.0};
  double weight_c{0.0};
  double planning_time{5.0};
  double path_resolution{0.25};
  std::string cliffmap_filename;
};

struct VehicleParameters {
  mrpt::math::CPolygon footprint;
  double turning_radius{0.5};
  double max_vehicle_speed{1.0};
  double inflation_radius{0.25};
};

class MultipleCirclesReedsSheppCarPlanner {
  mm::COccupancyGridMap2D grid_map_;
  bool no_map_;

  PlannerParameters planner_params_;
  VehicleParameters vehicle_params_;

  std::string cliffmap_filename;

 public:
  boost::shared_ptr<og::SimpleSetup> ss;

  MultipleCirclesReedsSheppCarPlanner(
      const PlannerParameters &planner_params,
      const VehicleParameters &vehicle_params,
      const nav_msgs::OccupancyGridConstPtr &occ_map_ptr);

  ~MultipleCirclesReedsSheppCarPlanner() {}

  bool plan(const geometry_msgs::Pose &startState,
            const geometry_msgs::Pose &goalState,
            geometry_msgs::PoseArray *path);
};
}
