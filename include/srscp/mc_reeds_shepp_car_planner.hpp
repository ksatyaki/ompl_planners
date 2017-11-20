#include "srscp/mc_state_validity_checker.hpp"
#include <boost/math/constants/constants.hpp>

#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/sst/SST.h>

#include <mrpt/math/CPolygon.h>

namespace mm = mrpt::maps;
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

namespace srscp {

struct State {
  double x;
  double y;
  double theta;
};

class MultipleCirclesReedsSheppCarPlanner {
  mm::COccupancyGridMap2D grid_map_;
  std::vector<double> x_coords_;
  std::vector<double> y_coords_;
  bool no_map_;
  double robot_radius_;

public:
  MultipleCirclesReedsSheppCarPlanner(const char *mapFileName,
                                      double mapResolution, double robotRadius,
                                      const mrpt::math::CPolygon &footprint);
  ~MultipleCirclesReedsSheppCarPlanner() {}

  bool plan(const State &startState, const State &goalState,
            double distanceBetweenPathPoints, double turningRadius,
            std::vector<State> &path);
};
}
