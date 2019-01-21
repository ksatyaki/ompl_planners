#pragma once

#include <mrpt/maps/COccupancyGridMap2D.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

namespace mm = mrpt::maps;
namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl_planners_ros {

class MultipleCircleStateValidityChecker : public ob::StateValidityChecker {
public:
  mm::COccupancyGridMap2D &grid_map_;
  double radius_;
  std::vector<double> &x_coords_;
  std::vector<double> &y_coords_;
  bool no_map_;

  MultipleCircleStateValidityChecker(const ob::SpaceInformationPtr &si,
                                     mm::COccupancyGridMap2D &gridMap,
                                     double _radius,
                                     std::vector<double> &xCoords,
                                     std::vector<double> &yCoords)
      : ob::StateValidityChecker(si), grid_map_(gridMap), radius_(_radius),
        x_coords_(xCoords), y_coords_(yCoords) {
    std::cout << "State validity checker initialized.";
    no_map_ = false;
  }
  // MultipleCircleStateValidityChecker(const ob::SpaceInformationPtr &si)
  //     : ob::StateValidityChecker(si) {
  //   no_map_ = true;
  //   std::cout << "Using empty map for validity checking" << std::endl;
  // }

  virtual bool isValid(const ob::State *state) const;
};
  
}
