/*
 *   Copyright (c) Chittaranjan Srinivas Swaminathan
 *   Copyright (c) Federico Pecora
 *   This file is part of ompl_planners_ros.
 *
 *   ompl_planners_ros is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   ompl_planners_ros is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with ompl_planners_ros.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <mrpt/maps/COccupancyGridMap2D.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

namespace mm = mrpt::maps;
namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl_planners_ros {

class MultipleCircleStateValidityChecker: public ob::StateValidityChecker {
public:
  mm::COccupancyGridMap2D &grid_map_;
  double radius_;
  std::vector<double> x_coords_;
  std::vector<double> y_coords_;
  bool no_map_;

  MultipleCircleStateValidityChecker(const ob::SpaceInformationPtr &si,
      mm::COccupancyGridMap2D &gridMap, double _radius,
      std::vector<double> &xCoords, std::vector<double> &yCoords) :
      ob::StateValidityChecker(si), grid_map_(gridMap), radius_(_radius), x_coords_(
          xCoords), y_coords_(yCoords) {
    std::cout << "State validity checker initialized.";
    no_map_ = false;
  }
  // MultipleCircleStateValidityChecker(const ob::SpaceInformationPtr &si)
  //     : ob::StateValidityChecker(si) {
  //   no_map_ = true;
  //   std::cout << "Using empty map for validity checking" << std::endl;
  // }
  virtual ~MultipleCircleStateValidityChecker() {
  }

  virtual bool isValid(const ob::State *state) const;
};

}
