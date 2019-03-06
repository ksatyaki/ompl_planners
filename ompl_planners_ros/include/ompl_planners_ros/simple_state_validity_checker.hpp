
/*
 *   Copyright (c) Chittaranjan Srinivas Swaminathan
 *   Copyright (c) Federico Pecora
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

#include <mrpt/maps/COccupancyGridMap2D.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

using namespace mrpt::maps;
namespace ob = ompl::base;
namespace og = ompl::geometric;

class SimpleStateValidityChecker: public ob::StateValidityChecker {
public:
  COccupancyGridMap2D gridmap;
  float radius;
  SimpleStateValidityChecker(const ob::SpaceInformationPtr &si,
      const char *mapFilename, float mapResolution, float _radius) :
      ob::StateValidityChecker(si) {
    radius = _radius;
    gridmap.loadFromBitmapFile(mapFilename, mapResolution, 0.0f, 0.0f);
    std::cout << "Loaded map " << mapFilename << std::endl;
  }

  virtual bool isValid(const ob::State *state) const;
};

