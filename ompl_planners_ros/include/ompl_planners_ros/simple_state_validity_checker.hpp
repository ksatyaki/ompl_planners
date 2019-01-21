/**
 * Copyright 2017 Federico Pecora
 */
#pragma once

#include <mrpt/maps/COccupancyGridMap2D.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

using namespace mrpt::maps;
namespace ob = ompl::base;
namespace og = ompl::geometric;

class SimpleStateValidityChecker : public ob::StateValidityChecker {
public:
  COccupancyGridMap2D gridmap;
  float radius;
  SimpleStateValidityChecker(const ob::SpaceInformationPtr &si,
                             const char *mapFilename, float mapResolution,
                             float _radius)
      : ob::StateValidityChecker(si) {
    radius = _radius;
    gridmap.loadFromBitmapFile(mapFilename, mapResolution, 0.0f, 0.0f);
    std::cout << "Loaded map " << mapFilename << std::endl;
  }

  virtual bool isValid(const ob::State *state) const;
};

