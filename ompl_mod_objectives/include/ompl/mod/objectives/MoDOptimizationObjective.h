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
 *   along with ompl_mod_objectives. If not, see <https://www.gnu.org/licenses/>
 */

#pragma once

#include <ompl/base/OptimizationObjective.h>

namespace ompl {
namespace mod {

enum class MapType { STeFMap = 0, GMMTMap = 1, CLiFFMap = 2, NOTSET = 101};

class MoDOptimizationObjective : public ompl::base::OptimizationObjective {
protected:
  /// The weight associated with Euclidean distance cost.
  double weight_d_;

  /// The weight associated with quaternion distance cost.
  double weight_q_;

  /// The weight associated with Down-The-CLiFF cost.
  double weight_c_;

  MapType map_type_{MapType::NOTSET};

  inline MoDOptimizationObjective(const ompl::base::SpaceInformationPtr &si,
                           double weight_d, double weight_q, double weight_c,
                           MapType map_type)
      : ompl::base::OptimizationObjective(si), weight_d_(weight_d),
        weight_q_(weight_q), weight_c_(weight_c), map_type_(map_type) {}

public:
  inline std::string getMapTypeStr() const {
    switch(map_type_) {
    case MapType::STeFMap:
      return "STeF-map";
    case MapType::GMMTMap:
      return "GMMT-map";
    case MapType::CLiFFMap:
      return "CLiFF-map";
    default:
      return "Not set.";
    }
  }
  inline MapType getMapType() const {
    return map_type_;
  }
};
} // namespace mod
}