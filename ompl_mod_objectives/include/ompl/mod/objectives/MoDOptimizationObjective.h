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

enum class MapType {
  CLiFFMap = 0,
  STeFMap = 1,
  GMMTMap = 2,
  WHyTeMap = 3,
  NOTSET = 101
};

struct Cost {
  /// The last computed distance cost
  double cost_d_{0.0};

  /// The last computed quaternion distance cost
  double cost_q_{0.0};

  /// The last computed MoD cost
  double cost_c_{0.0};

  inline Cost operator+(Cost b) {
    Cost result;
    result.cost_c_ = this->cost_c_ + b.cost_c_;
    result.cost_d_ = this->cost_d_ + b.cost_d_;
    result.cost_q_ = this->cost_q_ + b.cost_q_;
    return result;
  }
};

class MoDOptimizationObjective : public ompl::base::OptimizationObjective {
protected:
  /// The weight associated with Euclidean distance cost.
  double weight_d_;

  /// The weight associated with quaternion distance cost.
  double weight_q_;

  /// The weight associated with Down-The-CLiFF cost.
  double weight_c_;

  mutable Cost last_cost_;

  MapType map_type_{MapType::NOTSET};

  inline MoDOptimizationObjective(const ompl::base::SpaceInformationPtr &si,
                                  double weight_d, double weight_q,
                                  double weight_c, MapType map_type)
      : ompl::base::OptimizationObjective(si), weight_d_(weight_d),
        weight_q_(weight_q), weight_c_(weight_c), map_type_(map_type) {}

public:
  inline double getLastCostD() const { return last_cost_.cost_d_; }
  inline double getLastCostQ() const { return last_cost_.cost_q_; }
  inline double getLastCostC() const { return last_cost_.cost_c_; }
  inline Cost getLastCost() const { return last_cost_; }

  inline std::string getMapTypeStr() const {
    switch (map_type_) {
    case MapType::STeFMap:
      return "STeF-map";
    case MapType::GMMTMap:
      return "GMMT-map";
    case MapType::CLiFFMap:
      return "CLiFF-map";
    case MapType::WHyTeMap:
      return "WHyTe-map";
    default:
      return "Not set.";
    }
  }
  inline MapType getMapType() const { return map_type_; }
};
} // namespace mod
}