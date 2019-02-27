/*
 *   Copyright (c) Chittaranjan Srinivas Swaminathan
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

#include <ompl/base/PlannerData.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

namespace ompl_planners_ros {

class Visualization {
 protected:
  ros::NodeHandle nh_;

  ros::Publisher markers_pub_;

  geometry_msgs::Point stateToPointMsg(const ompl::base::State* state);

  geometry_msgs::Point stateToPointMsg(
      int vertex_id, const ompl::base::PlannerData& planner_data);

  bool interpolateLine(const geometry_msgs::Point& p1,
                       const geometry_msgs::Point& p2,
                       visualization_msgs::Marker* marker,
                       const std_msgs::ColorRGBA color);

 public:
  Visualization() {
    markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "visualization_marker", 10);
    ROS_INFO_STREAM("\x1b[34m[Visualization]: READY!");
  }

  virtual ~Visualization() {}

  void publishPlanningGraph(const ompl::base::PlannerData& pData);
};

} /* namespace ompl_planners_ros */
