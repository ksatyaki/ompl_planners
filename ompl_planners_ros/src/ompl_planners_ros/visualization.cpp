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

#include <ompl_planners_ros/visualization.hpp>

namespace ompl_planners_ros {

void Visualization::publishSolutionPath(ompl::geometric::PathGeometric &sPath) {
  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker m;
  setMarkerCommonProperties(&m);
  m.color.r = 0.2;
  m.color.g = 0.9;
  m.color.b = 0.0;
  m.scale.x = 0.02;
  for (const auto &state : sPath.getStates()) {
    m.points.push_back(stateToPointMsg(state));
    m.colors.push_back(m.color);
  }
  markers.markers.push_back(m);
  markers_pub_.publish(markers);
}

void Visualization::publishPlanningGraph(const ompl::base::PlannerData &pData) {
  visualization_msgs::MarkerArray markers;

  int index = 0;
  // Loop through all verticies
  for (std::size_t vertex_id = 0; vertex_id < pData.numVertices();
       ++vertex_id) {
    // Get the out edges from the current vertex
    std::vector<unsigned int> edge_list;
    pData.getEdges(vertex_id, edge_list);

    // Now loop through each edge
    for (std::vector<unsigned int>::const_iterator edge_it = edge_list.begin();
         edge_it != edge_list.end(); ++edge_it) {
      ompl::geometric::PathGeometric segment(
          pData.getSpaceInformation(), pData.getVertex(vertex_id).getState(),
          pData.getVertex(*edge_it).getState());
      segment.interpolate(segment.length() / 0.05);

      visualization_msgs::Marker marker;
      setMarkerCommonProperties(&marker);
      marker.id = index++;
      for (const auto &state : segment.getStates()) {
        marker.points.push_back(stateToPointMsg(state));
        marker.colors.push_back(marker.color);
      }
      markers.markers.push_back(marker);
    }
  }
  markers_pub_.publish(markers);
}

void Visualization::setMarkerCommonProperties(
    visualization_msgs::Marker *marker) {

  marker->header.frame_id = "map";
  marker->header.stamp = ros::Time::now();
  marker->ns = "planning_graph";
  marker->type = visualization_msgs::Marker::LINE_STRIP;
  marker->action = visualization_msgs::Marker::ADD;
  marker->lifetime = ros::Duration();

  marker->pose.position.x = 0.0;
  marker->pose.position.y = 0.0;
  marker->pose.position.z = 0.0;

  marker->pose.orientation.x = 0.0;
  marker->pose.orientation.y = 0.0;
  marker->pose.orientation.z = 0.0;
  marker->pose.orientation.w = 1.0;

  marker->scale.x = 0.01;
  marker->scale.y = 0.01;

  // TODO: Customizable?
  marker->color.r = 0.3;
  marker->color.g = 0.3;
  marker->color.b = 0.6;
  marker->color.a = 1.0;
}

geometry_msgs::Point Visualization::stateToPointMsg(
    const ompl::base::State *state) {
  if (!state) {
    ROS_FATAL("No state found for a vertex");
    exit(1);
  }

  // Convert to RealVectorStateSpace
  auto real_state = state->as<ompl::base::ReedsSheppStateSpace::StateType>();

  geometry_msgs::Point temp_point;
  // Create point
  temp_point.x = real_state->getX();
  temp_point.y = real_state->getY();
  temp_point.z = 0.0;
  return temp_point;
}

} /* namespace ompl_planners_ros */
