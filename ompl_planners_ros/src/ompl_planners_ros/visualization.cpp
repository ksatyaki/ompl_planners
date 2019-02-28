/*
 * visualization.cpp
 *
 *  Created on: Feb 26, 2019
 *      Author: ksatyaki
 */

#include <ompl_planners_ros/visualization.hpp>

namespace ompl_planners_ros {

void Visualization::publishPlanningGraph(const ompl::base::PlannerData &pData) {
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on
  // these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique
  // ID TODO: Make this customizable?
  marker.ns = "planning_graph";

  // Set the marker type.
  marker.type = visualization_msgs::Marker::LINE_LIST;

  // Set the marker action.  Options are ADD and DELETE
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = 0;
  marker.lifetime = ros::Duration(60.0);

  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.005;

  // TODO: Customizable?
  marker.color.r = 0.2;
  marker.color.g = 0.6;
  marker.color.b = 0.2;
  marker.color.a = 1.0;

  geometry_msgs::Point this_vertex;
  geometry_msgs::Point next_vertex;

  // Loop through all verticies
  for (std::size_t vertex_id = 0; vertex_id < pData.numVertices();
       ++vertex_id) {
    this_vertex = this->stateToPointMsg(vertex_id, pData);

    // Get the out edges from the current vertex
    std::vector<unsigned int> edge_list;
    pData.getEdges(vertex_id, edge_list);

    // Now loop through each edge
    for (std::vector<unsigned int>::const_iterator edge_it = edge_list.begin();
         edge_it != edge_list.end(); ++edge_it) {
      // Convert vertex id to next coordinates
      next_vertex = stateToPointMsg(*edge_it, pData);

      interpolateLine(this_vertex, next_vertex, &marker, marker.color);
    }
  }

  // Send to Rviz
  // Maybe this node will publish more than one tree?
  // Make the set of markers global
  visualization_msgs::MarkerArray markers;
  markers.markers.push_back(marker);
  markers_pub_.publish(markers);
}

geometry_msgs::Point Visualization::stateToPointMsg(
    int vertex_id, const ompl::base::PlannerData &planner_data) {
  const ompl::base::PlannerDataVertex *vertex =
      &planner_data.getVertex(vertex_id);
  return stateToPointMsg(vertex->getState());
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

bool Visualization::interpolateLine(const geometry_msgs::Point &p1,
                                    const geometry_msgs::Point &p2,
                                    visualization_msgs::Marker *marker,
                                    const std_msgs::ColorRGBA color) {
  // Copy to non-const
  geometry_msgs::Point point_a = p1;
  geometry_msgs::Point point_b = p2;

  // Get the heights
  point_a.z = 0.0;
  point_b.z = 0.0;

  // Switch the coordinates such that x1 < x2
  if (point_a.x > point_b.x) {
    // Swap the coordinates
    geometry_msgs::Point point_temp = point_a;
    point_a = point_b;
    point_b = point_temp;
  }

  // Show start and end point
  // Interpolate the line

  // Calculate slope between the lines
  double m = (point_b.y - point_a.y) / (point_b.x - point_a.x);

  // Calculate the y-intercep
  double b = point_a.y - m * point_a.x;

  // Define the interpolation interval
  double interval = 0.01;

  // Make new locations
  geometry_msgs::Point temp_a = point_a;  // remember the last point
  geometry_msgs::Point temp_b = point_a;  // move along this point

  // Loop through the line adding segements along the cost map
  for (temp_b.x = point_a.x + interval; temp_b.x <= point_b.x;
       temp_b.x += interval) {
    // publishSphere(temp_b, color2);

    // Find the y coordinate
    temp_b.y = m * temp_b.x + b;

    // Add the new heights
    temp_a.z = 0.0;
    temp_b.z = 0.0;

    // Add the point pair to the line message
    marker->points.push_back(temp_a);
    marker->points.push_back(temp_b);
    // Add colors
    marker->colors.push_back(color);
    marker->colors.push_back(color);

    // Remember the last coordiante for next iteration
    temp_a = temp_b;
  }

  // Finish the line for non-even interval lengths
  marker->points.push_back(temp_a);
  marker->points.push_back(point_b);
  // Add colors
  marker->colors.push_back(color);
  marker->colors.push_back(color);

  return true;
}

} /* namespace ompl_planners_ros */
