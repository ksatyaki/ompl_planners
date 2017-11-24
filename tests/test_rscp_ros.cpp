#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include "srscp/mc_reeds_shepp_car_planner.hpp"
#include <mrpt/math/CPolygon.h>

int main(int argn, char *args[]) {

  ros::init(argn, args, "rscp_node");
  ros::NodeHandle nh;

  ros::Publisher path_pub = nh.advertise<geometry_msgs::PoseArray>("/path", 1);

  ros::ServiceClient map_client =
      nh.serviceClient<nav_msgs::GetMap>("/map_server/static_map");
  map_client.waitForExistence();
  nav_msgs::GetMap get_map_;
  if (!map_client.call(get_map_)) {
    ROS_FATAL("Failed to call the map server for map!");
  }
  ROS_INFO("Got a map!");

  mrpt::math::CPolygon footprint;
  footprint.AddVertex(0.5, 0.5);
  footprint.AddVertex(0.5, -0.5);
  footprint.AddVertex(-0.5, 0.5);
  footprint.AddVertex(-0.5, -0.5);

  // srscp::MultipleCirclesReedsSheppCarPlanner planner(args[1],
  // std::atof(args[2]), 0.25, footprint);
  nav_msgs::OccupancyGridPtr occ_map =
      nav_msgs::OccupancyGridPtr(new nav_msgs::OccupancyGrid);
  *occ_map = get_map_.response.map;
  srscp::MultipleCirclesReedsSheppCarPlanner planner(occ_map, 0.25, footprint);

  std::vector<srscp::State> path;
  planner.plan({2.0, 2.0, 0.0}, {6.0, 2.0, 0.0}, 0.4, 2.0, path);

  geometry_msgs::PoseArray poses;
  poses.header.frame_id = "map";
  poses.header.stamp = ros::Time::now();
  for (auto p : path) {
    geometry_msgs::Pose pose;
    pose.position.x = p.x;
    pose.position.y = p.y;
    pose.orientation.z = sin(p.theta / 2);
    pose.orientation.w = cos(p.theta / 2);
    poses.poses.push_back(pose);
  }

  path_pub.publish(poses);

  while (ros::ok()) {
    ros::spinOnce();
    path_pub.publish(poses);
  }

  return 0;
}
