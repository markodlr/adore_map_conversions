/********************************************************************************
 * Copyright (c) 2025 Contributors to the Eclipse Foundation
 *
 * See the NOTICE file(s) distributed with this work for additional
 * information regarding copyright ownership.
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * https://www.eclipse.org/legal/epl-2.0
 *
 * SPDX-License-Identifier: EPL-2.0
 ********************************************************************************/

#include "adore_map_conversions.hpp"

namespace adore
{
namespace map

{
namespace conversions
{


// Convert MapPoint to ROS message
adore_ros2_msgs::msg::MapPoint
to_ros_msg( const MapPoint& cpp_point )
{
  adore_ros2_msgs::msg::MapPoint ros_point;
  ros_point.x         = cpp_point.x;
  ros_point.y         = cpp_point.y;
  ros_point.s         = cpp_point.s;
  ros_point.parent_id = cpp_point.parent_id;
  if( cpp_point.max_speed.has_value() )
    ros_point.max_speed = cpp_point.max_speed.value();
  else
    ros_point.max_speed = -1; // any negative values are interpreted as no optional set
  return ros_point;
}

// Convert ROS MapPoint message to C++ MapPoint
MapPoint
to_cpp_type( const adore_ros2_msgs::msg::MapPoint& ros_point )
{
  MapPoint cpp_point;
  cpp_point.x         = ros_point.x;
  cpp_point.y         = ros_point.y;
  cpp_point.s         = ros_point.s;
  cpp_point.parent_id = ros_point.parent_id;
  if( ros_point.max_speed >= 0 )
    cpp_point.max_speed = ros_point.max_speed;
  else
    cpp_point.max_speed = std::nullopt;
  return cpp_point;
}

// Convert Lane to ROS message
adore_ros2_msgs::msg::MapLane
to_ros_msg( const Lane& cpp_lane )
{
  adore_ros2_msgs::msg::MapLane ros_lane;
  for( const auto& point : cpp_lane.borders.inner.interpolated_points )
  {
    ros_lane.inner_points.push_back( to_ros_msg( point ) ); // Convert each MapPoint
  }
  for( const auto& point : cpp_lane.borders.outer.interpolated_points )
  {
    ros_lane.outer_points.push_back( to_ros_msg( point ) ); // Convert each MapPoint
  }
  for( const auto& point : cpp_lane.borders.center.interpolated_points )
  {
    ros_lane.center_points.push_back( to_ros_msg( point ) ); // Convert each MapPoint
  }
  ros_lane.type              = static_cast<uint8_t>( cpp_lane.type );
  ros_lane.material          = static_cast<uint8_t>( cpp_lane.material );
  ros_lane.id                = cpp_lane.id;
  ros_lane.road_id           = cpp_lane.road_id;
  ros_lane.speed_limit       = cpp_lane.speed_limit;
  ros_lane.left_of_reference = cpp_lane.left_of_reference;
  ros_lane.lateral_offset    = cpp_lane.lateral_offset;
  return ros_lane;
}

// Convert ROS Lane message to C++ Lane
Lane
to_cpp_type( const adore_ros2_msgs::msg::MapLane& ros_lane )
{
  Lane cpp_lane;
  for( const auto& point : ros_lane.inner_points )
  {
    cpp_lane.borders.inner.interpolated_points.push_back( to_cpp_type( point ) ); // Convert each MapPoint
  }
  for( const auto& point : ros_lane.outer_points )
  {
    cpp_lane.borders.outer.interpolated_points.push_back( to_cpp_type( point ) ); // Convert each MapPoint
  }
  for( const auto& point : ros_lane.center_points )
  {
    cpp_lane.borders.center.interpolated_points.push_back( to_cpp_type( point ) ); // Convert each MapPoint
  }
  cpp_lane.type              = static_cast<LaneType>( ros_lane.type );
  cpp_lane.material          = static_cast<LaneMaterial>( ros_lane.material );
  cpp_lane.id                = ros_lane.id;
  cpp_lane.road_id           = ros_lane.road_id;
  cpp_lane.speed_limit       = ros_lane.speed_limit;
  cpp_lane.left_of_reference = ros_lane.left_of_reference;
  cpp_lane.lateral_offset    = ros_lane.lateral_offset;
  return cpp_lane;
}

// Convert Connection to ROS message
adore_ros2_msgs::msg::MapConnection
to_ros_msg( const Connection& cpp_connection )
{
  adore_ros2_msgs::msg::MapConnection ros_connection;
  ros_connection.from_id         = cpp_connection.from_id;
  ros_connection.to_id           = cpp_connection.to_id;
  ros_connection.connection_type = static_cast<uint8_t>( cpp_connection.connection_type );
  ros_connection.weight          = cpp_connection.weight;
  return ros_connection;
}

// Convert ROS Connection message to C++ Connection
Connection
to_cpp_type( const adore_ros2_msgs::msg::MapConnection& ros_connection )
{
  Connection cpp_connection;
  cpp_connection.from_id         = ros_connection.from_id;
  cpp_connection.to_id           = ros_connection.to_id;
  cpp_connection.connection_type = static_cast<ConnectionType>( ros_connection.connection_type );
  cpp_connection.weight          = ros_connection.weight;
  return cpp_connection;
}

// Convert Road to ROS message
adore_ros2_msgs::msg::MapRoad
to_ros_msg( const Road& cpp_road )
{
  adore_ros2_msgs::msg::MapRoad ros_road;
  for( const auto& lane : cpp_road.lanes )
  {
    ros_road.lanes.push_back( to_ros_msg( *lane ) );
  }
  ros_road.id       = cpp_road.id;
  ros_road.name     = cpp_road.name;
  ros_road.one_way  = cpp_road.one_way;
  ros_road.category = static_cast<uint8_t>( cpp_road.category );
  return ros_road;
}

// Convert ROS Road message to C++ Road
Road
to_cpp_type( const adore_ros2_msgs::msg::MapRoad& ros_road )
{
  Road cpp_road;
  for( const auto& ros_lane : ros_road.lanes )
  {
    auto lane_ptr = std::make_shared<Lane>( to_cpp_type( ros_lane ) );
    cpp_road.lanes.insert( lane_ptr );
    cpp_road.lane_offset_to_lane[ros_lane.lateral_offset] = lane_ptr;
  }
  cpp_road.name     = ros_road.name;
  cpp_road.one_way  = ros_road.one_way;
  cpp_road.id       = ros_road.id;
  cpp_road.category = static_cast<RoadCategory>( ros_road.category );
  return cpp_road;
}

// Convert Map to ROS message
adore_ros2_msgs::msg::Map
to_ros_msg( const Map& cpp_map )
{
  adore_ros2_msgs::msg::Map ros_map;

  // Convert connections
  for( const auto& connection : cpp_map.lane_graph.all_connections )
  {
    ros_map.connections.push_back( to_ros_msg( connection ) );
  }

  // Convert roads
  for( const auto& [id, road] : cpp_map.roads )
  {
    adore_ros2_msgs::msg::MapRoad ros_road = to_ros_msg( road );
    ros_map.roads.push_back( ros_road );
  }

  ros_map.x_max = cpp_map.quadtree.boundary.x_max;
  ros_map.x_min = cpp_map.quadtree.boundary.x_min;
  ros_map.y_max = cpp_map.quadtree.boundary.y_max;
  ros_map.y_min = cpp_map.quadtree.boundary.y_min;

  ros_map.header.frame_id = "world";

  return ros_map;
}

// Convert ROS Map message to C++ Map
Map
to_cpp_type( const adore_ros2_msgs::msg::Map& ros_map )
{
  Map cpp_map;

  cpp_map.quadtree.boundary.x_max = ros_map.x_max;
  cpp_map.quadtree.boundary.x_min = ros_map.x_min;
  cpp_map.quadtree.boundary.y_min = ros_map.y_min;
  cpp_map.quadtree.boundary.y_max = ros_map.y_max;
  cpp_map.quadtree.capacity       = 10;

  // Convert connections
  for( const auto& ros_connection : ros_map.connections )
  {
    auto connection = to_cpp_type( ros_connection );
    cpp_map.lane_graph.add_connection( connection );
  }

  // Convert roads
  for( const auto& ros_road : ros_map.roads )
  {
    Road cpp_road              = to_cpp_type( ros_road );
    cpp_map.roads[cpp_road.id] = cpp_road;
    for( const auto& lane : cpp_road.lanes )
    {
      // add lane
      cpp_map.lanes[lane->id] = lane;
      // add points
      for( const auto& point : lane->borders.center.interpolated_points )
      {
        cpp_map.quadtree.insert( point );
      }
    }
  }

  return cpp_map;
}

RouteSection
to_cpp_type( const adore_ros2_msgs::msg::RouteSection& msg )
{
  RouteSection section;
  section.end_s   = msg.end_s;
  section.start_s = msg.start_s;
  section.route_s = msg.route_s;
  section.lane_id = msg.lane_id;
  return section;
}

adore_ros2_msgs::msg::RouteSection
to_ros_msg( const RouteSection& section )
{
  adore_ros2_msgs::msg::RouteSection msg;
  msg.end_s   = section.end_s;
  msg.start_s = section.start_s;
  msg.route_s = section.route_s;
  msg.lane_id = section.lane_id;
  return msg;
}

Route
to_cpp_type( const adore_ros2_msgs::msg::Route& msg )
{
  Route route;
  for( const auto& section : msg.sections )
  {
    auto                          cpp_section = to_cpp_type( section );
    std::shared_ptr<RouteSection> section_ptr = std::make_shared<RouteSection>( cpp_section );
    route.sections.push_back( section_ptr );
    route.lane_to_sections[section.lane_id] = section_ptr;
  }

  for( const auto& point_msg : msg.center_points )
  {
    // Convert from ROS to internal MapPoint
    auto mp = to_cpp_type( point_msg );

    // Find which RouteSection this point belongs to, via lane_id
    auto it = route.lane_to_sections.find( mp.parent_id );
    if( it == route.lane_to_sections.end() )
    {
      // std::cerr << "section not found when converting  " << mp.parent_id << std::endl;
      continue;
    }
    auto   section_ptr            = it->second; // shared_ptr<RouteSection>
    bool   forward                = ( section_ptr->end_s >= section_ptr->start_s );
    double local_s                = forward ? ( mp.s - section_ptr->start_s ) : ( section_ptr->start_s - mp.s );
    double route_s                = section_ptr->route_s + local_s;
    route.reference_line[route_s] = mp;
  }


  // Convert start and destination points
  route.start.x       = msg.start.x;
  route.start.y       = msg.start.y;
  route.destination.x = msg.goal.x;
  route.destination.y = msg.goal.y;
  route.map           = std::make_shared<Map>( to_cpp_type( msg.map ) );


  return route;
}

// Convert from Route (C++ struct) to Route.msg
adore_ros2_msgs::msg::Route
to_ros_msg( const Route& route )
{
  adore_ros2_msgs::msg::Route msg;

  if( route.map )
  {
    msg.map = to_ros_msg( *route.map );
  }

  // Convert lane IDs
  for( const auto& section : route.sections )
  {
    auto section_msg = to_ros_msg( *section );
    msg.sections.push_back( section_msg );
  }

  // Convert center points
  for( const auto& [s, map_point] : route.reference_line )
  {
    msg.center_points.push_back( to_ros_msg( map_point ) );
  }

  // Convert start and destination points
  msg.start.x = route.start.x;
  msg.start.y = route.start.y;
  msg.goal.x  = route.destination.x;
  msg.goal.y  = route.destination.y;

  msg.header.frame_id = "world";

  return msg;
}

// Convert from DrivableArea.msg to DrivableArea C++ type
adore::planner::DrivableArea
to_cpp_type( const adore_ros2_msgs::msg::DrivableArea& msg )
{
  using adore::map::MapPoint;
  using adore::planner::DrivableArea;

  DrivableArea cpp;

  // reference_line
  for( const auto& mp_msg : msg.reference_line )
  {
    MapPoint mp = to_cpp_type( mp_msg );
    cpp.reference_line.emplace( mp.s, mp ); // key by longitudinal s
  }

  // left_boundary
  for( const auto& mp_msg : msg.left_boundary )
  {
    MapPoint mp = to_cpp_type( mp_msg );
    cpp.left_boundary.emplace( mp.s, mp );
  }

  // right_boundary
  for( const auto& mp_msg : msg.right_boundary )
  {
    MapPoint mp = to_cpp_type( mp_msg );
    cpp.right_boundary.emplace( mp.s, mp );
  }

  return cpp;
}

// Convert from DrivableArea (C++ struct/class) to DrivableArea.msg
adore_ros2_msgs::msg::DrivableArea
to_ros_msg( const adore::planner::DrivableArea& area )
{
  adore_ros2_msgs::msg::DrivableArea msg;

  // reference_line
  const auto& ref = area.reference_line;
  msg.reference_line.reserve( ref.size() );
  for( const auto& kv : ref )
  {
    const auto& mp = kv.second;
    msg.reference_line.push_back( to_ros_msg( mp ) );
  }

  // left_boundary
  const auto& left = area.left_boundary;
  msg.left_boundary.reserve( left.size() );
  for( const auto& kv : left )
  {
    const auto& mp = kv.second;
    msg.left_boundary.push_back( to_ros_msg( mp ) );
  }

  // right_boundary
  const auto& right = area.right_boundary;
  msg.right_boundary.reserve( right.size() );
  for( const auto& kv : right )
  {
    const auto& mp = kv.second;
    msg.right_boundary.push_back( to_ros_msg( mp ) );
  }

  msg.header.frame_id = "world";
  return msg;
}

} // namespace conversions
} // namespace map
} // namespace adore