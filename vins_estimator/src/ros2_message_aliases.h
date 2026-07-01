#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/channel_float32.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace std_msgs
{
using Bool = msg::Bool;
using ColorRGBA = msg::ColorRGBA;
using Float32 = msg::Float32;
using Header = msg::Header;
}  // namespace std_msgs

namespace sensor_msgs
{
using ChannelFloat32 = msg::ChannelFloat32;
using Image = msg::Image;
using Imu = msg::Imu;
using PointCloud = msg::PointCloud;
}  // namespace sensor_msgs

namespace geometry_msgs
{
using Point = msg::Point;
using Point32 = msg::Point32;
using PointStamped = msg::PointStamped;
using PoseStamped = msg::PoseStamped;
}  // namespace geometry_msgs

namespace nav_msgs
{
using Odometry = msg::Odometry;
using Path = msg::Path;
}  // namespace nav_msgs

namespace visualization_msgs
{
using Marker = msg::Marker;
using MarkerArray = msg::MarkerArray;
}  // namespace visualization_msgs
