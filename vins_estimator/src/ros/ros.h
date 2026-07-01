#pragma once

#include <cassert>
#include <cstdlib>
#include <sstream>

#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <rcutils/logging_macros.h>

namespace ros
{
inline builtin_interfaces::msg::Time Time(double seconds)
{
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(seconds);
    stamp.nanosec = static_cast<uint32_t>((seconds - stamp.sec) * 1e9);
    return stamp;
}

inline builtin_interfaces::msg::Duration Duration()
{
    builtin_interfaces::msg::Duration duration;
    duration.sec = 0;
    duration.nanosec = 0;
    return duration;
}
}  // namespace ros

namespace vins_ros2
{
inline double stampToSec(const builtin_interfaces::msg::Time &stamp)
{
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}
}  // namespace vins_ros2

#define ROS_INFO(...) RCUTILS_LOG_INFO_NAMED("vins", __VA_ARGS__)
#define ROS_WARN(...) RCUTILS_LOG_WARN_NAMED("vins", __VA_ARGS__)
#define ROS_ERROR(...) RCUTILS_LOG_ERROR_NAMED("vins", __VA_ARGS__)
#define ROS_DEBUG(...) RCUTILS_LOG_DEBUG_NAMED("vins", __VA_ARGS__)

#define ROS_INFO_STREAM(args) do { std::ostringstream s; s << args; ROS_INFO("%s", s.str().c_str()); } while (0)
#define ROS_WARN_STREAM(args) do { std::ostringstream s; s << args; ROS_WARN("%s", s.str().c_str()); } while (0)
#define ROS_ERROR_STREAM(args) do { std::ostringstream s; s << args; ROS_ERROR("%s", s.str().c_str()); } while (0)
#define ROS_DEBUG_STREAM(args) do { std::ostringstream s; s << args; ROS_DEBUG("%s", s.str().c_str()); } while (0)

#define ROS_ASSERT(expr) assert(expr)
#define ROS_ASSERT_MSG(expr, ...) do { if (!(expr)) { ROS_ERROR(__VA_ARGS__); assert(expr); } } while (0)
#define ROS_BREAK() std::abort()
