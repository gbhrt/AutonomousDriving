// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autoware_auto_msgs:msg/PointClusters.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_AUTO_MSGS__MSG__DETAIL__POINT_CLUSTERS__TRAITS_HPP_
#define AUTOWARE_AUTO_MSGS__MSG__DETAIL__POINT_CLUSTERS__TRAITS_HPP_

#include "autoware_auto_msgs/msg/detail/point_clusters__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

// Include directives for member types
// Member 'clusters'
#include "sensor_msgs/msg/detail/point_cloud2__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const autoware_auto_msgs::msg::PointClusters & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: clusters
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.clusters.size() == 0) {
      out << "clusters: []\n";
    } else {
      out << "clusters:\n";
      for (auto item : msg.clusters) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const autoware_auto_msgs::msg::PointClusters & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<autoware_auto_msgs::msg::PointClusters>()
{
  return "autoware_auto_msgs::msg::PointClusters";
}

template<>
inline const char * name<autoware_auto_msgs::msg::PointClusters>()
{
  return "autoware_auto_msgs/msg/PointClusters";
}

template<>
struct has_fixed_size<autoware_auto_msgs::msg::PointClusters>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autoware_auto_msgs::msg::PointClusters>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autoware_auto_msgs::msg::PointClusters>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTOWARE_AUTO_MSGS__MSG__DETAIL__POINT_CLUSTERS__TRAITS_HPP_
