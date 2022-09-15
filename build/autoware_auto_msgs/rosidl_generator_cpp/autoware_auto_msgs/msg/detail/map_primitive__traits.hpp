// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from autoware_auto_msgs:msg/MapPrimitive.idl
// generated code does not contain a copyright notice

#ifndef AUTOWARE_AUTO_MSGS__MSG__DETAIL__MAP_PRIMITIVE__TRAITS_HPP_
#define AUTOWARE_AUTO_MSGS__MSG__DETAIL__MAP_PRIMITIVE__TRAITS_HPP_

#include "autoware_auto_msgs/msg/detail/map_primitive__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const autoware_auto_msgs::msg::MapPrimitive & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: primitive_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "primitive_type: ";
    value_to_yaml(msg.primitive_type, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const autoware_auto_msgs::msg::MapPrimitive & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<autoware_auto_msgs::msg::MapPrimitive>()
{
  return "autoware_auto_msgs::msg::MapPrimitive";
}

template<>
inline const char * name<autoware_auto_msgs::msg::MapPrimitive>()
{
  return "autoware_auto_msgs/msg/MapPrimitive";
}

template<>
struct has_fixed_size<autoware_auto_msgs::msg::MapPrimitive>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<autoware_auto_msgs::msg::MapPrimitive>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<autoware_auto_msgs::msg::MapPrimitive>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTOWARE_AUTO_MSGS__MSG__DETAIL__MAP_PRIMITIVE__TRAITS_HPP_
