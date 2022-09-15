// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from autoware_auto_msgs:msg/Complex32.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "autoware_auto_msgs/msg/detail/complex32__rosidl_typesupport_introspection_c.h"
#include "autoware_auto_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "autoware_auto_msgs/msg/detail/complex32__functions.h"
#include "autoware_auto_msgs/msg/detail/complex32__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void Complex32__rosidl_typesupport_introspection_c__Complex32_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  autoware_auto_msgs__msg__Complex32__init(message_memory);
}

void Complex32__rosidl_typesupport_introspection_c__Complex32_fini_function(void * message_memory)
{
  autoware_auto_msgs__msg__Complex32__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Complex32__rosidl_typesupport_introspection_c__Complex32_message_member_array[2] = {
  {
    "real",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autoware_auto_msgs__msg__Complex32, real),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "imag",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(autoware_auto_msgs__msg__Complex32, imag),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Complex32__rosidl_typesupport_introspection_c__Complex32_message_members = {
  "autoware_auto_msgs__msg",  // message namespace
  "Complex32",  // message name
  2,  // number of fields
  sizeof(autoware_auto_msgs__msg__Complex32),
  Complex32__rosidl_typesupport_introspection_c__Complex32_message_member_array,  // message members
  Complex32__rosidl_typesupport_introspection_c__Complex32_init_function,  // function to initialize message memory (memory has to be allocated)
  Complex32__rosidl_typesupport_introspection_c__Complex32_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Complex32__rosidl_typesupport_introspection_c__Complex32_message_type_support_handle = {
  0,
  &Complex32__rosidl_typesupport_introspection_c__Complex32_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_autoware_auto_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, autoware_auto_msgs, msg, Complex32)() {
  if (!Complex32__rosidl_typesupport_introspection_c__Complex32_message_type_support_handle.typesupport_identifier) {
    Complex32__rosidl_typesupport_introspection_c__Complex32_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Complex32__rosidl_typesupport_introspection_c__Complex32_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
