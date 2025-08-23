// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/TrajectoryBezier.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/trajectory_bezier__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__TrajectoryBezier__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xbb, 0x66, 0xc4, 0xd5, 0x77, 0x72, 0x11, 0xfe,
      0x92, 0x4f, 0x9e, 0x86, 0x50, 0xcd, 0x57, 0xd2,
      0x1e, 0x66, 0x00, 0xb3, 0xab, 0x50, 0xde, 0xba,
      0x9f, 0x4f, 0x63, 0x9f, 0xcd, 0x96, 0xb8, 0x7d,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__TrajectoryBezier__TYPE_NAME[] = "px4_msgs/msg/TrajectoryBezier";

// Define type names, field names, and default values
static char px4_msgs__msg__TrajectoryBezier__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__TrajectoryBezier__FIELD_NAME__position[] = "position";
static char px4_msgs__msg__TrajectoryBezier__FIELD_NAME__yaw[] = "yaw";
static char px4_msgs__msg__TrajectoryBezier__FIELD_NAME__delta[] = "delta";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__TrajectoryBezier__FIELDS[] = {
  {
    {px4_msgs__msg__TrajectoryBezier__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TrajectoryBezier__FIELD_NAME__position, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TrajectoryBezier__FIELD_NAME__yaw, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TrajectoryBezier__FIELD_NAME__delta, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__TrajectoryBezier__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__TrajectoryBezier__TYPE_NAME, 29, 29},
      {px4_msgs__msg__TrajectoryBezier__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Bezier Trajectory description. See also Mavlink TRAJECTORY msg\n"
  "# The topic trajectory_bezier describe each waypoint defined in vehicle_trajectory_bezier\n"
  "\n"
  "uint64 timestamp\\t\\t# time since system start (microseconds)\n"
  "\n"
  "float32[3] position     # local position x,y,z (metres)\n"
  "float32 yaw             # yaw angle (rad)\n"
  "float32 delta           # time it should take to get to this waypoint, if this is the final waypoint (seconds)";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__TrajectoryBezier__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__TrajectoryBezier__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 425, 425},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__TrajectoryBezier__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__TrajectoryBezier__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
