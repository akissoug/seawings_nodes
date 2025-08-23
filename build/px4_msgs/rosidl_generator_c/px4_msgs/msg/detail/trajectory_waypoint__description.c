// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/TrajectoryWaypoint.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/trajectory_waypoint__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__TrajectoryWaypoint__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x3d, 0x31, 0x2b, 0xfa, 0x36, 0xfb, 0x34, 0xc8,
      0x6e, 0xcb, 0xce, 0x3d, 0x2b, 0xd2, 0x93, 0xee,
      0x38, 0x4d, 0x4f, 0x3f, 0xdb, 0x31, 0x28, 0x9c,
      0x93, 0x54, 0x4a, 0x9b, 0xb3, 0xcc, 0xe4, 0x7a,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__TrajectoryWaypoint__TYPE_NAME[] = "px4_msgs/msg/TrajectoryWaypoint";

// Define type names, field names, and default values
static char px4_msgs__msg__TrajectoryWaypoint__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__TrajectoryWaypoint__FIELD_NAME__position[] = "position";
static char px4_msgs__msg__TrajectoryWaypoint__FIELD_NAME__velocity[] = "velocity";
static char px4_msgs__msg__TrajectoryWaypoint__FIELD_NAME__acceleration[] = "acceleration";
static char px4_msgs__msg__TrajectoryWaypoint__FIELD_NAME__yaw[] = "yaw";
static char px4_msgs__msg__TrajectoryWaypoint__FIELD_NAME__yaw_speed[] = "yaw_speed";
static char px4_msgs__msg__TrajectoryWaypoint__FIELD_NAME__point_valid[] = "point_valid";
static char px4_msgs__msg__TrajectoryWaypoint__FIELD_NAME__type[] = "type";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__TrajectoryWaypoint__FIELDS[] = {
  {
    {px4_msgs__msg__TrajectoryWaypoint__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TrajectoryWaypoint__FIELD_NAME__position, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TrajectoryWaypoint__FIELD_NAME__velocity, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TrajectoryWaypoint__FIELD_NAME__acceleration, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TrajectoryWaypoint__FIELD_NAME__yaw, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TrajectoryWaypoint__FIELD_NAME__yaw_speed, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TrajectoryWaypoint__FIELD_NAME__point_valid, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__TrajectoryWaypoint__FIELD_NAME__type, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__TrajectoryWaypoint__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__TrajectoryWaypoint__TYPE_NAME, 31, 31},
      {px4_msgs__msg__TrajectoryWaypoint__FIELDS, 8, 8},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Waypoint Trajectory description. See also Mavlink TRAJECTORY msg\n"
  "# The topic trajectory_waypoint describe each waypoint defined in vehicle_trajectory_waypoint\n"
  "\n"
  "uint64 timestamp\\t\\t# time since system start (microseconds)\n"
  "\n"
  "float32[3] position\n"
  "float32[3] velocity\n"
  "float32[3] acceleration\n"
  "float32 yaw\n"
  "float32 yaw_speed\n"
  "\n"
  "bool point_valid\n"
  "uint8 type";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__TrajectoryWaypoint__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__TrajectoryWaypoint__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 345, 345},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__TrajectoryWaypoint__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__TrajectoryWaypoint__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
