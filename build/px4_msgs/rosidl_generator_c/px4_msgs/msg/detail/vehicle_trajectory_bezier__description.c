// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/VehicleTrajectoryBezier.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/vehicle_trajectory_bezier__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__VehicleTrajectoryBezier__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x9f, 0x4f, 0x18, 0x3d, 0x25, 0x28, 0x1f, 0x63,
      0xbe, 0x8a, 0xa8, 0x00, 0x8e, 0x6b, 0x0a, 0xf0,
      0xb3, 0x83, 0x7e, 0x64, 0x45, 0x2a, 0xf0, 0x58,
      0x37, 0x30, 0xba, 0x81, 0x04, 0x21, 0xec, 0xbc,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "px4_msgs/msg/detail/trajectory_bezier__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t px4_msgs__msg__TrajectoryBezier__EXPECTED_HASH = {1, {
    0xbb, 0x66, 0xc4, 0xd5, 0x77, 0x72, 0x11, 0xfe,
    0x92, 0x4f, 0x9e, 0x86, 0x50, 0xcd, 0x57, 0xd2,
    0x1e, 0x66, 0x00, 0xb3, 0xab, 0x50, 0xde, 0xba,
    0x9f, 0x4f, 0x63, 0x9f, 0xcd, 0x96, 0xb8, 0x7d,
  }};
#endif

static char px4_msgs__msg__VehicleTrajectoryBezier__TYPE_NAME[] = "px4_msgs/msg/VehicleTrajectoryBezier";
static char px4_msgs__msg__TrajectoryBezier__TYPE_NAME[] = "px4_msgs/msg/TrajectoryBezier";

// Define type names, field names, and default values
static char px4_msgs__msg__VehicleTrajectoryBezier__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__VehicleTrajectoryBezier__FIELD_NAME__control_points[] = "control_points";
static char px4_msgs__msg__VehicleTrajectoryBezier__FIELD_NAME__bezier_order[] = "bezier_order";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__VehicleTrajectoryBezier__FIELDS[] = {
  {
    {px4_msgs__msg__VehicleTrajectoryBezier__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleTrajectoryBezier__FIELD_NAME__control_points, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_ARRAY,
      5,
      0,
      {px4_msgs__msg__TrajectoryBezier__TYPE_NAME, 29, 29},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleTrajectoryBezier__FIELD_NAME__bezier_order, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription px4_msgs__msg__VehicleTrajectoryBezier__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {px4_msgs__msg__TrajectoryBezier__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__VehicleTrajectoryBezier__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__VehicleTrajectoryBezier__TYPE_NAME, 36, 36},
      {px4_msgs__msg__VehicleTrajectoryBezier__FIELDS, 3, 3},
    },
    {px4_msgs__msg__VehicleTrajectoryBezier__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&px4_msgs__msg__TrajectoryBezier__EXPECTED_HASH, px4_msgs__msg__TrajectoryBezier__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = px4_msgs__msg__TrajectoryBezier__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Vehicle Waypoints Trajectory description. See also MAVLink MAV_TRAJECTORY_REPRESENTATION msg\n"
  "# The topic vehicle_trajectory_bezier is used to send a smooth flight path from the\n"
  "# companion computer / avoidance module to the position controller.\n"
  "\n"
  "uint64 timestamp\\t\\t# time since system start (microseconds)\n"
  "\n"
  "uint8 POINT_0 = 0\n"
  "uint8 POINT_1 = 1\n"
  "uint8 POINT_2 = 2\n"
  "uint8 POINT_3 = 3\n"
  "uint8 POINT_4 = 4\n"
  "\n"
  "uint8 NUMBER_POINTS = 5\n"
  "\n"
  "TrajectoryBezier[5] control_points\n"
  "uint8 bezier_order\n"
  "\n"
  "# TOPICS vehicle_trajectory_bezier";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__VehicleTrajectoryBezier__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__VehicleTrajectoryBezier__TYPE_NAME, 36, 36},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 514, 514},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__VehicleTrajectoryBezier__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__VehicleTrajectoryBezier__get_individual_type_description_source(NULL),
    sources[1] = *px4_msgs__msg__TrajectoryBezier__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
