// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/DifferentialDriveSetpoint.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/differential_drive_setpoint__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__DifferentialDriveSetpoint__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xec, 0x4f, 0x52, 0x06, 0x3a, 0x19, 0xbb, 0x82,
      0xba, 0xb9, 0x3e, 0x16, 0x80, 0x95, 0xf2, 0x86,
      0xa5, 0xa3, 0xab, 0x46, 0xa1, 0x07, 0x5d, 0x41,
      0xe2, 0x69, 0x87, 0xf9, 0x21, 0x1b, 0xb5, 0x23,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__DifferentialDriveSetpoint__TYPE_NAME[] = "px4_msgs/msg/DifferentialDriveSetpoint";

// Define type names, field names, and default values
static char px4_msgs__msg__DifferentialDriveSetpoint__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__DifferentialDriveSetpoint__FIELD_NAME__speed[] = "speed";
static char px4_msgs__msg__DifferentialDriveSetpoint__FIELD_NAME__closed_loop_speed_control[] = "closed_loop_speed_control";
static char px4_msgs__msg__DifferentialDriveSetpoint__FIELD_NAME__yaw_rate[] = "yaw_rate";
static char px4_msgs__msg__DifferentialDriveSetpoint__FIELD_NAME__closed_loop_yaw_rate_control[] = "closed_loop_yaw_rate_control";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__DifferentialDriveSetpoint__FIELDS[] = {
  {
    {px4_msgs__msg__DifferentialDriveSetpoint__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__DifferentialDriveSetpoint__FIELD_NAME__speed, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__DifferentialDriveSetpoint__FIELD_NAME__closed_loop_speed_control, 25, 25},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__DifferentialDriveSetpoint__FIELD_NAME__yaw_rate, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__DifferentialDriveSetpoint__FIELD_NAME__closed_loop_yaw_rate_control, 28, 28},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__DifferentialDriveSetpoint__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__DifferentialDriveSetpoint__TYPE_NAME, 38, 38},
      {px4_msgs__msg__DifferentialDriveSetpoint__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp # time since system start (microseconds)\n"
  "\n"
  "float32 speed # [m/s] collective roll-off speed in body x-axis\n"
  "bool closed_loop_speed_control # true if speed is controlled using estimator feedback, false if direct feed-forward\n"
  "float32 yaw_rate # [rad/s] yaw rate\n"
  "bool closed_loop_yaw_rate_control # true if yaw rate is controlled using gyroscope feedback, false if direct feed-forward\n"
  "\n"
  "# TOPICS differential_drive_setpoint differential_drive_control_output";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__DifferentialDriveSetpoint__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__DifferentialDriveSetpoint__TYPE_NAME, 38, 38},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 468, 468},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__DifferentialDriveSetpoint__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__DifferentialDriveSetpoint__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
