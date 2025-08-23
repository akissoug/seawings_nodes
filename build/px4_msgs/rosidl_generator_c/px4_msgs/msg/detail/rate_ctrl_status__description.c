// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/RateCtrlStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/rate_ctrl_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__RateCtrlStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xbd, 0x24, 0x7d, 0xf5, 0x10, 0x55, 0xa4, 0x81,
      0xb5, 0x36, 0x40, 0xc9, 0x60, 0x3a, 0x49, 0x81,
      0x68, 0x13, 0x77, 0xe6, 0x81, 0xe9, 0x1b, 0x90,
      0xb5, 0x58, 0x95, 0xff, 0x65, 0x5a, 0xf3, 0x02,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__RateCtrlStatus__TYPE_NAME[] = "px4_msgs/msg/RateCtrlStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__RateCtrlStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__RateCtrlStatus__FIELD_NAME__rollspeed_integ[] = "rollspeed_integ";
static char px4_msgs__msg__RateCtrlStatus__FIELD_NAME__pitchspeed_integ[] = "pitchspeed_integ";
static char px4_msgs__msg__RateCtrlStatus__FIELD_NAME__yawspeed_integ[] = "yawspeed_integ";
static char px4_msgs__msg__RateCtrlStatus__FIELD_NAME__wheel_rate_integ[] = "wheel_rate_integ";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__RateCtrlStatus__FIELDS[] = {
  {
    {px4_msgs__msg__RateCtrlStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RateCtrlStatus__FIELD_NAME__rollspeed_integ, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RateCtrlStatus__FIELD_NAME__pitchspeed_integ, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RateCtrlStatus__FIELD_NAME__yawspeed_integ, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RateCtrlStatus__FIELD_NAME__wheel_rate_integ, 16, 16},
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
px4_msgs__msg__RateCtrlStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__RateCtrlStatus__TYPE_NAME, 27, 27},
      {px4_msgs__msg__RateCtrlStatus__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp\\t\\t# time since system start (microseconds)\n"
  "\n"
  "# rate controller integrator status\n"
  "float32 rollspeed_integ\n"
  "float32 pitchspeed_integ\n"
  "float32 yawspeed_integ\n"
  "float32 wheel_rate_integ\\t# FW only and optional";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__RateCtrlStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__RateCtrlStatus__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 216, 216},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__RateCtrlStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__RateCtrlStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
