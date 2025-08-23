// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/PositionSetpointTriplet.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/position_setpoint_triplet__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__PositionSetpointTriplet__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x59, 0x7c, 0xa5, 0x14, 0xdd, 0x31, 0x2f, 0xfb,
      0xee, 0xbc, 0x6f, 0xcc, 0x9f, 0x51, 0x57, 0xaa,
      0xad, 0x0e, 0x61, 0x66, 0x08, 0x93, 0x27, 0x3b,
      0x3a, 0x61, 0x0e, 0xad, 0xed, 0xc7, 0x83, 0x32,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "px4_msgs/msg/detail/position_setpoint__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t px4_msgs__msg__PositionSetpoint__EXPECTED_HASH = {1, {
    0x5b, 0x07, 0x80, 0x7d, 0x3f, 0x56, 0x48, 0xe0,
    0x81, 0x76, 0x41, 0xdd, 0xae, 0x1d, 0x13, 0xa0,
    0x54, 0x5e, 0x7f, 0xf2, 0x3f, 0x10, 0xa7, 0x5c,
    0xcd, 0xe5, 0x4c, 0xc2, 0x5b, 0x28, 0x26, 0x2b,
  }};
#endif

static char px4_msgs__msg__PositionSetpointTriplet__TYPE_NAME[] = "px4_msgs/msg/PositionSetpointTriplet";
static char px4_msgs__msg__PositionSetpoint__TYPE_NAME[] = "px4_msgs/msg/PositionSetpoint";

// Define type names, field names, and default values
static char px4_msgs__msg__PositionSetpointTriplet__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__PositionSetpointTriplet__FIELD_NAME__previous[] = "previous";
static char px4_msgs__msg__PositionSetpointTriplet__FIELD_NAME__current[] = "current";
static char px4_msgs__msg__PositionSetpointTriplet__FIELD_NAME__next[] = "next";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__PositionSetpointTriplet__FIELDS[] = {
  {
    {px4_msgs__msg__PositionSetpointTriplet__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__PositionSetpointTriplet__FIELD_NAME__previous, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {px4_msgs__msg__PositionSetpoint__TYPE_NAME, 29, 29},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__PositionSetpointTriplet__FIELD_NAME__current, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {px4_msgs__msg__PositionSetpoint__TYPE_NAME, 29, 29},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__PositionSetpointTriplet__FIELD_NAME__next, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {px4_msgs__msg__PositionSetpoint__TYPE_NAME, 29, 29},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription px4_msgs__msg__PositionSetpointTriplet__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {px4_msgs__msg__PositionSetpoint__TYPE_NAME, 29, 29},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__PositionSetpointTriplet__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__PositionSetpointTriplet__TYPE_NAME, 36, 36},
      {px4_msgs__msg__PositionSetpointTriplet__FIELDS, 4, 4},
    },
    {px4_msgs__msg__PositionSetpointTriplet__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&px4_msgs__msg__PositionSetpoint__EXPECTED_HASH, px4_msgs__msg__PositionSetpoint__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = px4_msgs__msg__PositionSetpoint__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Global position setpoint triplet in WGS84 coordinates.\n"
  "# This are the three next waypoints (or just the next two or one).\n"
  "\n"
  "uint64 timestamp\\t\\t# time since system start (microseconds)\n"
  "\n"
  "PositionSetpoint previous\n"
  "PositionSetpoint current\n"
  "PositionSetpoint next";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__PositionSetpointTriplet__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__PositionSetpointTriplet__TYPE_NAME, 36, 36},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 258, 258},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__PositionSetpointTriplet__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__PositionSetpointTriplet__get_individual_type_description_source(NULL),
    sources[1] = *px4_msgs__msg__PositionSetpoint__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
