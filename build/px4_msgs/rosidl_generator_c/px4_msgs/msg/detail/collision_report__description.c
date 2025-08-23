// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/CollisionReport.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/collision_report__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__CollisionReport__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe2, 0xbb, 0xcf, 0xdf, 0x03, 0x4b, 0x3d, 0xe2,
      0xcc, 0x36, 0x74, 0x6d, 0x3e, 0xb2, 0xef, 0x60,
      0x3f, 0xc8, 0x61, 0xc7, 0x2d, 0x9a, 0xba, 0xe5,
      0x6b, 0x88, 0x55, 0x84, 0x79, 0xa9, 0xd2, 0x45,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__CollisionReport__TYPE_NAME[] = "px4_msgs/msg/CollisionReport";

// Define type names, field names, and default values
static char px4_msgs__msg__CollisionReport__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__CollisionReport__FIELD_NAME__src[] = "src";
static char px4_msgs__msg__CollisionReport__FIELD_NAME__id[] = "id";
static char px4_msgs__msg__CollisionReport__FIELD_NAME__action[] = "action";
static char px4_msgs__msg__CollisionReport__FIELD_NAME__threat_level[] = "threat_level";
static char px4_msgs__msg__CollisionReport__FIELD_NAME__time_to_minimum_delta[] = "time_to_minimum_delta";
static char px4_msgs__msg__CollisionReport__FIELD_NAME__altitude_minimum_delta[] = "altitude_minimum_delta";
static char px4_msgs__msg__CollisionReport__FIELD_NAME__horizontal_minimum_delta[] = "horizontal_minimum_delta";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__CollisionReport__FIELDS[] = {
  {
    {px4_msgs__msg__CollisionReport__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__CollisionReport__FIELD_NAME__src, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__CollisionReport__FIELD_NAME__id, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__CollisionReport__FIELD_NAME__action, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__CollisionReport__FIELD_NAME__threat_level, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__CollisionReport__FIELD_NAME__time_to_minimum_delta, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__CollisionReport__FIELD_NAME__altitude_minimum_delta, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__CollisionReport__FIELD_NAME__horizontal_minimum_delta, 24, 24},
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
px4_msgs__msg__CollisionReport__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__CollisionReport__TYPE_NAME, 28, 28},
      {px4_msgs__msg__CollisionReport__FIELDS, 8, 8},
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
  "uint8 src\n"
  "uint32 id\n"
  "uint8 action\n"
  "uint8 threat_level\n"
  "float32 time_to_minimum_delta\n"
  "float32 altitude_minimum_delta\n"
  "float32 horizontal_minimum_delta";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__CollisionReport__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__CollisionReport__TYPE_NAME, 28, 28},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 205, 205},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__CollisionReport__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__CollisionReport__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
