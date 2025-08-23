// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/ActionRequest.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/action_request__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__ActionRequest__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x75, 0x97, 0xe7, 0x47, 0x68, 0xec, 0x16, 0x29,
      0xf0, 0xa3, 0x3b, 0x4d, 0x14, 0xbb, 0x57, 0x85,
      0xd1, 0xe9, 0x69, 0x88, 0xe3, 0xb6, 0x58, 0xa4,
      0x3a, 0xbb, 0xc4, 0x3d, 0xef, 0xb9, 0x62, 0x93,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__ActionRequest__TYPE_NAME[] = "px4_msgs/msg/ActionRequest";

// Define type names, field names, and default values
static char px4_msgs__msg__ActionRequest__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__ActionRequest__FIELD_NAME__action[] = "action";
static char px4_msgs__msg__ActionRequest__FIELD_NAME__source[] = "source";
static char px4_msgs__msg__ActionRequest__FIELD_NAME__mode[] = "mode";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__ActionRequest__FIELDS[] = {
  {
    {px4_msgs__msg__ActionRequest__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ActionRequest__FIELD_NAME__action, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ActionRequest__FIELD_NAME__source, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ActionRequest__FIELD_NAME__mode, 4, 4},
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
px4_msgs__msg__ActionRequest__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__ActionRequest__TYPE_NAME, 26, 26},
      {px4_msgs__msg__ActionRequest__FIELDS, 4, 4},
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
  "uint8 action # what action is requested\n"
  "uint8 ACTION_DISARM = 0\n"
  "uint8 ACTION_ARM = 1\n"
  "uint8 ACTION_TOGGLE_ARMING = 2\n"
  "uint8 ACTION_UNKILL = 3\n"
  "uint8 ACTION_KILL = 4\n"
  "uint8 ACTION_SWITCH_MODE = 5\n"
  "uint8 ACTION_VTOL_TRANSITION_TO_MULTICOPTER = 6\n"
  "uint8 ACTION_VTOL_TRANSITION_TO_FIXEDWING = 7\n"
  "\n"
  "uint8 source # how the request was triggered\n"
  "uint8 SOURCE_RC_STICK_GESTURE = 0\n"
  "uint8 SOURCE_RC_SWITCH = 1\n"
  "uint8 SOURCE_RC_BUTTON = 2\n"
  "uint8 SOURCE_RC_MODE_SLOT = 3\n"
  "\n"
  "uint8 mode # for ACTION_SWITCH_MODE what mode is requested according to vehicle_status_s::NAVIGATION_STATE_*";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__ActionRequest__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__ActionRequest__TYPE_NAME, 26, 26},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 618, 618},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__ActionRequest__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__ActionRequest__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
