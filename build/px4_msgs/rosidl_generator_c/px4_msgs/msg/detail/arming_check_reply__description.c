// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/ArmingCheckReply.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/arming_check_reply__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__ArmingCheckReply__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe0, 0x49, 0x95, 0x7c, 0x9a, 0x75, 0xef, 0x53,
      0xde, 0xef, 0xcd, 0x39, 0x9f, 0xd2, 0x9c, 0x47,
      0x3b, 0x66, 0x6c, 0x3b, 0x26, 0x68, 0xe6, 0x5a,
      0x00, 0xa4, 0x3f, 0x8f, 0x58, 0xba, 0x1a, 0x9f,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "px4_msgs/msg/detail/event__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t px4_msgs__msg__Event__EXPECTED_HASH = {1, {
    0x49, 0x64, 0x81, 0x2e, 0xb9, 0x82, 0xe8, 0xd7,
    0x85, 0x5e, 0x07, 0x63, 0x2a, 0x5b, 0x46, 0x2b,
    0x0b, 0xee, 0xb3, 0xa1, 0x9e, 0x11, 0x69, 0x13,
    0x50, 0x6b, 0x33, 0x9e, 0xb1, 0xc8, 0x76, 0xa9,
  }};
#endif

static char px4_msgs__msg__ArmingCheckReply__TYPE_NAME[] = "px4_msgs/msg/ArmingCheckReply";
static char px4_msgs__msg__Event__TYPE_NAME[] = "px4_msgs/msg/Event";

// Define type names, field names, and default values
static char px4_msgs__msg__ArmingCheckReply__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__ArmingCheckReply__FIELD_NAME__request_id[] = "request_id";
static char px4_msgs__msg__ArmingCheckReply__FIELD_NAME__registration_id[] = "registration_id";
static char px4_msgs__msg__ArmingCheckReply__FIELD_NAME__health_component_index[] = "health_component_index";
static char px4_msgs__msg__ArmingCheckReply__FIELD_NAME__health_component_is_present[] = "health_component_is_present";
static char px4_msgs__msg__ArmingCheckReply__FIELD_NAME__health_component_warning[] = "health_component_warning";
static char px4_msgs__msg__ArmingCheckReply__FIELD_NAME__health_component_error[] = "health_component_error";
static char px4_msgs__msg__ArmingCheckReply__FIELD_NAME__can_arm_and_run[] = "can_arm_and_run";
static char px4_msgs__msg__ArmingCheckReply__FIELD_NAME__num_events[] = "num_events";
static char px4_msgs__msg__ArmingCheckReply__FIELD_NAME__events[] = "events";
static char px4_msgs__msg__ArmingCheckReply__FIELD_NAME__mode_req_angular_velocity[] = "mode_req_angular_velocity";
static char px4_msgs__msg__ArmingCheckReply__FIELD_NAME__mode_req_attitude[] = "mode_req_attitude";
static char px4_msgs__msg__ArmingCheckReply__FIELD_NAME__mode_req_local_alt[] = "mode_req_local_alt";
static char px4_msgs__msg__ArmingCheckReply__FIELD_NAME__mode_req_local_position[] = "mode_req_local_position";
static char px4_msgs__msg__ArmingCheckReply__FIELD_NAME__mode_req_local_position_relaxed[] = "mode_req_local_position_relaxed";
static char px4_msgs__msg__ArmingCheckReply__FIELD_NAME__mode_req_global_position[] = "mode_req_global_position";
static char px4_msgs__msg__ArmingCheckReply__FIELD_NAME__mode_req_mission[] = "mode_req_mission";
static char px4_msgs__msg__ArmingCheckReply__FIELD_NAME__mode_req_home_position[] = "mode_req_home_position";
static char px4_msgs__msg__ArmingCheckReply__FIELD_NAME__mode_req_prevent_arming[] = "mode_req_prevent_arming";
static char px4_msgs__msg__ArmingCheckReply__FIELD_NAME__mode_req_manual_control[] = "mode_req_manual_control";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__ArmingCheckReply__FIELDS[] = {
  {
    {px4_msgs__msg__ArmingCheckReply__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ArmingCheckReply__FIELD_NAME__request_id, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ArmingCheckReply__FIELD_NAME__registration_id, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ArmingCheckReply__FIELD_NAME__health_component_index, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ArmingCheckReply__FIELD_NAME__health_component_is_present, 27, 27},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ArmingCheckReply__FIELD_NAME__health_component_warning, 24, 24},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ArmingCheckReply__FIELD_NAME__health_component_error, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ArmingCheckReply__FIELD_NAME__can_arm_and_run, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ArmingCheckReply__FIELD_NAME__num_events, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ArmingCheckReply__FIELD_NAME__events, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_ARRAY,
      5,
      0,
      {px4_msgs__msg__Event__TYPE_NAME, 18, 18},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ArmingCheckReply__FIELD_NAME__mode_req_angular_velocity, 25, 25},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ArmingCheckReply__FIELD_NAME__mode_req_attitude, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ArmingCheckReply__FIELD_NAME__mode_req_local_alt, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ArmingCheckReply__FIELD_NAME__mode_req_local_position, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ArmingCheckReply__FIELD_NAME__mode_req_local_position_relaxed, 31, 31},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ArmingCheckReply__FIELD_NAME__mode_req_global_position, 24, 24},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ArmingCheckReply__FIELD_NAME__mode_req_mission, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ArmingCheckReply__FIELD_NAME__mode_req_home_position, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ArmingCheckReply__FIELD_NAME__mode_req_prevent_arming, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__ArmingCheckReply__FIELD_NAME__mode_req_manual_control, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription px4_msgs__msg__ArmingCheckReply__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {px4_msgs__msg__Event__TYPE_NAME, 18, 18},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__ArmingCheckReply__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__ArmingCheckReply__TYPE_NAME, 29, 29},
      {px4_msgs__msg__ArmingCheckReply__FIELDS, 20, 20},
    },
    {px4_msgs__msg__ArmingCheckReply__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&px4_msgs__msg__Event__EXPECTED_HASH, px4_msgs__msg__Event__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = px4_msgs__msg__Event__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp # time since system start (microseconds)\n"
  "\n"
  "uint8 request_id\n"
  "uint8 registration_id\n"
  "\n"
  "uint8 HEALTH_COMPONENT_INDEX_NONE = 0\n"
  "uint8 HEALTH_COMPONENT_INDEX_AVOIDANCE = 19\n"
  "\n"
  "uint8 health_component_index      # HEALTH_COMPONENT_INDEX_*\n"
  "bool health_component_is_present\n"
  "bool health_component_warning\n"
  "bool health_component_error\n"
  "\n"
  "bool can_arm_and_run              # whether arming is possible, and if it's a navigation mode, if it can run\n"
  "\n"
  "uint8 num_events\n"
  "\n"
  "Event[5] events\n"
  "\n"
  "# Mode requirements\n"
  "bool mode_req_angular_velocity\n"
  "bool mode_req_attitude\n"
  "bool mode_req_local_alt\n"
  "bool mode_req_local_position\n"
  "bool mode_req_local_position_relaxed\n"
  "bool mode_req_global_position\n"
  "bool mode_req_mission\n"
  "bool mode_req_home_position\n"
  "bool mode_req_prevent_arming\n"
  "bool mode_req_manual_control\n"
  "\n"
  "\n"
  "uint8 ORB_QUEUE_LENGTH = 4";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__ArmingCheckReply__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__ArmingCheckReply__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 811, 811},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__ArmingCheckReply__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__ArmingCheckReply__get_individual_type_description_source(NULL),
    sources[1] = *px4_msgs__msg__Event__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
