// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/NavigatorMissionItem.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/navigator_mission_item__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__NavigatorMissionItem__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd9, 0x50, 0x5a, 0xaf, 0x5b, 0xb5, 0xbe, 0x0e,
      0x91, 0xdd, 0xb6, 0x45, 0xba, 0x0f, 0x78, 0x06,
      0x11, 0x71, 0x37, 0xd0, 0x12, 0x74, 0x55, 0x97,
      0x92, 0xb1, 0x64, 0xeb, 0x92, 0xea, 0x53, 0x60,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__NavigatorMissionItem__TYPE_NAME[] = "px4_msgs/msg/NavigatorMissionItem";

// Define type names, field names, and default values
static char px4_msgs__msg__NavigatorMissionItem__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__NavigatorMissionItem__FIELD_NAME__instance_count[] = "instance_count";
static char px4_msgs__msg__NavigatorMissionItem__FIELD_NAME__sequence_current[] = "sequence_current";
static char px4_msgs__msg__NavigatorMissionItem__FIELD_NAME__nav_cmd[] = "nav_cmd";
static char px4_msgs__msg__NavigatorMissionItem__FIELD_NAME__latitude[] = "latitude";
static char px4_msgs__msg__NavigatorMissionItem__FIELD_NAME__longitude[] = "longitude";
static char px4_msgs__msg__NavigatorMissionItem__FIELD_NAME__time_inside[] = "time_inside";
static char px4_msgs__msg__NavigatorMissionItem__FIELD_NAME__acceptance_radius[] = "acceptance_radius";
static char px4_msgs__msg__NavigatorMissionItem__FIELD_NAME__loiter_radius[] = "loiter_radius";
static char px4_msgs__msg__NavigatorMissionItem__FIELD_NAME__yaw[] = "yaw";
static char px4_msgs__msg__NavigatorMissionItem__FIELD_NAME__altitude[] = "altitude";
static char px4_msgs__msg__NavigatorMissionItem__FIELD_NAME__frame[] = "frame";
static char px4_msgs__msg__NavigatorMissionItem__FIELD_NAME__origin[] = "origin";
static char px4_msgs__msg__NavigatorMissionItem__FIELD_NAME__loiter_exit_xtrack[] = "loiter_exit_xtrack";
static char px4_msgs__msg__NavigatorMissionItem__FIELD_NAME__force_heading[] = "force_heading";
static char px4_msgs__msg__NavigatorMissionItem__FIELD_NAME__altitude_is_relative[] = "altitude_is_relative";
static char px4_msgs__msg__NavigatorMissionItem__FIELD_NAME__autocontinue[] = "autocontinue";
static char px4_msgs__msg__NavigatorMissionItem__FIELD_NAME__vtol_back_transition[] = "vtol_back_transition";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__NavigatorMissionItem__FIELDS[] = {
  {
    {px4_msgs__msg__NavigatorMissionItem__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__NavigatorMissionItem__FIELD_NAME__instance_count, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__NavigatorMissionItem__FIELD_NAME__sequence_current, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__NavigatorMissionItem__FIELD_NAME__nav_cmd, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__NavigatorMissionItem__FIELD_NAME__latitude, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__NavigatorMissionItem__FIELD_NAME__longitude, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__NavigatorMissionItem__FIELD_NAME__time_inside, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__NavigatorMissionItem__FIELD_NAME__acceptance_radius, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__NavigatorMissionItem__FIELD_NAME__loiter_radius, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__NavigatorMissionItem__FIELD_NAME__yaw, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__NavigatorMissionItem__FIELD_NAME__altitude, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__NavigatorMissionItem__FIELD_NAME__frame, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__NavigatorMissionItem__FIELD_NAME__origin, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__NavigatorMissionItem__FIELD_NAME__loiter_exit_xtrack, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__NavigatorMissionItem__FIELD_NAME__force_heading, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__NavigatorMissionItem__FIELD_NAME__altitude_is_relative, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__NavigatorMissionItem__FIELD_NAME__autocontinue, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__NavigatorMissionItem__FIELD_NAME__vtol_back_transition, 20, 20},
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
px4_msgs__msg__NavigatorMissionItem__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__NavigatorMissionItem__TYPE_NAME, 33, 33},
      {px4_msgs__msg__NavigatorMissionItem__FIELDS, 18, 18},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp                 # time since system start (microseconds)\n"
  "\n"
  "uint32 instance_count            # Instance count of this mission. Increments monotonically whenever the mission is modified\n"
  "\n"
  "uint16 sequence_current          # Sequence of the current mission item\n"
  "\n"
  "uint16 nav_cmd\n"
  "\n"
  "float32 latitude\n"
  "float32 longitude\n"
  "\n"
  "float32 time_inside              # time that the MAV should stay inside the radius before advancing in seconds\n"
  "float32 acceptance_radius        # default radius in which the mission is accepted as reached in meters\n"
  "float32 loiter_radius            # loiter radius in meters, 0 for a VTOL to hover, negative for counter-clockwise\n"
  "float32 yaw                      # in radians NED -PI..+PI, NAN means don't change yaw\n"
  "float32 altitude                 # altitude in meters (AMSL)\n"
  "\n"
  "uint8 frame                      # mission frame\n"
  "uint8 origin                     # mission item origin (onboard or mavlink)\n"
  "\n"
  "bool loiter_exit_xtrack          # exit xtrack location: 0 for center of loiter wp, 1 for exit location\n"
  "bool force_heading               # heading needs to be reached\n"
  "bool altitude_is_relative        # true if altitude is relative from start point\n"
  "bool autocontinue                # true if next waypoint should follow after this one\n"
  "bool vtol_back_transition        # part of the vtol back transition sequence";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__NavigatorMissionItem__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__NavigatorMissionItem__TYPE_NAME, 33, 33},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1341, 1341},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__NavigatorMissionItem__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__NavigatorMissionItem__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
