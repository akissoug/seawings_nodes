// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/CellularStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/cellular_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__CellularStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xfa, 0xac, 0x66, 0xc9, 0x0d, 0xc6, 0x8d, 0x2c,
      0xc3, 0x5f, 0x92, 0x59, 0x01, 0xa8, 0x01, 0xd7,
      0x5d, 0xd7, 0x84, 0xa6, 0x4a, 0x3c, 0xbd, 0xdd,
      0xe0, 0x5f, 0x6a, 0x37, 0x1f, 0x36, 0x91, 0x3b,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__CellularStatus__TYPE_NAME[] = "px4_msgs/msg/CellularStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__CellularStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__CellularStatus__FIELD_NAME__status[] = "status";
static char px4_msgs__msg__CellularStatus__FIELD_NAME__failure_reason[] = "failure_reason";
static char px4_msgs__msg__CellularStatus__FIELD_NAME__type[] = "type";
static char px4_msgs__msg__CellularStatus__FIELD_NAME__quality[] = "quality";
static char px4_msgs__msg__CellularStatus__FIELD_NAME__mcc[] = "mcc";
static char px4_msgs__msg__CellularStatus__FIELD_NAME__mnc[] = "mnc";
static char px4_msgs__msg__CellularStatus__FIELD_NAME__lac[] = "lac";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__CellularStatus__FIELDS[] = {
  {
    {px4_msgs__msg__CellularStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__CellularStatus__FIELD_NAME__status, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__CellularStatus__FIELD_NAME__failure_reason, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__CellularStatus__FIELD_NAME__type, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__CellularStatus__FIELD_NAME__quality, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__CellularStatus__FIELD_NAME__mcc, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__CellularStatus__FIELD_NAME__mnc, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__CellularStatus__FIELD_NAME__lac, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__CellularStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__CellularStatus__TYPE_NAME, 27, 27},
      {px4_msgs__msg__CellularStatus__FIELDS, 8, 8},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp\\t# time since system start (microseconds)\n"
  "\n"
  "uint8 CELLULAR_STATUS_FLAG_UNKNOWN=0 # State unknown or not reportable\n"
  "uint8 CELLULAR_STATUS_FLAG_FAILED=1 # velocity setpoint\n"
  "uint8 CELLULAR_STATUS_FLAG_INITIALIZING=2 # Modem is being initialized\n"
  "uint8 CELLULAR_STATUS_FLAG_LOCKED=3\\t# Modem is locked\n"
  "uint8 CELLULAR_STATUS_FLAG_DISABLED=4\\t# Modem is not enabled and is powered down\n"
  "uint8 CELLULAR_STATUS_FLAG_DISABLING=5\\t# Modem is currently transitioning to the CELLULAR_STATUS_FLAG_DISABLED state\n"
  "uint8 CELLULAR_STATUS_FLAG_ENABLING=6 \\t# Modem is currently transitioning to the CELLULAR_STATUS_FLAG_ENABLED state\n"
  "uint8 CELLULAR_STATUS_FLAG_ENABLED=7  # Modem is enabled and powered on but not registered with a network provider and not available for data connections\n"
  "uint8 CELLULAR_STATUS_FLAG_SEARCHING=8  # Modem is searching for a network provider to register\n"
  "uint8 CELLULAR_STATUS_FLAG_REGISTERED=9  # Modem is registered with a network provider, and data connections and messaging may be available for use\n"
  "uint8 CELLULAR_STATUS_FLAG_DISCONNECTING=10  # Modem is disconnecting and deactivating the last active packet data bearer. This state will not be entered if more than one packet data bearer is active and one of the active bearers is deactivated\n"
  "uint8 CELLULAR_STATUS_FLAG_CONNECTING=11  # Modem is activating and connecting the first packet data bearer. Subsequent bearer activations when another bearer is already active do not cause this state to be entered\n"
  "uint8 CELLULAR_STATUS_FLAG_CONNECTED=12  # One or more packet data bearers is active and connected\n"
  "\n"
  "uint8 CELLULAR_NETWORK_FAILED_REASON_NONE=0 # No error\n"
  "uint8 CELLULAR_NETWORK_FAILED_REASON_UNKNOWN=1 # Error state is unknown\n"
  "uint8 CELLULAR_NETWORK_FAILED_REASON_SIM_MISSING=2 # SIM is required for the modem but missing\n"
  "uint8 CELLULAR_NETWORK_FAILED_REASON_SIM_ERROR=3 # SIM is available, but not usable for connection\n"
  "\n"
  "uint16 status \\t# Status bitmap 1: Roaming is active\n"
  "uint8 failure_reason #Failure reason when status in in CELLUAR_STATUS_FAILED\n"
  "uint8 type \\t# Cellular network radio type 0: none 1: gsm 2: cdma 3: wcdma 4: lte\n"
  "uint8 quality\\t# Cellular network RSSI/RSRP in dBm, absolute value\n"
  "uint16 mcc\\t# Mobile country code. If unknown, set to: UINT16_MAX\n"
  "uint16 mnc\\t# Mobile network code. If unknown, set to: UINT16_MAX\n"
  "uint16 lac\\t# Location area code. If unknown, set to: 0";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__CellularStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__CellularStatus__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 2366, 2366},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__CellularStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__CellularStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
