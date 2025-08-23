// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/AutotuneAttitudeControlStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/autotune_attitude_control_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__AutotuneAttitudeControlStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x77, 0x67, 0x73, 0x0b, 0x69, 0x59, 0x75, 0x1d,
      0xb3, 0x58, 0xd3, 0xfa, 0xdd, 0x0e, 0x14, 0x89,
      0xa8, 0xb8, 0xe5, 0x02, 0x69, 0x8d, 0x0a, 0x2e,
      0x80, 0xdc, 0x0f, 0xbb, 0xc6, 0x6d, 0xc0, 0x77,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__AutotuneAttitudeControlStatus__TYPE_NAME[] = "px4_msgs/msg/AutotuneAttitudeControlStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__AutotuneAttitudeControlStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__AutotuneAttitudeControlStatus__FIELD_NAME__coeff[] = "coeff";
static char px4_msgs__msg__AutotuneAttitudeControlStatus__FIELD_NAME__coeff_var[] = "coeff_var";
static char px4_msgs__msg__AutotuneAttitudeControlStatus__FIELD_NAME__fitness[] = "fitness";
static char px4_msgs__msg__AutotuneAttitudeControlStatus__FIELD_NAME__innov[] = "innov";
static char px4_msgs__msg__AutotuneAttitudeControlStatus__FIELD_NAME__dt_model[] = "dt_model";
static char px4_msgs__msg__AutotuneAttitudeControlStatus__FIELD_NAME__kc[] = "kc";
static char px4_msgs__msg__AutotuneAttitudeControlStatus__FIELD_NAME__ki[] = "ki";
static char px4_msgs__msg__AutotuneAttitudeControlStatus__FIELD_NAME__kd[] = "kd";
static char px4_msgs__msg__AutotuneAttitudeControlStatus__FIELD_NAME__kff[] = "kff";
static char px4_msgs__msg__AutotuneAttitudeControlStatus__FIELD_NAME__att_p[] = "att_p";
static char px4_msgs__msg__AutotuneAttitudeControlStatus__FIELD_NAME__rate_sp[] = "rate_sp";
static char px4_msgs__msg__AutotuneAttitudeControlStatus__FIELD_NAME__u_filt[] = "u_filt";
static char px4_msgs__msg__AutotuneAttitudeControlStatus__FIELD_NAME__y_filt[] = "y_filt";
static char px4_msgs__msg__AutotuneAttitudeControlStatus__FIELD_NAME__state[] = "state";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__AutotuneAttitudeControlStatus__FIELDS[] = {
  {
    {px4_msgs__msg__AutotuneAttitudeControlStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AutotuneAttitudeControlStatus__FIELD_NAME__coeff, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      5,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AutotuneAttitudeControlStatus__FIELD_NAME__coeff_var, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      5,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AutotuneAttitudeControlStatus__FIELD_NAME__fitness, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AutotuneAttitudeControlStatus__FIELD_NAME__innov, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AutotuneAttitudeControlStatus__FIELD_NAME__dt_model, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AutotuneAttitudeControlStatus__FIELD_NAME__kc, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AutotuneAttitudeControlStatus__FIELD_NAME__ki, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AutotuneAttitudeControlStatus__FIELD_NAME__kd, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AutotuneAttitudeControlStatus__FIELD_NAME__kff, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AutotuneAttitudeControlStatus__FIELD_NAME__att_p, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AutotuneAttitudeControlStatus__FIELD_NAME__rate_sp, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AutotuneAttitudeControlStatus__FIELD_NAME__u_filt, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AutotuneAttitudeControlStatus__FIELD_NAME__y_filt, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AutotuneAttitudeControlStatus__FIELD_NAME__state, 5, 5},
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
px4_msgs__msg__AutotuneAttitudeControlStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__AutotuneAttitudeControlStatus__TYPE_NAME, 42, 42},
      {px4_msgs__msg__AutotuneAttitudeControlStatus__FIELDS, 15, 15},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp                # time since system start (microseconds)\n"
  "\n"
  "float32[5] coeff                # coefficients of the identified discrete-time model\n"
  "float32[5] coeff_var            # coefficients' variance of the identified discrete-time model\n"
  "float32 fitness                 # fitness of the parameter estimate\n"
  "float32 innov\n"
  "float32 dt_model\n"
  "\n"
  "float32 kc\n"
  "float32 ki\n"
  "float32 kd\n"
  "float32 kff\n"
  "float32 att_p\n"
  "\n"
  "float32[3] rate_sp\n"
  "\n"
  "float32 u_filt\n"
  "float32 y_filt\n"
  "\n"
  "uint8 STATE_IDLE = 0\n"
  "uint8 STATE_INIT = 1\n"
  "uint8 STATE_ROLL = 2\n"
  "uint8 STATE_ROLL_PAUSE = 3\n"
  "uint8 STATE_PITCH = 4\n"
  "uint8 STATE_PITCH_PAUSE = 5\n"
  "uint8 STATE_YAW = 6\n"
  "uint8 STATE_YAW_PAUSE = 7\n"
  "uint8 STATE_VERIFICATION = 8\n"
  "uint8 STATE_APPLY = 9\n"
  "uint8 STATE_TEST = 10\n"
  "uint8 STATE_COMPLETE = 11\n"
  "uint8 STATE_FAIL = 12\n"
  "uint8 STATE_WAIT_FOR_DISARM = 13\n"
  "\n"
  "uint8 state";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__AutotuneAttitudeControlStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__AutotuneAttitudeControlStatus__TYPE_NAME, 42, 42},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 818, 818},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__AutotuneAttitudeControlStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__AutotuneAttitudeControlStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
