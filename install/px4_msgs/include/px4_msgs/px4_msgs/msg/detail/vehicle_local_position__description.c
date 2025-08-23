// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/VehicleLocalPosition.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/vehicle_local_position__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__VehicleLocalPosition__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd7, 0x95, 0x8e, 0x55, 0x9f, 0x84, 0x34, 0xb5,
      0xaf, 0x0f, 0x4e, 0xe7, 0x31, 0x39, 0xf0, 0x8b,
      0xa5, 0x21, 0x74, 0xf4, 0xdf, 0xae, 0x72, 0x4a,
      0x18, 0xca, 0xa6, 0x08, 0x59, 0x7f, 0x82, 0x0b,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__VehicleLocalPosition__TYPE_NAME[] = "px4_msgs/msg/VehicleLocalPosition";

// Define type names, field names, and default values
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__timestamp_sample[] = "timestamp_sample";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__xy_valid[] = "xy_valid";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__z_valid[] = "z_valid";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__v_xy_valid[] = "v_xy_valid";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__v_z_valid[] = "v_z_valid";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__x[] = "x";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__y[] = "y";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__z[] = "z";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__delta_xy[] = "delta_xy";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__xy_reset_counter[] = "xy_reset_counter";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__delta_z[] = "delta_z";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__z_reset_counter[] = "z_reset_counter";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__vx[] = "vx";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__vy[] = "vy";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__vz[] = "vz";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__z_deriv[] = "z_deriv";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__delta_vxy[] = "delta_vxy";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__vxy_reset_counter[] = "vxy_reset_counter";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__delta_vz[] = "delta_vz";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__vz_reset_counter[] = "vz_reset_counter";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__ax[] = "ax";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__ay[] = "ay";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__az[] = "az";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__heading[] = "heading";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__heading_var[] = "heading_var";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__unaided_heading[] = "unaided_heading";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__delta_heading[] = "delta_heading";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__heading_reset_counter[] = "heading_reset_counter";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__heading_good_for_control[] = "heading_good_for_control";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__tilt_var[] = "tilt_var";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__xy_global[] = "xy_global";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__z_global[] = "z_global";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__ref_timestamp[] = "ref_timestamp";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__ref_lat[] = "ref_lat";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__ref_lon[] = "ref_lon";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__ref_alt[] = "ref_alt";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__dist_bottom[] = "dist_bottom";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__dist_bottom_valid[] = "dist_bottom_valid";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__dist_bottom_sensor_bitfield[] = "dist_bottom_sensor_bitfield";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__eph[] = "eph";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__epv[] = "epv";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__evh[] = "evh";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__evv[] = "evv";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__dead_reckoning[] = "dead_reckoning";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__vxy_max[] = "vxy_max";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__vz_max[] = "vz_max";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__hagl_min[] = "hagl_min";
static char px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__hagl_max[] = "hagl_max";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__VehicleLocalPosition__FIELDS[] = {
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__timestamp_sample, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__xy_valid, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__z_valid, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__v_xy_valid, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__v_z_valid, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__z, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__delta_xy, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__xy_reset_counter, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__delta_z, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__z_reset_counter, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__vx, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__vy, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__vz, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__z_deriv, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__delta_vxy, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__vxy_reset_counter, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__delta_vz, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__vz_reset_counter, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__ax, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__ay, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__az, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__heading, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__heading_var, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__unaided_heading, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__delta_heading, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__heading_reset_counter, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__heading_good_for_control, 24, 24},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__tilt_var, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__xy_global, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__z_global, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__ref_timestamp, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__ref_lat, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__ref_lon, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__ref_alt, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__dist_bottom, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__dist_bottom_valid, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__dist_bottom_sensor_bitfield, 27, 27},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__eph, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__epv, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__evh, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__evv, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__dead_reckoning, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__vxy_max, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__vz_max, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__hagl_min, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleLocalPosition__FIELD_NAME__hagl_max, 8, 8},
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
px4_msgs__msg__VehicleLocalPosition__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__VehicleLocalPosition__TYPE_NAME, 33, 33},
      {px4_msgs__msg__VehicleLocalPosition__FIELDS, 49, 49},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Fused local position in NED.\n"
  "# The coordinate system origin is the vehicle position at the time when the EKF2-module was started.\n"
  "\n"
  "uint64 timestamp\\t\\t\\t# time since system start (microseconds)\n"
  "uint64 timestamp_sample                 # the timestamp of the raw data (microseconds)\n"
  "\n"
  "bool xy_valid\\t\\t\\t\\t# true if x and y are valid\n"
  "bool z_valid\\t\\t\\t\\t# true if z is valid\n"
  "bool v_xy_valid\\t\\t\\t\\t# true if vx and vy are valid\n"
  "bool v_z_valid\\t\\t\\t\\t# true if vz is valid\n"
  "\n"
  "# Position in local NED frame\n"
  "float32 x\\t\\t\\t\\t# North position in NED earth-fixed frame, (metres)\n"
  "float32 y\\t\\t\\t\\t# East position in NED earth-fixed frame, (metres)\n"
  "float32 z\\t\\t\\t\\t# Down position (negative altitude) in NED earth-fixed frame, (metres)\n"
  "\n"
  "# Position reset delta\n"
  "float32[2] delta_xy\\t\\t\\t# Amount of lateral shift of position estimate in latest reset (in x and y) [m]\n"
  "uint8 xy_reset_counter\\t\\t\\t# Index of latest lateral position estimate reset\n"
  "float32 delta_z\\t\\t\\t\\t# Amount of vertical shift of position estimate in latest reset [m]\n"
  "uint8 z_reset_counter\\t\\t\\t# Index of latest vertical position estimate reset\n"
  "\n"
  "# Velocity in NED frame\n"
  "float32 vx \\t\\t\\t\\t# North velocity in NED earth-fixed frame, (metres/sec)\n"
  "float32 vy\\t\\t\\t\\t# East velocity in NED earth-fixed frame, (metres/sec)\n"
  "float32 vz\\t\\t\\t\\t# Down velocity in NED earth-fixed frame, (metres/sec)\n"
  "float32 z_deriv\\t\\t\\t\\t# Down position time derivative in NED earth-fixed frame, (metres/sec)\n"
  "\n"
  "# Velocity reset delta\n"
  "float32[2] delta_vxy\\t\\t\\t# Amount of lateral shift of velocity estimate in latest reset (in x and y) [m/s]\n"
  "uint8 vxy_reset_counter\\t\\t\\t# Index of latest vertical velocity estimate reset\n"
  "float32 delta_vz\\t\\t\\t# Amount of vertical shift of velocity estimate in latest reset [m/s]\n"
  "uint8 vz_reset_counter\\t\\t\\t# Index of latest vertical velocity estimate reset\n"
  "\n"
  "# Acceleration in NED frame\n"
  "float32 ax        # North velocity derivative in NED earth-fixed frame, (metres/sec^2)\n"
  "float32 ay        # East velocity derivative in NED earth-fixed frame, (metres/sec^2)\n"
  "float32 az        # Down velocity derivative in NED earth-fixed frame, (metres/sec^2)\n"
  "\n"
  "float32 heading\\t\\t\\t\\t# Euler yaw angle transforming the tangent plane relative to NED earth-fixed frame, -PI..+PI,  (radians)\n"
  "float32 heading_var\n"
  "float32 unaided_heading                 # Same as heading but generated by integrating corrected gyro data only\n"
  "float32 delta_heading\\t\\t\\t# Heading delta caused by latest heading reset [rad]\n"
  "uint8 heading_reset_counter\\t\\t# Index of latest heading reset\n"
  "bool heading_good_for_control\n"
  "\n"
  "float32 tilt_var\n"
  "\n"
  "# Position of reference point (local NED frame origin) in global (GPS / WGS84) frame\n"
  "bool xy_global\\t\\t\\t\\t# true if position (x, y) has a valid global reference (ref_lat, ref_lon)\n"
  "bool z_global\\t\\t\\t\\t# true if z has a valid global reference (ref_alt)\n"
  "uint64 ref_timestamp\\t\\t\\t# Time when reference position was set since system start, (microseconds)\n"
  "float64 ref_lat\\t\\t\\t\\t# Reference point latitude, (degrees)\n"
  "float64 ref_lon\\t\\t\\t\\t# Reference point longitude, (degrees)\n"
  "float32 ref_alt\\t\\t\\t\\t# Reference altitude AMSL, (metres)\n"
  "\n"
  "# Distance to surface\n"
  "float32 dist_bottom\\t\\t\\t# Distance from from bottom surface to ground, (metres)\n"
  "bool dist_bottom_valid\\t\\t\\t# true if distance to bottom surface is valid\n"
  "uint8 dist_bottom_sensor_bitfield\\t# bitfield indicating what type of sensor is used to estimate dist_bottom\n"
  "uint8 DIST_BOTTOM_SENSOR_NONE = 0\n"
  "uint8 DIST_BOTTOM_SENSOR_RANGE = 1\\t# (1 << 0) a range sensor is used to estimate dist_bottom field\n"
  "uint8 DIST_BOTTOM_SENSOR_FLOW = 2\\t# (1 << 1) a flow sensor is used to estimate dist_bottom field (mostly fixed-wing use case)\n"
  "\n"
  "float32 eph\\t\\t\\t\\t# Standard deviation of horizontal position error, (metres)\n"
  "float32 epv\\t\\t\\t\\t# Standard deviation of vertical position error, (metres)\n"
  "float32 evh\\t\\t\\t\\t# Standard deviation of horizontal velocity error, (metres/sec)\n"
  "float32 evv\\t\\t\\t\\t# Standard deviation of vertical velocity error, (metres/sec)\n"
  "\n"
  "bool dead_reckoning                     # True if this position is estimated through dead-reckoning\n"
  "\n"
  "# estimator specified vehicle limits\n"
  "float32 vxy_max\\t\\t\\t\\t# maximum horizontal speed - set to 0 when limiting not required (meters/sec)\n"
  "float32 vz_max\\t\\t\\t\\t# maximum vertical speed - set to 0 when limiting not required (meters/sec)\n"
  "float32 hagl_min\\t\\t\\t# minimum height above ground level - set to 0 when limiting not required (meters)\n"
  "float32 hagl_max\\t\\t\\t# maximum height above ground level - set to 0 when limiting not required (meters)\n"
  "\n"
  "# TOPICS vehicle_local_position vehicle_local_position_groundtruth external_ins_local_position\n"
  "# TOPICS estimator_local_position";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__VehicleLocalPosition__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__VehicleLocalPosition__TYPE_NAME, 33, 33},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 4506, 4506},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__VehicleLocalPosition__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__VehicleLocalPosition__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
