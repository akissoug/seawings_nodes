// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/EstimatorEventFlags.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/estimator_event_flags__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__EstimatorEventFlags__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x94, 0x3c, 0x1c, 0x96, 0x1c, 0x12, 0x32, 0x4e,
      0x89, 0x4e, 0xb6, 0x01, 0xae, 0x4a, 0xcf, 0x27,
      0xf5, 0xe7, 0x7d, 0xb8, 0x6e, 0xca, 0xe4, 0xf5,
      0x98, 0x2c, 0x0e, 0x23, 0x63, 0x26, 0xd5, 0x59,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__EstimatorEventFlags__TYPE_NAME[] = "px4_msgs/msg/EstimatorEventFlags";

// Define type names, field names, and default values
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__timestamp_sample[] = "timestamp_sample";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__information_event_changes[] = "information_event_changes";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__gps_checks_passed[] = "gps_checks_passed";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_vel_to_gps[] = "reset_vel_to_gps";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_vel_to_flow[] = "reset_vel_to_flow";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_vel_to_vision[] = "reset_vel_to_vision";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_vel_to_zero[] = "reset_vel_to_zero";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_pos_to_last_known[] = "reset_pos_to_last_known";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_pos_to_gps[] = "reset_pos_to_gps";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_pos_to_vision[] = "reset_pos_to_vision";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__starting_gps_fusion[] = "starting_gps_fusion";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__starting_vision_pos_fusion[] = "starting_vision_pos_fusion";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__starting_vision_vel_fusion[] = "starting_vision_vel_fusion";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__starting_vision_yaw_fusion[] = "starting_vision_yaw_fusion";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__yaw_aligned_to_imu_gps[] = "yaw_aligned_to_imu_gps";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_hgt_to_baro[] = "reset_hgt_to_baro";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_hgt_to_gps[] = "reset_hgt_to_gps";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_hgt_to_rng[] = "reset_hgt_to_rng";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_hgt_to_ev[] = "reset_hgt_to_ev";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__warning_event_changes[] = "warning_event_changes";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__gps_quality_poor[] = "gps_quality_poor";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__gps_fusion_timout[] = "gps_fusion_timout";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__gps_data_stopped[] = "gps_data_stopped";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__gps_data_stopped_using_alternate[] = "gps_data_stopped_using_alternate";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__height_sensor_timeout[] = "height_sensor_timeout";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__stopping_navigation[] = "stopping_navigation";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__invalid_accel_bias_cov_reset[] = "invalid_accel_bias_cov_reset";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__bad_yaw_using_gps_course[] = "bad_yaw_using_gps_course";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__stopping_mag_use[] = "stopping_mag_use";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__vision_data_stopped[] = "vision_data_stopped";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__emergency_yaw_reset_mag_stopped[] = "emergency_yaw_reset_mag_stopped";
static char px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__emergency_yaw_reset_gps_yaw_stopped[] = "emergency_yaw_reset_gps_yaw_stopped";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__EstimatorEventFlags__FIELDS[] = {
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__timestamp_sample, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__information_event_changes, 25, 25},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__gps_checks_passed, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_vel_to_gps, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_vel_to_flow, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_vel_to_vision, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_vel_to_zero, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_pos_to_last_known, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_pos_to_gps, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_pos_to_vision, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__starting_gps_fusion, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__starting_vision_pos_fusion, 26, 26},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__starting_vision_vel_fusion, 26, 26},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__starting_vision_yaw_fusion, 26, 26},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__yaw_aligned_to_imu_gps, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_hgt_to_baro, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_hgt_to_gps, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_hgt_to_rng, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__reset_hgt_to_ev, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__warning_event_changes, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__gps_quality_poor, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__gps_fusion_timout, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__gps_data_stopped, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__gps_data_stopped_using_alternate, 32, 32},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__height_sensor_timeout, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__stopping_navigation, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__invalid_accel_bias_cov_reset, 28, 28},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__bad_yaw_using_gps_course, 24, 24},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__stopping_mag_use, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__vision_data_stopped, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__emergency_yaw_reset_mag_stopped, 31, 31},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__EstimatorEventFlags__FIELD_NAME__emergency_yaw_reset_gps_yaw_stopped, 35, 35},
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
px4_msgs__msg__EstimatorEventFlags__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__EstimatorEventFlags__TYPE_NAME, 32, 32},
      {px4_msgs__msg__EstimatorEventFlags__FIELDS, 33, 33},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp                        # time since system start (microseconds)\n"
  "uint64 timestamp_sample                 # the timestamp of the raw data (microseconds)\n"
  "\n"
  "# information events\n"
  "uint32 information_event_changes        # number of information event changes\n"
  "bool gps_checks_passed                  #  0 - true when gps quality checks are passing passed\n"
  "bool reset_vel_to_gps                   #  1 - true when the velocity states are reset to the gps measurement\n"
  "bool reset_vel_to_flow                  #  2 - true when the velocity states are reset using the optical flow measurement\n"
  "bool reset_vel_to_vision                #  3 - true when the velocity states are reset to the vision system measurement\n"
  "bool reset_vel_to_zero                  #  4 - true when the velocity states are reset to zero\n"
  "bool reset_pos_to_last_known            #  5 - true when the position states are reset to the last known position\n"
  "bool reset_pos_to_gps                   #  6 - true when the position states are reset to the gps measurement\n"
  "bool reset_pos_to_vision                #  7 - true when the position states are reset to the vision system measurement\n"
  "bool starting_gps_fusion                #  8 - true when the filter starts using gps measurements to correct the state estimates\n"
  "bool starting_vision_pos_fusion         #  9 - true when the filter starts using vision system position measurements to correct the state estimates\n"
  "bool starting_vision_vel_fusion         # 10 - true when the filter starts using vision system velocity measurements to correct the state estimates\n"
  "bool starting_vision_yaw_fusion         # 11 - true when the filter starts using vision system yaw  measurements to correct the state estimates\n"
  "bool yaw_aligned_to_imu_gps             # 12 - true when the filter resets the yaw to an estimate derived from IMU and GPS data\n"
  "bool reset_hgt_to_baro                  # 13 - true when the vertical position state is reset to the baro measurement\n"
  "bool reset_hgt_to_gps                   # 14 - true when the vertical position state is reset to the gps measurement\n"
  "bool reset_hgt_to_rng                   # 15 - true when the vertical position state is reset to the rng measurement\n"
  "bool reset_hgt_to_ev                    # 16 - true when the vertical position state is reset to the ev measurement\n"
  "\n"
  "# warning events\n"
  "uint32 warning_event_changes            # number of warning event changes\n"
  "bool gps_quality_poor                   #  0 - true when the gps is failing quality checks\n"
  "bool gps_fusion_timout                  #  1 - true when the gps data has not been used to correct the state estimates for a significant time period\n"
  "bool gps_data_stopped                   #  2 - true when the gps data has stopped for a significant time period\n"
  "bool gps_data_stopped_using_alternate   #  3 - true when the gps data has stopped for a significant time period but the filter is able to use other sources of data to maintain navigation\n"
  "bool height_sensor_timeout              #  4 - true when the height sensor has not been used to correct the state estimates for a significant time period\n"
  "bool stopping_navigation                #  5 - true when the filter has insufficient data to estimate velocity and position and is falling back to an attitude, height and height rate mode of operation\n"
  "bool invalid_accel_bias_cov_reset       #  6 - true when the filter has detected bad acceerometer bias state esitmstes and has reset the corresponding covariance matrix elements\n"
  "bool bad_yaw_using_gps_course           #  7 - true when the filter has detected an invalid yaw estimate and has reset the yaw angle to the GPS ground course\n"
  "bool stopping_mag_use                   #  8 - true when the filter has detected bad magnetometer data and is stopping further use of the magnetometer data\n"
  "bool vision_data_stopped                #  9 - true when the vision system data has stopped for a significant time period\n"
  "bool emergency_yaw_reset_mag_stopped    # 10 - true when the filter has detected bad magnetometer data, has reset the yaw to anothter source of data and has stopped further use of the magnetometer data\n"
  "bool emergency_yaw_reset_gps_yaw_stopped # 11 - true when the filter has detected bad GNSS yaw data, has reset the yaw to anothter source of data and has stopped further use of the GNSS yaw data";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__EstimatorEventFlags__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__EstimatorEventFlags__TYPE_NAME, 32, 32},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 4316, 4316},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__EstimatorEventFlags__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__EstimatorEventFlags__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
