# SeaWings System

A comprehensive ROS2-based monitoring system for PX4-powered fixed-wing craft, providing triple-redundant flight safety through battery management, sensor health monitoring, and mission supervision.

##  Overview

The system consists of three independent but coordinated nodes that ensure safe autonomous flight operations:

- **Power Monitor**: Manages battery safety and return-to-launch decisions based on power availability
- **Fault Detector**: Monitors sensor health and system integrity
- **Mission Supervisor**: Oversees mission execution quality and progress

Each node operates independently with clear separation of duties, preventing conflicting commands while ensuring comprehensive safety coverage.

##  Key Features

- **Multi-layered safety**: Three independent nodes monitoring different aspects
- **Intelligent RTL triggering**: Based on battery, time, sensors, or mission issues  
- **SITL/Real craft compatibility**: Automatic threshold adjustments
- **WIG effect tracking**: Monitors ground effect efficiency (informational)
- **Session statistics**: Tracks performance across multiple missions
- **Graceful coordination**: Nodes respect each other's decisions and PX4 failsafes

  In the diagrams directory there ara diagrams explaining the stracture of each node and also the way that the whole system works. At some points there may be some minor changes due to node optimisation.

##  Prerequisites


- ROS 2 Jazzy
- PX4 main version
- QGroundControl
- MicroXRCE-DDS agent
- px4.msgs for main px4 version



## Usage *(after colcon build command in sw_ws direcotry)

### Basic Launch Sequence

1. **Terminal 1 - Start PX4 SITL**:
```bash
cd ~/PX4-Autopilot
make px4_sitl gz_rc_cessna
```

2. **Terminal 2 - Start MicroXRCEDDS Agent**:
```bash
MicroXRCEAgent udp4 -p 8888
```

3. **Terminal 3 - Launch all safety nodes**:
```bash
source ~/sw_ws/install/setup.bash
ros2 launch seawings_mission_management seawings_mission.launch.py
```

Or launch nodes individually:

```bash
# Terminal 3 - Power Monitor
ros2 run seawings_mission_management power_monitor --ros-args --params-file ~/sw_ws/src/seawings_mission_management/config/missiom_params.yaml

# Terminal 4 - Fault Detector  
ros2 run seawings_mission_management fault_detector --ros-args --params-file ~/sw_ws/src/seawings_mission_management/config/mission_params.yaml

# Terminal 5 - Mission Supervisor
ros2 run seawings_mission_management mission_supervisor --ros-args --params-file ~/sw_ws/src/seawings_mission_management/config/mission_params.yaml
```

4. **Start QGroundControl** and connect to vehicle

### Creating a Test Mission

1. In QGroundControl:
   - Click on "Plan" tab
   - Create waypoints 
   - Set appropriate altitudes
   - Upload mission to vehicle
   - Switch to "Fly" tab

2. Execute mission:
   - Arm vehicle (slide to arm)
   - Takeoff in Position mode
   - Switch to Mission mode
   - Monitor node outputs

### Switching Between SITL and Real Craft

1. **For SITL** (simulation):
```yaml
# In params.yaml, set:
sitl_mode: true
```

2. **For Real Craft**:
```yaml
# In params.yaml, set:
sitl_mode: false
```

Key differences:
- SITL: Relaxed thresholds, extended timeouts, warnings only for some conditions
- Real: Strict thresholds, normal timeouts, full safety actions

## 📊 Node Descriptions

### Power Monitor (`power_monitor`)
Monitors battery status and calculates return-to-launch requirements based on:
- Battery percentage threshold (default 30%)
- Remaining flight time vs return time
- Distance to home with safety margins
- Current consumption averaging
- Multi-mission battery tracking
- WIG effect efficiency monitoring (informational)
- Conservative RTL calculations for safety

### Fault Detector (`fault_detector`)
Monitors system sensors and triggers emergency procedures for:
- GPS failures (satellites, fix quality, jamming)
- IMU inconsistencies
- Magnetometer issues
- Barometer failures
- RC signal quality
- Estimator health
- Consecutive failure counting
- Sensor-specific timeout tracking
- SITL/Real adaptive thresholds

### Mission Supervisor (`mission_supervisor`)
Oversees mission execution quality:
- Waypoint progress and timeouts
- Vehicle stuck detection
- Geofence violations
- Mission duration limits
- Loiter state management
- Mission retry logic
- Comprehensive mission statistics
- Stuck detection using position history

## 🔧 Configuration

All parameters are configured in the yaml file. Key parameters:


### Tuning for Your Aircraft
1. **Battery capacity**: Set actual battery mAh
2. **Return speed**: Set cruise speed for your aircraft  
3. **Wing span**: Set for accurate WIG calculations
4. **Loiter radius**: Match your aircraft's turn radius

## 🧪 Testing & Validation

### Quick Functionality Test
```bash
# Test parameter setting
ros2 param set /power_monitor_multi_mission rtl_battery_remaining 85.0

# Monitor topics
ros2 topic echo /power_monitor/status
ros2 topic echo /fault_detector/status
ros2 topic echo /mission_supervisor/status

```

### Simulated Failure Testing
Since PX4 failure injection may not work, use these alternatives:

1. **Battery failure**: Adjust threshold parameter during flight
2. **GPS failure**: Set min_satellites to unreachable value
3. **Stuck detection**: Create waypoint in unreachable location
4. **Mission timeout**: Set very short timeout parameter


## Extras! Important!

## Modify PX4 code in order to be able to subscribe to these topics
```bash
#In terminal
cd PX4-Autopilot/src/modules/uxrce_dds_client
gedit dds_topics.yaml
---
#Add under publications

  - topic: /fmu/out/mission_result
    type: px4_msgs::msg::MissionResult
  
  - topic: /fmu/out/geofence_result
    type: px4_msgs::msg::GeofenceResult
    
  - topic: /fmu/out/vehicle_imu_status
    type: px4_msgs::msg::VehicleImuStatus
  
  - topic: /fmu/out/sensor_mag
    type: px4_msgs::msg::SensorMag
    
  - topic: /fmu/out/sensor_baro
    type: px4_msgs::msg::SensorBaro
  
  - topic: /fmu/out/vehicle_air_data
    type: px4_msgs::msg::VehicleAirData
  
  - topic: /fmu/out/rc_channels
    type: px4_msgs::msg::RcChannels
    

#Run the following command in terminal
python3 generate_dds_topics.py -m ../../msg -y dds_topics.yaml -u . -t dds_topics.h.em

```

## Modify battery_simulator in order for the SITL to be as realistic as possible
```bash
#In terminal
cd /PX4-Autopilot/src/modules/simulation/battery_simulator
gedit BatterySimulator.cpp

#Comment out this line:
float ibatt = -1.0f; // no current sensor in simulation	

#Instead add this: 
float ibatt = _armed ? 15.0f : 0.0f;  // Fixed 2A when armed, 0A when disarmed



