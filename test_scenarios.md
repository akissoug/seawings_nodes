# SeaWings Test scenarios

## Pre-Test Setup
1. **Terminal Layout** :
   - Terminal 1: PX4 SITL
   - Terminal 2: MicroXRCEDDS Agent
   - Terminal 3: Power Monitor 
   - Terminal 4: Fault Detector 
   - Terminal 5: Mission Supervisor 
   - QGroundControl on separate screen/window

2. **QGC Configuration**:
   - Set up a simple rectangular mission (a few waypoints)
   - Set RTL altitude 
   - Set loiter radius 
   - Enable mission upload confirmation

---

## TEST SCENARIO 1: Normal Mission Execution
**Demonstrates:** All nodes working in harmony

### Steps:
1. **Start all nodes** - Show startup messages and parameter display
2. **Arm and takeoff manually** 
3. **Switch to AUTO.MISSION**
   - All nodes should log: "Mission #1 started (Manual to Auto transition)"
4. **Let mission run** for some waypoints
   - Show periodic status logs from all nodes
   - Point out battery monitoring, sensor health, waypoint progress
5. **Complete mission normally**
   - Show mission completion logs
6. **Land and disarm**
   - Show session statistics

### Key Points to Highlight:
- Synchronized mission detection
- No false positives
- Clean separation of duties

---

## TEST SCENARIO 2: Battery-Triggered RTL
**Demonstrates:** Power Monitor's battery management

### Steps:
1. **Modify power_monitor parameters** before starting:
   ```yaml
   rtl_battery_remaining: 90.0  # High threshold for quick trigger
   ```
2. **Start mission normally**
3. **Wait for battery to drop below 85%** (usually 1-2 minutes in SITL)
4. **Observe Power Monitor triggering RTL**:
   - "⚠️ POWER MONITOR RTL NEEDED: battery at threshold"
   - Other nodes log: "RTL detected (triggered by PX4 or other node)"
5. **Watch RTL execution and loiter at home**
6. **Switch back to AUTO.MISSION**

### Key Points:
- Only Power Monitor triggers on battery
- Other nodes respect and log the trigger source
- Mission resume detection works correctly

---

## TEST SCENARIO 3: GPS Failure Simulation (Alternative Methods) 
**Demonstrates:** Fault Detector's sensor monitoring

### Method A - Parameter Manipulation:
1. **Start mission normally**
2. **During flight, use parameter change**:
   ```bash
   ros2 param set /fault_detector min_satellites 20
   ```
3. **This simulates GPS degradation** (can't get 20 satellites)
4. **Watch consecutive failure counting**
5. **After threshold, Fault Detector triggers RTL**

### Method B - QGC GPS Disable:
1. In QGC, go to Vehicle Setup → Parameters
2. Search for GPS or EKF2_GPS_CHECK
3. Disable GPS checks temporarily
4. Monitor Fault Detector response

### Key Points:
- Fault Detector counts failures before acting
- Grace period prevents false positives
- Clear logging of sensor issues

---

## TEST SCENARIO 4: Stuck Detection 
**Demonstrates:** Mission Supervisor's stuck detection

### Steps:
1. **Create a mission with a problematic waypoint**:
   - Set one waypoint acceptance radius very small (1m)
   - Or set one waypoint inside an obstacle
2. **Start mission**
3. **Vehicle will loiter trying to reach waypoint**
4. **After 60 seconds**, Mission Supervisor detects stuck:
   - "⚠️ Vehicle stuck! Moved only Xm in 60s"
   - Triggers RTL
5. **Show statistics with stuck events**

### Alternative - Manual Stuck:
1. During mission, switch to POSITION mode
2. Hold position for 60+ seconds
3. Switch back to AUTO.MISSION
4. Supervisor detects lack of progress

---

## TEST SCENARIO 5: Power moinitor triggers RTL by calculating battery remaing time and battery needed to return home 

**Demonstrates:** RTL and distance calculation

### Steps:
1. **Start mission normally**
2. **Wait until RTL triggers**
3. **Observe Power Monitor triggering RTL**:
   - "⚠️ POWER MONITOR RTL NEEDED..."
   - Other nodes log: "RTL detected (triggered by PX4 or other node)"
4. **Watch RTL execution and loiter at home**

### Key Points:
- Only Power Monitor triggers on battery
- Other nodes respect and log the trigger source
- Correct battery time remaining calculation and comparison with time needed for home return  

---

## TEST SCENARIO 6: Mode Confusion Test 
**Demonstrates:** Proper mode transition handling

### Steps:
1. **Rapid mode switching**:
   - MANUAL → AUTO.MISSION → POSITION → AUTO.MISSION
   - Verify only first transition triggers new mission
   
2. **Waypoint hold vs RTL**:
   - Let mission reach waypoint → AUTO.LOITER
   - Should log "Mission paused - Loitering at waypoint"
   - Not "Mission started"
   
3. **Manual RTL**:
   - Use RTL switch in QGC
   - All nodes detect external RTL
   - Complete RTL → loiter
   - Resume mission (should be new mission #2)

---

## TEST SCENARIO 7: Failsafe Priority Test 
**Demonstrates:** Nodes respecting PX4 failsafes

### Steps:
1. **Trigger PX4 failsafe** (if possible):
   - Set very low battery failsafe in PX4 (if configurable)
   - Or trigger RC failsafe by closing QGC
   
2. **Observe all nodes standing down**:
   - "PX4 failsafe active - standing by"
   - No competing RTL commands
   
3. **Clear failsafe** and continue

---


