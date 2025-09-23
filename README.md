# ELRS Robot Control System

A professional RC-controlled robot using ExpressLRS (ELRS) wireless communication system with differential steering and safety features.

## 🚀 Features

- **Professional RC Control**: 4#### Quick Start with Launcher Script
```bash
# Make the launcher executable (first time only)
chmod +x run_plotter.sh

# Run with live Arduino data
./run_plotter.sh

# Run in demo mode (no Arduino needed)
./run_plotter.sh --demo

# Specify custom serial port
./run_plotter.sh --port /dev/ttyUSB0
```

#### Manual Installation
```bash
# Install Python dependencies
pip install -r requirements.txt

# Or install individually
pip install pyserial matplotlib numpy
```

#### Manual Usage
```bash
# Run the real-time plotter
python3 motor_plotter.py

# Run in demo mode with simulated data (no Arduino needed)
python3 motor_plotter.py --demo

# Specify custom serial port
python3 motor_plotter.py --port /dev/ttyUSB0

# Or make executable and run
chmod +x motor_plotter.py
./motor_plotter.py --demo
```F/ELRS communication
- **16-Channel Support**: Full RC receiver capability with microsecond precision
- **Simulated Mecanum Wheels**: Strafe left/right with differential steering
- **Battery Voltage Monitoring**: Real-time battery voltage display for power diagnostics
- **Differential Steering**: Realistic car-like turning behavior
- **ELRS Reset Protection**: Advanced filtering prevents receiver resets during rapid speed changes
- **Safety Systems**: Motor lock/unlock, link monitoring, failsafe protection
- **Real-time Control**: Low latency wireless control with excellent range
- **Speed Control**: Variable speed adjustment via RC channel with exponential smoothing
- **Status Monitoring**: Real-time telemetry and debugging output

## 📡 Hardware Requirements

### Arduino Board
- **Arduino Uno R4 WiFi** (Renesas RA framework)
- Uses Serial1 (pins 0/1) for ELRS communication
- USB Serial for debugging and status output

### Motor Driver
- **L298N Dual H-Bridge Motor Driver**
- Controls two DC motors independently
- PWM speed control for variable speed

### ELRS System
- **ELRS Receiver Module** (e.g., RadioMaster RP series)
- **ELRS Transmitter** (e.g., RadioMaster TX16S or compatible)
- **Communication**: 420,000 baud serial protocol

## 🔌 Pin Connections

### Motor Driver (L298N)
```
Arduino Pin → L298N Pin → Function
     3      →    ENA     → Left Motor PWM Speed
     4      →    IN1     → Left Motor Direction 1
     5      →    IN2     → Left Motor Direction 2
     6      →    ENB     → Right Motor PWM Speed
     7      →    IN3     → Right Motor Direction 1
     8      →    IN4     → Right Motor Direction 2
```

### ELRS Receiver
```
Arduino Pin → ELRS Receiver → Function
     0      →      TX      → Serial1 RX (Arduino receives)
     1      →      RX      → Serial1 TX (Arduino transmits)
    5V      →      VCC     → Power (5V)
    GND     →      GND     → Ground
```

### Battery Voltage Monitoring
```
Arduino Pin → Voltage Divider → Function
    A0      →   Battery+     → Battery voltage input (voltage divider required)
    GND     →   Battery-     → Ground reference
```

## 📻 Channel Mapping

The robot uses 5 RC channels for complete control:

| Channel | Function | Low Value (1000) | Center (1500) | High Value (2000) |
|---------|----------|------------------|---------------|-------------------|
| **CH1** | Strafe Control | Strafe Left | Center (No Strafe) | Strafe Right |
| **CH2** | Speed Control | Minimum Speed (60) | Base Speed (60) | Maximum Speed (255) |
| **CH3** | Throttle | Backward | Stop (1472) | Forward |
| **CH4** | Steering | Turn Right | Straight | Turn Left |
| **CH8** | Motor Lock | Locked (No Movement) | - | Unlocked (Movement Enabled) |

### Detailed Channel Behavior

#### Channel 1 - Strafe Control (Priority)
- **< 1400**: Strafe left (simulated mecanum wheel behavior)
- **1400-1600**: No strafe (normal throttle/steering control)
- **> 1600**: Strafe right (simulated mecanum wheel behavior)
- **Priority**: Overrides throttle control when active

#### Channel 3 - Throttle Control
- **< 1472**: Move backward
- **= 1472**: Stop (deadzone)
- **> 1473**: Move forward
- **Combined with CH4**: Differential steering while moving

#### Channel 2 - Speed Control
- **< 1500**: Decrease speed (minimum 60)
- **> 1500**: Increase speed (maximum 255)
- **Protection**: Exponential smoothing prevents ELRS resets
- **Rate Limited**: Maximum 15 units change per 100ms update

#### Channel 4 - Steering Control
- **< 1500**: Turn right (differential steering)
- **> 1500**: Turn left (differential steering)
- **Combined with throttle**: Realistic turning while moving

#### Channel 8 - Safety Lock
- **1000**: Motors locked (safety on)
- **2000**: Motors unlocked (ready to drive)
- **Emergency safety**: Prevents accidental movement

## 🎮 Operation Modes

### 1. Strafe Movement (Channel 1 Priority)
When strafe channel (CH1) is active, throttle and steering are ignored:
- **Strafe Left**: Left motor faster forward, right motor slower forward (leftward drift)
- **Strafe Right**: Right motor faster forward, left motor slower forward (rightward drift)
- **Status**: Monitor serial output for "STRAFE LEFT" or "STRAFE RIGHT" direction

### 2. Forward/Backward with Steering
When throttle (CH3) is active and strafe is centered:
- **Forward + Left**: Left wheels full speed, right wheels 50% speed
- **Forward + Right**: Right wheels full speed, left wheels 50% speed
- **Backward + Left**: Same differential logic in reverse
- **Backward + Right**: Same differential logic in reverse
- **Status**: Monitor serial output for "FORWARD LEFT/RIGHT" or "BACKWARD LEFT/RIGHT" direction

### 3. Spot Turning
When throttle (CH3) is centered (1472) and strafe is centered:
- **Left Turn**: Left wheels 30% forward, right wheels 100% forward
- **Right Turn**: Right wheels 30% forward, left wheels 100% forward
- **Status**: Monitor serial output for "TURN LEFT" or "TURN RIGHT" direction

### 4. Stopped State
When throttle is centered and no movement requested:
- **All motors stopped**
- **Status**: Monitor serial output shows no active direction

### 5. Safety States
- **Motors Locked**: All movement disabled regardless of other inputs
- **Link Down**: Motors automatically stop if ELRS connection lost
- **Failsafe**: Automatic emergency stop on signal loss

## ️ ELRS Reset Protection System

Advanced protection prevents ELRS receiver resets during rapid speed changes:

### Protection Features
- **Exponential Smoothing**: Filters speed channel with α=0.1 for smooth transitions
- **Rate Limiting**: Maximum 15 speed units change per 100ms update
- **Dead Zone**: 20-unit dead zone around center prevents jitter
- **Extreme Change Detection**: Warns on changes >300 units with extra recovery time
- **Channel Validation**: Rejects invalid channel values (outside 500-2500 range)

### How It Works
1. **Raw channel values** are exponentially smoothed to prevent sudden jumps
2. **Filtered values** are rate-limited to gradual speed changes
3. **Extreme movements** trigger warnings and extended recovery delays
4. **Invalid data** is rejected to maintain system stability

### Protection Benefits
- **Prevents ELRS resets** during rapid "speed up then backward" movements
- **Maintains stable connection** even with aggressive RC stick movements
- **Smooth speed transitions** improve control feel
- **Automatic recovery** from connection issues

## 🛠️ Installation & Setup

### 1. Hardware Assembly
1. Connect L298N motor driver to Arduino pins as shown above
2. Connect DC motors to L298N output terminals
3. Connect ELRS receiver to Serial1 (pins 0/1)
4. **Optional: Battery Voltage Monitoring**
   - Build voltage divider: Battery+ → 10kΩ → A0 → 3.3kΩ → GND
   - Adjust `VOLTAGE_DIVIDER_RATIO` in code based on your resistor values
   - Ratio = (R1 + R2) / R2, where R1=10kΩ, R2=3.3kΩ, Ratio≈4.0
5. Power both Arduino and L298N appropriately

### 2. Software Installation
```bash
# Clone the repository
git clone <repository-url>
cd wifi-robot-mecanum-whls/firmware

# Install PlatformIO dependencies
pio lib install

# Build and upload firmware
pio run --target upload

# Monitor serial output
pio device monitor --baud 115200
```

### 3. ELRS Binding
1. Put ELRS receiver in bind mode
2. Put ELRS transmitter in bind mode
3. Wait for successful binding
4. Power cycle both devices
5. Verify link status via serial monitor

## 📊 Monitoring & Debugging

### Serial Output
The system provides real-time status updates every 500ms:
```
Link: UP | Lock: OFF | Dir: FORWARD LEFT | Speed: 180 | Batt:7.4V | CH1:1500 CH2:1750 CH3:1650 CH4:1650 CH8:2000 | FilteredCH2:1523
```

### Python Real-time Plotter
For advanced monitoring and debugging, use the included Python plotter:

#### Installation
```bash
# Install Python dependencies
pip install -r requirements.txt

# Or install individually
pip install pyserial matplotlib numpy
```

#### Usage
```bash
# Run the real-time plotter (requires Arduino connected)
python motor_plotter.py

# Run in demo mode with simulated data (no Arduino needed)
python motor_plotter.py --demo

# Specify custom serial port
python motor_plotter.py --port /dev/ttyUSB0

# Or make executable and run
chmod +x motor_plotter.py
./motor_plotter.py --demo
```

#### Plotter Features
- **Motor PWM Visualization**: Real-time left/right motor PWM signals
- **Speed & Battery Monitoring**: Live speed and battery voltage graphs
- **ELRS Status Tracking**: Link status and motor lock indicators
- **Direction Display**: Current movement direction text
- **30-Second History**: Rolling window of recent data
- **Configurable Serial Port**: Edit `SERIAL_PORT` in the script

#### Plot Layout
```
┌─────────────────┬─────────────────┐
│  Motor PWM      │  Speed & Batt   │
│  Signals        │  Voltage        │
├─────────────────┼─────────────────┤
│  ELRS Link &    │  Direction &    │
│  Lock Status    │  Status Text    │
└─────────────────┴─────────────────┘
```

### Status Information
- **Link**: ELRS connection status (UP/DOWN)
- **Lock**: Motor safety lock status (ON/OFF)
- **Dir**: Current movement direction (FORWARD/BACKWARD/STRAFE LEFT/etc.)
- **Speed**: Current motor speed (60-255)
- **Batt**: Battery voltage (e.g., 7.4V) - monitor for power supply issues
- **CH1-CH8**: Live channel values from ELRS receiver
- **FilteredCH2**: Smoothed speed channel value (after exponential filtering)

### Protection Warnings
```
WARNING: Extreme speed change detected! CH2: 1804 (previous: 1500) -> Filtered: 1523 -> Speed: 180
```
- **Triggered** when channel changes exceed 300 units
- **Shows raw vs filtered values** for debugging
- **Indicates protection activation** during rapid movements

## ⚙️ Configuration Options

### Speed Control Parameters
```cpp
#define SPEED_MIN 60           // Minimum motor speed
#define SPEED_MAX 255          // Maximum motor speed
#define SPEED_CENTER 1500      // Speed channel center value
#define SPEED_DEADZONE 20      // Dead zone around center (prevents jitter)
int baseSpeed = 60;            // Default base speed
```

### ELRS Protection Settings
```cpp
#define SPEED_CHANGE_DELAY 100     // Minimum time between speed updates (ms)
#define SPEED_FILTER_ALPHA 0.1     // Exponential smoothing factor (0.0-1.0)
#define FILTER_UPDATE_INTERVAL 20  // Filter update frequency (ms)
#define DIRECTION_CHANGE_DELAY 15  // Minimum delay between direction changes (ms)
#define VOLTAGE_DIVIDER_RATIO 3.0  // Battery voltage divider ratio (adjust for your resistors)
```
// Rate limiting
int maxChange = 15;                // Maximum speed change per update
#define SPEED_DEADZONE 20          // Dead zone around center values
```

### Channel Thresholds
```cpp
#define THROTTLE_BACKWARD_MAX 1472
#define THROTTLE_STOP 1472
#define THROTTLE_FORWARD_MIN 1473
#define STEERING_CENTER 1500
#define STRAFE_CENTER 1500
#define MOTOR_LOCKED 1000
#define MOTOR_UNLOCKED 2000
```

### Movement Ratios
```cpp
// Differential steering ratios
int steeringRatio = 0.5;        // 50% speed reduction for gradual turns
int spotTurnRatio = 0.3;        // 30% speed for tight spot turns
int strafeRatio = 0.3;          // 30% speed for strafe simulation
```

## � Power Supply Solutions

### **⚠️ Critical Issue: ELRS Receiver Power Supply**

The ELRS receiver resets are caused by **power supply voltage drops** during high motor current draw, not software issues. When motors accelerate to full speed (255) then reverse direction, the current spike causes voltage sag that resets the ELRS receiver.

### **🛠️ Hardware Solutions (Recommended)**

#### **Option 1: Separate Power Supplies (Best)**
```
Motor Power (7.2V-12V, 2A+) → L298N Motor Driver
Logic Power (5V regulated) → Arduino + ELRS Receiver
```
- **Benefits**: Complete isolation, no voltage sag on logic circuits
- **Implementation**: Use separate battery or regulated 5V supply for Arduino/ELRS

#### **Option 2: Higher Capacity Battery**
- **Current Battery**: Replace with higher capacity (2000mAh+)
- **Voltage**: Ensure stable 7.2V-12V under load
- **Current Rating**: 3A+ continuous capacity

#### **Option 3: Add Decoupling Capacitors**
```
Arduino 5V → 1000µF electrolytic capacitor
ELRS VCC → 470µF electrolytic capacitor  
Motor Driver → 2200µF electrolytic capacitor
```
- **Placement**: As close as possible to power pins
- **Purpose**: Smooth voltage during current spikes

### **💻 Software Solutions (Implemented)**

#### **Current Limiting**
- **Acceleration Limiting**: Maximum 15 units speed increase per update cycle
- **Direction Change Delay**: 15ms minimum between direction changes
- **Gradual Speed Changes**: Prevents sudden current spikes that cause voltage drops

#### **Protection Features**
```cpp
// Rate limiting in processELRS function
if (millis() - lastSpeedChange > SPEED_CHANGE_DELAY) {
  int maxChange = 15; // Maximum speed change per update
  int speedChange = newTargetSpeed - targetMotorSpeed;
  
  if (abs(speedChange) > maxChange) {
    targetMotorSpeed += (speedChange > 0) ? maxChange : -maxChange;
  }
}

// Direction change protection
if (millis() - lastDirectionChange > DIRECTION_CHANGE_DELAY) {
  // Allow direction change
}
```

## � Troubleshooting

### **🔋 Power Supply Issues (Most Common)**

#### ELRS Receiver Resets During Speed Changes
**Symptoms**: Link goes DOWN when accelerating to max speed then reversing
**Cause**: Power supply voltage sag during motor current spikes
**Solutions**:
1. **Use separate power supplies** for motors vs logic circuits
2. **Add decoupling capacitors** (1000µF+) near power pins
3. **Use higher capacity battery** (2000mAh+, 3A rating)
4. **Check voltage stability** during rapid speed changes

#### USB Cable Dependency
**Symptoms**: Works with USB connected, fails without USB
**Cause**: USB provides additional power/current capacity
**Solutions**:
1. **Use dedicated battery power** with sufficient capacity
2. **Separate power supplies** for motors and logic
3. **Add voltage regulator** for stable 5V to Arduino/ELRS

### **🎮 Control Issues**

#### No Movement
1. Check Channel 8 - ensure motor lock is OFF (value ~2000)
2. Verify ELRS link is UP in serial monitor
3. Check power connections to L298N
4. Verify motor connections
5. Monitor serial output for movement status and direction

#### Erratic Movement
1. **Check channel calibration** on transmitter
2. **Monitor filtered values** - should change smoothly, not jump
3. **Verify power supply stability** during movement
4. **Check for loose motor connections**

#### Strafe Not Working (CH1)
1. Ensure CH1 is assigned in transmitter
2. Check channel values in serial output
3. Strafe overrides throttle - center CH1 for normal driving
4. Monitor serial output for strafe direction status

#### Speed Control Issues
1. Check CH2 values in serial output
2. Monitor "FilteredCH2" and "TargetSpd" values
3. Speed changes should be gradual, not instant
4. Protection may limit rapid changes to prevent ELRS resets

#### Wrong Turning Direction
1. Swap motor connections on one side
2. Or invert steering logic in firmware
3. Verify channel 4 direction in transmitter settings

#### Compilation Errors
```bash
# Clean and rebuild
pio run --target clean
pio lib install
pio run
```

### ELRS Connection Issues
1. Verify baud rate (420,000)
2. Check physical connections (RX/TX swapped?)
3. Ensure receiver is bound and powered
4. Monitor for "Link: UP" in serial output

## 📈 Performance Specifications

- **Communication**: 420,000 baud ELRS/CRSF with reset protection
- **Update Rate**: Real-time RC control with 20ms filter updates
- **Range**: Several kilometers (depending on ELRS hardware)
- **Latency**: <10ms typical + filtering delay
- **Channels**: 16 available (5 actively used)
- **Precision**: Microsecond-level channel resolution
- **Speed Control**: Exponential smoothing with rate limiting
- **Protection**: Automatic ELRS reset prevention
- **Movement Modes**: Forward/Backward/Left/Right/Strafe/Spot Turn
- **Memory Usage**: ~14.0% RAM, ~19.0% Flash (optimized)

## 🔄 Development

### Building from Source
```bash
# Prerequisites
pip install platformio

# Build
pio run

# Upload
pio run --target upload

# Monitor
pio device monitor
```

### Python Development Tools
```bash
# Quick launcher script (recommended)
./run_plotter.sh --demo

# Manual Python execution
pip install -r requirements.txt
python motor_plotter.py --demo
```

### Dependencies
- **AlfredoCRSF** v1.0.1 - ELRS/CRSF protocol library
- **Arduino Framework** for Renesas RA
- **PlatformIO** build system
- **Python Tools**: PySerial, Matplotlib, NumPy (for plotting)
- **Shell Scripts**: `run_plotter.sh` (automated Python environment setup)

## 📝 Version History

### v1.4.0 - Performance Optimization & ELRS Stability Fix
- **Removed Global Speed Ramping**: Eliminated ramping system that was causing ELRS receiver resets due to inconsistent motor timing
- **Removed LED Matrix Code**: Optimized performance by removing visual feedback system
- **Reduced Direction Change Delay**: From 50ms to 15ms for faster response while maintaining stability
- **Consistent Motor Control**: All motor functions now use direct speed values for uniform timing
- **Memory Optimization**: Reduced RAM usage to ~14.0%, Flash to ~19.0%
- **ELRS Stability**: Resolved receiver resets caused by speed ramping timing conflicts

### v1.3.0 - Power Supply Protection & Current Limiting
- **Root Cause Identified**: ELRS resets caused by power supply voltage sag during motor current spikes
- **Hardware Solutions**: Documented separate power supplies, decoupling capacitors, higher capacity batteries
- **Software Protection**: Added current limiting (5 units max acceleration), direction change delays (50ms)
- **Gradual Motor Control**: Prevents sudden current spikes that cause voltage drops
- **Enhanced Troubleshooting**: Power supply diagnostics and testing procedures

### v1.2.0 - Enhanced Protection & Visual Feedback
- Implemented ELRS reset protection with exponential smoothing
- Added simulated mecanum wheel strafing (CH1 control)
- Enhanced speed control with rate limiting and dead zones
- Improved status monitoring with filtered values
- Added extreme change detection and recovery delays

### v1.1.0 - Improved Steering
- Added differential steering
- Fixed turning behavior
- Inverted steering controls
- Enhanced status monitoring

### v1.0.0 - Initial ELRS Integration
- Basic ELRS channel reception
- Motor control integration
- Safety systems implementation

## 🤝 Contributing

1. Fork the repository
2. Create feature branch
3. Make changes
4. Test thoroughly
5. Submit pull request

## 📄 License

This project is open source. See LICENSE file for details.

## 🆘 Support

For issues and questions:
1. Check troubleshooting section
2. Review serial monitor output
3. Verify hardware connections
4. Create GitHub issue with details

---

**⚠️ Safety Notice**: Always ensure motors are locked (CH8 = 1000) when testing or making changes. The robot can move unexpectedly if ELRS signal is present and motors are unlocked. Monitor the serial output for lock status and movement direction.

**🔋 Power Supply Warning**: ELRS receiver resets during rapid speed changes are caused by power supply voltage sag. Use separate power supplies for motors and logic circuits, or add decoupling capacitors to prevent voltage drops. The software includes global speed ramping and current limiting but cannot compensate for inadequate power supplies.