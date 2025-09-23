# ELRS Robot Control System

A professional RC-controlled robot using ExpressLRS (ELRS) wireless communication system with differential steering and safety features.

## 🚀 Features

- **Professional RC Control**: 420,000 baud CRSF/ELRS communication
- **16-Channel Support**: Full RC receiver capability with microsecond precision
- **Differential Steering**: Realistic car-like turning behavior
- **Safety Systems**: Motor lock/unlock, link monitoring, failsafe protection
- **Real-time Control**: Low latency wireless control with excellent range
- **Speed Control**: Variable speed adjustment via RC channel
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

## 📻 Channel Mapping

The robot uses 4 main RC channels for control:

| Channel | Function | Low Value (1000) | Center (1500) | High Value (2000) |
|---------|----------|------------------|---------------|-------------------|
| **CH2** | Speed Control | Minimum Speed (60) | Base Speed (60) | Maximum Speed (255) |
| **CH3** | Throttle | Backward | Stop (1472) | Forward |
| **CH4** | Steering | Turn Right | Straight | Turn Left |
| **CH8** | Motor Lock | Locked (No Movement) | - | Unlocked (Movement Enabled) |

### Detailed Channel Behavior

#### Channel 3 - Throttle Control
- **< 1472**: Move backward
- **= 1472**: Stop (deadzone)
- **> 1473**: Move forward

#### Channel 2 - Speed Control
- **< 1500**: Decrease speed (minimum 60)
- **> 1500**: Increase speed (maximum 255)
- Real-time speed adjustment while driving

#### Channel 4 - Steering Control
- **< 1500**: Turn right (differential steering)
- **> 1500**: Turn left (differential steering)
- Combined with throttle for realistic turning while moving

#### Channel 8 - Safety Lock
- **1000**: Motors locked (safety on)
- **2000**: Motors unlocked (ready to drive)
- Emergency safety feature to prevent accidental movement

## 🎮 Operation Modes

### 1. Forward/Backward with Steering
When throttle (CH3) is active:
- **Forward + Left**: Left wheels full speed, right wheels 50% speed
- **Forward + Right**: Right wheels full speed, left wheels 50% speed
- **Backward + Left**: Same differential logic in reverse
- **Backward + Right**: Same differential logic in reverse

### 2. Spot Turning
When throttle (CH3) is centered (1472):
- **Left Turn**: Left wheels 30% forward, right wheels 100% forward
- **Right Turn**: Right wheels 30% forward, left wheels 100% forward
- Enables tight turns without forward/backward movement

### 3. Safety States
- **Motors Locked**: All movement disabled regardless of other inputs
- **Link Down**: Motors automatically stop if ELRS connection lost
- **Failsafe**: Automatic emergency stop on signal loss

## 🛠️ Installation & Setup

### 1. Hardware Assembly
1. Connect L298N motor driver to Arduino pins as shown above
2. Connect DC motors to L298N output terminals
3. Connect ELRS receiver to Serial1 (pins 0/1)
4. Power both Arduino and L298N appropriately

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
Link: UP | Lock: OFF | Dir: FORWARD LEFT | Speed: 180 | CH3:1650 CH2:1750 CH4:1650 CH8:2000
```

### Status Information
- **Link**: ELRS connection status (UP/DOWN)
- **Lock**: Motor safety lock status (ON/OFF)  
- **Dir**: Current movement direction
- **Speed**: Current motor speed (60-255)
- **CH3/CH2/CH4/CH8**: Live channel values

### Debugging Commands
Connect to serial monitor at 115200 baud to see live telemetry.

## ⚙️ Configuration Options

### Speed Adjustment
Modify these constants in `main.cpp`:
```cpp
#define SPEED_MIN 60      // Minimum motor speed
#define SPEED_MAX 255     // Maximum motor speed
int baseSpeed = 60;       // Default base speed
```

### Steering Sensitivity
Adjust differential ratios:
```cpp
// For gradual turns while moving
int rightSpeed = motorSpeed * 0.5;  // 50% = gradual, 30% = tight

// For spot turns
int leftSpeed = speed * 0.3;  // 30% = tight spot turns
```

### Channel Thresholds
Modify trigger points:
```cpp
#define THROTTLE_STOP 1472          // Throttle stop point
#define STEERING_CENTER 1500        // Steering center point
#define MOTOR_LOCKED 1000           // Lock threshold
#define MOTOR_UNLOCKED 2000         // Unlock threshold
```

## 🔧 Troubleshooting

### Common Issues

#### No Movement
1. Check Channel 8 - ensure motor lock is OFF (value ~2000)
2. Verify ELRS link is UP in serial monitor
3. Check power connections to L298N
4. Verify motor connections

#### Erratic Movement
1. Check ELRS signal quality and range
2. Verify channel calibration on transmitter
3. Check for loose connections
4. Monitor serial output for channel values

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

- **Communication**: 420,000 baud ELRS/CRSF
- **Update Rate**: Real-time RC control
- **Range**: Several kilometers (depending on ELRS hardware)
- **Latency**: <10ms typical
- **Channels**: 16 available (4 actively used)
- **Precision**: Microsecond-level channel resolution
- **Memory Usage**: 13.9% RAM, 18.3% Flash

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

### Dependencies
- **AlfredoCRSF** v1.0.1 - ELRS/CRSF protocol library
- **Arduino Framework** for Renesas RA
- **PlatformIO** build system

## 📝 Version History

### v1.0.0 - Initial ELRS Integration
- Basic ELRS channel reception
- Motor control integration
- Safety systems implementation

### v1.1.0 - Improved Steering
- Added differential steering
- Fixed turning behavior
- Inverted steering controls
- Enhanced status monitoring

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

**⚠️ Safety Notice**: Always ensure motors are locked (CH8 = 1000) when testing or making changes. The robot can move unexpectedly if ELRS signal is present and motors are unlocked.