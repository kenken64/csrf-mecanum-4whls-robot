#include <AlfredoCRSF.h>

// L298N motor driver pin connections
#define ENA 3              // Left motor PWM
#define IN1 4              // Left motor direction 1
#define IN2 5              // Left motor direction 2
#define ENB 6              // Right motor PWM
#define IN3 7              // Right motor direction 1
#define IN4 8              // Right motor direction 2

#define PIN_RX 0
#define PIN_TX 1
#define BATTERY_PIN A0      // Analog pin for battery voltage monitoring

AlfredoCRSF crsf;

// Robot control variables
int motorSpeed = 60;       // Last applied speed (PWM magnitude 60-255)
int targetMotorSpeed = 60; // Target speed after filtering/ramping
int baseSpeed = 60;        // Minimum commanded speed when moving
int previousSpeed = 60;    // For debugging/rate checks
bool motorsLocked = true;  // Channel 8 motor lock state

// Direction enumeration replaces dynamic String usage to avoid fragmentation
enum Direction : uint8_t {
  DIR_STOPPED,
  DIR_FORWARD,
  DIR_BACKWARD,
  DIR_FORWARD_LEFT,
  DIR_FORWARD_RIGHT,
  DIR_BACKWARD_LEFT,
  DIR_BACKWARD_RIGHT,
  DIR_DRIFT_LEFT,
  DIR_DRIFT_RIGHT,
  DIR_TURN_LEFT,
  DIR_TURN_RIGHT,
  DIR_NO_LINK,
  DIR_LOCKED
};

Direction currentDirection = DIR_STOPPED;
Direction previousDirection = DIR_STOPPED;

const char* directionToString(Direction d) {
  switch(d) {
    case DIR_STOPPED: return "STOPPED";
    case DIR_FORWARD: return "FORWARD";
    case DIR_BACKWARD: return "BACKWARD";
    case DIR_FORWARD_LEFT: return "FORWARD LEFT";
    case DIR_FORWARD_RIGHT: return "FORWARD RIGHT";
    case DIR_BACKWARD_LEFT: return "BACKWARD LEFT";
    case DIR_BACKWARD_RIGHT: return "BACKWARD RIGHT";
    case DIR_DRIFT_LEFT: return "DRIFT LEFT";
    case DIR_DRIFT_RIGHT: return "DRIFT RIGHT";
    case DIR_TURN_LEFT: return "TURN LEFT";
    case DIR_TURN_RIGHT: return "TURN RIGHT";
    case DIR_NO_LINK: return "NO LINK";
    case DIR_LOCKED: return "LOCKED";
    default: return "UNKNOWN";
  }
}

// Direction change tracking to prevent current spikes
unsigned long lastDirectionChange = 0;
#define DIRECTION_CHANGE_DELAY 15  // ms guard between directional state changes

// Battery voltage monitoring
#define VOLTAGE_DIVIDER_RATIO 3.0  // (R1+R2)/R2 from physical divider
float batteryVoltage = 0.0f;
// Low battery failsafe thresholds (example for 2S LiPo; adjust as needed)
#define LOW_BATTERY_WARN 7.2f     // Warning threshold (V)
#define LOW_BATTERY_LIMIT 6.8f    // Enter throttled mode (V)
#define LOW_BATTERY_CUTOFF 6.6f   // Hard cutoff stop (V)
#define LOW_BATTERY_RELEASE 7.0f  // Hysteresis release voltage (V)
bool lowBatteryWarned = false;
bool lowBatteryThrottled = false;
bool lowBatteryCutoff = false;

// UNO R4 has a 12-bit ADC (0-4095). If using a different board adjust ADC_MAX.
#ifndef ADC_MAX
#define ADC_MAX 4095.0f
#endif
#ifndef ADC_VREF
#define ADC_VREF 5.0f
#endif

// Simple EMA filter for battery voltage
float batteryVoltageFiltered = 0.0f;
const float BATTERY_FILTER_ALPHA = 0.1f; // 0<alpha<=1

float readBatteryVoltage() {
  int raw = analogRead(BATTERY_PIN);
  float v = (raw / ADC_MAX) * ADC_VREF * VOLTAGE_DIVIDER_RATIO;
  if (batteryVoltageFiltered == 0.0f) {
    batteryVoltageFiltered = v;
  } else {
    batteryVoltageFiltered += BATTERY_FILTER_ALPHA * (v - batteryVoltageFiltered);
  }
  return batteryVoltageFiltered;
}

// Channel change detection
int previousSpeedCh = 1500;
unsigned long lastSpeedChange = 0;
#define SPEED_CHANGE_DELAY 100  // Increased from 50ms to 100ms for better protection
#define SPEED_FILTER_ALPHA 0.1  // Exponential smoothing factor (lower = more smoothing)
#define SPEED_DEADZONE 20       // Dead zone around center to prevent jitter

// Speed filtering variables
float filteredSpeedCh = 1500.0;  // Filtered channel value
unsigned long lastFilterUpdate = 0;
#define FILTER_UPDATE_INTERVAL 20  // ms between filter updates

// Channel mapping constants
#define CH_STRAFE 1        // Channel 1: Strafe left/right (mecanum wheels)
#define CH_SPEED 2         // Channel 2: Speed control
#define CH_THROTTLE 3      // Channel 3: Throttle (backward/stop/forward)
#define CH_STEERING 4      // Channel 4: Steering (left/right)
#define CH_LOCK 8          // Channel 8: Motor lock/unlock

// Throttle thresholds
#define THROTTLE_BACKWARD_MAX 1472
#define THROTTLE_STOP 1472
#define THROTTLE_FORWARD_MIN 1473

// Speed control
#define SPEED_CENTER 1500
#define SPEED_MIN 60
#define SPEED_MAX 255

// Integer scaling factors (replaces floating multipliers)
#define SCALE_30(percentVal) ((percentVal) * 30 / 100) // 30% of value
#define SCALE_50(percentVal) ((percentVal) >> 1)       // 50% of value

// Steering
#define STEERING_CENTER 1500

// Strafe (Channel 1)
#define STRAFE_CENTER 1500

// Motor lock
#define MOTOR_LOCKED 1000
#define MOTOR_UNLOCKED 2000

// Function declarations
void moveForward(int speed);
void moveBackward(int speed);
void strafeLeft(int speed);   // Simulated strafe left (drift)
void strafeRight(int speed);  // Simulated strafe right (drift)
void turnLeft(int speed);
void turnRight(int speed);
void stopMotors();
void processELRS();
void printChannels();
void printStatus();

void setup()
{
  Serial.begin(115200);
  Serial.println("=== ELRS Robot Control ===");
  Serial.println("COM Serial initialized");
  
  // Initialize motor control pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Initialize battery voltage monitoring pin
  pinMode(BATTERY_PIN, INPUT);

  // Start with motors stopped and locked
  stopMotors();
  
  // Use Serial1 for CRSF communication
  Serial1.begin(CRSF_BAUDRATE);
  
  crsf.begin(Serial1);
  
  Serial.println("Channel Mapping:");
  Serial.println("  CH1: Strafe (>1500=right drift, <1500=left drift) - Simulated");
  Serial.println("  CH2: Speed (1500=base, higher=faster, lower=slower)");
  Serial.println("  CH3: Throttle (<1472=backward, 1472=stop, >1473=forward)");
  Serial.println("  CH4: Steering (>1500=left, <1500=right)");
  Serial.println("  CH8: Motor Lock (1000=locked, 2000=unlocked)");
  Serial.println("===============================");
}

void loop()
{
    // Must call crsf.update() in loop() to process data
    crsf.update();
    
    // Process ELRS control
    processELRS();
    
    // Print debug info every 500ms
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 500) {
        printStatus();
        lastDebug = millis();
    }
}

// Motor control functions with global speed ramping
void moveForward(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);
}

void moveBackward(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speed);
}

// Simulated mecanum wheel strafing functions for 2-motor setup
void strafeRight(int speed) {
  // Simulated strafe right: Rotate right with slight forward bias
  // Left motor faster forward, right motor slower forward
  int leftSpeed = speed;
  int rightSpeed = SCALE_30(speed);  // Much slower right side creates rightward drift
  
  digitalWrite(IN1, HIGH);  // Left motor forward (faster)
  digitalWrite(IN2, LOW);
  analogWrite(ENA, leftSpeed);

  digitalWrite(IN3, HIGH);  // Right motor forward (slower)
  digitalWrite(IN4, LOW);
  analogWrite(ENB, rightSpeed);
}

void strafeLeft(int speed) {
  // Simulated strafe left: Rotate left with slight forward bias  
  // Right motor faster forward, left motor slower forward
  int leftSpeed = SCALE_30(speed);  // Much slower left side creates leftward drift
  int rightSpeed = speed;
  
  digitalWrite(IN1, HIGH);  // Left motor forward (slower)
  digitalWrite(IN2, LOW);
  analogWrite(ENA, leftSpeed);

  digitalWrite(IN3, HIGH);  // Right motor forward (faster)
  digitalWrite(IN4, LOW);
  analogWrite(ENB, rightSpeed);
}

// Pivot turning (in-place): one motor forward, one backward
void turnLeft(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speed);
}

void turnRight(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}

// ELRS processing function
void processELRS() {
  // Check if link is up and handle connection issues
  if (!crsf.isLinkUp()) {
    stopMotors();
    currentDirection = DIR_NO_LINK;
    previousSpeedCh = 1500;
    filteredSpeedCh = 1500.0f;
    targetMotorSpeed = baseSpeed;
    previousSpeed = baseSpeed;
    // Throttle link-loss logging (every 2s max)
    static unsigned long lastLossLog=0; 
    if (millis() - lastLossLog > 2000) {
      Serial.println("ELRS Link Lost - Resetting speed values");
      lastLossLog = millis();
    }
    return;
  }

  // Stale frame watchdog: if channels not changing for extended period while link claims up, could indicate freeze
  static unsigned long lastFrameActivity = 0;
  bool frameActive = false; // set true when we detect any channel deviation > threshold

  // Get channel values with validation
  // Snapshot channels once for consistency
  int strafe = crsf.getChannel(CH_STRAFE);
  int speedCh = crsf.getChannel(CH_SPEED);
  int throttle = crsf.getChannel(CH_THROTTLE);
  int steering = crsf.getChannel(CH_STEERING);
  int lockCh = crsf.getChannel(CH_LOCK);

  // Detect activity
  if (abs(speedCh - previousSpeedCh) > 5) frameActive = true;
  if (frameActive) lastFrameActivity = millis();
  else if (millis() - lastFrameActivity > 1500) {
    // Failsafe if stale >1.5s
    stopMotors();
    currentDirection = DIR_STOPPED;
    // Don't return immediately; still allow battery processing, but freeze motion
    return;
  }

  // Validate channel values (CRSF should be 1000-2000)
  bool invalid = false;
  if (speedCh < 800 || speedCh > 2200) { invalid = true; }
  if (throttle < 800 || throttle > 2200) { invalid = true; }
  if (invalid) {
    // Fail-safe: stop motion but keep processing lock + battery
    stopMotors();
    currentDirection = DIR_STOPPED;
    return; // Keep last good filtered values
  }

  // Update motor lock status
  if (lockCh >= 1900) {  // Channel 8 = 2000 (unlocked)
    motorsLocked = false;
  } else if (lockCh <= 1100) {  // Channel 8 = 1000 (locked)
    motorsLocked = true;
  }

  // If motors are locked, stop and return
  if (motorsLocked) {
    stopMotors();
    currentDirection = DIR_LOCKED;
    return;
  }

  // Battery voltage & failsafe state update
  batteryVoltage = readBatteryVoltage();
  if (!lowBatteryCutoff) {
    if (batteryVoltage <= LOW_BATTERY_CUTOFF) {
      lowBatteryCutoff = true;
      Serial.println("LOW BATTERY CUTOFF - Motors disabled");
      stopMotors();
    } else if (batteryVoltage <= LOW_BATTERY_LIMIT) {
      if (!lowBatteryThrottled) {
        Serial.println("LOW BATTERY - Throttling speed");
      }
      lowBatteryThrottled = true;
    } else if (batteryVoltage <= LOW_BATTERY_WARN) {
      if (!lowBatteryWarned) {
        Serial.println("LOW BATTERY WARNING");
        lowBatteryWarned = true;
      }
    } else if (batteryVoltage >= LOW_BATTERY_RELEASE) {
      // Recover flags when voltage rises sufficiently (e.g., after load removed)
      if (lowBatteryThrottled || lowBatteryWarned) {
        Serial.println("Battery level recovered - exiting low battery mode");
      }
      lowBatteryWarned = false;
      lowBatteryThrottled = false;
      // Cutoff stays latched until reset (safety) unless you wish to auto-clear:
      // lowBatteryCutoff = false; // uncomment if auto clear desired
    }
  }
  if (lowBatteryCutoff) {
    stopMotors();
    currentDirection = DIR_STOPPED;
    return; // Hard stop
  }

  // Apply exponential smoothing to speed channel to prevent rapid changes
  // Update filter at regular intervals
  if (millis() - lastFilterUpdate > FILTER_UPDATE_INTERVAL) {
    // Apply exponential smoothing: new_value = alpha * raw + (1-alpha) * previous
    filteredSpeedCh = SPEED_FILTER_ALPHA * speedCh + (1.0 - SPEED_FILTER_ALPHA) * filteredSpeedCh;
    lastFilterUpdate = millis();
  }

  // Use filtered value for speed calculation
  int smoothedSpeedCh = round(filteredSpeedCh);

  // Check for dead zone around center to prevent jitter
  if (abs(smoothedSpeedCh - SPEED_CENTER) < SPEED_DEADZONE) {
    smoothedSpeedCh = SPEED_CENTER;
  }

  // Calculate target speed from smoothed channel value
  int newTargetSpeed;
  if (smoothedSpeedCh > SPEED_CENTER) {
    // Above center - increase speed
    newTargetSpeed = map(smoothedSpeedCh, SPEED_CENTER, 2000, baseSpeed, SPEED_MAX);
  } else {
    // Below center - decrease speed
    newTargetSpeed = map(smoothedSpeedCh, 1000, SPEED_CENTER, SPEED_MIN, baseSpeed);
  }
  newTargetSpeed = constrain(newTargetSpeed, SPEED_MIN, SPEED_MAX);
  if (lowBatteryThrottled) {
    // Scale down target speed (e.g., 60%) during throttle mode
    newTargetSpeed = (newTargetSpeed * 60) / 100;
    if (newTargetSpeed < SPEED_MIN) newTargetSpeed = SPEED_MIN;
  }

  // Update target speed with rate limiting
  if (millis() - lastSpeedChange > SPEED_CHANGE_DELAY) {
    // Limit speed change rate to prevent sudden jumps that could reset ELRS
    int maxChange = 15; // Reduced from 30 to 15 for more protection
    int speedChange = newTargetSpeed - targetMotorSpeed;

    if (abs(speedChange) > maxChange) {
      if (speedChange > 0) {
        targetMotorSpeed += maxChange;
      } else {
        targetMotorSpeed -= maxChange;
      }
    } else {
      targetMotorSpeed = newTargetSpeed;
    }

    // Ensure motor speed stays within bounds
    targetMotorSpeed = constrain(targetMotorSpeed, SPEED_MIN, SPEED_MAX);
  int oldApplied = motorSpeed;
  motorSpeed = targetMotorSpeed; // desired new applied speed
    lastSpeedChange = millis();

    // Detect and warn about rapid channel changes that could cause ELRS resets
    int channelChange = abs(speedCh - previousSpeedCh);
    if (channelChange > 300) { // extreme raw delta
      Serial.print("WARNING: Extreme speed change detected! CH2: ");
      Serial.print(speedCh);
      Serial.print(" (previous: ");
      Serial.print(previousSpeedCh);
      Serial.print(") -> Filtered: ");
      Serial.print(smoothedSpeedCh);
      Serial.print(" -> Speed: ");
      Serial.println(motorSpeed);
    }

    previousSpeedCh = speedCh;
  }

  // If speed changed but direction didn't update (e.g., staying FORWARD) refresh PWM outputs
  static int lastAppliedPwm = -1;
  if (motorSpeed != lastAppliedPwm) {
    switch(currentDirection) {
      case DIR_FORWARD: moveForward(motorSpeed); break;
      case DIR_BACKWARD: moveBackward(motorSpeed); break;
      case DIR_DRIFT_LEFT: strafeLeft(motorSpeed); break;
      case DIR_DRIFT_RIGHT: strafeRight(motorSpeed); break;
      case DIR_FORWARD_LEFT:
      case DIR_FORWARD_RIGHT:
      case DIR_BACKWARD_LEFT:
      case DIR_BACKWARD_RIGHT:
      case DIR_TURN_LEFT:
      case DIR_TURN_RIGHT:
      case DIR_STOPPED:
      case DIR_NO_LINK:
      case DIR_LOCKED:
      default: /* specific branches already set outputs */ break;
    }
    lastAppliedPwm = motorSpeed;
  }

  // First check for strafe movement (Channel 1) - priority over throttle
  if (strafe > STRAFE_CENTER + 100) {
    // Strafe right (simulated with forward + right turn)
    if (currentDirection != DIR_DRIFT_RIGHT && millis() - lastDirectionChange > DIRECTION_CHANGE_DELAY) {
      strafeRight(motorSpeed);
      currentDirection = DIR_DRIFT_RIGHT;
      previousDirection = currentDirection;
      lastDirectionChange = millis();
      return;
    }
  } else if (strafe < STRAFE_CENTER - 100) {
    // Strafe left (simulated with forward + left turn)
    if (currentDirection != DIR_DRIFT_LEFT && millis() - lastDirectionChange > DIRECTION_CHANGE_DELAY) {
      strafeLeft(motorSpeed);
      currentDirection = DIR_DRIFT_LEFT;
      previousDirection = currentDirection;
      lastDirectionChange = millis();
      return;
    }
  }

  // Process throttle (Channel 3) if no strafe movement
  if (throttle < THROTTLE_BACKWARD_MAX) {
    // Move backward with steering
    if (steering > STEERING_CENTER + 100) {
      // Backward + Left steering (right side slower) - INVERTED
      if (currentDirection != DIR_BACKWARD_LEFT && millis() - lastDirectionChange > DIRECTION_CHANGE_DELAY) {
  int leftSpeed = motorSpeed;
  int rightSpeed = SCALE_50(motorSpeed);  // Right side slower for left turn

        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(ENA, leftSpeed);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENB, rightSpeed);
  currentDirection = DIR_BACKWARD_LEFT;
        previousDirection = currentDirection;
        lastDirectionChange = millis();
      }
    } else if (steering < STEERING_CENTER - 100) {
      // Backward + Right steering (left side slower) - INVERTED
      if (currentDirection != DIR_BACKWARD_RIGHT && millis() - lastDirectionChange > DIRECTION_CHANGE_DELAY) {
  int leftSpeed = SCALE_50(motorSpeed);   // Left side slower for right turn
        int rightSpeed = motorSpeed;

        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(ENA, leftSpeed);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENB, rightSpeed);
  currentDirection = DIR_BACKWARD_RIGHT;
        previousDirection = currentDirection;
        lastDirectionChange = millis();
      }
    } else {
      // Straight backward
      if (currentDirection != DIR_BACKWARD && millis() - lastDirectionChange > DIRECTION_CHANGE_DELAY) {
        moveBackward(motorSpeed);
        currentDirection = DIR_BACKWARD;
        previousDirection = currentDirection;
        lastDirectionChange = millis();
      }
    }
  } else if (throttle > THROTTLE_FORWARD_MIN) {
    // Move forward with steering
    if (steering > STEERING_CENTER + 100) {
      // Forward + Left steering (right side slower) - INVERTED
      if (currentDirection != DIR_FORWARD_LEFT && millis() - lastDirectionChange > DIRECTION_CHANGE_DELAY) {
  int leftSpeed = motorSpeed;
  int rightSpeed = SCALE_50(motorSpeed);  // Right side slower for left turn

        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, leftSpeed);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENB, rightSpeed);
  currentDirection = DIR_FORWARD_LEFT;
        previousDirection = currentDirection;
        lastDirectionChange = millis();
      }
    } else if (steering < STEERING_CENTER - 100) {
      // Forward + Right steering (left side slower) - INVERTED
      if (currentDirection != DIR_FORWARD_RIGHT && millis() - lastDirectionChange > DIRECTION_CHANGE_DELAY) {
  int leftSpeed = SCALE_50(motorSpeed);   // Left side slower for right turn
        int rightSpeed = motorSpeed;

        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, leftSpeed);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENB, rightSpeed);
  currentDirection = DIR_FORWARD_RIGHT;
        previousDirection = currentDirection;
        lastDirectionChange = millis();
      }
    } else {
      // Straight forward
      if (currentDirection != DIR_FORWARD && millis() - lastDirectionChange > DIRECTION_CHANGE_DELAY) {
        moveForward(motorSpeed);
        currentDirection = DIR_FORWARD;
        previousDirection = currentDirection;
        lastDirectionChange = millis();
      }
    }
  } else {
    // Throttle at center - check steering only for spot turns
    if (steering > STEERING_CENTER + 100) {
      // Turn left in place - INVERTED
      if (currentDirection != DIR_TURN_LEFT && millis() - lastDirectionChange > DIRECTION_CHANGE_DELAY) {
        turnLeft(motorSpeed);
        currentDirection = DIR_TURN_LEFT;
        previousDirection = currentDirection;
        lastDirectionChange = millis();
      }
    } else if (steering < STEERING_CENTER - 100) {
      // Turn right in place - INVERTED
      if (currentDirection != DIR_TURN_RIGHT && millis() - lastDirectionChange > DIRECTION_CHANGE_DELAY) {
        turnRight(motorSpeed);
        currentDirection = DIR_TURN_RIGHT;
        previousDirection = currentDirection;
        lastDirectionChange = millis();
      }
    } else {
      // Stop
      if (currentDirection != DIR_STOPPED && millis() - lastDirectionChange > DIRECTION_CHANGE_DELAY) {
        stopMotors();
        currentDirection = DIR_STOPPED;
        previousDirection = currentDirection;
        lastDirectionChange = millis();
      }
    }
  }
}

//Use crsf.getChannel(x) to get us channel values (1-16).
void printChannels()
{
  for (int ChannelNum = 1; ChannelNum <= 16; ChannelNum++)
  {
    Serial.print(crsf.getChannel(ChannelNum));
    Serial.print(", ");
  }
  Serial.println(" ");
}

void printStatus() {
  // Read battery voltage
  batteryVoltage = readBatteryVoltage();
  
  Serial.print("Link: ");
  Serial.print(crsf.isLinkUp() ? "UP" : "DOWN");
  Serial.print(" | Lock: ");
  Serial.print(motorsLocked ? "ON" : "OFF");
  Serial.print(" | Dir: ");
  Serial.print(directionToString(currentDirection));
  Serial.print(" | Speed: ");
  Serial.print(motorSpeed);
  Serial.print(" | Batt:");
  Serial.print(batteryVoltage, 1);
  Serial.print("V");
  Serial.print(" | CH1:");
  Serial.print(crsf.getChannel(1));
  Serial.print(" CH2:");
  Serial.print(crsf.getChannel(2));
  Serial.print(" CH3:");
  Serial.print(crsf.getChannel(3));
  Serial.print(" CH4:");
  Serial.print(crsf.getChannel(4));
  Serial.print(" CH8:");
  Serial.print(crsf.getChannel(8));

  // Add filtered speed info for debugging
  Serial.print(" | FilteredCH2:");
  Serial.print(round(filteredSpeedCh));
  Serial.println();
}