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
int motorSpeed = 60;  // Current speed (60-255)
int baseSpeed = 60;   // Minimum speed
int previousSpeed = 60;  // Previous speed for change detection
String currentDirection = "STOPPED";
bool motorsLocked = true;  // Channel 8 motor lock state

// Direction change tracking to prevent current spikes
unsigned long lastDirectionChange = 0;
#define DIRECTION_CHANGE_DELAY 15  // Reduced from 50ms to 15ms for smoother transitions
String previousDirection = "STOPPED";

// Battery voltage monitoring
#define VOLTAGE_DIVIDER_RATIO 3.0  // Adjust based on your voltage divider (R1+R2)/R2
float batteryVoltage = 0.0;

// Function to read battery voltage
float readBatteryVoltage() {
  int adcValue = analogRead(BATTERY_PIN);
  // Convert ADC value to voltage (5.0V reference, 10-bit ADC = 1023 max)
  float adcVoltage = (adcValue / 1023.0) * 5.0;
  // Apply voltage divider ratio
  return adcVoltage * VOLTAGE_DIVIDER_RATIO;
}

// Channel change detection
int previousSpeedCh = 1500;
unsigned long lastSpeedChange = 0;
#define SPEED_CHANGE_DELAY 100  // Increased from 50ms to 100ms for better protection
#define SPEED_FILTER_ALPHA 0.1  // Exponential smoothing factor (lower = more smoothing)
#define SPEED_DEADZONE 20       // Dead zone around center to prevent jitter

// Speed filtering variables
float filteredSpeedCh = 1500.0;  // Filtered channel value
int targetMotorSpeed = 60;       // Target speed after filtering
unsigned long lastFilterUpdate = 0;
#define FILTER_UPDATE_INTERVAL 20  // Update filter every 20ms

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
  int rightSpeed = speed * 0.3;  // Much slower right side creates rightward drift
  
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
  int leftSpeed = speed * 0.3;  // Much slower left side creates leftward drift
  int rightSpeed = speed;
  
  digitalWrite(IN1, HIGH);  // Left motor forward (slower)
  digitalWrite(IN2, LOW);
  analogWrite(ENA, leftSpeed);

  digitalWrite(IN3, HIGH);  // Right motor forward (faster)
  digitalWrite(IN4, LOW);
  analogWrite(ENB, rightSpeed);
}

void turnLeft(int speed) {
  // Left wheels slower, right wheels at full speed for gradual left turn
  int leftSpeed = speed * 0.3;   // Left side much slower
  int rightSpeed = speed;        // Right side full speed
  
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, leftSpeed);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, rightSpeed);
}

void turnRight(int speed) {
  // Right wheels slower, left wheels at full speed for gradual right turn  
  int leftSpeed = speed;         // Left side full speed
  int rightSpeed = speed * 0.3;  // Right side much slower
  
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, leftSpeed);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, rightSpeed);
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
    currentDirection = "NO LINK";
    
    // Reset previous values when connection lost
    previousSpeedCh = 1500;
    motorSpeed = baseSpeed;
    previousSpeed = baseSpeed;
    
    Serial.println("ELRS Link Lost - Resetting speed values");
    return;
  }

  // Get channel values with validation
  int strafe = crsf.getChannel(CH_STRAFE);        // Channel 1
  int speedCh = crsf.getChannel(CH_SPEED);        // Channel 2  
  int throttle = crsf.getChannel(CH_THROTTLE);    // Channel 3
  int steering = crsf.getChannel(CH_STEERING);    // Channel 4
  int lockCh = crsf.getChannel(CH_LOCK);          // Channel 8

  // Validate channel values (CRSF should be 1000-2000)
  if (speedCh < 500 || speedCh > 2500) {
    Serial.print("Invalid CH2 value: ");
    Serial.println(speedCh);
    return; // Skip this cycle if invalid data
  }
  
  if (throttle < 500 || throttle > 2500) {
    Serial.print("Invalid CH3 value: ");
    Serial.println(throttle);
    return; // Skip this cycle if invalid data
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
    currentDirection = "LOCKED";
    return;
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
    motorSpeed = targetMotorSpeed;
    lastSpeedChange = millis();

    // Detect and warn about rapid channel changes that could cause ELRS resets
    int channelChange = abs(speedCh - previousSpeedCh);
    if (channelChange > 300) { // Increased threshold for warning
      Serial.print("WARNING: Extreme speed change detected! CH2: ");
      Serial.print(speedCh);
      Serial.print(" (previous: ");
      Serial.print(previousSpeedCh);
      Serial.print(") -> Filtered: ");
      Serial.print(smoothedSpeedCh);
      Serial.print(" -> Speed: ");
      Serial.println(motorSpeed);

      // Additional protection: temporarily increase delay after extreme changes
      lastSpeedChange = millis() + 200; // Add extra 200ms delay
    }

    previousSpeedCh = speedCh;
  }

  // First check for strafe movement (Channel 1) - priority over throttle
  if (strafe > STRAFE_CENTER + 100) {
    // Strafe right (simulated with forward + right turn)
    if (currentDirection != "DRIFT RIGHT" && millis() - lastDirectionChange > DIRECTION_CHANGE_DELAY) {
      strafeRight(motorSpeed);
      currentDirection = "DRIFT RIGHT";
      previousDirection = currentDirection;
      lastDirectionChange = millis();
      return;
    }
  } else if (strafe < STRAFE_CENTER - 100) {
    // Strafe left (simulated with forward + left turn)
    if (currentDirection != "DRIFT LEFT" && millis() - lastDirectionChange > DIRECTION_CHANGE_DELAY) {
      strafeLeft(motorSpeed);
      currentDirection = "DRIFT LEFT";
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
      if (currentDirection != "BACKWARD LEFT" && millis() - lastDirectionChange > DIRECTION_CHANGE_DELAY) {
        int leftSpeed = motorSpeed;
        int rightSpeed = motorSpeed * 0.5;  // Right side slower for left turn

        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(ENA, leftSpeed);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENB, rightSpeed);
        currentDirection = "BACKWARD LEFT";
        previousDirection = currentDirection;
        lastDirectionChange = millis();
      }
    } else if (steering < STEERING_CENTER - 100) {
      // Backward + Right steering (left side slower) - INVERTED
      if (currentDirection != "BACKWARD RIGHT" && millis() - lastDirectionChange > DIRECTION_CHANGE_DELAY) {
        int leftSpeed = motorSpeed * 0.5;   // Left side slower for right turn
        int rightSpeed = motorSpeed;

        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(ENA, leftSpeed);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENB, rightSpeed);
        currentDirection = "BACKWARD RIGHT";
        previousDirection = currentDirection;
        lastDirectionChange = millis();
      }
    } else {
      // Straight backward
      if (currentDirection != "BACKWARD" && millis() - lastDirectionChange > DIRECTION_CHANGE_DELAY) {
        moveBackward(motorSpeed);
        currentDirection = "BACKWARD";
        previousDirection = currentDirection;
        lastDirectionChange = millis();
      }
    }
  } else if (throttle > THROTTLE_FORWARD_MIN) {
    // Move forward with steering
    if (steering > STEERING_CENTER + 100) {
      // Forward + Left steering (right side slower) - INVERTED
      if (currentDirection != "FORWARD LEFT" && millis() - lastDirectionChange > DIRECTION_CHANGE_DELAY) {
        int leftSpeed = motorSpeed;
        int rightSpeed = motorSpeed * 0.5;  // Right side slower for left turn

        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, leftSpeed);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENB, rightSpeed);
        currentDirection = "FORWARD LEFT";
        previousDirection = currentDirection;
        lastDirectionChange = millis();
      }
    } else if (steering < STEERING_CENTER - 100) {
      // Forward + Right steering (left side slower) - INVERTED
      if (currentDirection != "FORWARD RIGHT" && millis() - lastDirectionChange > DIRECTION_CHANGE_DELAY) {
        int leftSpeed = motorSpeed * 0.5;   // Left side slower for right turn
        int rightSpeed = motorSpeed;

        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, leftSpeed);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENB, rightSpeed);
        currentDirection = "FORWARD RIGHT";
        previousDirection = currentDirection;
        lastDirectionChange = millis();
      }
    } else {
      // Straight forward
      if (currentDirection != "FORWARD" && millis() - lastDirectionChange > DIRECTION_CHANGE_DELAY) {
        moveForward(motorSpeed);
        currentDirection = "FORWARD";
        previousDirection = currentDirection;
        lastDirectionChange = millis();
      }
    }
  } else {
    // Throttle at center - check steering only for spot turns
    if (steering > STEERING_CENTER + 100) {
      // Turn left in place - INVERTED
      if (currentDirection != "TURN LEFT" && millis() - lastDirectionChange > DIRECTION_CHANGE_DELAY) {
        turnLeft(motorSpeed);
        currentDirection = "TURN LEFT";
        previousDirection = currentDirection;
        lastDirectionChange = millis();
      }
    } else if (steering < STEERING_CENTER - 100) {
      // Turn right in place - INVERTED
      if (currentDirection != "TURN RIGHT" && millis() - lastDirectionChange > DIRECTION_CHANGE_DELAY) {
        turnRight(motorSpeed);
        currentDirection = "TURN RIGHT";
        previousDirection = currentDirection;
        lastDirectionChange = millis();
      }
    } else {
      // Stop
      if (currentDirection != "STOPPED" && millis() - lastDirectionChange > DIRECTION_CHANGE_DELAY) {
        stopMotors();
        currentDirection = "STOPPED";
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
  Serial.print(currentDirection);
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