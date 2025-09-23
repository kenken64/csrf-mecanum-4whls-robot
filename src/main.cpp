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

AlfredoCRSF crsf;

// Robot control variables
int motorSpeed = 60;  // Current speed (60-255)
int baseSpeed = 60;   // Minimum speed
String currentDirection = "STOPPED";
bool motorsLocked = true;  // Channel 8 motor lock state

// Channel mapping constants
#define CH_THROTTLE 3      // Channel 3: Throttle (backward/stop/forward)
#define CH_SPEED 2         // Channel 2: Speed control
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

// Motor lock
#define MOTOR_LOCKED 1000
#define MOTOR_UNLOCKED 2000

// Function declarations
void moveForward(int speed);
void moveBackward(int speed);
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

  // Start with motors stopped and locked
  stopMotors();
  
  // Use Serial1 for CRSF communication
  Serial1.begin(CRSF_BAUDRATE);
  
  crsf.begin(Serial1);
  
  Serial.println("Channel Mapping:");
  Serial.println("  CH3: Throttle (<1472=backward, 1472=stop, >1473=forward)");
  Serial.println("  CH2: Speed (1500=base, higher=faster, lower=slower)");
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

// Motor control functions
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
  // Check if link is up
  if (!crsf.isLinkUp()) {
    stopMotors();
    currentDirection = "NO LINK";
    return;
  }

  // Get channel values
  int throttle = crsf.getChannel(CH_THROTTLE);    // Channel 3
  int speedCh = crsf.getChannel(CH_SPEED);        // Channel 2  
  int steering = crsf.getChannel(CH_STEERING);    // Channel 4
  int lockCh = crsf.getChannel(CH_LOCK);          // Channel 8

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

  // Calculate speed from Channel 2
  if (speedCh > SPEED_CENTER) {
    // Above 1500 - increase speed
    motorSpeed = map(speedCh, SPEED_CENTER, 2000, baseSpeed, SPEED_MAX);
  } else {
    // Below 1500 - decrease speed  
    motorSpeed = map(speedCh, 1000, SPEED_CENTER, SPEED_MIN, baseSpeed);
  }
  motorSpeed = constrain(motorSpeed, SPEED_MIN, SPEED_MAX);

  // Process throttle (Channel 3)
  if (throttle < THROTTLE_BACKWARD_MAX) {
    // Move backward with steering
    if (steering > STEERING_CENTER + 100) {
      // Backward + Left steering (right side slower) - INVERTED
      int leftSpeed = motorSpeed;
      int rightSpeed = motorSpeed * 0.5;  // Right side slower for left turn
      
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, leftSpeed);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENB, rightSpeed);
      currentDirection = "BACKWARD LEFT";
    } else if (steering < STEERING_CENTER - 100) {
      // Backward + Right steering (left side slower) - INVERTED
      int leftSpeed = motorSpeed * 0.5;   // Left side slower for right turn
      int rightSpeed = motorSpeed;
      
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, leftSpeed);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENB, rightSpeed);
      currentDirection = "BACKWARD RIGHT";
    } else {
      // Straight backward
      moveBackward(motorSpeed);
      currentDirection = "BACKWARD";
    }
  } else if (throttle > THROTTLE_FORWARD_MIN) {
    // Move forward with steering
    if (steering > STEERING_CENTER + 100) {
      // Forward + Left steering (right side slower) - INVERTED
      int leftSpeed = motorSpeed;
      int rightSpeed = motorSpeed * 0.5;  // Right side slower for left turn
      
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, leftSpeed);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, rightSpeed);
      currentDirection = "FORWARD LEFT";
    } else if (steering < STEERING_CENTER - 100) {
      // Forward + Right steering (left side slower) - INVERTED
      int leftSpeed = motorSpeed * 0.5;   // Left side slower for right turn
      int rightSpeed = motorSpeed;
      
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, leftSpeed);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, rightSpeed);
      currentDirection = "FORWARD RIGHT";
    } else {
      // Straight forward
      moveForward(motorSpeed);
      currentDirection = "FORWARD";
    }
  } else {
    // Throttle at center - check steering only for spot turns
    if (steering > STEERING_CENTER + 100) {
      // Turn left in place - INVERTED
      turnLeft(motorSpeed);
      currentDirection = "TURN LEFT";
    } else if (steering < STEERING_CENTER - 100) {
      // Turn right in place - INVERTED
      turnRight(motorSpeed);
      currentDirection = "TURN RIGHT";
    } else {
      // Stop
      stopMotors();
      currentDirection = "STOPPED";
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
  Serial.print("Link: ");
  Serial.print(crsf.isLinkUp() ? "UP" : "DOWN");
  Serial.print(" | Lock: ");
  Serial.print(motorsLocked ? "ON" : "OFF");
  Serial.print(" | Dir: ");
  Serial.print(currentDirection);
  Serial.print(" | Speed: ");
  Serial.print(motorSpeed);
  Serial.print(" | CH3:");
  Serial.print(crsf.getChannel(3));
  Serial.print(" CH2:");
  Serial.print(crsf.getChannel(2));  
  Serial.print(" CH4:");
  Serial.print(crsf.getChannel(4));
  Serial.print(" CH8:");
  Serial.println(crsf.getChannel(8));
}