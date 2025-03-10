#include <Encoder.h> // Use the Encoder library for handling encoders
#include <Wire.h> // Include the Wire library for I2C

const float wheelDiameterInches = 6.0;
const float wheelbaseDiameterInches = 15.0; // Distance between wheels
const float countsPerRevolution = 64 * 50; // Encoder resolution * gear ratio
const float inchesPerCount = (wheelDiameterInches * PI) / countsPerRevolution;
const float wheelbaseRadius = wheelbaseDiameterInches / 2.0;

// Motor control variables
const float Battery_Voltage = 7.8; // Battery voltage
const int motor1PWM = 9; // Motor 1 PWM pin
const int motor2PWM = 10; // Motor 2 PWM pin
const int motor1Dir = 7; // Motor 1 direction pin
const int motor2Dir = 8; // Motor 2 direction pin
const int motorEnable = 4; // Motor enable pin

// Position control variables
float desiredAngle = 0;
float desiredRotationRadians = desiredAngle*(PI/180.0)*0.92; // Target rotation (90 degrees in radians)
float desiredPositionFeet = 1.0; // Set target position in feet
const float maxSpeed = 0.15; // Lower max speed to reduce overshoot
const float minSpeed = 0.05;
const float accelerationRate = 0.05;
const float accelerationPhase = desiredRotationRadians*0.15; // accelerates for 0.15 or 15% of movement
const float decelerationPhase = desiredRotationRadians*0.55; // decelerates for 1 - 0.55 = 0.45 or 45% of movement
const float accelerationPhasefeet = desiredPositionFeet*0.15; // accelerates for 0.15 or 15% of movement
const float decelerationPhasefeet = desiredPositionFeet*0.75; // decelerates for 1 - 0.55 = 0.45 or 45% of movement

float currentSpeed = 0.0;
long startPosition, currentPosition;
float remainingDistance = desiredPositionFeet;

// Encoder object
Encoder motorEncoder(3, 6); // Encoder pins (A and B)

// Timing variables
unsigned long last_time_ms, start_time_ms;
float current_time;

void setup() {
  // put your setup code here, to run once:
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor2PWM, OUTPUT);
  pinMode(motor1Dir, OUTPUT);
  pinMode(motor2Dir, OUTPUT);
  pinMode(motorEnable, OUTPUT);
  
  digitalWrite(motorEnable, HIGH); // Enable motor driver

  Serial.begin(115200);
  start_time_ms = millis();
  startPosition = motorEncoder.read(); // Record starting position
}

void loop() {
  // put your main code here, to run repeatedly:
  current_time = (millis() - start_time_ms) / 1000.0;

  // Read current position in counts
  long positionCounts = motorEncoder.read() - startPosition;

  // Calculate remaining rotation and run the if statement 
  float rotationRadians = (positionCounts * inchesPerCount) / wheelbaseRadius;
  float remainingRotation = desiredRotationRadians - rotationRadians;

  if (remainingRotation > 0) {
    // Determine speed profile
    if (rotationRadians < accelerationPhase) { // acceleration phase, rotationRadians is less than acceleration phase
      // Acceleration phase
      currentSpeed += accelerationRate * 0.01;
      if (currentSpeed > maxSpeed) {
        currentSpeed = maxSpeed;
      }
    } else if (rotationRadians > decelerationPhase) { // deceleration phase, rotationRadians is greater than deceleration phase
      currentSpeed -= accelerationRate * 0.01;
      if (currentSpeed < minSpeed) {
        currentSpeed = minSpeed;
      }
    } 
    else if ( accelerationPhase < rotationRadians && rotationRadians < decelerationPhase) { // hold phase, past acceleration but not into deceleration
      // hold speed
      currentSpeed = currentSpeed;
    }
    else { // stop phase, all other conditions fail
      // Stop
      currentSpeed = 0;
      remainingRotation = 0;
    }

    // Convert speed to motor voltage
    float appliedVoltage = currentSpeed * Battery_Voltage / 2 / maxSpeed;
    if (appliedVoltage > Battery_Voltage) appliedVoltage = Battery_Voltage / 2;

    // Set motor directions for rotation
    digitalWrite(motor1Dir, LOW);  // Motor 1 forward
    digitalWrite(motor2Dir, HIGH); // Motor 2 backward

    // Apply PWM signal
    int pwmValue = min(abs(appliedVoltage) * 255 / Battery_Voltage, 255);
    analogWrite(motor1PWM, pwmValue);
    analogWrite(motor2PWM, pwmValue*0.93);
  }
  if (remainingRotation <= 0 && remainingDistance > 0) {
    motorEncoder.readAndReset();
    long positionFeetCounts = motorEncoder.read() - positionCounts;
    float positionFeet = (positionFeetCounts * inchesPerCount) / 12.0; // Convert counts to feet
    remainingDistance = desiredPositionFeet - positionFeet;
    // Determine speed profile
    if (remainingDistance > decelerationPhasefeet) {
      // Acceleration phase
      currentSpeed += (accelerationRate * 0.01); // Increment speed
      if (currentSpeed > maxSpeed) {
        currentSpeed = maxSpeed;
      }
    } else if (remainingDistance > 0) {
      currentSpeed -= accelerationRate * 0.01;
      if (currentSpeed < minSpeed) {
        currentSpeed = minSpeed;
      }
    } else {
      // Stop
      currentSpeed = 0;

    }

    // Convert speed to motor voltage
    float appliedVoltage = currentSpeed * Battery_Voltage / 2 / maxSpeed;

    // Set motor direction
    digitalWrite(motor1Dir, appliedVoltage > 0 ? LOW : HIGH);
    digitalWrite(motor2Dir, appliedVoltage > 0 ? LOW : HIGH);
    // Apply PWM signal based on the voltage
    int pwmValue = min(abs(appliedVoltage) * 255 / Battery_Voltage, 255);
    analogWrite(motor1PWM, pwmValue);
    analogWrite(motor2PWM, pwmValue*0.93);
  }
  if (remainingDistance <= 0 && remainingRotation <= 0) {
    analogWrite(motor1PWM, 0); // Stop motors
    analogWrite(motor2PWM, 0);
    while (true); // Halt program
  }
  delay(5);
}
