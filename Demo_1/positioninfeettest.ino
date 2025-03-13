// positioninfeettest.ino
// Move a robot forward by a specified distance in feet
// use encoder feedback for speed control and smooth acceleration/deceleration.  

#include <Encoder.h> // Use the Encoder library for handling encoders
#include <Wire.h> // Include the Wire library for I2C

const float wheelDiameterInches = 6.0;
const float countsPerRevolution = 64 * 50; // Encoder resolution * gear ratio
const float inchesPerCount = (wheelDiameterInches * 3.14159) / countsPerRevolution;

// Motor control variables
const float Battery_Voltage = 7.8; // Battery voltage
const int motor1PWM = 9; // Motor PWM pin
const int motor2PWM = 10; // Motor PWM pin
const int motorDir = 7; // Motor direction pin
const int motorEnable = 4; // Motor enable pin

// Position control variables
float desiredPositionFeet = 2.0; // Set target position in feet
const float maxSpeed = 0.15; // Lower max speed to reduce overshoot
const float minSpeed = 0.025;
const float accelerationRate = 0.05; // Slower acceleration to avoid overshoot
const float decelerationDistance = 0.5; // Increase deceleration distance for smoother slowdown

float currentSpeed = 0.0;
long startPosition, currentPosition;

// Encoder object
Encoder motorEncoder(3, 6); // Encoder pins (A and B)

// Timing variables
unsigned long last_time_ms, start_time_ms;
float current_time;

void setup() {
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor2PWM, OUTPUT);
  pinMode(motorDir, OUTPUT);
  pinMode(motorEnable, OUTPUT);
  
  digitalWrite(motorEnable, HIGH); // Enable motor driver

  Serial.begin(115200);
  start_time_ms = millis();
  last_time_ms = start_time_ms;
  
  startPosition = motorEncoder.read(); // Record starting position
}

void loop() {
  current_time = (millis() - start_time_ms) / 1000.0;

  // Read current position in counts
  long positionCounts = motorEncoder.read() - startPosition;
  float positionFeet = positionCounts * inchesPerCount / 12.0; // Convert counts to feet
  float remainingDistance = desiredPositionFeet - positionFeet;

  // Determine speed profile
  if (remainingDistance > decelerationDistance) {
    // Acceleration phase
    currentSpeed += accelerationRate * 0.01; // Increment speed
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
  float appliedVoltage = currentSpeed * Battery_Voltage / maxSpeed;

  // Set motor direction
  digitalWrite(motorDir, appliedVoltage > 0 ? LOW : HIGH);

  // Apply PWM signal based on the voltage
  int pwmValue = min(abs(appliedVoltage) * 255 / Battery_Voltage, 255);
  analogWrite(motor1PWM, pwmValue);
  analogWrite(motor2PWM, pwmValue);

  // Debugging output
  Serial.print("Position (ft): ");
  Serial.print(positionFeet);
  Serial.print(" Speed (ft/s): ");
  Serial.print(currentSpeed);
  Serial.print(" PWM: ");
  Serial.println(pwmValue);

  // Stop the motor once the target position is reached
  if (remainingDistance <= 0) {
    analogWrite(motor1PWM, 0); // Stop motor
    analogWrite(motor2PWM, 0);
    while (true); // Halt program here
  }

  delay(5); // Wait for next loop iteration
}


