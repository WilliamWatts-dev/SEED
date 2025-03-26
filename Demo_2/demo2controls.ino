#include <Encoder.h> // Use the Encoder library for handling encoders
#include <Wire.h> // Include the Wire library for I2C

// Variables for I2C transfer 
volatile bool newData = false;
volatile uint8_t readData = 0;
uint8_t recievedData = 0;

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
float desiredAngle = 360.0;
float desiredRotationRadians = desiredAngle*(PI/180.0)*0.965; // Target rotation (90 degrees in radians)
float desiredPositionFeet = 2; // Set target position in feet
const float maxSpeed = 0.2; // Lower max speed to reduce overshoot
const float minSpeed = 0.05;
const float accelerationRate = 0.05;
const float accelerationPhase = desiredRotationRadians*0.15; // accelerates for 0.15 or 15% of movement
const float decelerationPhase = desiredRotationRadians*0.55; // decelerates for 1 - 0.55 = 0.45 or 45% of movement
const float decelerationPhasefeet = 1.7; // decelerates for 1 - 0.55 = 0.45 or 45% of movement

float currentSpeed = 0.0;
long startPosition, currentPosition;
float remainingDistance = desiredPositionFeet;
long postRotation;

// Encoder object
Encoder motorEncoder(3, 6); // Encoder pins (A and B)

// Timing variables
unsigned long last_time_ms, start_time_ms;
float current_time;

// State Machine Variables in order of use
int currentState = 0;
const int commTestState = 1; // Start with communications test, no errors, move on
const int initialRotationState = 2; // Start rotation, once qr is found, move on 
const int qrFoundState = 3; // Transfer initial desired angle + feet, move on
const int moveCycleState = 4; // Initiate loop on moving towards qr until at least 1.5 feet away then stop, move on
const int arrowReadState = 5; // Transfer arrow data to arduino, move on
const int arrowRotationState = 6; // Rotation 90 degrees left, right, or catch error, move on
const int continuedRotationState = 7; // Start rotation again, loop back to qrFoundState once found

void receiveEvent(int howMany) {
  while (Wire.available()) {
    readData = Wire.read();
    newData = true;
  }
  // Clear the buffer
  while (Wire.available()){
    Wire.read();
  }
}

void setup() {
  // put your setup code here, to run once:
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor2PWM, OUTPUT);
  pinMode(motor1Dir, OUTPUT);
  pinMode(motor2Dir, OUTPUT);
  pinMode(motorEnable, OUTPUT);
  
  digitalWrite(motorEnable, HIGH); // Enable motor driver

  Serial.begin(115200);

  // I2C bus, address 8
  Wire.begin(8);
  Wire.onReceive(recieveEvent);

  start_time_ms = millis();
  startPosition = motorEncoder.read(); // Record starting position
  currentState = commTestState;
}

void loop() {
  // put your main code here, to run repeatedly:
  current_time = (millis() - start_time_ms) / 1000.0;

  // Read current position in counts
  long positionCounts = motorEncoder.read() - startPosition;

  switch(currentState) {
    case(commTestState) {
      int test = 0;
      if(newData) {
        recievedData = readData;
        newData = false;
      }
      if(recievedData != test) {
        currentState = initialRotationState;
        break;
      }
      currentState = commTestState;
      break;
    }
    case(initialRotationState) {
      float rotationRadians = (positionCounts * inchesPerCount) / wheelbaseRadius;

      break;
    }
    case(qrFoundState) {
      break;
    }
    case(moveCycleState) {
      break;
    }
    case(arrowReadState) {
      break;
    }
    case(arrowRotationState) {
      break;
    }
    case(continuedRotationState) {
      break;
    }
  }
  /*
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

    }

    // Convert speed to motor voltage
    float appliedVoltage = currentSpeed * Battery_Voltage / 2 / maxSpeed;
    if (appliedVoltage > Battery_Voltage) appliedVoltage = Battery_Voltage / 2;

    // Set motor directions for rotation
    digitalWrite(motor1Dir, LOW);  // Motor 1 forward
    digitalWrite(motor2Dir, HIGH); // Motor 2 backward

    // Apply PWM signal
    int pwmValue = min(abs(appliedVoltage) * 255 / Battery_Voltage, 255);
    analogWrite(motor1PWM, pwmValue*1.03);
    analogWrite(motor2PWM, pwmValue*0.85);
    postRotation = positionCounts;
  }
  else if (remainingRotation <= 0.001 && remainingDistance > 0) {
    long positionCountsfeet = motorEncoder.read() - postRotation;
    float positionFeet = (positionCountsfeet * inchesPerCount) / 12.0; // Convert counts to feet
    remainingDistance = desiredPositionFeet - positionFeet;
    // Determine speed profile
    if (remainingDistance > decelerationPhasefeet) {
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
      remainingDistance = 0;
    }
    // Convert speed to motor voltage
    float appliedVoltage = currentSpeed * Battery_Voltage / 2 / maxSpeed;
    if (appliedVoltage > Battery_Voltage) appliedVoltage = Battery_Voltage / 2;

    // Set motor direction
    digitalWrite(motor1Dir, appliedVoltage > 0 ? LOW : HIGH);
    digitalWrite(motor2Dir, appliedVoltage > 0 ? LOW : HIGH);
    // Apply PWM signal based on the voltage
    float pwmValue = min(abs(appliedVoltage) * 255 / Battery_Voltage, 255);
    analogWrite(motor1PWM, (pwmValue*1.04));
    analogWrite(motor2PWM, (pwmValue*0.945));
    Serial.println(remainingDistance);
  }
  else if (remainingRotation <= 0 && remainingDistance <= 0) {
    analogWrite(motor1PWM, 0); // Stop motors
    analogWrite(motor2PWM, 0);
    while (true); // Halt program
  }
  */
  delay(5);
}
