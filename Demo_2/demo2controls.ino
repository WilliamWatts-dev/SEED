#include <Encoder.h> // Use the Encoder library for handling encoders
#include <Wire.h> // Include the Wire library for I2C

// Variables for I2C transfer 
volatile bool newData = false;
volatile uint8_t readData = 0; // Read From bus
volatile uint8_t readFeet;
volatile uint8_t readAngle;
volatile uint8_t readColor;

// Robot constants
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
float desiredPositionFeet = 2; // Set target position in feet
const float maxSpeed = 0.2; // Lower max speed to reduce overshoot
const float minSpeed = 0.05;
const float accelerationRate = 0.05;
const float decelerationPhasefeet = 1.7; // decelerates for 1 - 0.55 = 0.45 or 45% of movement

float currentSpeed = 0.0;
long startPosition, currentPosition;

// Encoder object
Encoder motorEncoder(3, 6); // Encoder pins (A and B)

// Timing variables
unsigned long last_time_ms, start_time_ms;
float current_time;

// Global Encoder References
long postRotation1;
long postFoundRotation;
long postApproachRotation;

// State Machine Variables in order of use
int currentState = 0;
const int commTestState = 1; // Start with communications test, no errors, move on
const int initialRotationState = 2; // Start rotation, once qr is found, move on 
const int qrFoundState = 3; // Transfer initial desired angle + feet, move on
const int moveCycleState = 4; // Initiate loop on moving towards qr until at least 1.5 feet away then stop, move on
const int arrowReadState = 5; // Transfer arrow data to arduino, move on
const int turnLeftState = 6; // Rotation 90 degrees left
const int turnRightState = 7; // Rotation 90 degrees right
const int continuedRotationState = 8; // Start rotation again, loop back to qrFoundState once found

void receiveEvent(int howMany) {
  while (Wire.available()) {
    // Reads the byta containing all of our transmitted data
    readData = Wire.read();

    // Extract the components from the packed data
    // [distance (4 bits)][angle (3 bits)][color (1 bit)]
    readFeet = (readData >> 4) & 0x0F;   // Extract bits 4-7
    readAngle = (readData >> 1) & 0x07;  // Extract bits 1-3
    readColor = readData & 0x01;         // Extract bit 0 (LSB)
    newData = true;
  

  // Debug print statements
  Serial.print("I2C Received - Feet: ");
  Serial.print(readFeet);
  Serial.print(", Angle: ");
  Serial.print(readAngle);
  Serial.print(", Color: ");
  Serial.println(readColor ? "Red/Right" : "Green/Left");
}
  // Clear the buffer
  while (Wire.available()){
    Wire.read();
  }
}

// Function for rotation.
void rotate(float angle, long posCounts, bool dir) {
  float desiredRotationRadians = angle*(PI/180.0)*0.965; // Target rotation (90 degrees in radians)
  float accelerationPhase = desiredRotationRadians*0.15; // accelerates for 0.15 or 15% of movement
  float decelerationPhase = desiredRotationRadians*0.55; // decelerates for 1 - 0.55 = 0.45 or 45% of movement
  float rotationRadians = (posCounts * inchesPerCount) / wheelbaseRadius;
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

        }

        // Convert speed to motor voltage
        float appliedVoltage = currentSpeed * Battery_Voltage / 2 / maxSpeed;
        if (appliedVoltage > Battery_Voltage) appliedVoltage = Battery_Voltage / 2;

        // Set motor directions for rotation
        if(dir) { // Right (maybe)
          digitalWrite(motor1Dir, LOW);  // Motor 1 forward
          digitalWrite(motor2Dir, HIGH); // Motor 2 backward 
        }
        else {
          digitalWrite(motor1Dir, HIGH);  // Motor 1 backward
          digitalWrite(motor2Dir, LOW); // Motor 2 forward
        }


        // Apply PWM signal
        int pwmValue = min(abs(appliedVoltage) * 255 / Battery_Voltage, 255);
        analogWrite(motor1PWM, pwmValue*1.03);
        analogWrite(motor2PWM, pwmValue*0.85);
}
}

// Function for position
void forward(float feet, long posCounts) {
    float positionFeet = (posCounts * inchesPerCount) / 12.0; // Convert counts to feet
    float remainingDistance = feet - positionFeet;
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
  Wire.onReceive(receiveEvent);

  start_time_ms = millis();
  startPosition = motorEncoder.read(); // Record starting position
  currentState = commTestState;
}

void loop() {
  // put your main code here, to run repeatedly:
  current_time = (millis() - start_time_ms) / 1000.0;
  switch(currentState) {
    case(commTestState): {
      Wire.beginTransmission(8);
      byte test = Wire.endTransmission();
      if(test != 0) {
        currentState = commTestState;
        Serial.println("I2C Error");
        break;
      }
      else {
        currentState = initialRotationState;
        Serial.println("I2C Found!");
        Serial.println("Entering initial rotation state.");
      }
    }
    case(initialRotationState): {
        long positionCounts = motorEncoder.read() - startPosition;
        rotate(360, positionCounts, 1);
        if(newData == true) {
          postRotation1 = positionCounts;
          currentState = qrFoundState;
          // Extract bits and store in the following 3 variables
          // IMPORTANT: Figure out conversions and bit interpretations with CV code
          readFeet = (readData >> 4) & 0x0F;
          readAngle = (readData >> 1) & 0x07;
          readColor = readData & 0x01;
          newData = false;
          Serial.println("QR Found!");
          Serial.println("Switching to QR found state.");
          break;
        }
      }
      break;
    case(qrFoundState): {
      // Finish rotation to 0 degrees
      long positionCounts = motorEncoder.read() - postRotation1;
      if(newData == true) {
        readFeet = (readData >> 4) & 0x0F;
        readAngle = (readData >> 1) & 0x07;
        readColor = readData & 0x01;
        newData = false;
      }
      // Need a way to determine dir, maybe unnecessary
      bool dir;
      rotate(readAngle, positionCounts, dir);
      if(readAngle <= 0.001) {
        currentState = moveCycleState;
        postFoundRotation = positionCounts;
        Serial.println("Angle Corrected");
        Serial.println("Switching to Move Cycle State");
        delay(250);
        break;
      }
      break;
    }
    case(moveCycleState): {
      // Adjust specified PWM values accordig to positive or negative angle. Implementing negative feedback
      long positionCounts = motorEncoder.read() - postFoundRotation;
      // Continously recieve angle + distance
      if(newData == true) {
        readFeet = (readData >> 4) & 0x0F;
        readAngle = (readData >> 1) & 0x07;
        readColor = readData & 0x01;
        newData = false;
      }
      // Pretty sure readFeet will be the remaining distance to the beacon
      //IMPORTANT: Solution to determining dir, predict overshoot?
      bool dir;
      rotate(readAngle, positionCounts, dir);
      forward(readFeet, positionCounts);
      // Break at remainingdistance <= 1.5 ft
      if(readFeet <= 1.5) {
        currentState = arrowReadState;
        postApproachRotation = positionCounts;
        Serial.println("Within 1.5 Feet");
        Serial.println("Switching to Arrow Read State");
        // while(true); // Use this statement for the Approach + Stop
        delay(250);
        break;
      }
      break;
    }
    case(arrowReadState): {
      //Complete stop at beginning of this state or end of last. Recieve arrow data
      if(newData == true) {
        readFeet = (readData >> 4) & 0x0F;
        readAngle = (readData >> 1) & 0x07;
        readColor = readData & 0x01;
        newData = false;
        if(readColor == 1) {
          currentState = turnRightState;
          Serial.println("Recieved: Right");
          Serial.println("Switching to Turn Right State");
          break;
        }
      }
      break;
    }
    case(turnLeftState): {
      // No data needed, for this and next, rotate until remaining = 0
      long positionCounts = motorEncoder.read() - postApproachRotation;
      rotate(90, positionCounts, 1);
      while(true); // and finally stop
      break;
    }
    case(turnRightState): {
      long positionCounts = motorEncoder.read() - postApproachRotation;
      rotate(90, positionCounts, 0);
      while(true); // and finally stop
      break;
    }
    case(continuedRotationState): {
      // Probably won't use this in demo 2
      break;
    }
  }
  delay(5);
}
