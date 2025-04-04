
#include <Encoder.h> // Use the Encoder library for handling encoders
#include <Wire.h> // Include the Wire library for I2C

// Variables for I2C transfer
uint8_t dataBuffer[9]; // Buffer for 9 bytes: 4 (distance) + 4 (angle) + 1 (color)
volatile bool newData = false;
volatile uint8_t readData = 0; // Read From bus
volatile float readFeet = 0.0;
volatile float readAngle = 0.0;
volatile uint8_t readColor = 0;

// Robot constants
const float wheelDiameterInches = 6.0;
const float wheelbaseDiameterInches = 15.0; // Distance between wheels
const float countsPerRevolution = 64 * 50; // Encoder resolution * gear ratio
const float inchesPerCount = (wheelDiameterInches * PI) / countsPerRevolution;
const float wheelbaseRadius = wheelbaseDiameterInches / 2.0;
float x_old = 0;
float y_old = 0;
float phi_old = 0;
float left_distance_old = 0;
float right_distance_old = 0;
bool stopNchop = 0;

// Motor control variables
const float Battery_Voltage = 7.8; // Battery voltage
const int motor1PWM = 9; // Motor 1 PWM pin
const int motor2PWM = 10; // Motor 2 PWM pin
const int motor1Dir = 7; // Motor 1 direction pin
const int motor2Dir = 8; // Motor 2 direction pin
const int motorEnable = 4; // Motor enable pin

// Position control variables

const float maxSpeed = 0.15; // Lower max speed to reduce overshoot
const float minSpeed = 0.07;
const float accelerationRate = 0.05;
const float decelerationPhasefeet = 1.7; // decelerates for 1 - 0.55 = 0.45 or 45% of movement

float currentSpeed = 0.0;
long startPosition, currentPosition;

// Encoder object
Encoder motor1Encoder(3, 6); // LEFT Encoder pins (A and B)
Encoder motor2Encoder(2, 5); // RIGHT

// Timing variables
unsigned long last_time_ms, start_time_ms;
float current_time;

// Global Encoder References
long postRotation1;
long postFoundRotation;
long postApproachRotation;
long lastPosition;

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
  Wire.read();

  if (Wire.available() >= 9) {
    for (int i = 0; i < 9; i++) {
      dataBuffer[i] = Wire.read();
    }
    Serial.print("Received dataBuffer: ");
    for (int i = 0; i < 9; i++) {
      Serial.print(dataBuffer[i]);
      Serial.print(" ");
    }
    Serial.println();
    newData = true;
  }
  // Clear the buffer
  while (Wire.available()){
    Wire.read();
  }
}

void stop() {
  analogWrite(motor1PWM, 0);
  analogWrite(motor2PWM, 0);
  stopNchop = 1;
}

// Function for rotation.
void rotate(float angle, long posCounts, bool dir) {
  float desiredRotationRadians = angle*(PI/180.0)*0.965; // Target rotation (90 degrees in radians)
  float accelerationPhase = desiredRotationRadians*0.1; // accelerates for 0.15 or 15% of movement
  float decelerationPhase = desiredRotationRadians*0.55; // decelerates for 1 - 0.55 = 0.45 or 45% of movement
   float remainingRotation = desiredRotationRadians - posCounts;
   if (remainingRotation > 0.001) {
        // Determine speed profile
        if (posCounts < accelerationPhase) { // acceleration phase, rotationRadians is less than acceleration phase
          // Acceleration phase
          currentSpeed += accelerationRate * 0.01;
          if (currentSpeed > maxSpeed) {
            currentSpeed = maxSpeed;
          }
        } else if (posCounts > decelerationPhase) { // deceleration phase, rotationRadians is greater than deceleration phase
          currentSpeed -= accelerationRate * 0.01;
          if (currentSpeed < minSpeed) {
            currentSpeed = minSpeed;
          }
        } 
        else if ( accelerationPhase < posCounts && posCounts < decelerationPhase) { // hold phase, past acceleration but not into deceleration
          // hold speed
          currentSpeed = currentSpeed;
        }
        else { // stop phase, all other conditions fail
          // Stop
          currentSpeed = 0;

        }

        // Convert speed to motor voltage
        float appliedVoltage = currentSpeed * Battery_Voltage / 3 / maxSpeed;
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
        analogWrite(motor1PWM, (pwmValue*1.03));
        analogWrite(motor2PWM, (pwmValue*0.85));
}
  else {
    stop();
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

// Function to determine rotation direction based on angle
bool determineRotationDirection(float angle) {
  // If angle is positive, turn right (dir = true)
  // If angle is negative, turn left (dir = false)
  return angle >= 0;
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
  startPosition = motor1Encoder.read(); // Record starting position
  currentState = initialRotationState;
}

void loop() {
  // put your main code here, to run repeatedly:
  // Position Tracking
  long posL_counts = motor1Encoder.read() - startPosition;
  long posR_counts = ((-1) * motor2Encoder.read()) - startPosition;
  float posL_rad, posR_rad;
  posL_rad = 2.0 * PI * (float)posL_counts / countsPerRevolution;
  posR_rad = 2.0 * PI * (float)posR_counts / countsPerRevolution;
  float x_new, y_new, phi_new;
  float left_distance_new, right_distance_new, left_change, right_change;
  left_distance_new = wheelDiameterInches * posL_rad;
  right_distance_new = wheelDiameterInches * posR_rad;
  left_change = left_distance_new - left_distance_old;
  right_change = right_distance_new - right_distance_old;
  phi_new = phi_old + ((1/wheelbaseDiameterInches) * (left_change - right_change)); //Changed one sign for direction correction
  x_new = x_old + (cos(phi_old) * (left_change + right_change) * 0.5);
  y_new = y_old + (sin(phi_old) * (left_change + right_change) * 0.5);
  //Position Tracking over
  current_time = (millis() - start_time_ms) / 1000.0;
  switch(currentState) {
    /**
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
        break;
      }
    }
    */
    case(initialRotationState): {
        rotate(30, posL_rad, true); // rotates clockwise because of true (angle is double)
        if(stopNchop) {
          delay(1000);
          stopNchop = 0;
          if(newData == true) {
            // Extract data from buffer
            memcpy(&readFeet, dataBuffer, 4);
            memcpy(&readAngle, dataBuffer+4, 4);
            readColor = dataBuffer[8] & 0x01;
            
            Serial.print("Received - Distance: ");
            Serial.print(readFeet);
            Serial.print("ft, Angle: ");
            Serial.print(readAngle);
            Serial.print("°, Color: ");
            Serial.println(readColor ? "Red" : "Green");
            
            newData = false;
            currentState = qrFoundState;
            startPosition = motor1Encoder.read();

            Serial.print("Aruco marker found! Switching to marker align state");
            Serial.println("Distance: " + String(readFeet) + "ft, Angle: " + String(readAngle) + "deg");
            break;
          }
          startPosition = motor1Encoder.read();
        }
        break;
      }
    case(qrFoundState): {
      // Finish rotation to 0 degrees
      // Update data if new information received
      if(newData == true) {
        // Extract data from buffer
        memcpy(&readFeet, dataBuffer, 4);
        memcpy(&readAngle, dataBuffer+4, 4);
        readColor = dataBuffer[8] & 0x01;
        
        Serial.print("Updated - Distance: ");
        Serial.print(readFeet);
        Serial.print("ft, Angle: ");
        Serial.print(readAngle);
        Serial.print("°, Color: ");
        Serial.println(readColor ? "Red" : "Green");
        
        newData = false;
      }
      
      // Need a way to determine dir, maybe unnecessary
      bool dir = determineRotationDirection(readAngle);

      // Rotate to align with the marker
      rotate(abs(readAngle), posL_rad, dir);
      
      if(stopNchop) {
        currentState = moveCycleState;
        //postFoundRotation = positionCounts;
        Serial.println("Angle Corrected");
        Serial.println("Switching to Move Cycle State");
        delay(250);
        break;
      }
      break;
    }
    case(moveCycleState): {
      // Adjust specified PWM values accordig to positive or negative angle. Implementing negative feedback

      
      // Update data if new information received
      if(newData == true) {
        // Extract data from buffer
        memcpy(&readFeet, dataBuffer, 4);
        memcpy(&readAngle, dataBuffer+4, 4);
        readColor = dataBuffer[8] & 0x01;
        
        Serial.print("Updated - Distance: ");
        Serial.print(readFeet);
        Serial.print("ft, Angle: ");
        Serial.print(readAngle);
        Serial.print("°, Color: ");
        Serial.println(readColor ? "Red" : "Green");
        
        newData = false;
      }
      // Pretty sure readFeet will be the remaining distance to the beacon
      
      //IMPORTANT: Solution to determining dir, predict overshoot?
      bool dir = determineRotationDirection(readAngle);

      if(abs(readAngle) > 3.0) { // If we're off by more than 3 degrees
        //rotate(readAngle, positionCounts, dir);
      } else { // Move forward
        //forward (readFeet, positionCounts);

      // Break at remainingdistance <= 1.5 ft
      if(readFeet <= 1.5) {
        currentState = arrowReadState;
        //postApproachRotation = positionCounts;
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
      // Read the arrow color/direction
      if(newData == true) {
        // Extract data from buffer
        memcpy(&readFeet, dataBuffer, 4);
        memcpy(&readAngle, dataBuffer+4, 4);
        readColor = dataBuffer[8] & 0x01;
        
        Serial.print("Arrow Data - Distance: ");
        Serial.print(readFeet);
        Serial.print("ft, Angle: ");
        Serial.print(readAngle);
        Serial.print("°, Color: ");
        Serial.println(readColor ? "Red" : "Green");
        
        newData = false;
        
        if(readColor == 1) { // Red = right
          currentState = turnRightState;
          Serial.println("Recieved: Right");
          Serial.println("Switching to Turn Right State");
        } else { // Green = left
          currentState = turnLeftState;
          Serial.println("Received: Left");
          Serial.println("Switching to Turn Left State");
        }
          break;
      }
      break;
    }
    case(turnLeftState): {
      // No data needed, for this and next, rotate until remaining = 0
      //rotate(90, positionCounts, false); // false = left
      break;
    }
    case(turnRightState): {
      //rotate(90, positionCounts, true); // true = right
      break;
    }
    case(continuedRotationState): {
      // Probably won't use this in demo 2
      break;
    }
  }
  phi_old = phi_new;
  x_old = x_new;
  y_old = y_new;
  left_distance_old = left_distance_new;
  right_distance_old = right_distance_new;
  delay(5);
}
}
