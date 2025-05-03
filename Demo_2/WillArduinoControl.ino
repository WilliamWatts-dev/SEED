#include <Wire.h>
#include <Encoder.h>

// --- I2C Communication Variables ---
uint8_t dataBuffer[9];    // 9 bytes: 4 (distance in feet) + 4 (angle in degrees) + 1 (arrow/color flag)
volatile bool newData = false;
volatile float readFeet = 0.0;
volatile float readAngle = 0.0;
volatile uint8_t readColor = 0;  // RedArrow flag: 0 = counterclockwise rotation, 1 = clockwise

// --- Robot Constants & Motor Pins ---
const float wheelDiameterInches = 6.0;
const float countsPerRevolution = 64 * 50;  // Encoder resolution * gear ratio
const float inchesPerCount = (wheelDiameterInches * PI) / countsPerRevolution;

const int motor1PWM = 9;   // PWM pin for motor 1
const int motor2PWM = 10;  // PWM pin for motor 2
const int motor1Dir = 7;   // Direction pin for motor 1
const int motor2Dir = 8;   // Direction pin for motor 2
const int motorEnable = 4; // Motor driver enable pin

// --- PWM Settings for different states ---
const int slowPWM    = 35;  // PWM for QRCodeFind rotation (coarse search)
const int adjustPWM  = 35;  // PWM for coarse angle adjustment
const int finePWM    = 35;   // PWM for fine angle adjustment
const int forwardPWM = 45;  // PWM for moving forward
const int rotationDelay = 3000;

// --- Encoder Setup (used by rotate function) ---
Encoder motor1Encoder(3, 6); // LEFT encoder pins (A and B)
long startPosition = 0;

// --- Motor Control Helper Functions ---
void setRotationClockwise(int pwm) {
  // For clockwise rotation: motor1 forward, motor2 backward.
  digitalWrite(motor1Dir, HIGH);
  digitalWrite(motor2Dir, LOW);
  analogWrite(motor1PWM, pwm);
  analogWrite(motor2PWM, pwm);
}

void setRotationCounterclockwise(int pwm) {
  // For counterclockwise rotation: motor1 backward, motor2 forward.
  digitalWrite(motor1Dir, LOW);
  digitalWrite(motor2Dir, HIGH);
  analogWrite(motor1PWM, pwm);
  analogWrite(motor2PWM, pwm);
}

void setForward(int pwm) {
  // For forward motion: both motors forward.
  digitalWrite(motor1Dir, LOW);
  digitalWrite(motor2Dir, LOW);
  analogWrite(motor1PWM, pwm);
  analogWrite(motor2PWM, pwm);
}

void stopMotors() {
  analogWrite(motor1PWM, 0);
  analogWrite(motor2PWM, 0);
}

// --- I2C Data Receive ---
void receiveEvent(int howMany) {
  // Clear first byte if needed
  Wire.read();
  
  if (Wire.available() >= 9) {
    for (int i = 0; i < 9; i++) {
      dataBuffer[i] = Wire.read();
    }
    // Serial.print("Received dataBuffer: ");
    // for (int i = 0; i < 9; i++) {
    //   Serial.print(dataBuffer[i]);
    //   Serial.print(" ");
    // }
    // Serial.println();
    newData = true;
  }
  // Flush any remaining bytes
  while (Wire.available()) {
    Wire.read();
  }
}

// --- Update Sensor Data from Buffer ---
void updateSensorData() {
  memcpy(&readFeet, dataBuffer, 4);
  memcpy(&readAngle, dataBuffer + 4, 4);
  readColor = dataBuffer[8] & 0x01;
  newData = false;
}

// --- Rotate Function (used in ArrowRotation) ---
// This is adapted from your provided code. It uses encoder counts and a speed profile
// to execute a rotation of the specified angle. The 'dir' parameter: false = counterclockwise, true = clockwise.
float currentSpeed = 0.0;
const float Battery_Voltage = 7.8;
const float maxSpeed = 0.15;
const float minSpeed = 0.07;
const float accelerationRate = 0.05;

// --- State Machine Setup ---
enum RobotState {
  QRCodeFind,      // 0: Robot rotates slowly looking for the QR code.
  AngleAdjust,     // 1: Coarse rotation based on the received angle.
  FineAngleAdjust, // 2: Fine rotation (with lower PWM) until within 0.3°.
  MoveForward,     // 3: Move forward toward the QR code.
  ArrowRotation    // 4: Rotate 90° based on arrow data.
};

RobotState currentState = QRCodeFind;

// --- Setup ---
void setup() {
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor2PWM, OUTPUT);
  pinMode(motor1Dir, OUTPUT);
  pinMode(motor2Dir, OUTPUT);
  pinMode(motorEnable, OUTPUT);
  digitalWrite(motorEnable, HIGH);  // Enable motor driver
  
  Serial.begin(115200);
  
  // Initialize I2C with address 8 and set the receive event handler.
  Wire.begin(8);
  Wire.onReceive(receiveEvent);
  
  startPosition = abs(motor1Encoder.read());
  Serial.println("Starting state: QRCodeFind");
}

// --- Main Loop ---
void loop() {
  // In each state we may update our sensor values if new data has arrived.
  if (newData) {
    updateSensorData();
  }
  
  switch (currentState) {
    case QRCodeFind:
      // Rotate slowly clockwise looking for the QR code.
      setRotationClockwise(slowPWM);
      // Once new data arrives, stop motors and switch state.
      if (newData == false && (readFeet != 0.0 || readAngle != 0.0)) {
        // Data was updated via I2C
        stopMotors();
        Serial.println("QR Code found. Switching to AngleAdjust.");
        delay(2500);  // Brief delay to ensure stop
        currentState = AngleAdjust;
      }
      break;
      
    case AngleAdjust:
      // Coarse angle correction: rotate in the direction indicated by the angle.
      // If angle is negative: rotate counterclockwise; if positive: rotate clockwise.
      if (readAngle < 0) {
        setRotationCounterclockwise(adjustPWM);
      } else {
        setRotationClockwise(adjustPWM);
      }
      // When within 3 degrees, stop and move to fine adjustment.
      delay(200);
      stopMotors();
      delay(1500);

      if (abs(readAngle) <= 5.0) {
        stopMotors();
        Serial.println("Coarse angle adjusted. Switching to FineAngleAdjust.");
        delay(1000);
        currentState = FineAngleAdjust;
      }
      break;
      
    case FineAngleAdjust:
      // Fine adjustment using lower PWM values.
      if (readAngle < 0) {
        setRotationCounterclockwise(finePWM);
      } else {
        setRotationClockwise(finePWM);
      }
      delay(200);
      // When within 0.3° of alignment, stop and move forward.
      stopMotors();
      delay(1500);
      if (abs(readAngle) <= 1) {
        stopMotors();
        Serial.println("Angle fine-adjusted. Switching to MoveForward.");
        delay(1000);
        currentState = MoveForward;
      }
      break;
      
    case MoveForward:
      // Move forward toward the QR code.
      setForward(forwardPWM);
      // If the distance to the QR code is less than or equal to 1.5 feet, stop and switch state.
      if (readFeet <= 1.5 && readFeet > 0) {
        stopMotors();
        Serial.println("Reached target distance. Switching to ArrowRotation.");
        delay(1000);
        currentState = ArrowRotation;
        delay(500000000);
      }
      break;
      
    case ArrowRotation:
      // Rotate 90 degrees based on arrow data.
      // If redArrow (readColor) equals 0, rotate 90° counterclockwise; otherwise, clockwise.
      {
        long posCounts = abs(motor1Encoder.read()) - startPosition;  // Use encoder reading for rotation
        if (readColor == 0) {
          Serial.println("Arrow indicates: Rotate counterclockwise 90°.");
          // Call rotate function with dir = false (counterclockwise)
          // For counterclockwise rotation: motor1 backward, motor2 forward.
          digitalWrite(motor1Dir, LOW);
          digitalWrite(motor2Dir, HIGH);
          analogWrite(motor1PWM, 45);
          analogWrite(motor2PWM, 45);
        } else {
          Serial.println("Arrow indicates: Rotate clockwise 90°.");
          digitalWrite(motor1Dir, HIGH);
          digitalWrite(motor2Dir, LOW);
          analogWrite(motor1PWM, 45);
          analogWrite(motor2PWM, 45);
        }
        // After rotation, you may restart the state machine. Here we return to QRCodeFind.
        delay(rotationDelay);
        stopMotors();
        delay(500000);
        currentState = QRCodeFind;
        // Reset sensor values if needed.
        readFeet = 0.0;
        readAngle = 0.0;
        readColor = 0;
        startPosition = abs(motor1Encoder.read());
        Serial.println("Rotation complete. Restarting QRCodeFind.");
      }
      break;
  }
  delay(5); // Small loop delay for stability
}