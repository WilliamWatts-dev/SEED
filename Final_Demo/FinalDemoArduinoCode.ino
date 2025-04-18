/* ISSUES TO BE SOLVED BEFORE FINAL DEMO:
      1.) When in the QRFind state, one motor doesn't turn as much as the other. 
        Turn up PWM voltage to it or turn down PWM voltage from more powerful motor?
      2.) When adjusting the angle, it does it in large increments. 
        Add some term that takes the angle of error and converts to a delay time, effectively angle?
        (Smaller delays with smaller angles, larger delays with larger angles effectively)
      3.) Robot drifts while moving forward. 
        Could be corrected with an if statement in the MoveForward state that checks the angle. 
        If too high, reduce PWM on one of the motors until within acceptable boundary.
      4.) Delays are very long and frequent, and the robot is slow. 
        Could be fixed by increasing PWM values to just keep friction, decrease delays to exactly camera delay, decrease Pi delay?
      5.) Final Demo will include a QR Code with no arrow. 
        Code only takes into account when there is a red arrow or when there isn't. 
        Will need 2 bits for Red arrow, Green arrow, and No arrow.
        (Use last bit for error state?)
      6.) The robot does not indicate when it is finished.
        Code should be written for the RobotFinished state.
*/

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
const int slowPWM    = 32;  // PWM for QRCodeFind rotation (coarse search)
const int adjustPWM  = 35;  // PWM for coarse angle adjustment
const int finePWM    = 35;   // PWM for fine angle adjustment
const int forwardPWM = 55;  // PWM for moving forward
const int rotationDelay = 2900;

// --- Motor Control Helper Functions ---
void setRotationClockwise(int pwm) {
  // For clockwise rotation: motor1 forward, motor2 backward.
  digitalWrite(motor1Dir, HIGH);
  digitalWrite(motor2Dir, LOW);
  analogWrite(motor1PWM, pwm*1.28);
  analogWrite(motor2PWM, pwm);
}

void setRotationCounterclockwise(int pwm) {
  // For counterclockwise rotation: motor1 backward, motor2 forward.
  digitalWrite(motor1Dir, LOW);
  digitalWrite(motor2Dir, HIGH);
  analogWrite(motor1PWM, pwm*1.28);
  analogWrite(motor2PWM, pwm);
}

void setForward(int pwm) {
  // For forward motion: both motors forward.
  digitalWrite(motor1Dir, LOW);
  digitalWrite(motor2Dir, LOW);
  analogWrite(motor1PWM, pwm*1.28);
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
    // DEBUGGING: code prints the data buffer to terminal.
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

// --- State Machine Setup ---
enum RobotState {
  QRCodeFind,      // 0: Robot rotates slowly looking for the QR code.
  AngleAdjust,     // 1: Coarse rotation based on the received angle.
  FineAngleAdjust, // 2: Fine rotation (with lower PWM) until within 0.3°.
  MoveForward,     // 3: Move forward toward the QR code.
  ArrowRotation,   // 4: Rotate 90° based on arrow data.
  RobotFinished    // 5: Robot has finished the course.
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
      // Rotate slowly counterclockwise looking for the QR code. (As specified by the Final Demo Instructions)
      setRotationCounterclockwise(slowPWM);
      // Once new data arrives, i.e. QR code is found, stop motors and switch state.
      if (newData == false && (readFeet != 0.0 || readAngle != 0.0)) { // No new data, but we have feet/angle measurements.
        // Data was updated via I2C
        stopMotors();
        Serial.println("QR Code found. Switching to AngleAdjust.");
        delay(1500);  // Brief delay to ensure stop
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
      delay(200); // Time the motors are on
      stopMotors();
      delay(1500); // Delay for camera to catch up

      // When within fine degree threshold, stop and move to fine adjustment.
      // Otherwise, just keep adjusting.
      if (abs(readAngle) <= 5.0) {
        stopMotors();
        Serial.println("Coarse angle adjusted. Switching to FineAngleAdjust.");
        delay(1000); // Delay for camera to catch up
        currentState = FineAngleAdjust;
      }
      break;
      
    case FineAngleAdjust:
      // Fine adjustment using lower PWM values, and less time. Creates smaller/finer angle adjustments.
      if (readAngle < 0) {
        setRotationCounterclockwise(finePWM);
      } else {
        setRotationClockwise(finePWM);
      }
      delay(150); // Time the motors are on
      stopMotors();
      delay(1500); // Delay for camera to catch up
      // When within 1° of alignment, stop and move forward.
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
      }
      break;
      
    case ArrowRotation:
      // Rotate 90 degrees based on arrow data.
      // If redArrow (readColor) equals 0, rotate 90° counterclockwise; otherwise, clockwise.
      {
        if (readColor == 0) { // Red Arrow
          Serial.println("Arrow indicates: Rotate counterclockwise 90°.");
          // For counterclockwise rotation: motor1 backward, motor2 forward.
          setRotationCounterclockwise(45);
        } else if (readColor == 1) { // Green Arrow
          Serial.println("Arrow indicates: Rotate clockwise 90°.");
          setRotationClockwise(45);
        } else if (readColor == 2){ // No Arrow
          // Robot is finished, make it do a cool dance or something.
          Serial.println("No Arrow Detected, Assuming Course is Finished.");
          currentState = RobotFinished;
          break;
        }
        else {
          Serial.println("No State Found");
        }
        // After rotation, you may restart the state machine.
        delay(rotationDelay); // How long it takes to turn 90 degrees approximately. 
        stopMotors();
        currentState = AngleAdjust; // Move back to Angle Adjust state.
        // Reset sensor values if needed.
        readFeet = 0.0;
        readAngle = 0.0;
        readColor = 0;
        Serial.println("Rotation complete. Restarting QRCodeFind.");
        delay(1500);  // Delay for camera to catch up and read QR code
      }
      break;

  
  case RobotFinished: 
    // Have the robot do a cool dance or something. Maybe turn on an LED to indicate we're finished.
    {
      // Cool dance code.
    }
  delay(5); // Small loop delay for stability
  }
}
