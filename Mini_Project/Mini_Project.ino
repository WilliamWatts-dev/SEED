// Arduino pin connections: 
#include <Encoder.h> // Use the Encoder library for handling encoders
#include <Wire.h> // Include the Wire library for I2C

const float pos_ONE = 0.76; // sets the wheel to the '1' position
const float pos_ZERO = 0; // sets the wheel to the '0' position

// Define pins for Motor 1
const int motor1PinA = 3; // Encoder A output
const int motor1PinB = 6; // Encoder B output
const int motor1PWM = 9;  // PWM signal for motor speed
const int motor1Dir = 7;  // Direction pin

// Define pins for Motor 2
const int motor2PinA = 2; // Encoder A output
const int motor2PinB = 5; // Encoder B output
const int motor2PWM = 10; // PWM signal for motor speed
const int motor2Dir = 8;  // Direction pin

// Motor enable pin
const int motorEnable = 4;

// Encoder resolution and gear ratio
const int encoderResolution = 64; // Counts per revolution of the motor shaft
const int gearRatio = 50;         // Gear ratio
const int countsPerRevolution = encoderResolution * gearRatio;

// Timing variables
const float desired_Ts = 10.0 / 1000.0; // Sample time in seconds (10 ms)
unsigned long last_time_ms, start_time_ms;
float current_time;

// Encoder objects
Encoder motor1Encoder(motor1PinA, motor1PinB);
Encoder motor2Encoder(motor2PinA, motor2PinB);

// Motor 1 Variables
float prevPos1_rad = 0.0, velocity1 = 0.0;
float integral_error1 = 0.0; // Integral error for motor 1

// Motor 2 Variables
float prevPos2_rad = 0.0, velocity2 = 0.0;
float integral_error2 = 0.0; // Integral error for motor 2

// Desired positions (0 or 1) for both motors
float desiredPos1 = 0; // Initial desired position for motor 1 (0 = starting, 1 = half rotation)
float desiredPos2 = 0; // Initial desired position for motor 2 (0 = starting, 1 = half rotation)

// Desired velocities (rad/s) for both motors
float desiredVelocity1 = 0; // Initial desired velocity for motor 1
float desiredVelocity2 = 0; // Initial desired velocity for motor 2

// Control gains for PI controller
float Kp_pos = 3.5;   // Proportional gain for position control
float Ki_pos = 0.7;    // Integral gain for position control

// Battery voltage
const float Battery_Voltage = 7.8;

// Control voltage for motors
float appliedVoltage1 = 0.0;
float appliedVoltage2 = 0.0;

// PWM values for motors
unsigned int PWM1 = 0;
unsigned int PWM2 = 0;

// Quadrant value inputted from raspberry pi
int quadrant = 0;

void setup() {
    // Set motor pins as outputs
    pinMode(motor1PWM, OUTPUT);
    pinMode(motor1Dir, OUTPUT);
    pinMode(motor2PWM, OUTPUT);
    pinMode(motor2Dir, OUTPUT);
    pinMode(motorEnable, OUTPUT);

    // Enable motors
    digitalWrite(motorEnable, HIGH);

    // Initialize Serial communication for debugging
    Serial.begin(115200);
    Serial.println("Time(s)\tVoltage1(V)\tVelocity1(rad/s)\tVoltage2(V)\tVelocity2(rad/s)"); // Column headers

    // Join I2C bus as slave with address 20
    Wire.begin(0x20);

    // Initialize timing variables
    last_time_ms = millis();
    start_time_ms = last_time_ms;
}

void loop() {
    // Get current time
    current_time = (float)(millis() - start_time_ms) / 1000.0;

    if (Wire.available()) { // loop through all but the last
      quadrant = Wire.read(); // receive pi values as uint8_t values
    }

    // Switch statement for recieving CV information.
    switch (quadrant) {
      case 0: // Northeast or Default
        desiredPos1 = pos_ZERO; // 0
        desiredPos2 = pos_ZERO; // 0
      case 1: // Northwest
        desiredPos1 = pos_ZERO; // 0
        desiredPos2 = pos_ONE; // 1
      case 2: // Southeast
        desiredPos1 = pos_ONE; // 1
        desiredPos2 = pos_ZERO; // 0
      case 3: // Southwest
        desiredPos1 = pos_ONE; // 1
        desiredPos2 = pos_ONE; // 1
    }
/* code for deugging desired position
    // Set desired positions based on time (0 or 1 for each motor)
    if (current_time <= 1.0) {
        desiredPos1 = desiredPos2 = pos_ZERO; // Starting position
    }
    else if (current_time <= 6.0 ) {
        desiredPos1 = pos_ZERO; // 1
        desiredPos2 = pos_ONE; // 1
    }
    else {
        desiredPos1 = desiredPos2 = pos_ZERO; // Back to starting position
    }
*/
    // Read motor position (in counts) and convert to radians
    long pos1_counts = motor1Encoder.read();
    float pos1_rad = 2 * PI * (float)pos1_counts / (countsPerRevolution * 4);

    long pos2_counts = motor2Encoder.read();
    float pos2_rad = 2 * PI * (float)pos2_counts / (countsPerRevolution * 4);

    // Compute velocity (rad/s)
    velocity1 = (pos1_rad - prevPos1_rad) / desired_Ts;
    prevPos1_rad = pos1_rad; // Store current position for next iteration

    velocity2 = (pos2_rad - prevPos2_rad) / desired_Ts;
    prevPos2_rad = pos2_rad; // Store current position for next iteration

    // Position control for Motor 1
    float pos_error1 = desiredPos1 - pos1_rad; // Position error
    integral_error1 += pos_error1 * desired_Ts; // Integrate position error
    float desiredSpeed1 = Kp_pos * pos_error1 + Ki_pos * integral_error1; // PI control for motor 1
    float error1 = desiredSpeed1 - velocity1; // Velocity error
    appliedVoltage1 = Kp_pos * error1; // Proportional control for motor 1

    // Position control for Motor 2
    float pos_error2 = desiredPos2 - pos2_rad; // Position error
    integral_error2 += pos_error2 * desired_Ts; // Integrate position error
    float desiredSpeed2 = Kp_pos * pos_error2 + Ki_pos * integral_error2; // PI control for motor 2
    float error2 = desiredSpeed2 - velocity2; // Velocity error
    appliedVoltage2 = Kp_pos * error2; // Proportional control for motor 2

    // Check the sign of voltage and set the motor driver sign pin as appropriate for Motor 1
    if (appliedVoltage1 > 0) {
        digitalWrite(motor1Dir, HIGH);
    } else {
        digitalWrite(motor1Dir, LOW);
    }

    // Check the sign of voltage and set the motor driver sign pin as appropriate for Motor 2
    if (appliedVoltage2 > 0) {
        digitalWrite(motor2Dir, HIGH);
    } else {
        digitalWrite(motor2Dir, LOW);
    }

    // Apply the requested voltage, up to the maximum available for Motor 1
    PWM1 = 255 * abs(appliedVoltage1) / Battery_Voltage;
    // Do not send voltage greater than 7.8V
    if (appliedVoltage1 >= Battery_Voltage) { appliedVoltage1 = Battery_Voltage; }
    PWM1 = min(PWM1, 255); // Ensure the PWM is within range
    analogWrite(motor1PWM, PWM1);

    // Apply the requested voltage, up to the maximum available for Motor 2
    PWM2 = 255 * abs(appliedVoltage2) / Battery_Voltage;
    // Do not send voltage greater than 7.8V
    if (appliedVoltage2 >= Battery_Voltage) { appliedVoltage2 = Battery_Voltage; }
    PWM2 = min(PWM2, 255); // Ensure the PWM is within range
    analogWrite(motor2PWM, PWM2);

    // Print Time, Voltage, and Velocity for both motors on the same line
    Serial.print(current_time, 3);
    Serial.print("\t");
    Serial.print(appliedVoltage1, 2);
    Serial.print("\t");
    Serial.print(velocity1, 4);
    Serial.print("\t");
    Serial.print(appliedVoltage2, 2);
    Serial.print("\t");
    Serial.println(velocity2, 4);

    // Ensure loop runs at the correct sample rate
    while (millis() < last_time_ms + desired_Ts * 1000) {
      // Do nothing (just wait to maintain the loop frequency)
    }
    last_time_ms = millis(); // Update time
}
