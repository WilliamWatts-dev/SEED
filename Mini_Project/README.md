# Mini Project  

## Features  

- Detect a marker in the image.  
- Continuously display the image with the marker's position.  
- Show wheel status on an LCD using threading.
- Send position to Arduino via IC2 transfer.
- Turn robot's wheels accordingly to represent marker's current quadrant

## Files  

- **Marker_Detect.py** – Main script for marker detection and display.
- **Mini_Project.ino** - Script for taking input from .py file and moving the robot's wheels accordingly
- **motor_control.slx** - Simulink file for simulating the motor control subsystem
- **PID_Motor_Control.slx** - Simulink file for simulating the motor control system with an added PID controller
- **PID_Tuner.m** - Matlab file for simulating the motor and demonstrating the system step response with and without the PI controller
- **README.md** – Project documentation.  

## Matlab Simulation Results  

![image](https://github.com/user-attachments/assets/8ed0ab4a-2e46-4bdd-be60-265e2de126c4)
