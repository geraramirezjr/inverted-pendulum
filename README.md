# Inverted Pendulum
Project repository for a cart-pole inverted pendulum system. Designed hardware and software to support feedback control of belt gantry, DC motor, encoders, motor driver, and microcontroller. Developed a self swing up startup sequence and a double PID control algorithm with no external libraries that can successfully balance an inverted pendulum for 30+ seconds.

![hero](images/swingup_and_balance.gif)

## System Overview
System consists of a pendulum and encoder assembly driven via a linear belt gantry and DC motor controlled with an Arduino equivalent microcontroller.

### Electrical System
![|](images/electrical_schematic_detailed.png)

### Mechanical Hardware
![|](images/mechanical_hardware.png)
- [OpenBuilds](https://openbuilds.com/) Hardware: V-slot linear rail, V-slot gantry, brackets, fasteners, t-nuts
- Modified Stepper Brackets: DC motor and pendulum encoder mounting
- 3D Prints: Gantry and limit switch brackets
- Timing belt and pulleys
- Pendulum rod

## Software and Control

### Closed Loop Control

The main software loop running on the microcontroller consists of a double PID control scheme for pendulum angle and cart position. The previous error and current error for each position are used to perform numerical differentiation and numerical integration (midpoint integration). A combined error is then used to determine motor control.

<p align="center">
    <img src="images/double_PID_closed_loop_control.png" width="400px">
</p>

A few functions are used in support of the main loop operation:
- Motor Function: Used to control motor direction and current with a PWM signal
- Encoder Functions: Interrupt pins on the arduino are used to increment or decrement the encoder count
- Limit Switches: Used for system safety. If any limit switch interrupt is triggered, motor current set to zero.

### Swing Up Program

An initial swing up program is used to bring pendulum from dead hang to an inverted position. Desired swing up behaviour can roughly be viewed as a growing oscillatory sin wave where pendulum swings with more and more force from left to right and eventually snaps into a control loop once upright. Once inverted the program does not immediately enter the main double PID loop, but instead enters a single PID loop just controlling the pendulum angle until. During testing the single PID control proved to be more robust to catching a swing up, and once stable the program continues to the main loop.

<p align="center">
    <img src="images/swing_up_program_overview.png" height="200px">
    <img src="images/swing_up_angle_vs_time_ideal.png" height="200px">
</p>

### Balancing
![hero](images/balance.gif)
