# 3-DoF Robotic Arm Control System (Bare-Metal C++)

A high-performance control firmware for a 3-Degree-of-Freedom (3-DoF) robotic manipulator implemented on the ATmega328P microcontroller without any high-level Arduino libraries.

## 📐 Kinematics & CAD Integration
* Inverse Kinematics (IK): Features a custom C++ mathematical engine using trigonometry (atan2, law of cosines) to calculate joint angles based on target spatial coordinates (X, Y, Z).
* CAD Constants: Link dimensions (\(L_1 = 100\) mm, \(L_2 = 120\) mm) derived from the Onshape CAD model are embedded directly into the firmware as geometric constraints.

## 💻 Low-Level Hardware Control (Bare-Metal)
* Hardware PWM Generation: Bypassed standard Arduino abstraction layers (Servo.h). Configured internal timers directly via microcode.
* Timer Registers: Utilized 16-bit Timer 1 (TCCR1A, TCCR1B, OCR1A, OCR1B) and 8-bit Timer 2 (TCCR2A, TCCR2B, OCR2A) to output hardware-accurate 50 Hz PWM signals (20ms period) required for high-torque MG996R servos.

## 🚀 Verification
* Fully verified and tested via hardware-in-the-loop emulation in the Wokwi simulation platform, confirming precise path generation and coordinate tracking.# RobotArm
3-DoF Robotic Arm control system based on ATmega328P registers (Bare-Metal C++) and Inverse Kinematics.
