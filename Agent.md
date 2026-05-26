# AGENT.md

## Project Overview

This project is a small Arduino-controlled car propelled by a piston moving back and forth.

The Arduino controls:

- A MOSFET on **Pin 2**
- A steering servo on **Pin 3**
- A left rear wheel limit switch on **Pin 4**
- A right rear wheel limit switch on **Pin 5**
- The Compass is at SCL **PinA5**, SDA **PinA4**, The package is LSM6, by Pololu, LIS3MDL, by Pololu, only x, y direction is needed

The MOSFET controls the piston actuator.  
The servo controls steering, the code should strictly limit the angle between 65-125, otherwise it will break the steering system, also note that the steering is not symmetric on each sides.  
The rear wheel limit switches are used only for monitoring wheel rotation.
When the solenoid is on, the compass is interfered, so don't rely on the reading when it's on

---

## Pin Assignments

| Function | Arduino Pin | Required Mode |
|---|---:|---|
| MOSFET / Piston Control | 2 | `OUTPUT` |
| Steering Servo | 3 | Servo output |
| Left Rear Wheel Limit Switch | 4 | `INPUT_PULLUP` |
| Right Rear Wheel Limit Switch | 5 | `INPUT_PULLUP` |

---

## Critical Limit Switch Safety Rule

The left and right rear wheel limit switches must be configured as input pins using the Arduino internal pull-up resistors.

Correct:

```cpp
pinMode(LEFT_REAR_LIMIT_PIN, INPUT_PULLUP);
pinMode(RIGHT_REAR_LIMIT_PIN, INPUT_PULLUP);