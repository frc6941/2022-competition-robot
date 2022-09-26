# IronPulse Robotics Team 6941 2022 Competition Robots

## Code Structure

1. Subsystems: the collection with the basic hardware, along with the basic functions that controls them.
2. Coordinators: higher level abstractions of multiple subsystem, carrying out a coordinated behavior.
3. Control Board: driver input related abstraction.
4. Auto: autonomous modes and routines.
5. Utils: convenience tools.

## Units and Positive Directions

1. Use METER as the unit of length. NO INCHES, FEET AND YARDS. NO CENTIMETERS AND MILLIMETERS. Transition will be stated explicitly.
2. Use DEGREE as the unit of angle. NO RADIANS.
3. Take COUNTER-CLOCKWISE (abbreviated as CCW) as the positive direction of rotation. NO CLOCKWISE (abbreviated as CW) unless stated explicitly.
4. Use RPM as the unit for rotational velocity.
