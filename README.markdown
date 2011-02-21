Spin Doctor - Arduino Tachometer/Speedometer with interactive LCD.
==================================================================

This circuit and firmware is used with a rotating magnet and fixed
hall-effect sensor to sense revolutions per minute, and display RPM
and surface speed (in several units) on an LCD.  Alternatively an
optical sensor may be used.

This project is intended for retrofitting digital display to rotary
tools such as lathes and saws.  It can also can be used for vehicle
speedometer/odometer or bike computer.

I was motivated to construct this project to confirm my estimation of the
undocumented output speeds of my second hand bandsaw and lathe.   

On the theory that anything worth doing is worth overdoing, I decided
to generalise it to act as a speedometer/odometer as well as tachometer.

Angular velocity is measured and displayed in RPM and Hz.

Using an entered diameter value, tangential (or road) speed is 
output in one of:

  - metres per minute
  - feet per minute
  - kilometres per hour
  - miles per hour



Hardware Overview
-----------------

### Input

### Output 

Firmware Overview
-----------------

### Pin assignment and setup

### Interrupts

### Main loop