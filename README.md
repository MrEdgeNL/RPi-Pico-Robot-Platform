# RPi-Pico-Robot-Platform

The RPi Pico W robot platform is alive.

The robot is a differential drive robot, including wireless communication over LAN (UPD messaging).
The main idea, is creating an architecture which could be re-used, when a drive train changes or when other peripherals are added.
For keeping everything simple, microPython is used on the Pico.

RPi Pico W is running:
* core 0: robot controller (loop, 50ms), managing:
  - real time controller: updating differential drive (takes 2~15ms),
  - real time controller: updating status led driver (<<ms).
* core 1: handle communication (loop; pose update ~250ms), via LAN.
* simple message queues (2x) (threat safe), so both processes could 'talk' to each other.

There is also a simple PC GUI (Tkinter), which:
* Send simple commands (buttons)
* Send user commands (textbox)
* Lists all in/out going messages (left side)
* Lists all 'LOG' (pose) updates from pico (right side)

## Robot platform:
![Screenshot robot platform.](/pics/RPi_Pico-W_Software_Architecture2.png)

## Coordinates & units used:
* Odometry: Absolute pose.
* Everything from GUI to robot.py: mm, deg, sec.
* Inside drive class: mm, rad, sec.
* Robot X: driving forwards. Theta+: CCW.

## Communication protocol:
Simple ascii message, format: "TARGET function [arg1, arg2]"
* TARGET: uppercase, 3 character string, identifies target of message:
  - MSG / LOG: data PICO → PC
  - RBT / DRV / LED: data PC → PICO-module
* function:
  - MSG: A message string.
  - LOG: format: microsecond pico controller, drive pose: (x [mm], y [mm], theta [deg]) (not strict yet)
  - to pico: lowercase, function name of module
* args: optional:
  - MSG & LOG: (currently) no arguments expected.
  - to pico: arguments necessary for current function.

### Some examples, send from GUI:
* "RBT stop": Stops all RTC activities and sets mode to 'Idle'.
* "RBT close": Closes the complete robot
* "DRV move_to 200,10,250": Move to location, with speed 250 mm/s.
* "DRV rotate_to -720,150": Depending on start angle, rotate 2 rotations CW, where the 150mm/s is converted to rad/sec.
* "DRV heading_to -90,150": Orientate robot to -90 degrees (shortest route), where the 150mm/s is converted to rad/sec.
* "DRV to_pose x,y,t,v": First do 'move_to' and finaly 'heading_to'.

The robot has the ability to store commands and execute these one after the other.
* "RBT add_cmd DRV,move_to,200,10,250"
* "RBT clear_cmds"; "RBT stop_cmds"; "RBT start_cmds"

The robot will transform new commands into 'GUI message string' and stores this in a special list.

## GUI:
![Screenshot gui.](/pics/RPi_Pico-W_Software_GUI_run_square.png)
Left side showing:
* Last bit of current drive settings (request by pressing Drive-Info button)
* New command was given: "RBT run_square 500,CCW,250"
* Right side is showing the final 2 pose updates for internal commands: "DRV move_to .." & " DRV heading_to ..".

## First steps...
[![first robot steps](/pics/RPi-Pico-Robot-Platform_youtube.jpg)](https://youtu.be/dP9MaJGwAcU)
Only calibration sofar: the average robot distance → ticks_per_mm.
Placing the robot in right orientation seems to be the hardest part.

## More information:
The why/how about this DIY adventure: [Pico/mPython – smart car DIY](https://retrobuildingtoys.nl/2024/rpi-pico-smart-car/).
