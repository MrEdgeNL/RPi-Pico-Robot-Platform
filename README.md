# RPi-Pico-Robot-Platform

Finaly build my first RPi Pico robot platform.

It uses the RPi pico for driving differential drive robot and able to communicate over LAN with a PC.
It (should) use both cores on the pico:
* core 0: robot controller (loop), managing:
*   real time controller: driving differential drive
*   real time controller: onboard led driver.
* core 1: handle communication, via LAN.
* using 2 message queues (threat safe), so both cores could 'talk' to each other.

There is also a simple PC GUI, which:
* Send simple commands (buttons)
* Send user commands (textbox)
* Lists all in/out going messages (left side)
* Lists all pose updates from pico (right side)

Robot platform:
![Screenshot robot platform.](/pics/RPi_Pico-W_Software_Architecture2.png)

Communication protocol:
For now simple asci message: "TARGET function [arg1, arg2]"
* TARGET: 3 character string, identifies target:
*   MSG / LOG: data PICO → PC
*   RBT / DRV / LED: data PC → PICO-module
* function:
*   MSG: A message string.
*   LOG: format: microsecond pico controller, drive pose: (x [mm], y [mm], theta [deg]) (not strict yet)
*   to pico: function name of module
* args: optional:
*   MSG & LOG: (currently) no arguments expected.
*   to pico: arguments necessar for current function.

Units used:
* Everything from GUI to robot.py: mm, deg, sec.
* Module: drive.py: mm, rad, sec.

More information (why/how) about this DIY adventure: [Pico/mPython – smart car DIY](https://retrobuildingtoys.nl/2024/rpi-pico-smart-car/).
