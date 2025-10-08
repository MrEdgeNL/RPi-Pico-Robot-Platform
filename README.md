# RPi-Pico-Robot-Platform

The RPi Pico W robot platform is alive.

The robot is a differential drive robot, including wireless communication over LAN (UPD messaging).
The main idea, is creating an architecture which could be re-used, when a drive train changes or when other peripherals are added.
For keeping everything simple, microPython is used on the Pico.

RPi Pico W is running:
* core 0: robot controller (loop, 50ms), managing:
  - real time controller: driving differential drive
  - real time controller: onboard led driver.
* core 1: handle communication (loop, pose update ~250ms), via LAN.
* message queues (2x) (threat safe), so both cores could 'talk' to each other.

There is also a simple PC GUI (Tkinter), which:
* Send simple commands (buttons)
* Send user commands (textbox)
* Lists all in/out going messages (left side)
* Lists all pose updates from pico (right side)

## Robot platform:
Graphical:
![Screenshot robot platform.](/pics/RPi_Pico-W_Software_Architecture2.png)

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

## Coordinates & units used:
* Odometry: absolute.
* Everything from GUI to robot.py: mm, deg, sec.
* Class drive.py: mm, rad, sec.

## Differential drive functionality:
__set_pwm(left_pwm=0, right_pwm=0):_
* Directly sets both motor pwm's. No further RTC control.

__get_deltaticks():_
* Read deltaticks from both encoders, since last _get_deltaticks():_.

_update():_
* Called periodically from robot.py loop:
1. Updates current pose (always)
2. Depending on RTC functionality: calculate new wheel speed & update with _set_pwm()_.

_stop():_
* Stops motor movement: _set_pwm(0,0).
* Disable RTC functionality.
* Set mode="idle".

_reset():_
* Call _stop()_
* Resets: current pose, goal, pid controllers, target speeds etc.

_close():_
* Stops motor movement: _set_pwm(0,0).
* Disable RTC functionality.

_set_wheel_speed(v_l=0.0, v_r=0.0):_
* If RTC is not yet set, reset PID controllers and set mode="direct"
* Execute PID control on wheel target speeds, during _update()_.

set_velocity(robot_v=0.0, robot_omega=0.0):
* Calculate target wheel speeds, based on wanted robot speeds.
* Inconsistancy(?): this function assumes already a running RTC flag.

_move_to(target_x=0.0, target_y=0.0, target_v=0.0):_
* Set goal pose (x,y), but not final heading, set mode="move_to" and set RTC flag high.

_rotate_to(target_theta_rad=0.0, target_v=0.0):_
* Set goal heading (theta), could be multiple rotations, set mode="rotate_to" and set RTC flag high.
  
_heading_to(target_theta_rad=0.0, target_v=0.0):_
* Set goal heading (theta), shortest route compared to current pose, set mode="heading_to" and set RTC flag high.

_to_pose(target_x=0.0, target_y=0.0, target_theta_rad=0.0, target_v=0.0):_
* Set goal pose (x,y,theta), set mode="to_pose" and finalize with set mode="heading_to". Sets RTC flag high.

And some drive settings:
* _calibrate_drive(ticks_to_mm=0.0, wheelbase_mm=0.0): #ticks_to_deg=0.0):
* set_rtc_update_period(update_period_ms=50):
* set_motor_pwm_range(pwm_min=0, pwm_max=65536):
* set_motor_pid(Kp=1.0, Ki=0.0, Kd=0.0):
* set_goal_seeker(Kp_speed=1.0, Kp_heading=1.0):
* set_goal_tolerances(tolerance_dist_mm=15.0, tolerance_heading_deg=5.0):_
 
## More information:
The why/how about this DIY adventure: [Pico/mPython – smart car DIY](https://retrobuildingtoys.nl/2024/rpi-pico-smart-car/).
