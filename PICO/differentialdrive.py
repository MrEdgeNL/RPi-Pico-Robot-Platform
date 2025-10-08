        
import math, time, _thread
from lib.motor import Motor
from lib.encoder_portable import Encoder

# ----------------------------
# Utility: Math functions
# ----------------------------
def hypot(a,b):
    return math.sqrt(a*a+b*b)

def wrap(angle):
    #dtheta_deg = abs((angle + 540) % 360 - 180)  # shortest signed diff in degrees
    #return math.atan2(math.sin(angle), math.cos(angle))  # shortest signed diff in radianss
    return (angle + math.pi) % (2 * math.pi) - math.pi    #np variant.
    
def clip(v, vmin, vmax):
    if v < vmin: return vmin
    if v > vmax: return vmax
    return v

class pose:
    def __init__(self, x=0.0, y=0.0, theta_rad=0.0):
        self.x = x
        self.y = y
        self.theta = theta_rad  #rad

    def __str__(self):        
        #return f"Pose: ({self.x:.1f}, {self.y:.1f}, {math.degrees(self.theta):.1f})"
        return f"Pose: ({int(self.x)}, {int(self.y)}, {int(math.degrees(self.theta))})"

# ----------------------------
# Utility: PID controller
# ----------------------------
class PID:
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd        
        self.integral = 0.0
        self.prev_err = 0.0
        self.initialized = False

    def reset(self):
        self.integral = 0.0
        self.prev_err = 0.0
        self.initialized = False

    def update(self, setpoint, measurement, dt=0.02, debug=False):
        err = setpoint - measurement
        if not self.initialized:
            self.prev_err = err
            self.initialized = True
        # PID terms
        self.integral += err * dt
        deriv = (err - self.prev_err) / dt
        self.prev_err = err
        out = self.Kp*err + self.Ki*self.integral + self.Kd*deriv      
        if debug:
            #print(f">debug: PID {err} {self.Kp*err:.1f} {self.Ki*self.integral:.1f} {self.Kd*deriv:.1f}")  
            print(f">debug: PID err: {err:.1f}; out: {out}")  
        return out




# ----------------------------
# Main Driver controller:
#   - SetPWM(pwm, pwm)
#   - SetSpeed(v, v)
#   - SetVelocity(v, o)
#   - MoveTo(x,y,t)
# ----------------------------
class Drive:
    def __init__(self): #q_out...
        # Pose state        
        self.pose = pose(0.0, 0.0, 0.0)
        self.goal = pose(0.0, 0.0, 0.0)
        self._lock = _thread.allocate_lock()

        # Configuration
        self.ticks_to_dist = 0.163760284  # [mm/tick]
        self.wheel_base = 100.0       # [mm]
        #self.wheel_diameter = 42.5    # [mm]

        # Internal state
        self._mode = "idle"  # direct, move, rotate
        self._do_rtc = False

        # Store last delta steps L & R (for debug only)
        self.dticks_l = 0
        self.dticks_r = 0        
        
        # Wheel speed estimation [mm/s]
        self.v_l_meas = 0.0
        self.v_r_meas = 0.0

        # Wheel speed targets [mm/s]
        self.v_l_target = 0.0
        self.v_r_target = 0.0

        # PWM limits
        self.pwm_min = 10000                # ~int(0.15 * 65536)
        self.pwm_max = 40000
        self.pwm_l = 0
        self.pwm_r = 0        

        # PID controller settings
        self.update_period_ms = 50                  # update interval (~dt)
        self._pid_control_l = PID( 50 , 100 , 0 )
        self._pid_control_r = PID( 50 , 100 , 0 )       
        self.last_update_ms = time.ticks_ms()       # needed for speed control

        # Goal settings
        self.Kp_speed = 2.5
        self.Kp_heading = 30
        self.tolerance_dist = 15                    # happy closing position within X [mm] radius
        self.tolerance_heading = math.radians(5)    # happy closing heading within X [rad]
        self.v_goal = 0.0
        self.w_goal = 0.0
        self._last_goal_dist = 0.0

        # Hardware classes:
        self.MtrA = Motor(8,9)
        self.MtrB = Motor(6,7)
        self.EncA = Encoder(10,11) 
        self.EncB = Encoder(12,13) 


    def _get_pwm(self, value=0):
        #if abs(value) < self.pwm_min/2:
        if value != 0:
            pwm = min(max(abs(value), self.pwm_min),self.pwm_max)  # ← [0, min..max]
            return int( math.copysign( pwm, value) )
        else:
            return 0

    # --------------------------
    # High-level interface
    # --------------------------
    def _set_pwm(self, left_pwm=0, right_pwm=0):
        # apply deadband compensation if non-zero intended speed
        self.pwm_l = self._get_pwm(left_pwm)
        self.pwm_r = self._get_pwm(right_pwm)
        self.MtrA.set_motor_pwm(self.pwm_l)
        self.MtrB.set_motor_pwm(self.pwm_r)
        #if self._send_debug:
        #    print(f">PWM: {left_pwm}; {right_pwm}.")

    def _get_deltaticks(self):
        return self.EncA.delta_ticks(), self.EncB.delta_ticks()

    def stop(self):        
        self._do_rtc = False  
        self._set_pwm(0,0)        
        self.v_l_target = 0
        self.v_r_target = 0
        self._mode = "idle"
        with self._lock:            
            self.pose.theta = wrap(self.pose.theta) #reset angle to: +/-pi.

    def reset(self):
        self.stop()
        self.set_current_pose(0.0,0.0,0.0)
        self.goal = pose(0.0,0.0,0.0)
        self._pid_control_l.reset()
        self._pid_control_r.reset()
        self.v_goal = 0.0
        self.w_goal = 0.0
        self._last_goal_dist = 0.0        
        self.e_angl = 0.0 
        self.e_dist = 0.0

    def close(self):
        self._do_rtc = False
        self._set_pwm(0,0)
        self._mode = "closing"

    def set_wheel_speed(self, v_l=0.0, v_r=0.0):
        """Set wheel speed targets [mm/s]."""
        self.v_l_target = v_l
        self.v_r_target = v_r
        if not self._do_rtc:  #basically setting new speed target
            self._mode = "direct"
            self._pid_control_l.reset()
            self._pid_control_r.reset()            
            self._do_rtc = True

    def set_velocity(self, robot_v=0.0, robot_omega=0.0):
        """ Set linear and angular velocity: v [mm/s], omega [rad/s] """
        self.v_l_target = robot_v - (robot_omega * self.wheel_base / 2.0)
        self.v_r_target = robot_v + (robot_omega * self.wheel_base / 2.0)        
    
    # 
    def _rtc_innitialize(self):
        self._pid_control_l.reset()
        self._pid_control_r.reset()
        self._goal_last_dist = 0.0
        self.w_goal = self.v_goal / self.wheel_base #rad/sec.
        with self._lock:            
            self.pose.theta = wrap(self.pose.theta) #reset angle to: +/-pi.        
        self._do_rtc = True

    def move_to(self, target_x=0.0, target_y=0.0, target_v=0.0):        
        self.goal = pose(target_x, target_y, 0)
        self.v_goal = target_v        
        self._mode = "move_to"
        self._rtc_innitialize()

    def rotate_to(self, target_theta_rad=0.0, target_v=0.0):           
        self.goal = pose(0, 0, target_theta_rad)        
        self.v_goal = 2*target_v        
        self._mode = "rotate_to"
        self._rtc_innitialize()
        
    def heading_to(self, target_theta_rad=0.0, target_v=0.0):           
        self.goal = pose(0, 0, target_theta_rad)
        self.v_goal = 2*target_v        
        self._mode = "heading_to"
        self._rtc_innitialize()

    def to_pose(self, target_x=0.0, target_y=0.0, target_theta_rad=0.0, target_v=0.0):           
        self.goal = pose(target_x, target_y, target_theta_rad)
        self.v_goal = target_v        
        self._mode = "to_pose"
        self._rtc_innitialize()

    # --------------------------
    # Periodic update (pose + speed + PID control)
    # Returns True if still active.
    # --------------------------
    def update(self):
        # ---- Update internal dt ----
        now = time.ticks_ms()
        if time.ticks_diff(now, self.last_update_ms) < self.update_period_ms:
            return False
        dt = time.ticks_diff(now, self.last_update_ms) / 1000.0
        self.last_update_ms = now

        # ---- Read encoders, update delta position ----
        with self._lock:
            self.dticks_l , self.dticks_r = self._get_deltaticks()        
        dl_mm = self.dticks_l * self.ticks_to_dist
        dr_mm = self.dticks_r * self.ticks_to_dist

        # ---- Update odometry ----
        dcenter = (dl_mm + dr_mm) / 2.0
        dtheta_rad_ave = (dr_mm - dl_mm) / self.wheel_base ####/2
        with self._lock:            
            self.pose.theta += dtheta_rad_ave             
            self.pose.x += dcenter * math.cos(self.pose.theta)  # calce new pose on average direction (=delta_theta/2)
            self.pose.y += dcenter * math.sin(self.pose.theta)
            #self.pose.theta += dtheta_rad_ave                   # add 2nd half. Doesnot seem to improve...

        # ---- Check if RTC is running ----
        if not self._do_rtc: 
            with self._lock:            
                self.pose.theta = wrap(self.pose.theta)         #reset angle to: +/-pi.
            return False

        # ---- RTC: Calculate current wheel speeds ----
        self.v_l_meas = int(dl_mm / dt) #if dt > 0 else 0
        self.v_r_meas = int(dr_mm / dt) #if dt > 0 else 0

        # ---- If position/heading → calculate setpoint wheel speeds ----
        if self._mode != "direct":                          
            dx = self.goal.x - self.pose.x
            dy = self.goal.y - self.pose.y

            if self._mode in ["to_pose", "move_to"]:  # move_to / to_pose:
                ang_pose_to_goal = math.atan2(dy, dx)
                self.e_angl = wrap(ang_pose_to_goal - self.pose.theta)
                self.e_dist = math.sqrt(dx*dx + dy*dy)
                if self.e_dist < self.tolerance_dist:          # 2. then check distance.
                    if self._mode == "to_pose":             # 3a. done moving, check heading.
                        self._mode = "heading_to"
                    else:                                   # 3b. DONE moving (heading is not relevant)
                        self._do_rtc = False

            if self._mode == "heading_to":       
                self.e_dist = 0
                self.e_angl = wrap(self.goal.theta - self.pose.theta)
                if abs(self.e_angl) < self.tolerance_heading:    # 1. check final heading.
                    self._do_rtc = False

            if self._mode == "rotate_to":       
                self.e_dist = 0
                self.e_angl = float(self.goal.theta - self.pose.theta)
                if abs(self.e_angl) < self.tolerance_heading:    # 1. check final heading.
                    self._do_rtc = False

            # Set new v_TARGETs:
            v_lin = clip(self.Kp_speed * self.e_dist, -self.v_goal, self.v_goal)  # mm/s
            w_rad_s = clip(self.Kp_heading * self.e_angl, -self.w_goal, self.w_goal)  # rad/s            
            self.set_velocity(v_lin, w_rad_s)            

        # ---- RTC: PID control (convert v_TARGETs -> PWMs) ----
        pwm_l = 0
        pwm_r = 0
        if self._do_rtc:                                  # RTC: update pwm
            if self.v_l_target != 0:
                pwm_l = int(self._pid_control_l.update(self.v_l_target, self.v_l_meas, dt) + math.copysign(self.pwm_min ,self.v_l_target) )
            if self.v_r_target != 0:
                pwm_r = int(self._pid_control_r.update(self.v_r_target, self.v_r_meas, dt) + math.copysign(self.pwm_min ,self.v_r_target) )
        else:                                               # RTC: reached target:
            #with self._lock:            
            #    self.pose.theta = wrap(self.pose.theta)     #reset angle to: +/-pi.
            print(f">RTC finalized: {self._mode}.")
            self._mode = "done"

        # ---- RTC: Send pwm to motors ----
        self._set_pwm(pwm_l, pwm_r)

        return self._do_rtc


    # --------------------------
    # Config / Calibration
    # --------------------------
    # def config_drive(self, wheelbase_mm=0.0, wheel_diameter_mm=0.0):
    #     self.wheel_base = wheelbase_mm
    #     self.wheel_diameter = wheel_diameter_mm

    def calibrate_drive(self, ticks_to_mm=0.0, wheelbase_mm=0.0): #ToDo: ticks_to_deg=0.0):
        self.ticks_to_dist = ticks_to_mm
        self.wheel_base = wheelbase_mm

    def set_rtc_update_period(self,  update_period_ms=100):
        self.update_period_ms = update_period_ms
    
    def set_motor_pwm_range(self, pwm_min=0, pwm_max=65536):
        self.pwm_min = pwm_min
        self.pwm_max = pwm_max

    def set_motor_pid(self, Kp=1.0, Ki=0.0, Kd=0.0):
        self._pid_control_l = PID( Kp , Ki , Kd )
        self._pid_control_r = PID( Kp , Ki , Kd )

    def set_goal_seeker(self, Kp_speed=1.0, Kp_heading=1.0):
        self.Kp_speed = Kp_speed
        self.Kp_heading = Kp_heading

    def set_goal_tolerances(self, tolerance_dist_mm=0.0, tolerance_heading_deg=0.0):
        self.tolerance_dist = tolerance_dist_mm
        self.tolerance_heading = math.radians(tolerance_heading_deg)

    # --------------------------
    # Pose helpers
    # --------------------------
    def set_current_pose(self, x=0.0, y=0.0, theta_rad=0.0):
        with self._lock:
            self.pose = pose(x, y, theta_rad)

    def get_current_pose(self):
        with self._lock:
            return (self.pose.x, self.pose.y, self.pose.theta)

    def get_current_goal(self):
        with self._lock:
            return (self.goal.x, self.goal.y, self.goal.theta)        






# ---------------------------------------------------------------------
# Example usage / wiring
# ---------------------------------------------------------------------
if __name__ == "__main__":    

    try:        
        # create drive
        drive = Drive()
        #drive.config_drive(wheelbase_mm=100, wheel_diameter_mm=42.5)   #should be deleted
        #drive.calibrate_drive(ticks_to_mm=1/5.852, ticks_to_deg=9.12)   # to be created: ticks_to_deg
        drive.calibrate_drive(ticks_to_mm=1/5.852, wheelbase_mm=100)   # inbetween version...   

        final_time = 10000 + time.ticks_ms()
        print(">pico.Drive: start loop")

        if False:
            drive.set_pwm(10*drive.pwm_min,drive.pwm_min)     #direct PWM werkt in idle mode.

        if False:                                                # lijkt te werken, zowel pos/neg snelheden, zowel set_wheel_speed als set_velocity.
            print("set")
            drive.update_period_ms = 100                    # 100ms +/-50: * te hoog: mist resolutie, dus grote snelheids vertaging. * te laag: geen ruimte voor andere taken.
            drive.set_motor_pid( 50 , 250 , 0 )             # idle: 50,250,0
            drive.set_mode("direct")        
            print("move")
            #drive.set_wheel_speed(-200,+800)
            drive.set_velocity( 200, 0 )

        if True:
            drive.set_motor_pid( 50 , 250 , 0 )
            drive.set_rtc_update_period(50)
            drive.set_goal_seeker(Kp_speed=10, Kp_heading=25)   #heading is niet zelf-sturend.
            drive.set_goal_tolerances(tolerance_dist_mm=15, tolerance_heading_deg=5)
            drive.move_to(1000,0,200)
            #drive.rotate_to(90,200)
            drive.to_pose(400,0,0,200)


        while final_time>time.ticks_ms():   #run x seconds:
            drive.update()

            if drive._mode == "idle":
                break
            #print(drive.GetCurrentPose()) 

        print(">pico.Drive: TEST loop stopped.")
        drive.stop()

    #except Exception as X:
    #    print(X)

    finally:        
        drive.close()
