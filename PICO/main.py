from communication import Comm, Message, SimpleQueue
from differentialdrive import Drive
from lib.statusled import StatusLED
import time, math
#from missions import IdleMission, LineFollowingMission



# -----------------------------
# Robot class (Core 0)
# -----------------------------
class Robot:
    def __init__(self, input_q, output_q):
        self.in_q = input_q         # incomming message que between cores
        self.out_q = output_q       # outgoing message que between cores
        self._cmds_list = []        # stores robot messages, like waypoints for drive etc.
        self._run_cmds = False
        self.led = StatusLED()
        self.led.on()
        self.drive = Drive()
        self.comm_ip = "unknown"
        self.comm_uart0_connected = False
        self.pose_update_ms = 200  #send pose at reqular intervals        
        self.last_pose_update_ms = time.ticks_ms()
        #self.mission = None #IdleMission(self.out_q)
        self._inf_loop = False

    def handle_command(self, msg):
        cmd = Message(msg)   
        try:
            if cmd.target=="RBT":           # All public robot messages:
                if cmd.command=="stop":                    
                    self.led.blink_slow()
                    self.stop()
                elif cmd.command=="close":
                    self.close()
                elif cmd.command=="add_cmd":
                    cmd_to_store = ""
                    for idx, attr in enumerate(cmd.args):
                        if idx==0:
                            cmd_to_store = attr.upper()
                        elif idx==1:
                            cmd_to_store += " " + attr.lower() + " "
                        else:
                            cmd_to_store += attr + ","
                    self._cmds_list.append(cmd_to_store)
                    print(f">pico: {cmd_to_store}.")
                    print(self._cmds_list)
                elif cmd.command=="clear_cmds":
                    self._run_cmds = False
                    self._cmds_list = []
                elif cmd.command=="stop_cmds":
                    self._run_cmds = False
                elif cmd.command=="start_cmds":
                    self._run_cmds = True
                elif cmd.command=="run_square":
                    self.drive.reset()
                    distancex = int(cmd.args[0])
                    if cmd.args[1]=="ccw":
                        distancey = distancex
                        direction = 90
                    else:
                        distancey = -distancex
                        direction = -90
                    speed = int(cmd.args[2])
                    self._cmds_list = []
                    self._cmds_list.append(f"DRV move_to {distancex},0,{speed}")
                    self._cmds_list.append(f"DRV heading_to {direction},{speed}")
                    self._cmds_list.append(f"DRV move_to {distancex},{distancey},{speed}")
                    self._cmds_list.append(f"DRV heading_to {2*direction},{speed}")
                    self._cmds_list.append(f"DRV move_to 0,{distancey},{speed}")
                    self._cmds_list.append(f"DRV heading_to {3*direction},{speed}")
                    self._cmds_list.append(f"DRV move_to 0,0,{speed}")
                    self._cmds_list.append(f"DRV heading_to 0,{speed}")                    
                    self._run_cmds = True
                elif cmd.command=="info":
                    self.out_q.put(f"MSG COM ip: {self.comm_ip}.")
                    self.out_q.put(f"MSG COM use uart0: {self.comm_uart0_connected}.")
                    self.out_q.put(f"MSG LED mode: {self.led.mode}.")    
                    self.out_q.put(f"MSG DRV mode: {self.drive._mode}.")            
                elif cmd.command=="ping":
                    self.led.toggle_onoff()
                    self.out_q.put("MSG RBT says: PONG!")
                elif cmd.command=="ip":
                    if len(cmd.args):
                        self.comm_ip = str(cmd.args[0])
                    if self.comm_ip == "none" or self.comm_ip =="unknown":
                        self.led.blink_fast()
                    else:
                        self.led.blink_slow()
                elif cmd.command=="uart0":
                    self.comm_uart0_connected = bool(cmd.args[0]=="true" or cmd.args[0]==0)
                else:
                    raise Exception("unkown RBT.cmd")

            elif cmd.target=="DRV":         # All public drive messages:
                if cmd.command=="reset":
                    self.drive.reset()
                    self.out_q.put(f"LOG (reset): {self.drive.pose}")
                elif cmd.command=="set_pwm":
                    if len(cmd.args)==2:
                        self.drive._set_pwm( int(cmd.args[0]), int(cmd.args[1]) )
                elif cmd.command=="set_wheel_speed":
                    if len(cmd.args)==2:
                        self.drive.set_wheel_speed( float(cmd.args[0]), float(cmd.args[1]) )
                elif cmd.command=="set_velocity":
                    if len(cmd.args)==2:
                        self.drive.set_velocity( float(cmd.args[0]), (2*math.pi*float(cmd.args[1])) )
                elif cmd.command=="move_to":
                        self.drive.move_to( float(cmd.args[0]), float(cmd.args[1]), float(cmd.args[2]) )
                elif cmd.command=="rotate_to":
                        self.drive.rotate_to( math.radians(float(cmd.args[0])), float(cmd.args[1]) )
                elif cmd.command=="heading_to":
                        self.drive.heading_to( math.radians(float(cmd.args[0])), float(cmd.args[1]) )
                elif cmd.command=="to_pose":
                    if len(cmd.args)==4:
                        self.drive.to_pose( float(cmd.args[0]), float(cmd.args[1]), math.radians(float(cmd.args[2])), float(cmd.args[3]) )
                elif cmd.command=="status":
                    self.out_q.put(f"MSG DRV mode: {self.drive._mode}.")
                    self.out_q.put(f"LOG {self.drive.pose}")
                elif cmd.command=="info":
                    self.out_q.put(f"MSG DRV mode: {self.drive._mode}.")
                    self.out_q.put(f"MSG Calibration: Ticks_to_Dist={self.drive.ticks_to_dist}; Wheel_base={self.drive.wheel_base}.") #update to ticks_to_deg→rad.
                    self.out_q.put(f"MSG RTC: Update period={self.drive.update_period_ms}.")
                    self.out_q.put(f"MSG Motor_PWM: min={self.drive.pwm_min}; max={self.drive.pwm_max}.")
                    self.out_q.put(f"MSG Motor_PID: ({self.drive._pid_control_l.Kp}, {self.drive._pid_control_l.Ki}, {self.drive._pid_control_l.Kd}).")
                    self.out_q.put(f"MSG Goal_Seeker: (Kp_speed={self.drive.Kp_speed}, Kp_heading={self.drive.Kp_heading})")
                    self.out_q.put(f"MSG Goal_Tolerance: (Distance={self.drive.tolerance_dist}, Heading={math.degrees(self.drive.tolerance_heading):.1f})")
                    self.out_q.put(f"MSG Goal_last: v={self.drive.v_goal:.1f}; w={self.drive.w_goal*math.pi*2:.1f}.")
                    self.out_q.put(f"MSG Goal_last: target={self.drive.goal}.")
                elif cmd.command=="set_pose":   #( x, y, t_deg )
                    self.drive.set_current_pose( float(cmd.args[0]), float(cmd.args[1]), math.radians(float(cmd.args[2])) )
                elif cmd.command=="calibrate_drive":    # ( ticks_to_mm, ticks_to_deg )
                    self.drive.calibrate_drive( float(cmd.args[0]), float(cmd.args[1]) ) #, float(cmd.args[1])
                elif cmd.command=="set_rtc_update_period":    # ( update_period_ms=100 )
                    self.drive.set_rtc_update_period( int(cmd.args[0]) )
                elif cmd.command=="set_motor_pwm_range":    # ( pwm_min )
                    self.drive.set_motor_pwm_range( int(cmd.args[0]), int(cmd.args[1]) )
                elif cmd.command=="set_motor_pid":    # ( Kp, Ki, Kd )
                    self.drive.set_motor_pid( float(cmd.args[0]), float(cmd.args[1]), float(cmd.args[2]) )
                elif cmd.command=="set_goal_seeker":    # ( Kp_speed=1.0, Kp_heading=1.0 )
                    self.drive.set_goal_seeker( float(cmd.args[0]), float(cmd.args[1]) )
                elif cmd.command=="set_goal_tolerances":    # ( tolerance_dist_mm, tolerance_heading_deg )
                    self.drive.set_goal_tolerances( float(cmd.args[0]), float(cmd.args[1]) )
                else:
                    raise Exception("unkown DRV.cmd")
            else:
                raise Exception("unkown target")
        except Exception as E:
            print(f">RBT msg issue ({E}): {cmd}.")



    def loop(self):
        if self.comm_uart0_connected:
            self.led.blink_slow
        else:
            self.led.blink_fast
        counter = 0
        self._inf_loop = True

        while self._inf_loop:
            t_calc = time.ticks_ms()

            # Process commands from PC
            while not self.in_q.empty():
                cmd_raw = self.in_q.get()
                self.handle_command(cmd_raw)

            # Update status led:
            self.led._update()

            # Check cmds list:
            if self._run_cmds:
                if self.drive._mode == "idle" and len(self._cmds_list)>0:
                    msg = self._cmds_list.pop(0)
                    print(f">pico: executing next cmd: {msg}.")
                    self.out_q.put(f"MSG executing: {msg}")
                    self.handle_command(msg)
                else:
                    self._run_cmds = bool(len(self._cmds_list)>0)

            # Step current drive/mission            
            if self.drive.update():
                t_calc = time.ticks_diff(time.ticks_ms(), t_calc) #calculate drive.update() time.
                if counter==4:
                    self.out_q.put(f"LOG ({self.drive.last_update_ms}): {self.drive.pose}")
                    counter = 0
                counter += 1
                #print(f">RBT({t_calc}); DRV {self.drive._mode}; pwm: {self.drive.pwm_l}/{self.drive.pwm_r}; v_real: {self.drive.v_l_meas}/{self.drive.v_r_meas}; {self.drive.pose}.")
                #print(f">RBT({t_calc}): pwm: {self.drive.pwm_l}/{self.drive.pwm_r}; {self.drive._mode}; dTicks: {self.drive.dticks_l}/{self.drive.dticks_r}; v_real: {self.drive.v_l_meas}/{self.drive.v_r_meas}; {self.drive.pose}.")
                print(f">RBT({t_calc}): pwm: {self.drive.pwm_l}/{self.drive.pwm_r}; v_real: {self.drive.v_l_meas}/{self.drive.v_r_meas}; E_dist {self.drive.e_dist:.1f}; E_deg: {math.degrees(self.drive.e_angl):.1f}; {self.drive.pose}.")

            if self.drive._mode == "done":
                self.out_q.put(f"LOG (done): {self.drive.pose}")
                self.drive._mode = "idle"

    def stop(self):
        self._run_cmds = False
        self.drive.stop()
        self.out_q.put("MSG Robot_is_stopped..")

    def close(self):
        self._run_cmds = False
        self._inf_loop = False
        self.out_q.put("MSG Robot_is_closing..")
        self.drive.close()
        self.led.off()
        
        

# -----------------------------
# System start: comms & robot loop
# -----------------------------
def main_robot_loop():
    input_q = SimpleQueue()
    output_q = SimpleQueue()

    comm = Comm(input_q, output_q)
    robot = Robot(input_q, output_q)

    try:
        # Run noneb-locking Comm on Core 1
        comm.loop()        
        # Run Robot-Loop on Core 0
        robot.loop()

    #except Exception as E:
    #    print(">pico: Exit by Exception:")
    #    print(E)
    except KeyboardInterrupt:
        print(">pico: Exit by KeyboardInterrupt")    
    finally:
        robot.close()
        comm.close()

# --- Run robot loop ---
main_robot_loop()